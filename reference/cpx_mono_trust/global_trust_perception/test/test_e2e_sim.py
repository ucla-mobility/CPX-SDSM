"""
End-to-end world simulation: scripted agent personas driven through the real
TrustEngine + SQLite tracker for many frames, asserting TRAJECTORY-level
outcomes (who ends up trusted, when verdicts flip, what the DB holds) rather
than single-frame values.

The "world" is two vehicles driving east at 1 m/s, observed perfectly by ego
(agent 1). Personas report per frame:

    honest          truth with a small (0.1 m) sensor offset
    liar            reports only a persistent fabricated object nobody else sees
                    (merely reporting nothing is forgiven now, so a liar has to
                    fabricate; driven with TrackData so the ledger grades it)
    ghost injector  truth PLUS a persistent fake object
    stale sender    honest data, but every message beyond the freshness cutoff
    dropout         honest, vanishes, returns after a long absence (DB-seeded)

Per-frame DB recording mirrors agent.py's flush loop (record + flush), so the
dropout scenarios exercise the same persistence path the live node uses.

No ROS: pure-Python engine drive (needs the MS-PSF fusion path, shapely/scipy
-> container-only, same as the other pipeline tests). Most tests pass no
TrackData, so the deferred ledger stays dormant (it needs stable track ids)
and other_only ghosts are held rather than graded. The liar tests DO pass
TrackData (via _run's track_agents) so the ledger back-charges the fabrication
over the grace window; the ledger's grade-over-time detail is covered by
test_confirmed_ghost_penalized, tracker-node dynamics by test_sort_min_hits.
"""

import math
import sqlite3
import time

import pytest

from global_trust_perception.trust_calculations.consistency import T_DEADLINE_FLUSHES as DEADLINE
from global_trust_perception.trust_calculations.consistency_checks.kinematic_checks import TrackData
from global_trust_perception.pipeline.persistent_reputation_tracker import PersistentReputationTracker
from global_trust_perception.trust_calculations.reputation import (
    DELTA,
    R_MAX,
    REPUTATION_DEFAULT,
    dynamic_threshold,
)
from global_trust_perception.trust_calculations.reputation_multipliers.absence_decay import GRACE_GAP_S, TAU_GAP_S
from global_trust_perception.trust_calculations.reputation_multipliers.kinematic_freshness import (
    D_THRESHOLD_M,
    F_FLOOR,
    GRACE_M,
    R_STAR,
    RHO_DIST,
)
from global_trust_perception.pipeline.trustworthy_perception import TrustEngine

DIMS = (2.0, 4.5, 1.5)
SPEED_X = 1.0            # m/s east
DT = 0.5                 # s per frame
GHOST = (60.0, 60.0, 0.0)

# Blind distance at which F reaches F_FLOOR exactly (solve
# RHO_DIST^(dist/D_THRESHOLD_M) == F_FLOOR for dist).
_DIST_FLOOR = GRACE_M + D_THRESHOLD_M * math.log(F_FLOOR) / math.log(RHO_DIST)

# An absurdly high stubbed speed: paired with _PLACEHOLDER_LATENCY_S below,
# comfortably past _DIST_FLOOR regardless of retuning.
_STALE_SPEED = _DIST_FLOOR * 100.0
_PLACEHOLDER_LATENCY_S = 1.0


class _AlwaysDiverged:
    """Test double for SenderMotionHistory: forces the kinematic freshness
    factor to F_FLOOR every frame by always reporting an absurdly high
    speed, standing in for 'every message this sender ever sends is beyond
    the freshness cutoff' without needing to fabricate genuine per-frame
    motion."""

    def update(self, agent_id, ref_x, ref_y, recv_t):
        return _STALE_SPEED

# First frame index entered with R_old >= R_STAR for a perfect climber
# (R rises by DELTA per agreement frame from REPUTATION_DEFAULT). Derived so
# retuning DELTA or the tau curve moves every trajectory assertion with it.
FIRST_TRUSTED = math.ceil((R_STAR - REPUTATION_DEFAULT) / DELTA)


# --- the world -------------------------------------------------------------------

def _truth(frame: int) -> list:
    """Two vehicles driving east; positions at the given frame."""
    dx = SPEED_X * DT * frame
    return [(3.0 + dx, 5.0, 0.0), (4.0 + dx, 2.0, 0.0)]


# --- personas: frame -> reported positions -----------------------------------------

def honest(frame):
    return [(x + 0.1, y, z) for x, y, z in _truth(frame)]


def liar(frame):
    """Reports only a persistent fake nobody else sees.

    Merely reporting nothing is forgiven now (all ego_only -> held), so a liar
    has to actually fabricate. Drive it with track_agents so the ghost confirms
    and the deferred ledger back-charges it.
    """
    return [GHOST]


def ghost_injector(frame):
    return honest(frame) + [GHOST]


# --- sim driver ---------------------------------------------------------------------

def _tracked(points: list) -> TrackData:
    """A stationary TrackData for each reported point (stable ids)."""
    return TrackData(
        track_ids=list(range(len(points))),
        kalman_x=[p[0] for p in points],
        kalman_y=[p[1] for p in points],
        kalman_vx=[0.0] * len(points),
        kalman_vy=[0.0] * len(points),
    )


def _run(engine, tracker, reporters, n_frames, start=0, ref_pos_agents=(),
         track_agents=()):
    """Drive n_frames through the engine, recording to the DB like agent.py.

    reporters: dict[agent_id -> persona function]. track_agents: ids whose
    detections are fed as TrackData, activating the deferred ledger
    for them (the rest stay ledger-dormant). ref_pos_agents: ids that get a
    ref_pos_by_agent entry every frame (feeding the kinematic freshness
    factor) -- the values themselves are placeholders unless engine's
    _sender_motion has been swapped for a test double, see
    test_stale_sender_scores_like_honest_but_is_never_admitted. Returns
    per-agent trajectories.
    """
    history = {aid: [] for aid in reporters}
    src_map = {aid: (aid, 0, 0, 0) for aid in reporters}
    for f in range(start, start + n_frames):
        positions = {aid: persona(f) for aid, persona in reporters.items()}
        dims = {aid: [DIMS] * len(p) for aid, p in positions.items()}
        ego = _truth(f)
        tracks = ({aid: _tracked(positions[aid]) for aid in track_agents}
                  if track_agents else None)
        ref_pos = ({aid: (0.0, 0.0, float(f), _PLACEHOLDER_LATENCY_S)
                    for aid in ref_pos_agents}
                   if ref_pos_agents else None)
        stats = engine.process_frame(
            positions, ego, dims, [DIMS] * len(ego), src_map,
            ref_pos_by_agent=ref_pos,
            tracks_by_agent=tracks,
        )
        for s in stats:
            history[s.agent_id].append(s)
            tracker.record(src_map[s.agent_id], s.r_new, f)
        tracker.flush(f)
    return history


def _fresh_engine():
    tracker = PersistentReputationTracker(':memory:')
    return TrustEngine(tracker), tracker


# --- single-persona arcs --------------------------------------------------------------

def test_honest_agent_earns_and_keeps_trust():
    engine, tracker = _fresh_engine()
    traj = _run(engine, tracker, {2: honest}, n_frames=10)[2]
    assert all(s.correct == 2 and s.incorrect == 0 for s in traj)
    assert traj[-1].r_new == R_MAX                    # saturates at the cap
    # untrusted while proving itself, trusted from the first frame entered
    # with R_old past the crossover — and never flips back
    assert all(not s.trusted for s in traj[:FIRST_TRUSTED])
    assert all(s.trusted for s in traj[FIRST_TRUSTED:])


def test_liar_sinks_to_zero_and_is_never_admitted():
    engine, tracker = _fresh_engine()
    traj = _run(engine, tracker, {2: liar}, n_frames=DEADLINE + 8,
                track_agents={2})[2]
    # occlusion assumed during the grace window (held, uncharged), then the ghost
    # is back-charged as Incorrect and drains the fabricator to the floor.
    assert all(s.incorrect == 0 for s in traj[:DEADLINE])
    assert traj[-1].incorrect > 0
    assert traj[-1].r_new == 0.0
    assert not any(s.trusted for s in traj)


def test_honest_and_liar_coexist_with_divergent_verdicts():
    """One shared world, opposite trajectories; the liar's presence does not
    contaminate the honest agent's scoring."""
    engine, tracker = _fresh_engine()
    hist = _run(engine, tracker, {2: honest, 3: liar}, n_frames=DEADLINE + 8,
                track_agents={3})
    assert hist[2][-1].r_new == 1.0
    assert hist[3][-1].r_new == 0.0
    assert all(s.trusted for s in hist[2][FIRST_TRUSTED:])
    assert not any(s.trusted for s in hist[3])


def test_ghost_injector_climbs_but_ghost_is_flagged_uncorroborated():
    """Without TrackData the deferred ledger stays dormant, so a ghost injector
    that agrees on real objects still climbs to full reputation (the ghost is
    held, not charged). But the ghost's peer support is 0 every frame (nobody
    else reports it), so it is flagged uncorroborated on EVERY frame regardless
    of any other agent's gate status — the support-mass signal the ledger would
    grade once track ids are present (see test_confirmed_ghost_penalized)."""
    engine, tracker = _fresh_engine()
    hist = _run(engine, tracker, {2: ghost_injector, 3: honest}, n_frames=8)
    injector = hist[2]
    assert injector[-1].r_new == 1.0                  # climbs: ledger dormant here
    assert all(s.held >= 1 for s in injector)         # ghost held every frame
    assert all(s.uncorroborated == 1 for s in injector)  # ghost never corroborated


def test_stale_sender_scores_like_honest_but_is_never_admitted():
    """Freshness gates admission only: the stale twin's reputation trajectory
    is IDENTICAL to the fresh twin's, yet it is never trusted."""
    fresh_engine, fresh_tracker = _fresh_engine()
    stale_engine, stale_tracker = _fresh_engine()
    stale_engine._sender_motion = _AlwaysDiverged()   # every message beyond the cutoff
    fresh_traj = _run(fresh_engine, fresh_tracker, {2: honest}, 8)[2]
    stale_traj = _run(stale_engine, stale_tracker, {2: honest}, 8,
                      ref_pos_agents={2})[2]
    assert [s.r_new for s in stale_traj] == [s.r_new for s in fresh_traj]
    assert not any(s.trusted for s in stale_traj)
    assert any(s.trusted for s in fresh_traj)


# --- dropout and return: absence decay through a process restart -----------------------

def _backdate_db(db_path: str, gap_s: float):
    conn = sqlite3.connect(db_path)
    conn.execute('UPDATE reputation_log SET ts = ts - ?', (gap_s,))
    conn.commit()
    conn.close()


def _phase(db_path, reporters, n_frames, start=0, track_agents=()):
    """One node lifetime: fresh engine + tracker over the given DB file."""
    tracker = PersistentReputationTracker(db_path)
    engine = TrustEngine(tracker)
    hist = _run(engine, tracker, reporters, n_frames, start=start,
                track_agents=track_agents)
    tracker.close()
    return hist


def test_return_after_half_life_seeds_decayed_and_stays_trusted(tmp_path):
    db = str(tmp_path / 'dropout.db')
    _phase(db, {2: honest}, n_frames=8)               # earns R = R_MAX, then gone
    _backdate_db(db, GRACE_GAP_S + TAU_GAP_S * math.log(2))  # one half-life of effective absence
    traj = _phase(db, {2: honest}, n_frames=3, start=100)[2]
    # deviation from baseline halved (R_MAX=1.0 -> 0.75 at current constants)
    expected_seed = REPUTATION_DEFAULT + (R_MAX - REPUTATION_DEFAULT) * 0.5
    assert traj[0].r_old == pytest.approx(expected_seed, abs=1e-3)
    # verdict follows the gate formula at the seeded value
    assert traj[0].trusted == (expected_seed >= dynamic_threshold(expected_seed))


def test_return_after_a_month_must_re_earn_trust(tmp_path):
    """The full arc: a month away resets an R=R_MAX agent to ~baseline; it
    re-enters as a stranger and re-earns admission over the same climb an
    unknown agent would need."""
    db = str(tmp_path / 'longgone.db')
    _phase(db, {2: honest}, n_frames=8)
    _backdate_db(db, 30 * 24 * 3600)
    traj = _phase(db, {2: honest}, n_frames=FIRST_TRUSTED + 3, start=100)[2]
    assert abs(traj[0].r_old - REPUTATION_DEFAULT) < 0.02
    assert not traj[0].trusted                        # stranger again
    assert traj[-1].trusted                           # trust re-earned by climbing


def test_liar_gains_nothing_from_a_month_away(tmp_path):
    """Downward-only decay, end to end: a burned agent (R=0) cannot launder
    its reputation by disappearing."""
    db = str(tmp_path / 'burned.db')
    _phase(db, {2: liar}, n_frames=DEADLINE + 8, track_agents={2})  # R driven to 0.0
    _backdate_db(db, 30 * 24 * 3600)
    traj = _phase(db, {2: liar}, n_frames=2, start=100, track_agents={2})[2]
    assert traj[0].r_old == pytest.approx(0.0, abs=1e-9)
    assert not traj[0].trusted


# --- persistence sanity -----------------------------------------------------------------

def test_db_holds_full_history_for_scored_agents_only(tmp_path):
    db = str(tmp_path / 'hist.db')
    n = 6
    _phase(db, {2: honest, 3: liar}, n_frames=n)
    conn = sqlite3.connect(db)
    def rows(src):
        return conn.execute(
            '''SELECT COUNT(*) FROM reputation_log r
               JOIN sessions s ON r.session_id = s.id
               WHERE s.source_id = ?''', (src,)
        ).fetchone()[0]
    # one initial-seed row + one row per frame, per scored agent
    assert rows('02 00 00 00') == n + 1
    assert rows('03 00 00 00') == n + 1
    # ego (agent 1) is ground truth: never scored, never written
    assert rows('01 00 00 00') == 0
    conn.close()
