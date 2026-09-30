# Ported verbatim (pure Python, no ROS deps) from CPX-Mono's
# ros2/src/global_trust_perception/global_trust_perception/trust_calculations/deferred.py
# — see that repo for the full design writeup. Only the import of consistency
# was repointed at this package; no logic changed.
"""
Deferred verdicts for uncorroborated other_only detections.

An other_only detection (something a peer reports that ego cannot see) is
ambiguous: it may be a fabricated ghost, or a real object genuinely occluded
from everyone else. The local score cannot tell them apart — high confidence is
exactly the signature of BOTH a confident liar and a confident unique witness.
TIME can: occlusion is transient in a moving scene, a fabrication is
uncorroborated forever. So judgment is DELAYED, not immediate.

Per (agent, track) this ledger holds an uncorroborated track from its FIRST
sighting and:
  - within T_DEADLINE_FLUSHES flushes of first sight -> held, charges nothing
    (occlusion assumed plausible);
  - corroborated in time (peer reputation mass reaches the threshold, or ego
    matches it) -> settle CORRECT, FULL per-frame back-pay of every held frame
    (the "good sensor in a bad spot" gets its full reward retroactively);
  - deadline expires with no corroboration -> settle INCORRECT, back-charge
    every held frame at pen(c); each subsequent uncorroborated frame then
    charges pen(c) instantly (persistent-ghost drain). Nothing charged is ever
    refunded: corroboration arriving after expiry stops the billing but does
    not return it, so a track that sat a whole grace window uncorroborated
    stays net-punished for it;
  - track dies still uncorroborated, having outlived the deadline -> settle
    INCORRECT with the same back-charge; a track that dies inside the grace
    window is forgiven (see the death rule below).

A sender's claim that an object EXISTS is taken at face value the moment it
makes it: the local pipeline has already dropped what its sender was unsure
of, so the global side does not re-filter on tracker confirmation. Time,
not a min_hits warm-up, is what separates a ghost from an occluded object
here, and the grace window above is the only delay before judgment.

Each held frame settles exactly once, to CORRECT xor INCORRECT: no
double-punishment and no verdict lost to a corroboration/expiry race. The
policy numbers (deadline, pen, threshold) are imported from consistency so the
"what is correct" definition has a single home; this module owns only the
"when and how it is accumulated" state. In-memory only — a restart forgives
pending verdicts (see trustworthy_perception's future-work notes).
"""

from dataclasses import dataclass, field

from sdsm_trust_perception.global_trust_perception.trust_calculations.consistency import (
    PENALTY_FLOOR_BETA,
    T_DEADLINE_FLUSHES,
    _clip01,
    is_corroborated,
    pen,
)


@dataclass
class _Entry:
    """One pending (agent, track): accumulated held frames not yet settled.

    sum_certainty and sum_credit both accumulate one value per held frame,
    but drive opposite outcomes: sum_certainty (raw local certainty) feeds
    _backcharge -- the INCORRECT side -- and is never discounted; sum_credit
    (certainty already discounted by kinematic plausibility, KDS, upstream)
    feeds the CORRECT-side back-pay. Keeping them separate means a detection
    that looks kinematically implausible earns less credit if it turns out
    genuine, without ALSO earning a softer penalty if it turns out to be a
    ghost -- KDS must only ever suppress unearned credit, never blunt a
    deserved charge.

    Expiry zeroes all three accumulators: those frames have been settled by the
    back-charge and must never be charged or paid twice. Nothing charged is
    ever refunded -- an expired track that is later corroborated simply stops
    being billed (see observe).
    """
    n_pending: int
    sum_certainty: float
    sum_credit: float
    first_frame: int
    expired: bool = False


@dataclass
class LedgerDelta:
    """What one agent's ledger contributes to this frame's verdict counts."""
    correct: float = 0.0     # back-paid credit to add to C
    incorrect: float = 0.0   # back-charge + post-expiry instant penalty to add to I
    pending: int = 0         # tracks still held this frame (for the held log)
    settled_correct: int = 0    # count of tracks settled correct this frame (diag)
    settled_incorrect: int = 0  # count of tracks settled/charged incorrect (diag)


def _backcharge(entry: _Entry) -> float:
    """Sum of pen(c) over an entry's held frames = n*beta + (1-beta)*sum_c."""
    return entry.n_pending * PENALTY_FLOOR_BETA + (1.0 - PENALTY_FLOOR_BETA) * entry.sum_certainty


class PendingVerdicts:
    """Per-engine ledger of deferred other_only verdicts, keyed (agent, track)."""

    def __init__(self, deadline_flushes: int = T_DEADLINE_FLUSHES, support_threshold: float = None):
        self._deadline = int(deadline_flushes)
        self._theta = support_threshold
        # agent_key -> {track_id -> _Entry}
        self._entries: dict = {}

    def observe(self,
                agent_key,
                frame_idx: int,
                other_only,
                vindicated_tids=()) -> LedgerDelta:
        """
        Advance one agent's ledger by a frame and return the verdict deltas.

        other_only     : iterable of (track_id, certainty, credit, support)
                         for this agent's other_only detections this frame.
                         `certainty` is the raw local certainty and drives
                         every INCORRECT-side amount (_backcharge, pen(c)) --
                         never discounted. `credit` is `certainty` already
                         discounted by kinematic plausibility (KDS) upstream,
                         and drives every CORRECT-side amount (corroboration,
                         back-pay, vindication) -- so a detection that looks
                         kinematically implausible earns less reward if it
                         turns out genuine, without earning a softer penalty
                         if it turns out to be a ghost. The deadline clock
                         starts at the FIRST sighting.
        vindicated_tids : track_ids that became ego-matched this frame. This
                         frame's credit is already given by the matched path, so
                         a vindicated pending track back-pays only its PRIOR held
                         frames (never double-counting the current one).

        A pending track absent from both inputs this frame is treated as a track
        death: settled INCORRECT if it outlived the deadline, else forgiven
        (unless already expired).
        """
        delta = LedgerDelta()
        entries = self._entries.setdefault(agent_key, {})
        vindicated = set(vindicated_tids)
        active = {tid for tid, _, _, _ in other_only} | vindicated

        # 1. Vindication: previously pending, now ego-matched.
        for tid in vindicated:
            e = entries.pop(tid, None)
            if e is not None and not e.expired:
                delta.correct += e.sum_credit
                delta.settled_correct += 1

        # 2. This frame's other_only detections.
        for tid, c, credit, support in other_only:
            c = _clip01(c)
            credit = _clip01(credit)
            e = entries.get(tid)

            # Corroborated -> settle correct, back-paying every prior held frame.
            if is_corroborated(support, self._theta):
                if e is None:
                    delta.correct += credit                  # corroborated on first sight
                elif e.expired:
                    delta.correct += credit                  # stop the bleeding; no refund
                    entries.pop(tid, None)
                else:
                    delta.correct += e.sum_credit + credit   # full back-pay of held frames
                    entries.pop(tid, None)
                delta.settled_correct += 1
                continue

            # Held this frame: open or extend the pending entry. The deadline runs
            # from first_frame (first sighting).
            if e is None:
                entries[tid] = _Entry(n_pending=1, sum_certainty=c, sum_credit=credit,
                                      first_frame=frame_idx)
                delta.pending += 1
            elif e.expired:
                delta.incorrect += pen(c)                    # past deadline: charge every frame
                delta.settled_incorrect += 1
            else:
                e.n_pending += 1
                e.sum_certainty += c
                e.sum_credit += credit
                if frame_idx - e.first_frame >= self._deadline:
                    delta.incorrect += _backcharge(e)        # deadline: retro-charge held frames
                    delta.settled_incorrect += 1
                    e.expired = True
                    e.n_pending = 0
                    e.sum_credit = 0.0
                    e.sum_certainty = 0.0
                else:
                    delta.pending += 1

        # 3. Deaths: pending tracks not reported and not vindicated this frame.
        #
        # A track that dies INSIDE the grace window is forgiven. Charging
        # it would bypass the window entirely: the whole point of the deadline
        # is that an uncorroborated track has not yet had a fair chance to be
        # witnessed, and dying does not change that. It matters in practice
        # because a real tracker re-IDs constantly -- ~10-14% of ids on this
        # source do not survive frame to frame -- so charging every young
        # death drains an honest sender's reputation at the churn rate,
        # regardless of how long the deadline is.
        #
        # The spawn-and-abandon hole this used to close stays closed for the
        # attacker it was aimed at: a fabricator's ghost has to persist to be
        # worth anything, and anything that lives past the deadline is still
        # charged on death (below) as well as on expiry.
        for tid in [t for t in entries if t not in active]:
            e = entries.pop(tid)
            lived = frame_idx - e.first_frame
            if not e.expired and lived >= self._deadline:
                delta.incorrect += _backcharge(e)
                delta.settled_incorrect += 1

        return delta

    def status(self, agent_key, track_id) -> str | None:
        """
        Read-only verdict state of one pending track, for DISPLAY only.

            'expired'  - the grace window closed with no corroboration, so this
                         track is now charged every frame (a persistent ghost)
            'deferred' - still inside the grace window: held, not yet charged
            None       - no open entry (never seen, or already settled)

        Returns a label rather than the _Entry, so callers never learn how the
        deadline is tracked; that stays this module's business. Pure read: it
        advances nothing, so calling it cannot perturb the ledger.
        """
        entry = self._entries.get(agent_key, {}).get(track_id)
        if entry is None:
            return None
        return 'expired' if entry.expired else 'deferred'

    def drop_agent(self, agent_key) -> None:
        """Forget an agent's ledger (e.g. when it leaves the scene for good)."""
        self._entries.pop(agent_key, None)
