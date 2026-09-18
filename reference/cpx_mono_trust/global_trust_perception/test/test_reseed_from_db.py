"""
Integration tests for the reseed seam: TrustEngine.get_reputation reading
the reputation DB and applying absence_decay to the stored value.

These pin the wiring that unit tests cannot see: gap units (seconds), the
downward-only decay applied at seed time, the NULL-ts reset policy, and
record_initial being reserved for genuinely new agents.

Requires TrustEngine (container-bound via mmcooper_fuse fusion path).
"""

import math
import sqlite3
import time

import pytest

from global_trust_perception.pipeline.persistent_reputation_tracker import PersistentReputationTracker
from global_trust_perception.trust_calculations.reputation import REPUTATION_DEFAULT
from global_trust_perception.trust_calculations.reputation_multipliers.absence_decay import (
    GRACE_GAP_S,
    TAU_GAP_S,
    absence_decay,
)
from global_trust_perception.pipeline.trustworthy_perception import TrustEngine

AGENT = 7
SRC = '07 00 00 00'
SRC_TUPLE = (7, 0, 0, 0)


def _tracker_with_history(db_path: str, r: float, age_s: float) -> PersistentReputationTracker:
    """A tracker whose DB holds one record for SRC, backdated by age_s."""
    t = PersistentReputationTracker(db_path)
    t.record(SRC_TUPLE, r, frame_num=1)
    t.flush(current_frame=1)
    t._conn.execute('UPDATE reputation_log SET ts = ?', (time.time() - age_s,))
    t._conn.commit()
    return t


# --- absence decay applied at seed time ------------------------------------------

def test_fresh_history_seeds_stored_value(tmp_path):
    t = _tracker_with_history(str(tmp_path / 'a.db'), 0.9, age_s=0.0)
    r = TrustEngine(t).get_reputation(AGENT, SRC)
    assert r == pytest.approx(0.9, abs=1e-3)


def test_half_life_gap_decays_halfway(tmp_path):
    """The seam test that catches unit bugs: a gap of one half-life must move
    0.9 exactly halfway to baseline (0.7). Wrong units (ms vs s) would land
    at ~0.5 or ~0.9 instead."""
    half_life = GRACE_GAP_S + TAU_GAP_S * math.log(2)
    t = _tracker_with_history(str(tmp_path / 'b.db'), 0.9, age_s=half_life)
    r = TrustEngine(t).get_reputation(AGENT, SRC)
    assert r == pytest.approx(0.7, abs=1e-3)


def test_month_gap_resets_high_rep_to_near_baseline(tmp_path):
    month = 30 * 24 * 3600
    t = _tracker_with_history(str(tmp_path / 'c.db'), 0.9, age_s=month)
    r = TrustEngine(t).get_reputation(AGENT, SRC)
    assert abs(r - REPUTATION_DEFAULT) < 0.02


def test_below_baseline_history_not_upgraded(tmp_path):
    """Downward-only through the seam: a distrusted agent does not launder
    its reputation by staying away for a month."""
    month = 30 * 24 * 3600
    t = _tracker_with_history(str(tmp_path / 'd.db'), 0.15, age_s=month)
    r = TrustEngine(t).get_reputation(AGENT, SRC)
    assert r == pytest.approx(0.15, abs=1e-6)


def test_seed_matches_pure_function(tmp_path):
    """Engine seed == absence_decay(stored, gap) for an arbitrary gap."""
    age = 3 * 24 * 3600
    t = _tracker_with_history(str(tmp_path / 'e.db'), 0.85, age_s=age)
    r = TrustEngine(t).get_reputation(AGENT, SRC)
    assert r == pytest.approx(absence_decay(0.85, age), abs=1e-3)


# --- caching and registration ------------------------------------------------------

def test_seed_happens_once_then_cached(tmp_path):
    """The DB is consulted only on first sight; later calls use engine state."""
    t = _tracker_with_history(str(tmp_path / 'f.db'), 0.9, age_s=0.0)
    engine = TrustEngine(t)
    r_first = engine.get_reputation(AGENT, SRC)
    t._conn.execute('UPDATE reputation_log SET r_new = 0.1')
    t._conn.commit()
    assert engine.get_reputation(AGENT, SRC) == r_first


def test_brand_new_agent_seeds_default_and_registers(tmp_path):
    t = PersistentReputationTracker(str(tmp_path / 'g.db'))
    engine = TrustEngine(t)
    assert engine.get_reputation(AGENT, SRC) == REPUTATION_DEFAULT
    t.flush(current_frame=0)
    assert t.get_last_reputation(SRC) == REPUTATION_DEFAULT   # record_initial ran


# --- NULL-ts legacy policy ----------------------------------------------------------

def _legacy_only_tracker(db_path: str, r: float) -> PersistentReputationTracker:
    """Tracker opened on a genuine pre-migration DB (schema without ts):
    opening it migrates the schema, leaving the old row with ts = NULL."""
    conn = sqlite3.connect(db_path)
    conn.execute('CREATE TABLE sessions (id INTEGER PRIMARY KEY, source_id TEXT NOT NULL)')
    conn.execute(
        'CREATE TABLE reputation_log ('
        'id INTEGER PRIMARY KEY, '
        'session_id INTEGER NOT NULL REFERENCES sessions(id), '
        'r_new REAL NOT NULL)'
    )
    cur = conn.execute('INSERT INTO sessions (source_id) VALUES (?)', (SRC,))
    conn.execute(
        'INSERT INTO reputation_log (session_id, r_new) VALUES (?, ?)',
        (cur.lastrowid, r),
    )
    conn.commit()
    conn.close()
    return PersistentReputationTracker(db_path)


def test_legacy_agent_resets_to_default(tmp_path):
    """POLICY (documented in get_last_reputation_with_ts): history of
    unknowable age is treated as no usable history -> reset to 0.5."""
    t = _legacy_only_tracker(str(tmp_path / 'h.db'), 0.9)
    assert TrustEngine(t).get_reputation(AGENT, SRC) == REPUTATION_DEFAULT


def test_legacy_agent_gets_no_duplicate_session(tmp_path):
    """A legacy agent is already registered: reseeding must not open a second
    session or write a fake initial row on top of real history."""
    t = _legacy_only_tracker(str(tmp_path / 'i.db'), 0.9)
    TrustEngine(t).get_reputation(AGENT, SRC)
    t.flush(current_frame=0)
    n_sessions = t._conn.execute(
        'SELECT COUNT(*) FROM sessions WHERE source_id = ?', (SRC,)
    ).fetchone()[0]
    n_rows = t._conn.execute('SELECT COUNT(*) FROM reputation_log').fetchone()[0]
    assert n_sessions == 1
    assert n_rows == 1                       # only the legacy row, no initial row
