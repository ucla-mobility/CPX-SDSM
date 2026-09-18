"""
Unit tests for PersistentReputationTracker: sessions, batching, timeout
re-sessioning, retrieval ordering, close-flush, and the NULL-ts legacy path.

Pure sqlite3 + stdlib — runs anywhere. File-backed DBs (tmp_path) are used
where persistence across close() matters; ':memory:' elsewhere.
"""

import sqlite3
import time

from global_trust_perception.pipeline.persistent_reputation_tracker import (
    SESSION_TIMEOUT,
    PersistentReputationTracker,
)

SRC = '01 00 00 00'          # source_id key format: 4 hex bytes
SRC_TUPLE = (1, 0, 0, 0)     # record() takes the raw tuple
OTHER_SRC = '02 00 00 00'


def _sessions_for(tracker, src=SRC) -> int:
    return tracker._conn.execute(
        'SELECT COUNT(*) FROM sessions WHERE source_id = ?', (src,)
    ).fetchone()[0]


def _log_rows(tracker) -> list:
    return tracker._conn.execute(
        'SELECT session_id, r_new, ts FROM reputation_log ORDER BY id'
    ).fetchall()


# --- record / flush / retrieval -------------------------------------------------

def test_unknown_agent_returns_none():
    t = PersistentReputationTracker(':memory:')
    assert t.get_last_reputation(SRC) is None
    assert t.get_last_reputation_with_ts(SRC) is None


def test_record_is_buffered_until_flush():
    """record() alone does not hit the DB; flush() writes the batch."""
    t = PersistentReputationTracker(':memory:')
    t.record(SRC_TUPLE, 0.6, frame_num=1)
    assert t.get_last_reputation(SRC) is None      # still only in the buffer
    t.flush(current_frame=1)
    assert t.get_last_reputation(SRC) == 0.6


def test_last_reputation_is_most_recent_row():
    t = PersistentReputationTracker(':memory:')
    for i, r in enumerate([0.6, 0.7, 0.8]):
        t.record(SRC_TUPLE, r, frame_num=i)
    t.flush(current_frame=3)
    assert t.get_last_reputation(SRC) == 0.8


def test_with_ts_returns_recent_wall_clock():
    t = PersistentReputationTracker(':memory:')
    before = time.time()
    t.record(SRC_TUPLE, 0.75, frame_num=1)
    t.flush(current_frame=1)
    r, ts, risk_l = t.get_last_reputation_with_ts(SRC)
    assert r == 0.75
    assert before <= ts <= time.time()
    assert risk_l == 0.0


def test_agents_are_isolated():
    t = PersistentReputationTracker(':memory:')
    t.record(SRC_TUPLE, 0.9, frame_num=1)
    t.record((2, 0, 0, 0), 0.1, frame_num=1)
    t.flush(current_frame=1)
    assert t.get_last_reputation(SRC) == 0.9
    assert t.get_last_reputation(OTHER_SRC) == 0.1


# --- sessions --------------------------------------------------------------------

def test_single_session_while_continuously_seen():
    t = PersistentReputationTracker(':memory:')
    for i in range(5):
        t.record(SRC_TUPLE, 0.6, frame_num=i)
        t.flush(current_frame=i)
    assert _sessions_for(t) == 1


def test_new_session_after_timeout_gap():
    t = PersistentReputationTracker(':memory:')
    t.record(SRC_TUPLE, 0.6, frame_num=1)
    # flush at a frame far enough ahead that the agent counts as silent
    t.flush(current_frame=1 + SESSION_TIMEOUT)
    t.record(SRC_TUPLE, 0.7, frame_num=10)
    t.flush(current_frame=10)
    assert _sessions_for(t) == 2
    # history survives re-sessioning: latest value still retrievable
    assert t.get_last_reputation(SRC) == 0.7


def test_no_new_session_before_timeout():
    t = PersistentReputationTracker(':memory:')
    t.record(SRC_TUPLE, 0.6, frame_num=1)
    t.flush(current_frame=1 + SESSION_TIMEOUT - 1)   # one frame short of stale
    t.record(SRC_TUPLE, 0.7, frame_num=2)
    t.flush(current_frame=2)
    assert _sessions_for(t) == 1


def test_get_last_reputation_spans_sessions():
    """Retrieval must cross session boundaries: latest row of ANY session."""
    t = PersistentReputationTracker(':memory:')
    t.record(SRC_TUPLE, 0.9, frame_num=1)
    t.flush(current_frame=1 + SESSION_TIMEOUT)       # session 1 closed at 0.9
    t.record(SRC_TUPLE, 0.3, frame_num=20)           # session 2 opens at 0.3
    t.flush(current_frame=20)
    assert _sessions_for(t) == 2
    assert t.get_last_reputation(SRC) == 0.3         # newest, not highest


# --- record_initial ---------------------------------------------------------------

def test_record_initial_opens_session_and_buffers_default():
    t = PersistentReputationTracker(':memory:')
    t.record_initial(SRC, 0.5)
    assert _sessions_for(t) == 1                     # session commits immediately
    t.flush(current_frame=0)
    assert t.get_last_reputation(SRC) == 0.5


def test_record_after_record_initial_reuses_session():
    t = PersistentReputationTracker(':memory:')
    t.record_initial(SRC, 0.5)
    t.record(SRC_TUPLE, 0.6, frame_num=1)
    t.flush(current_frame=1)
    assert _sessions_for(t) == 1


# --- close() durability ------------------------------------------------------------

def test_close_flushes_pending_buffer(tmp_path):
    db = str(tmp_path / 'rep.db')
    t = PersistentReputationTracker(db)
    t.record(SRC_TUPLE, 0.8, frame_num=1)
    t.close()                                        # no explicit flush
    reopened = PersistentReputationTracker(db)
    assert reopened.get_last_reputation(SRC) == 0.8
    reopened.close()


# --- NULL-ts legacy rows -------------------------------------------------------------

def _make_pre_migration_db(db_path: str, src: str, r: float):
    """Build a DB with the ORIGINAL schema (no ts column) and one record.

    Opening it with PersistentReputationTracker triggers the ALTER TABLE
    migration, which adds ts as a nullable column — the pre-existing row
    keeps ts = NULL. (The current schema declares ts NOT NULL, so NULL-ts
    rows can ONLY come from this migration path.)
    """
    conn = sqlite3.connect(db_path)
    conn.execute('CREATE TABLE sessions (id INTEGER PRIMARY KEY, source_id TEXT NOT NULL)')
    conn.execute(
        'CREATE TABLE reputation_log ('
        'id INTEGER PRIMARY KEY, '
        'session_id INTEGER NOT NULL REFERENCES sessions(id), '
        'r_new REAL NOT NULL)'
    )
    cur = conn.execute('INSERT INTO sessions (source_id) VALUES (?)', (src,))
    conn.execute(
        'INSERT INTO reputation_log (session_id, r_new) VALUES (?, ?)',
        (cur.lastrowid, r),
    )
    conn.commit()
    conn.close()


def test_null_ts_rows_visible_plain_but_not_with_ts(tmp_path):
    """The documented legacy split: get_last_reputation sees pre-migration
    rows; get_last_reputation_with_ts skips them (returns None), which is
    what triggers the engine's reset-to-default policy."""
    db = str(tmp_path / 'legacy.db')
    _make_pre_migration_db(db, SRC, 0.9)
    t = PersistentReputationTracker(db)              # runs the ts migration
    assert t.get_last_reputation(SRC) == 0.9
    assert t.get_last_reputation_with_ts(SRC) is None
    t.close()


def test_with_ts_finds_timestamped_rows_after_migration(tmp_path):
    """New records written AFTER migrating a legacy DB are timestamped and
    become the with_ts result, ending the reset-to-default phase."""
    db = str(tmp_path / 'mixed.db')
    _make_pre_migration_db(db, SRC, 0.9)             # legacy row, ts NULL
    t = PersistentReputationTracker(db)
    t.record(SRC_TUPLE, 0.7, frame_num=1)            # post-migration record
    t.flush(current_frame=1)
    assert t.get_last_reputation(SRC) == 0.7         # plain: newest row
    r, ts, risk_l = t.get_last_reputation_with_ts(SRC)
    assert r == 0.7                                  # with_ts: newest usable row
    assert ts is not None
    assert risk_l == 0.0
    t.close()
