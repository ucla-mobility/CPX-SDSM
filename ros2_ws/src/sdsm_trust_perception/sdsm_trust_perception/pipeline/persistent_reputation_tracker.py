# Ported verbatim (pure Python, no ROS deps) from CPX-Mono's
# ros2/src/global_trust_perception/global_trust_perception/pipeline/persistent_reputation_tracker.py
# — see that repo for the full design writeup. No logic changed.
"""
SQLite-backed store for per-agent reputation history.

Each agent gets one session row per continuous interaction (pop-up → drop-off).
r_new values are buffered in memory and written in batches for performance.
"""

import logging
import signal
import sqlite3
import time

_log = logging.getLogger(__name__)

# --- Tunable constants -------------------------------------------------------
BATCH_SIZE      = 1   # write buffer to DB every N frame flushes (1 = every frame, for testing; 10 for production)
SESSION_TIMEOUT = 3   # frames of silence before a session is considered closed
# -----------------------------------------------------------------------------


class PersistentReputationTracker:
    """
    Buffers per-frame r_new scores in memory and writes them to SQLite in
    batches of BATCH_SIZE frame flushes.

    A new session row is opened the first time an agent is seen, and again
    if it reappears after SESSION_TIMEOUT consecutive frames of silence.
    Calling close() flushes any remaining buffer — this handles Ctrl+C.
    """

    def __init__(self, db_path: str):
        self._conn = sqlite3.connect(db_path)
        self._conn.execute('PRAGMA journal_mode=WAL')
        # NORMAL skips the per-commit fsync (WAL still guarantees the DB can
        # never corrupt); an OS crash / power loss may drop the last few
        # commits. Accepted tradeoff for edge latency — see README
        # "Reputation database".
        self._conn.execute('PRAGMA synchronous=NORMAL')
        self._conn.execute('''
            CREATE TABLE IF NOT EXISTS sessions (
                id        INTEGER PRIMARY KEY,
                source_id TEXT NOT NULL
            )
        ''')
        self._conn.execute('''
            CREATE TABLE IF NOT EXISTS reputation_log (
                id         INTEGER PRIMARY KEY,
                session_id INTEGER NOT NULL REFERENCES sessions(id),
                r_new      REAL    NOT NULL,
                ts         REAL    NOT NULL
            )
        ''')
        # migrate pre-timestamp / pre-risk_l DBs
        cols = [row[1] for row in self._conn.execute('PRAGMA table_info(reputation_log)')]
        if 'ts' not in cols:
            self._conn.execute('ALTER TABLE reputation_log ADD COLUMN ts REAL')
        if 'risk_l' not in cols:
            self._conn.execute('ALTER TABLE reputation_log ADD COLUMN risk_l REAL DEFAULT 0.0')
        self._conn.commit()

        # (session_id, r_new, ts, risk_l) tuples waiting to be written; ts is the
        # wall-clock time.time() at record() time, not DB-write time
        self._buffer: list[tuple[int, float, float, float]] = []

        # frame_num of last record() call per agent key
        self._last_seen: dict[str, int] = {}

        # currently open session id per agent key
        self._active_session: dict[str, int] = {}

    def record(self, source_id_tuple: tuple, r_new: float, frame_num: int,
               risk_l: float = 0.0):
        """
        Buffer one r_new value for this agent.
        Opens a new session if the agent is new or returning after a gap.
        source_id_tuple is the full 4-element int32[4] from the SDSM message.
        """
        key = ' '.join(f'{b:02x}' for b in source_id_tuple)

        if key not in self._active_session:
            cursor = self._conn.execute(
                'INSERT INTO sessions (source_id) VALUES (?)', (key,)
            )
            self._conn.commit()
            self._active_session[key] = cursor.lastrowid

        self._last_seen[key] = frame_num
        self._buffer.append((self._active_session[key], r_new, time.time(), risk_l))

    def flush(self, current_frame: int):
        """
        Write the buffer to DB, then clear in-memory session state for agents
        that have been silent for >= SESSION_TIMEOUT frames so they get a new
        session row if they reappear.
        Call this every BATCH_SIZE frame flushes.
        """
        self._write_buffer()

        stale = [
            key for key, last in self._last_seen.items()
            if current_frame - last >= SESSION_TIMEOUT
        ]
        for key in stale:
            del self._active_session[key]
            del self._last_seen[key]

    def close(self):
        """Flush any remaining buffered data and close the DB connection.
        Always call this on node shutdown to avoid losing the last batch.

        SIGINT is blocked around the write so the Ctrl+C that triggered
        shutdown cannot interrupt commit() mid-flight (which would roll the
        transaction back and lose the last batch). Any pending SIGINT is
        delivered once the mask is lifted.
        """
        signal.pthread_sigmask(signal.SIG_BLOCK, {signal.SIGINT})
        try:
            self._write_buffer()
            self._conn.close()
        finally:
            signal.pthread_sigmask(signal.SIG_UNBLOCK, {signal.SIGINT})

    def get_last_reputation(self, source_id_str: str) -> float | None:
        """Return the most recent r_new for this agent, or None if never seen."""
        row = self._conn.execute(
            '''SELECT r.r_new
               FROM reputation_log r
               JOIN sessions s ON r.session_id = s.id
               WHERE s.source_id = ?
               ORDER BY r.id DESC LIMIT 1''',
            (source_id_str,),
        ).fetchone()
        return row[0] if row else None

    def get_last_reputation_with_ts(self, source_id_str: str) -> tuple[float, float, float] | None:
        """Return (r_new, ts, risk_l) for the most recent timestamped record, or None.

        ts is the wall-clock time.time() value recorded when that r_new was
        buffered. Use the gap between ts and time.time() to apply absence_decay
        for agents returning after a long absence, and to decay risk_l via
        PersistencePenalty.seed().

        Rows from before the ts migration (ts IS NULL) are skipped: an agent
        whose ONLY history predates the migration returns None here, and the
        engine deliberately reseeds it at REPUTATION_DEFAULT (0.5) — history
        of unknowable age is treated as no usable history. See
        TrustEngine.get_reputation.
        """
        row = self._conn.execute(
            '''SELECT r.r_new, r.ts, COALESCE(r.risk_l, 0.0)
               FROM reputation_log r
               JOIN sessions s ON r.session_id = s.id
               WHERE s.source_id = ? AND r.ts IS NOT NULL
               ORDER BY r.id DESC LIMIT 1''',
            (source_id_str,),
        ).fetchone()
        return (row[0], row[1], row[2]) if row else None

    def record_initial(self, source_id_str: str, r_default: float):
        """Open a new session and buffer the default reputation for a brand-new agent."""
        cursor = self._conn.execute(
            'INSERT INTO sessions (source_id) VALUES (?)', (source_id_str,)
        )
        self._conn.commit()
        self._active_session[source_id_str] = cursor.lastrowid
        self._last_seen[source_id_str] = -1
        self._buffer.append((cursor.lastrowid, r_default, time.time(), 0.0))

    def _write_buffer(self):
        if not self._buffer:
            return
        session_to_source = {sid: src for src, sid in self._active_session.items()}
        display: dict[str, list] = {}
        for session_id, r_new, _ts, _risk_l in self._buffer:
            src = session_to_source.get(session_id, str(session_id))
            display.setdefault(src, []).append(round(r_new, 4))
        _log.info('Adding to database: %s', display)
        self._conn.executemany(
            'INSERT INTO reputation_log (session_id, r_new, ts, risk_l) VALUES (?, ?, ?, ?)',
            self._buffer,
        )
        self._conn.commit()
        self._buffer.clear()
