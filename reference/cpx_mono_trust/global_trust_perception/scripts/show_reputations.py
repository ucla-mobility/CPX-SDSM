#!/usr/bin/env python3
"""
Pretty-print or clear the reputation history stored by an agent node.

Usage:
    python3 show_reputations.py <agent_id>           # resolves the repo default path
    python3 show_reputations.py <path_to.db>         # explicit database file
    python3 show_reputations.py --clear <agent_id>   # delete all rows from the DB
    python3 show_reputations.py --clear <path_to.db>
    python3 show_reputations.py --clear-all          # clear every agent DB in the data dir

Prints one aligned table with column headers:
    source_id | session | r_new | recorded_at
"""

import glob
import os
import sqlite3
import sys

_QUERY = '''
    SELECT s.source_id, s.id AS session, r.r_new,
           datetime(r.ts, 'unixepoch', 'localtime') AS recorded_at
    FROM reputation_log r
    JOIN sessions s ON r.session_id = s.id
    ORDER BY s.source_id, s.id, r.id
'''

# fallback for DBs written before the ts column existed (the agent node
# migrates them on its next start; this viewer never alters the file)
_QUERY_NO_TS = '''
    SELECT s.source_id, s.id AS session, r.r_new, NULL AS recorded_at
    FROM reputation_log r
    JOIN sessions s ON r.session_id = s.id
    ORDER BY s.source_id, s.id, r.id
'''


def _data_dir() -> str:
    prefix = os.environ.get('COLCON_PREFIX_PATH', '').split(os.pathsep)[0]
    if prefix:
        return os.path.join(os.path.dirname(prefix), 'src', 'CPX-Mono', 'ros2', 'data')
    return os.getcwd()


def _resolve_db_path(arg: str) -> str:
    """A path ending in .db is used as-is; anything else is treated as an agent id."""
    if arg.endswith('.db'):
        return arg
    return os.path.join(_data_dir(), f'historical_reputations_{arg}.db')


def _clear(db_path: str) -> int:
    if not os.path.exists(db_path):
        print(f'no database at: {db_path}', file=sys.stderr)
        return 1
    conn = sqlite3.connect(db_path)
    try:
        conn.execute('DELETE FROM reputation_log')
        conn.execute('DELETE FROM sessions')
        # sqlite_sequence only exists if a table uses AUTOINCREMENT; with plain
        # INTEGER PRIMARY KEY the ids restart at 1 on their own after a full delete
        if conn.execute("SELECT 1 FROM sqlite_master WHERE name = 'sqlite_sequence'").fetchone():
            conn.execute("DELETE FROM sqlite_sequence WHERE name IN ('sessions', 'reputation_log')")
        conn.commit()
    finally:
        conn.close()
    print(f'cleared: {db_path}')
    return 0


def _clear_all() -> int:
    data = _data_dir()
    dbs = sorted(glob.glob(os.path.join(data, 'historical_reputations_*.db')))
    if not dbs:
        print(f'no reputation databases found in: {data}', file=sys.stderr)
        return 1
    code = 0
    for db in dbs:
        code |= _clear(db)
    return code


def main(argv: list) -> int:
    if len(argv) == 2 and argv[1] == '--clear-all':
        return _clear_all()
    if len(argv) == 3 and argv[1] == '--clear':
        return _clear(_resolve_db_path(argv[2]))
    if len(argv) != 2:
        print(f'usage: {argv[0]} [--clear <agent_id | path_to.db> | --clear-all]', file=sys.stderr)
        return 2

    db_path = _resolve_db_path(argv[1])
    if not os.path.exists(db_path):
        print(f'no database at: {db_path}', file=sys.stderr)
        return 1

    conn = sqlite3.connect(db_path)
    try:
        cols = [row[1] for row in conn.execute('PRAGMA table_info(reputation_log)')]
        rows = conn.execute(_QUERY if 'ts' in cols else _QUERY_NO_TS).fetchall()
    finally:
        conn.close()

    print(f'database: {db_path}')
    if not rows:
        print('(no reputation rows yet)')
        return 0

    headers = ('source_id', 'session', 'r_new', 'recorded_at')
    table = [headers] + [
        (str(s), str(sess), f'{r:.4f}', ts if ts is not None else '-')
        for s, sess, r, ts in rows
    ]
    widths = [max(len(row[c]) for row in table) for c in range(len(headers))]

    def _fmt(row):
        return '  '.join(cell.ljust(widths[c]) for c, cell in enumerate(row))

    print(_fmt(headers))
    print('  '.join('-' * w for w in widths))
    for row in table[1:]:
        print(_fmt(row))
    print(f'\n{len(rows)} row(s)')
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv))
