#!/usr/bin/env bash
# Start roscore (if needed), Verizon relay, and ROS1 converter in background.
set -euo pipefail
ROLE="${1:-}"
PROJECT_ROOT="$(cd "$(dirname "$0")/.." && pwd)"
# shellcheck disable=SC1091
source "$PROJECT_ROOT/scripts/load_role.sh" "$ROLE"

STATE_DIR="$PROJECT_ROOT/run/$ROLE"
PID_FILE="$STATE_DIR/node.pid"
mkdir -p "$STATE_DIR" "$PROJECT_ROOT/logs"
if [ -f "$PID_FILE" ]; then
  OLD_PID="$(cat "$PID_FILE" 2>/dev/null || true)"
  if [ -n "$OLD_PID" ] && kill -0 "$OLD_PID" 2>/dev/null; then
    echo "$ROLE node is already running (supervisor PID $OLD_PID)." >&2
    exit 2
  fi
fi

RUN_ID="${ROLE}_$(date -u +%Y%m%dT%H%M%SZ)"
LOG_DIR="$PROJECT_ROOT/logs/$RUN_ID"
mkdir -p "$LOG_DIR"
ln -sfn "$LOG_DIR" "$PROJECT_ROOT/logs/${ROLE}_current"

(
  set -euo pipefail
  CHILDREN=()
  cleanup() {
    trap - INT TERM EXIT
    for pid in "${CHILDREN[@]:-}"; do kill -TERM "$pid" 2>/dev/null || true; done
    sleep 1
    for pid in "${CHILDREN[@]:-}"; do kill -KILL "$pid" 2>/dev/null || true; done
    wait 2>/dev/null || true
    rm -f "$PID_FILE"
  }
  trap cleanup INT TERM EXIT

  if ! rosparam list >/dev/null 2>&1; then
    roscore >"$LOG_DIR/roscore.log" 2>&1 &
    CHILDREN+=("$!")
    for _ in $(seq 1 30); do
      rosparam list >/dev/null 2>&1 && break
      sleep 1
    done
    rosparam list >/dev/null 2>&1 || {
      echo "roscore failed; see $LOG_DIR/roscore.log" >&2
      exit 4
    }
  fi

  bash "$PROJECT_ROOT/scripts/run_relay.sh" "$ROLE" "$LOG_DIR" \
    >"$LOG_DIR/relay.console.log" 2>&1 &
  RELAY_PID=$!
  CHILDREN+=("$RELAY_PID")
  READY=0
  for _ in $(seq 1 50); do
    if grep -Fq 'ENDPOINT_READY' \
        "$LOG_DIR/relay.console.log" 2>/dev/null; then
      READY=1
      break
    fi
    kill -0 "$RELAY_PID" 2>/dev/null || {
      echo "relay exited; see $LOG_DIR/relay.console.log" >&2
      exit 5
    }
    sleep 1
  done
  [ "$READY" -eq 1 ] || {
    echo "relay did not become ready; see $LOG_DIR/relay.console.log" >&2
    exit 5
  }

  bash "$PROJECT_ROOT/scripts/run_converter.sh" "$ROLE" "$LOG_DIR" \
    >"$LOG_DIR/converter.console.log" 2>&1 &
  CONVERTER_PID=$!
  CHILDREN+=("$CONVERTER_PID")
  sleep 2
  kill -0 "$CONVERTER_PID" 2>/dev/null || {
    echo "converter exited; see $LOG_DIR/converter.console.log" >&2
    exit 6
  }

  cat >"$STATE_DIR/state.env" <<EOF
ROLE=$ROLE
RUN_ID=$RUN_ID
LOG_DIR=$LOG_DIR
SUPERVISOR_PID=$BASHPID
RELAY_PID=$RELAY_PID
CONVERTER_PID=$CONVERTER_PID
EOF
  echo "NODE_READY role=$ROLE run=$RUN_ID logs=$LOG_DIR"
  wait -n
) >"$STATE_DIR/supervisor.log" 2>&1 &

SUPERVISOR_PID=$!
echo "$SUPERVISOR_PID" >"$PID_FILE"
for _ in $(seq 1 60); do
  if grep -Fq NODE_READY "$STATE_DIR/supervisor.log" 2>/dev/null; then
    cat "$STATE_DIR/supervisor.log"
    exit 0
  fi
  kill -0 "$SUPERVISOR_PID" 2>/dev/null || {
    cat "$STATE_DIR/supervisor.log" >&2
    exit 7
  }
  sleep 1
done
echo "Timed out; inspect $STATE_DIR/supervisor.log" >&2
exit 7
