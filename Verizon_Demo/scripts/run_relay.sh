#!/usr/bin/env bash
set -euo pipefail
ROLE="${1:-}"
LOG_DIR="${2:-}"
PROJECT_ROOT="$(cd "$(dirname "$0")/.." && pwd)"
# shellcheck disable=SC1091
source "$PROJECT_ROOT/scripts/load_role.sh" "$ROLE"
[ -n "$LOG_DIR" ] || { echo "Usage: run_relay.sh role log_dir" >&2; exit 2; }

if ! command -v conda >/dev/null 2>&1; then
  for prefix in "$HOME/miniforge3" "$HOME/anaconda3"; do
    [ -x "$prefix/bin/conda" ] && export PATH="$prefix/bin:$PATH" && break
  done
fi
eval "$(conda shell.bash hook)"
conda activate etx
export PYTHONPATH="$PROJECT_ROOT/app:$PROJECT_ROOT/vendor/python-etx-samples/src${PYTHONPATH:+:$PYTHONPATH}"
export ETX_FORCE_CODEC_LITE=1

exec python "$PROJECT_ROOT/app/etx_dual_relay.py" \
  --role "$ROLE" \
  --client-mode "$CLIENT_MODE" \
  --sender-id "$SENDER_ID" \
  --device-file "$PROJECT_ROOT/$REGISTRATION_REL" \
  --expected-identity "$EXPECTED_IDENTITY" \
  --default-lat "$DEFAULT_LAT" \
  --default-lon "$DEFAULT_LON" \
  --publish-type "$PUBLISH_TYPE" \
  --receive-type "$RECEIVE_TYPE" \
  --max-publish-rate "$MAX_PUBLISH_RATE" \
  --control-hello-seconds "$CONTROL_HELLO_SECONDS" \
  --log-dir "$LOG_DIR" \
  "${@:3}"
