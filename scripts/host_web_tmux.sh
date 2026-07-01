#!/usr/bin/env bash
set -euo pipefail

SESSION="${SESSION:-ohmni-web}"
ROOT_DIR="$(CDPATH= cd "$(dirname "$0")/.." && pwd)"
WEB_DIR="$ROOT_DIR/web"
PORT="${PORT:-5173}"
DOCKER_NAME="${DOCKER_NAME:-ohmni-web-vite}"

if ! command -v tmux >/dev/null 2>&1; then
  echo "tmux is not installed" >&2
  exit 1
fi

tmux has-session -t "$SESSION" 2>/dev/null && tmux kill-session -t "$SESSION"

if command -v npm >/dev/null 2>&1; then
  tmux new-session -d -s "$SESSION" -c "$WEB_DIR" \
    "if [ ! -d node_modules ]; then npm install; fi; npm run dev -- --host 0.0.0.0 --port $PORT"
elif command -v docker >/dev/null 2>&1; then
  docker rm -f "$DOCKER_NAME" >/dev/null 2>&1 || true
  tmux new-session -d -s "$SESSION" -c "$WEB_DIR" \
    "docker run --rm -it --name \"$DOCKER_NAME\" -p $PORT:$PORT -v \"$ROOT_DIR:/app\" -w /app/web node:20-alpine sh -lc 'if [ ! -d node_modules ]; then npm install; fi; npm run dev -- --host 0.0.0.0 --port $PORT'"
else
  echo "npm is not installed and docker is not available" >&2
  exit 1
fi

echo "Started tmux session: $SESSION"
echo "Attach: tmux attach -t $SESSION"
echo "Open: http://localhost:$PORT"
