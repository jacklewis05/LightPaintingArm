#!/bin/sh
# Start the light-painting program (sets up its Python environment on first run).
cd "$(dirname "$0")" || exit 1
if [ ! -x .venv/bin/python ]; then
  python3 -m venv .venv && .venv/bin/pip install -q -r requirements.txt || exit 1
fi
exec .venv/bin/python -m lightpaint "$@"
