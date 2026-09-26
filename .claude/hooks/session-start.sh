#!/usr/bin/env bash
# https://code.claude.com/docs/en/cloud-environments#install-dependencies-with-a-sessionstart-hook
set -euo pipefail
if [[ "${CLAUDE_CODE_REMOTE:-}" != "true" ]]; then
    exit 0
fi
repo_dir="${CLAUDE_PROJECT_DIR:-$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)}"
cd "$repo_dir"
python3 -m pip install -e ".[hpp,dev]" pytest-timeout
