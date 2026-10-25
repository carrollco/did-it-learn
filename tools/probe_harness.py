"""Check the v2 harness (locked-down scene process) runs inside a Managed Agents sandbox (Node 22)."""
import json, os, pathlib, sys, time
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1] / "loop"))
from kit import AGENT, client, harness_sources, ROOT
c = client()
src = harness_sources("explorer") + [{"type": "inline", "target": "/workspace/scene.js", "content": (ROOT / "scenes/baseline-v2.js").read_text()},
                                    {"type": "inline", "target": "/workspace/escape.js", "content": (ROOT / "tmp/escape/escape.js").read_text()}]
t = time.time()
it = c.interactions.create(agent=AGENT, environment={"type": "remote", "sources": src}, input=(
    "Run exactly these commands and reply with their complete output verbatim, nothing else:\n"
    "cd /workspace/harness && npm install --no-audit --no-fund >/dev/null 2>&1; node -v; "
    "node run.mjs /workspace/scene.js --split val | head -3; node run.mjs /workspace/escape.js --split smoke | head -2"))
print(json.dumps({"seconds": round(time.time() - t, 1), "output": it.output_text}, indent=1))
