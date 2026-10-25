"""Check the drone harness runs inside a Managed Agents sandbox (Node 22), explorer and reproducer copies."""
import json, sys, pathlib, time
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1] / "loop"))
from kit import AGENT, client, sources, run_interaction
c = client(); t = time.time()
it = run_interaction(c, agent=AGENT, environment={"type": "remote", "sources": sources("drone", "explorer")}, input=(
    "Run exactly these commands and reply with their complete output verbatim, nothing else:\n"
    "cd /workspace/harness && npm install --no-audit --no-fund >/dev/null 2>&1; cd /workspace; node -v; "
    "cp tasks/drone/baselines/stabilizer.js controller.js; node tasks/drone/run.mjs /workspace/controller.js --split val | head -2; "
    "node tasks/drone/run.mjs /workspace/controller.js --split smoke | head -1; ls tasks/drone/baselines"))
print(json.dumps({"seconds": round(time.time() - t, 1), "output": it.output_text}, indent=1))
