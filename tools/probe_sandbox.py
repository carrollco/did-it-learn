"""Probe what a Managed Agents sandbox can run: Node, npm installs, headless Chromium with WebGL.

Writes the raw command output to a file inside the sandbox, downloads it, and saves it under tmp/probe/.
Uses the agent's default model (gemini-3.8-flash) because the probe only needs a shell.
"""
import json, os, sys, time
from google import genai

PROBE = r'''
set -x
uname -a; whoami; nproc; free -h; df -h /workspace | tail -1
node -v; npm -v; python3 -V
mkdir -p /workspace/p && cd /workspace/p && npm init -y >/dev/null
npm i three cannon-es playwright 2>&1 | tail -3
npx playwright install --with-deps chromium 2>&1 | tail -5
cat > gl.mjs <<'JS'
import { chromium } from 'playwright';
const b = await chromium.launch({ args: ['--use-angle=swiftshader', '--enable-unsafe-swiftshader'] });
const p = await b.newPage();
await p.setContent('<canvas id=c width=64 height=64></canvas>');
const r = await p.evaluate(() => {
  const gl = document.getElementById('c').getContext('webgl2');
  if (!gl) return 'NO_WEBGL2';
  const ext = gl.getExtension('WEBGL_debug_renderer_info');
  return gl.getParameter(ext ? ext.UNMASKED_RENDERER_WEBGL : gl.RENDERER);
});
console.log('WEBGL_RENDERER=' + r);
await b.close();
JS
node gl.mjs 2>&1 | tail -3
'''

def main():
    client = genai.Client(api_key=os.environ["GOOGLE_API_KEY"], http_options={"timeout": 900_000})
    t = time.time()
    it = client.interactions.create(
        agent="antigravity-preview-09-2026",
        input=("Save this script as /workspace/probe.sh and run it with `bash /workspace/probe.sh > /workspace/probe.txt 2>&1`. "
               "Do not change the script. When it finishes, reply with the last 40 lines of /workspace/probe.txt verbatim.\n\n" + PROBE),
        environment="remote",
    )
    out = {"seconds": round(time.time() - t, 1), "environment_id": it.environment_id, "status": getattr(it, "status", None),
           "output_text": it.output_text}
    u = getattr(it, "usage", None)
    out["usage"] = u.model_dump(exclude_none=True) if hasattr(u, "model_dump") else str(u)
    try:
        f = client.environments.files.download(environment=it.environment_id, path="/workspace/probe.txt")
        data = f if isinstance(f, (bytes, bytearray)) else getattr(f, "content", None) or getattr(f, "data", None) or f
        open("tmp/probe/probe.txt", "wb").write(data if isinstance(data, (bytes, bytearray)) else str(data).encode())
        out["download"] = "ok"
    except Exception as e:  # the files API shape is not yet verified
        out["download"] = f"failed: {e!r}"
    json.dump(out, open("tmp/probe/result.json", "w"), indent=1, default=str)
    print(json.dumps(out, indent=1, default=str)[:4000])

if __name__ == "__main__":
    main()
