"""Shared helpers: the Gemini client, sandbox file bundles, downloads, Argon calls with images, local renders."""
import base64, hashlib, json, os, pathlib, re, subprocess, time
from google import genai

ROOT = pathlib.Path(__file__).resolve().parents[1]
HARNESS = ROOT / "harness"
ARGON = "gemini-4-argon"
AGENT = "antigravity-preview-09-2026"
HARNESS_FILES = ["package.json", "task.mjs", "world.mjs", "trialrun.mjs", "sim.mjs", "host.mjs", "run.mjs", "view.html", "view.mjs", "shoot.mjs"]
PLAYWRIGHT_LIBS = pathlib.Path.home() / ".cache/ms-playwright-local-libs/root/usr/lib/x86_64-linux-gnu"


def client():
    return genai.Client(api_key=os.environ["GOOGLE_API_KEY"], http_options={"timeout": 2_400_000})


def harness_sources(variant):
    """Inline sources for a sandbox. Only 'explorer' sandboxes get val.mjs, the 16 validation trials."""
    names = HARNESS_FILES + (["val.mjs"] if variant == "explorer" else [])
    return [{"type": "inline", "target": f"/workspace/harness/{n}", "content": (HARNESS / n).read_text()} for n in names]


def heldout_seeds(secret, n=64):
    """Held-out trial seeds, derived from a secret only this Python side holds (Doppler HELDOUT_SECRET)."""
    return [100000 + int.from_bytes(hashlib.sha256(f"{secret}:{i}".encode()).digest()[:4], "big") % 900_000_000 for i in range(n)]


def download(c, env_id, path):
    data = c.environments.files.download(environment=env_id, path=path)
    if isinstance(data, (bytes, bytearray)):
        return bytes(data)
    for attr in ("content", "data"):
        if getattr(data, attr, None) is not None:
            return getattr(data, attr)
    return str(data).encode()


def usage(it):
    u = getattr(it, "usage", None)
    return u.model_dump(exclude_none=True) if hasattr(u, "model_dump") else {}


def cost(u, model=ARGON):
    """Introductory Argon price through 2026-11-30: $2 per 1M input, $10 per 1M output (thinking billed as output)."""
    i = u.get("total_input_tokens", 0) - u.get("total_cached_tokens", 0) + 0.05 * u.get("total_cached_tokens", 0)
    o = u.get("total_output_tokens", 0) + u.get("total_thought_tokens", 0)
    return round(i * 2e-6 + o * 10e-6, 4)


def steps_of(it):
    out = []
    for s in getattr(it, "steps", None) or []:
        out.append(s.model_dump(exclude_none=True) if hasattr(s, "model_dump") else str(s))
    return out


def save_interaction(path, it, extra=None):
    path = pathlib.Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    rec = {"id": it.id, "environment_id": getattr(it, "environment_id", None), "status": getattr(it, "status", None),
           "output_text": it.output_text, "usage": usage(it), "cost_usd": cost(usage(it)), "steps": steps_of(it), **(extra or {})}
    path.write_text(json.dumps(rec, indent=1, default=str))
    return rec


def image_part(png_path):
    return {"type": "image", "data": base64.b64encode(pathlib.Path(png_path).read_bytes()).decode(), "mime_type": "image/png"}


def argon(c, parts, thinking="medium", schema=None):
    kw = {"response_format": {"type": "text", "mime_type": "application/json", "schema": schema}} if schema else {}
    return c.interactions.create(model=ARGON, input=parts, generation_config={"thinking_level": thinking}, **kw)


def node_env():
    env = dict(os.environ)
    if PLAYWRIGHT_LIBS.exists():
        env["LD_LIBRARY_PATH"] = str(PLAYWRIGHT_LIBS)
    return env


def trials(scene, split=None, seeds=None, secret=None):
    """Run the frozen judge locally. split is val, smoke or test; test seeds go in on stdin, never on a command line."""
    scene = pathlib.Path(scene).resolve()
    out = scene.with_suffix(f".{split or 'custom'}.json")
    cmd, stdin = ["node", "run.mjs", str(scene), "--json", str(out)], None
    if split == "test":
        cmd.append("--seeds-stdin")
        stdin = json.dumps(heldout_seeds(secret or os.environ["HELDOUT_SECRET"]))
    elif seeds:
        cmd += ["--seeds", ",".join(map(str, seeds))]
    else:
        cmd += ["--split", split]
    subprocess.run(cmd, cwd=HARNESS, env=node_env(), input=stdin, capture_output=True, text=True, timeout=1800)
    return json.loads(out.read_text())


def render(scene, out_dir, seed, cams="overview,closeup,low,top", t="end", trail=True, video=False):
    scene, out_dir = pathlib.Path(scene).resolve(), pathlib.Path(out_dir).resolve()
    cmd = ["node", "shoot.mjs", str(scene), "--out", str(out_dir), "--seed", str(seed), "--cams", cams, "--t", str(t)]
    cmd += ["--trail"] if trail else []
    cmd += ["--video"] if video else []
    r = subprocess.run(cmd, cwd=HARNESS, env=node_env(), capture_output=True, text=True, timeout=600)
    if r.returncode != 0:
        raise RuntimeError(f"render failed: {r.stderr[-800:]}")
    return json.loads(r.stdout.strip().splitlines()[-1])


SCENE_API = """\
The scene is one JavaScript file, /workspace/scene.js, that exports `export default function build({ THREE, add, task })`.
It may not import anything or touch globals; it gets exactly these arguments, and the judge runs it in a separate
locked-down process that sees nothing but what is described here:
- `task`: the frozen task numbers (read-only; see /workspace/harness/task.mjs). Units are metres. The desk top is y = 0
  and spans x -1..1, z -0.5..0.5. The marble (radius 0.025 m) is dropped from about [-0.85, 0.8, -0.3]. The cup (inner
  radius 0.07 m, wall height 0.06 m) sits at [0.8, 0, 0.3]; the harness builds the desk and the cup.
- `add.box({ size: [x, y, z], position: [x, y, z], rotation: [rx, ry, rz], color })`: a static physical box (rotation
  is Euler XYZ in radians, like THREE). `add.cylinder({ radius, height, position, rotation, color })`: a static cylinder
  along local y. `add.decor(object3D)`: any THREE object, visual only, no physics, unlimited.
- `add.mover({ size, path: (t, marble) => ({ position, rotation }), color })`: a paddle. 60 times a second it is told the
  time t and the marble's `position` and `velocity` (arrays), and returns where it wants to be. It moves toward that pose
  no faster than 1.0 m/s and 6 rad/s.
- Budget: at most 8 static pieces and 2 paddles. Every side of a static piece is 0.012 m to 1.1 m; paddle sides at most
  0.25 m. Material: the sum over static pieces of each piece's largest face (length x width) is at most 0.40 m^2. Pieces
  must sit inside x -1.1..1.1, y 0..1.2, z -0.6..0.6. Breaking a rule makes the scene invalid.
A trial drops the marble with a small random offset and drift, a random friction (0.02 to 0.3) and bounciness (0.05 to
0.45), and bumps the table twice at random moments (a sudden kick of 0.4 to 1.2 m/s between 1 and 5 s, then 0.2 to
0.8 m/s between 5 and 9 s, in random directions, plus a small upward hop). A trial passes if the marble is resting
inside the cup after 12 seconds.
Tools in /workspace/harness (run `npm install --no-audit --no-fund` there first):
- `node run.mjs /workspace/scene.js --split smoke` runs the 4 smoke trials; `--seeds 1,2,3` runs any trials you choose.
"""

EXPLORER_TOOLS = """\
- `node run.mjs /workspace/scene.js --split val` runs the 16 validation trials (listed in val.mjs).
- Renders: run `npx playwright install --with-deps chromium` once, then
  `node shoot.mjs /workspace/scene.js --out /workspace/shots --seed <seed> --cams overview,closeup,low,top --trail`
  writes PNGs (the marble's path is drawn green if it ended in the cup, red if not); look at them with view_file.
"""
