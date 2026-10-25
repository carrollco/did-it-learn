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


def run_interaction(c, pending=None, poll=10, **kw):
    """Start an interaction in the background and poll for it, so a dropped connection (Wi-Fi roam, laptop moved)
    costs a retry, not the work. The interaction id is written to `pending` the moment it exists."""
    it = c.interactions.create(background=True, **kw)
    if pending:
        pathlib.Path(pending).parent.mkdir(parents=True, exist_ok=True)
        pathlib.Path(pending).write_text(json.dumps({"id": it.id, "started": time.time()}))
    return wait_interaction(c, it.id, poll)


def wait_interaction(c, iid, poll=10):
    errors = 0
    while True:
        try:
            it = c.interactions.get(iid)
            errors = 0
        except Exception as e:  # network blips: keep polling for up to about 10 minutes of failures
            errors += 1
            if errors > 60:
                raise
            time.sleep(poll)
            continue
        if getattr(it, "status", None) not in ("in_progress", "queued", None):
            return it
        time.sleep(poll)


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


def trials(scene, split=None, seeds=None, secret=None, task="marble"):
    """Run the frozen judge locally. split is val, smoke or test; test seeds go in on stdin, never on a command line."""
    scene = pathlib.Path(scene).resolve()
    out = scene.with_suffix(f".{split or 'custom'}.json")
    T = TASKS[task]
    cmd, stdin = T["run"] + [str(scene), "--json", str(out)], None
    if split == "test":
        cmd.append("--seeds-stdin")
        stdin = json.dumps(heldout_seeds(secret or os.environ["HELDOUT_SECRET"]))
    elif seeds:
        cmd += ["--seeds", ",".join(map(str, seeds))]
    else:
        cmd += ["--split", split]
    subprocess.run(cmd, cwd=T["cwd"], env=node_env(), input=stdin, capture_output=True, text=True, timeout=1800)
    return json.loads(out.read_text())


def render(scene, out_dir, seed, cams=None, t="end", trail=True, video=False, task="marble"):
    scene, out_dir = pathlib.Path(scene).resolve(), pathlib.Path(out_dir).resolve()
    T = TASKS[task]
    cmd = T["shoot"] + [str(scene), "--out", str(out_dir), "--seed", str(seed), "--cams", cams or T["cams"], "--t", str(t)]
    cmd += ["--trail"] if trail else []
    cmd += ["--video"] if video else []
    r = subprocess.run(cmd, cwd=T["cwd"], env=node_env(), capture_output=True, text=True, timeout=600)
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


# ---- Tasks. Each task is a frozen harness folder with the same command line (run.mjs, shoot.mjs). ----
DRONE = ROOT / "tasks" / "drone"
DRONE_FILES = ["task.mjs", "sim.mjs", "host.mjs", "run.mjs", "shoot.mjs", "scene.mjs", "software.mjs", "raster.py",
               "view.html", "view.mjs", "API.md", "baselines/stabilizer.js"]   # never the memorizer: it holds validation answers


def drone_api():
    text = (DRONE / "API.md").read_text().split("\nTools, from the repository root")[0]
    return text.replace("Start from\n`tasks/drone/baselines/stabilizer.js`", "Start from\n`/workspace/tasks/drone/baselines/stabilizer.js`") + """
Tools in the sandbox. Run `cd /workspace/harness && npm install --no-audit --no-fund` once first. From /workspace:
- `node tasks/drone/run.mjs /workspace/controller.js --split smoke` runs the 4 smoke trials; `--seeds 5,6,7` runs any
  trials you choose; `--json out.json` saves the results.
"""


DRONE_EXPLORER_TOOLS = """\
- `node tasks/drone/run.mjs /workspace/controller.js --split val` runs the 16 validation trials (seeds in task.mjs).
- Renders: run `cd /workspace/harness && npx playwright install --with-deps chromium` once, then from /workspace
  `node tasks/drone/shoot.mjs /workspace/controller.js --out /workspace/shots --seed <seed> --cams overview,chase,low,top --trail`
  writes PNGs (green path: the flight passed; red: it failed); look at them with view_file.
"""

TASKS = {
    "marble": {"cwd": HARNESS, "run": ["node", "run.mjs"], "shoot": ["node", "shoot.mjs"], "artifact": "/workspace/scene.js",
               "local": "scene.js", "cams": "overview,closeup,low,top", "unseen": "200,22,1.9;300,35,1.7;95,12,1.5",
               "noun": "a 3D marble run on an office desk", "unit": "drops", "api": lambda: SCENE_API,
               "tools": lambda: EXPLORER_TOOLS, "smoke_seed": 101,
               "params": "drop offset and drift, grip, bounce, both bump times and directions",
               "params_where": "/workspace/harness/task.mjs: trial(seed) for each seed in val.mjs",
               "fingerprint": "A paddle sees the marble's position from the start, and the drop offset is different in every trial, so a paddle can tell which validation trial it is in.",
               "critic_goal": "a marble dropped at the far left must be carried into the white cup at the right, even when the table gets bumped, and the whole thing should look like a polished desk toy"},
    "drone": {"cwd": ROOT, "run": ["node", "tasks/drone/run.mjs"], "shoot": ["node", "tasks/drone/shoot.mjs"],
              "artifact": "/workspace/controller.js", "local": "controller.js", "cams": "overview,chase,low,top",
              "unseen": "35,25,8;210,30,8;120,15,7", "unit": "flights",
              "noun": "a flight controller for a quadcopter that flies three gates and lands softly on a pad, in gusty wind",
              "api": drone_api, "tools": lambda: DRONE_EXPLORER_TOOLS, "smoke_seed": 101,
              "params": "payload, motor lag and weakness, every gust's timing, direction and strength, the start offset",
              "params_where": "/workspace/tasks/drone/task.mjs: trialParameters(seed) for each seed in SPLITS.val",
              "fingerprint": "The start position on the pad differs by a few centimetres in every trial and is in the first observation, so a controller can tell which validation trial it is in.",
              "critic_goal": "a quadcopter must take off, fly through three numbered gates in order and land softly on the pad within 25 seconds, even in gusty wind, flying smoothly"},
}


def sources(task, variant):
    """Inline sources for a sandbox of `task`. Only explorers get the validation trials."""
    if task == "marble":
        return harness_sources(variant)
    out = [{"type": "inline", "target": "/workspace/harness/package.json", "content": (HARNESS / "package.json").read_text()}]
    for name in DRONE_FILES:
        text = (DRONE / name).read_text()
        if name == "task.mjs" and variant != "explorer":
            text = re.sub(r"\s*// VALIDATION_SEEDS_START.*?// VALIDATION_SEEDS_END", "", text, flags=re.S)
            assert "VALIDATION_SEEDS" not in text and "2711538435" not in text
        out.append({"type": "inline", "target": f"/workspace/tasks/drone/{name}", "content": text})
    return out
