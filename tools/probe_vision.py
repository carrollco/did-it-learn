"""Can a Managed Agents sandbox agent look at an image file it made? Draws a secret colour and side, asks the agent."""
import json, os, random, time
from google import genai

COLORS = {"red": (220, 40, 40), "green": (40, 170, 60), "blue": (40, 80, 220), "yellow": (240, 210, 30), "purple": (140, 50, 170)}
name = random.choice(list(COLORS)); side = random.choice(["left", "right"])
r, g, b = COLORS[name]
MAKER = f'''
import zlib, struct
W, H = 200, 100
rows = []
for y in range(H):
    row = bytearray([0])
    for x in range(W):
        filled = (x < W // 2) == ({side!r} == "left")
        row += bytes(({r}, {g}, {b}) if filled else (255, 255, 255))
    rows.append(bytes(row))
def chunk(t, d): return struct.pack(">I", len(d)) + t + d + struct.pack(">I", zlib.crc32(t + d) & 0xffffffff)
png = b"\\x89PNG\\r\\n\\x1a\\n" + chunk(b"IHDR", struct.pack(">IIBBBBB", W, H, 8, 2, 0, 0, 0)) + chunk(b"IDAT", zlib.compress(b"".join(rows))) + chunk(b"IEND", b"")
open("/workspace/img.png", "wb").write(png)
'''
client = genai.Client(api_key=os.environ["GOOGLE_API_KEY"], http_options={"timeout": 900_000})
model = os.environ.get("PROBE_MODEL")  # unset: the agent's default model
t = time.time()
kw = {"agent_config": {"type": "antigravity", "model": model}} if model else {}
it = client.interactions.create(
    agent="antigravity-preview-09-2026", environment="remote", **kw,
    input=("Run `python3 /workspace/make.py` (it is already in the workspace). It writes /workspace/img.png. "
           "Do not read the script or print pixel values with code. Look at the image itself with your image viewing ability "
           "and answer in one line: COLOR=<colour name> SIDE=<left|right half filled>. If you cannot view images, answer CANNOT_VIEW."),
    environment_config=None,
) if False else client.interactions.create(
    agent="antigravity-preview-09-2026", **kw,
    environment={"type": "remote", "sources": [{"type": "inline", "target": "/workspace/make.py", "content": MAKER}]},
    input=("Run `python3 /workspace/make.py` (it is already in the workspace). It writes /workspace/img.png. "
           "Do not read the script and do not inspect pixel values with code. Look at the image itself with your image viewing ability "
           "and answer in one line: COLOR=<colour name> SIDE=<left|right half filled>. If you cannot view images, answer CANNOT_VIEW."),
)
steps = []
for s in (getattr(it, "steps", None) or []):
    d = s.model_dump(exclude_none=True) if hasattr(s, "model_dump") else str(s)
    steps.append(json.dumps(d, default=str)[:400])
print(json.dumps({"truth": f"COLOR={name} SIDE={side}", "answer": it.output_text, "seconds": round(time.time() - t, 1),
                  "model": model or "default", "steps": steps[-12:]}, indent=1, default=str))
