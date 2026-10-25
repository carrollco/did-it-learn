import contextlib
import functools
import http.client
import json
import pathlib
import subprocess
import sys
import threading
from http.server import ThreadingHTTPServer
import pytest

ROOT = pathlib.Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "loop"))

import serve  # noqa: E402


@pytest.fixture()
def sandbox_server(monkeypatch, tmp_path):
    """Create an isolated repo tree in tmp_path and start serve.Handler on 127.0.0.1:0."""
    (tmp_path / "runs" / "d1" / "honest" / "round1").mkdir(parents=True)
    controller = tmp_path / "runs" / "d1" / "honest" / "round1" / "controller.js"
    controller.write_text("export default function makeController(){return {step(){return {thrust:0,roll:0,pitch:0,yawRate:0};}};}\n")

    (tmp_path / "tasks" / "drone" / "baselines").mkdir(parents=True)
    baseline = tmp_path / "tasks" / "drone" / "baselines" / "stabilizer.js"
    baseline.write_text((ROOT / "tasks" / "drone" / "baselines" / "stabilizer.js").read_text())

    (tmp_path / "private").mkdir()
    (tmp_path / "private" / "secret.txt").write_text("TOP_SECRET_PRIVATE")

    (tmp_path / ".git").mkdir()
    (tmp_path / ".git" / "HEAD").write_text("ref: refs/heads/main\n")

    (tmp_path / "tmp" / "live").mkdir(parents=True)
    (tmp_path / "tmp" / "scratch.txt").write_text("SCRATCH_INTERNAL")
    (tmp_path / "tmp" / "live" / "seed-555555.json").write_text('{"seed":555555,"frames":[]}')

    video_bytes = b"0123456789abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ"
    (tmp_path / "runs" / "d1" / "clip.mp4").write_bytes(video_bytes)

    outside_js = tmp_path / "outside.js"
    outside_js.write_text("export default function(){}\n")

    monkeypatch.setattr(serve, "ROOT", tmp_path)
    monkeypatch.setattr(serve, "LIVE", tmp_path / "tmp" / "live")

    def fake_run(cmd, **kwargs):
        # Extract --json and --record and seed to simulate tasks/drone/run.mjs quickly without spawning Node
        idx_json = cmd.index("--json")
        json_out = pathlib.Path(cmd[idx_json + 1])
        idx_seed = cmd.index("--seeds")
        seed = int(cmd[idx_seed + 1])
        idx_rec = cmd.index("--record")
        rec_dir = pathlib.Path(cmd[idx_rec + 1])
        (rec_dir / f"seed-{seed}.json").write_text(json.dumps({"seed": seed, "frames": []}))
        json_out.write_text(
            json.dumps(
                {
                    "valid": True,
                    "passed": 1,
                    "total": 1,
                    "results": [{"seed": seed, "success": True, "reason": "landed", "gatesPassed": 3, "landingError": 0.04, "simulatedSeconds": 12.5}],
                }
            )
        )
        return subprocess.CompletedProcess(cmd, 0, stdout="", stderr="")

    monkeypatch.setattr(serve.subprocess, "run", fake_run)

    httpd = ThreadingHTTPServer(("127.0.0.1", 0), functools.partial(serve.Handler, directory=str(tmp_path)))
    port = httpd.server_address[1]
    thread = threading.Thread(target=httpd.serve_forever, daemon=True)
    thread.start()
    try:
        yield {"host": "127.0.0.1", "port": port, "root": tmp_path, "video_bytes": video_bytes}
    finally:
        httpd.shutdown()
        httpd.server_close()


def _request(srv, method, path, headers=None):
    conn = http.client.HTTPConnection(srv["host"], srv["port"], timeout=5)
    try:
        conn.request(method, path, headers=headers or {})
        resp = conn.getresponse()
        body = resp.read()
        hdrs = {k.lower(): v for k, v in resp.getheaders()}
        return resp.status, hdrs, body
    finally:
        conn.close()


def test_studied_seeds_refused_by_api_fly(sandbox_server):
    """All 16 validation seeds and all 4 smoke seeds are refused by /api/fly with HTTP 400."""
    assert len(serve.STUDIED) == 20
    for smoke_seed in (101, 102, 103, 104):
        assert smoke_seed in serve.STUDIED
        code, _, body = _request(
            sandbox_server,
            "GET",
            f"/api/fly?artifact=tasks/drone/baselines/stabilizer.js&seed={smoke_seed}",
        )
        assert code == 400
        assert "loops studied" in json.loads(body)["error"]

    for val_seed in sorted(serve.STUDIED - {101, 102, 103, 104}):
        code, obj = serve.fly("tasks/drone/baselines/stabilizer.js", val_seed)
        assert code == 400
        assert "loops studied" in obj["error"]


@pytest.mark.parametrize("bad_seed", ["abc", "12.34", "NaN", "1e6", "true", "101px"])
def test_non_integer_seed_returns_400(sandbox_server, bad_seed):
    """/api/fly answers 400 to any non-integer seed query parameter."""
    code, _, body = _request(
        sandbox_server,
        "GET",
        f"/api/fly?artifact=tasks/drone/baselines/stabilizer.js&seed={bad_seed}",
    )
    assert code == 400
    assert json.loads(body) == {"error": "seed must be a whole number"}


def test_api_fly_refuses_artifacts_outside_runs_and_baselines(sandbox_server):
    """Refuses direct outside paths, ../ traversal, and symlink tricks escaping runs/ or tasks/drone/baselines/."""
    root = sandbox_server["root"]

    # Create symlinks inside runs/ and baselines/ that point to outside.js
    symlink_in_runs = root / "runs" / "d1" / "escape_link.js"
    symlink_in_runs.symlink_to(root / "outside.js")
    symlink_in_baselines = root / "tasks" / "drone" / "baselines" / "escape_base.js"
    symlink_in_baselines.symlink_to(root / "outside.js")
    symlink_to_etc = root / "runs" / "d1" / "etc_link.js"
    symlink_to_etc.symlink_to("/etc/hosts")

    forbidden_artifacts = [
        "outside.js",
        "loop/serve.py",
        "tasks/drone/sim.mjs",
        "../outside.js",
        "runs/../../outside.js",
        "runs/d1/honest/round1/../../../../outside.js",
        "tasks/drone/baselines/../../../outside.js",
        "runs/d1/escape_link.js",
        "tasks/drone/baselines/escape_base.js",
        "runs/d1/etc_link.js",
    ]
    for art in forbidden_artifacts:
        code, _, body = _request(sandbox_server, "GET", f"/api/fly?artifact={art}&seed=999999")
        assert code == 400, f"Expected 400 for {art}, got {code}: {body}"
        assert "artifact must be a controller under runs/ or the drone baselines" in json.loads(body)["error"]

    # Non-existent or non-.js file inside runs/ returns 404
    code_missing, _, _ = _request(sandbox_server, "GET", "/api/fly?artifact=runs/d1/missing.js&seed=999999")
    assert code_missing == 404
    code_mp4, _, _ = _request(sandbox_server, "GET", "/api/fly?artifact=runs/d1/clip.mp4&seed=999999")
    assert code_mp4 == 404


@pytest.mark.xfail(
    strict=True,
    reason="BUG in loop/serve.py:25: str(path).startswith(str(ROOT / 'runs')) lacks a trailing path separator, allowing sibling directories like runs-evil/ or tasks/drone/baselines-evil/",
)
def test_api_fly_refuses_sibling_prefix_directory(sandbox_server):
    """A sibling directory named `runs-evil/` is outside `runs/` and must be refused with 400."""
    root = sandbox_server["root"]
    evil_dir = root / "runs-evil"
    evil_dir.mkdir(exist_ok=True)
    (evil_dir / "controller.js").write_text("export default function(){}\n")

    code, obj = serve.fly("runs-evil/controller.js", 999999)
    assert code == 400


def test_api_fly_succeeds_for_valid_controller_and_unstudied_seed(sandbox_server):
    """Valid controller under runs/ or baselines/ with an unstudied or omitted seed returns 200 and track URL."""
    code, _, body = _request(
        sandbox_server,
        "GET",
        "/api/fly?artifact=runs/d1/honest/round1/controller.js&seed=424242",
    )
    assert code == 200
    payload = json.loads(body)
    assert payload["seed"] == 424242
    assert payload["result"]["success"] is True
    assert payload["track"].startswith("/tmp/live/")
    assert payload["track"].endswith("/seed-424242.json")


def test_private_git_and_tmp_blocked_except_tmp_live(sandbox_server):
    """/private, /.git and /tmp/ are never served (404), except /tmp/live/ which is served (200)."""
    for blocked in ["/private/secret.txt", "/private/", "/.git/HEAD", "/.git/", "/tmp/scratch.txt", "/tmp/"]:
        code, _, body = _request(sandbox_server, "GET", blocked)
        assert code == 404, f"Expected 404 for {blocked}, got {code}"
        assert json.loads(body) == {"error": "not served"}

    code_live, _, body_live = _request(sandbox_server, "GET", "/tmp/live/seed-555555.json")
    assert code_live == 200
    assert json.loads(body_live) == {"seed": 555555, "frames": []}


@pytest.mark.xfail(
    strict=True,
    reason="BUG in loop/serve.py:57: tuple has '/tmp/' with trailing slash instead of '/tmp', so GET /tmp returns 301 instead of 404",
)
def test_tmp_exact_path_without_trailing_slash_returns_404(sandbox_server):
    """GET /tmp (without trailing slash) must be blocked with 404 just like /private and /.git."""
    code, _, _ = _request(sandbox_server, "GET", "/tmp")
    assert code == 404


@pytest.mark.xfail(
    strict=True,
    reason="BUG in loop/serve.py:57: u.path is checked before normalization/unquoting, so /tmp/live/../scratch.txt and /runs/../private/secret.txt bypass the check and return 200",
)
def test_traversal_and_url_encoded_requests_to_private_git_tmp_blocked(sandbox_server):
    """Dot-segment and percent-encoded paths resolving into /private, /.git, or /tmp (outside /tmp/live) must not be served."""
    for bypass in [
        "/tmp/live/../scratch.txt",
        "/runs/../private/secret.txt",
        "/runs/../.git/HEAD",
        "/%70rivate/secret.txt",
    ]:
        code, _, body = _request(sandbox_server, "GET", bypass)
        assert code == 404, f"Bypass {bypass} returned {code} with body {body!r}"


def test_byte_range_requests_return_206_and_correct_content_range(sandbox_server):
    """Byte-range requests return 206 Partial Content with accurate Content-Range, Content-Length, and body slice."""
    data = sandbox_server["video_bytes"]
    size = len(data)

    # 1. Closed range bytes=0-9
    code, hdrs, body = _request(sandbox_server, "GET", "/runs/d1/clip.mp4", headers={"Range": "bytes=0-9"})
    assert code == 206
    assert hdrs["content-range"] == f"bytes 0-9/{size}"
    assert hdrs["content-length"] == "10"
    assert body == data[0:10]

    # 2. Middle range bytes=10-25
    code, hdrs, body = _request(sandbox_server, "GET", "/runs/d1/clip.mp4", headers={"Range": "bytes=10-25"})
    assert code == 206
    assert hdrs["content-range"] == f"bytes 10-25/{size}"
    assert hdrs["content-length"] == "16"
    assert body == data[10:26]

    # 3. Single byte range bytes=0-0
    code, hdrs, body = _request(sandbox_server, "GET", "/runs/d1/clip.mp4", headers={"Range": "bytes=0-0"})
    assert code == 206
    assert hdrs["content-range"] == f"bytes 0-0/{size}"
    assert hdrs["content-length"] == "1"
    assert body == data[0:1]

    # 4. Open-ended range bytes=20-
    code, hdrs, body = _request(sandbox_server, "GET", "/runs/d1/clip.mp4", headers={"Range": "bytes=20-"})
    assert code == 206
    assert hdrs["content-range"] == f"bytes 20-{size - 1}/{size}"
    assert hdrs["content-length"] == str(size - 20)
    assert body == data[20:]

    # 5. Suffix range bytes=-12
    code, hdrs, body = _request(sandbox_server, "GET", "/runs/d1/clip.mp4", headers={"Range": "bytes=-12"})
    assert code == 206
    assert hdrs["content-range"] == f"bytes {size - 12}-{size - 1}/{size}"
    assert hdrs["content-length"] == "12"
    assert body == data[-12:]

    # 6. Out-of-bounds range returns 416
    code, hdrs, _ = _request(sandbox_server, "GET", "/runs/d1/clip.mp4", headers={"Range": f"bytes={size + 10}-{size + 20}"})
    assert code == 416
    assert hdrs["content-range"] == f"bytes */{size}"
