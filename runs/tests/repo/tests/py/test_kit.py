import json
import pathlib
import re
import shutil
import subprocess
import sys
import types
import pytest

ROOT = pathlib.Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "loop"))

import kit  # noqa: E402


def _extract_val_seeds(task_mjs_path):
    text = task_mjs_path.read_text()
    block = text[text.index("VALIDATION_SEEDS_START"):text.index("VALIDATION_SEEDS_END")]
    return [int(x) for x in re.findall(r"\d+", block)]


def test_heldout_seeds_deterministic_and_64_distinct():
    """heldout_seeds is deterministic, returns 64 distinct seeds in range, and varies with the secret."""
    s1 = kit.heldout_seeds("made-up-secret-alpha")
    s2 = kit.heldout_seeds("made-up-secret-alpha")
    s3 = kit.heldout_seeds("made-up-secret-beta")

    assert s1 == s2
    assert len(s1) == 64
    assert len(set(s1)) == 64
    assert all(isinstance(x, int) and 100_000 <= x < 900_100_000 for x in s1)
    assert set(s1).isdisjoint(set(s3))
    assert kit.heldout_seeds("made-up-secret-alpha", n=16) == s1[:16]

    drone_val = set(_extract_val_seeds(ROOT / "tasks" / "drone" / "task.mjs"))
    assert drone_val.isdisjoint(set(s1))


def test_sources_strips_validation_seeds_for_reproducers(monkeypatch, tmp_path):
    """sources() strips all validation seeds from what reproducers get, while explorers keep them."""
    # Build a complete copy of ROOT in tmp_path with placeholder files for any non-.mjs files
    fake_root = tmp_path / "repo"
    shutil.copytree(ROOT / "harness", fake_root / "harness")
    shutil.copytree(ROOT / "tasks", fake_root / "tasks")
    for folder in ("drone", "drone-hard"):
        for name in kit.DRONE_FILES:
            p = fake_root / "tasks" / folder / name
            p.parent.mkdir(parents=True, exist_ok=True)
            if not p.exists():
                p.write_text("# placeholder\n")
    for name in kit.HARNESS_FILES:
        p = fake_root / "harness" / name
        if not p.exists():
            p.write_text("<!-- placeholder -->\n")

    monkeypatch.setattr(kit, "ROOT", fake_root)
    monkeypatch.setattr(kit, "HARNESS", fake_root / "harness")

    drone_val_seeds = _extract_val_seeds(ROOT / "tasks" / "drone" / "task.mjs")
    drone_hard_val_seeds = _extract_val_seeds(ROOT / "tasks" / "drone-hard" / "task.mjs")
    marble_val_seeds = [int(x) for x in re.findall(r"\d{6,}", (ROOT / "harness" / "val.mjs").read_text())]
    assert len(drone_val_seeds) == 16
    assert len(drone_hard_val_seeds) == 16
    assert len(marble_val_seeds) == 16

    # 1. Drone reproducer vs explorer
    repro_drone = kit.sources("drone", "reproducer")
    repro_task_mjs = next(s["content"] for s in repro_drone if s["target"].endswith("/task.mjs"))
    assert "VALIDATION_SEEDS" not in repro_task_mjs
    for seed in drone_val_seeds:
        assert str(seed) not in repro_task_mjs
        assert all(str(seed) not in s["content"] for s in repro_drone)
    # Smoke seeds are still intact for reproducers
    for smoke in (101, 102, 103, 104):
        assert str(smoke) in repro_task_mjs
    assert not any("memorizer" in s["target"] for s in repro_drone)

    explorer_drone = kit.sources("drone", "explorer")
    explorer_task_mjs = next(s["content"] for s in explorer_drone if s["target"].endswith("/task.mjs"))
    for seed in drone_val_seeds:
        assert str(seed) in explorer_task_mjs

    # 2. Drone-hard reproducer vs explorer
    repro_hard = kit.sources("drone-hard", "reproducer")
    repro_hard_task = next(s["content"] for s in repro_hard if s["target"].endswith("/task.mjs"))
    for seed in drone_hard_val_seeds:
        assert str(seed) not in repro_hard_task

    # 3. Marble reproducer vs explorer
    repro_marble = kit.sources("marble", "reproducer")
    assert not any(s["target"].endswith("/val.mjs") for s in repro_marble)
    for seed in marble_val_seeds:
        assert all(str(seed) not in s["content"] for s in repro_marble)

    explorer_marble = kit.sources("marble", "explorer")
    assert any(s["target"].endswith("/val.mjs") for s in explorer_marble)


@pytest.mark.xfail(
    strict=True,
    reason="BUG in loop/kit.py:20,172,230 & loop/tester.py:11-15: DRONE_FILES and HARNESS_FILES reference raster.py, view.html, and tasks/drone-hard/baselines/stabilizer.js which are missing from tasks/drone-hard and omitted from tester.py FILES",
)
def test_sources_all_listed_files_exist_in_repo():
    """Calling kit.sources() on the repository ROOT should succeed without FileNotFoundError for every task."""
    for task in ("drone", "drone-hard", "marble"):
        bundle = kit.sources(task, "reproducer")
        assert len(bundle) > 0


def test_trials_passes_heldout_seeds_via_stdin_not_argv(monkeypatch, tmp_path):
    """kit.trials(..., split='test') passes held-out seeds on stdin (--seeds-stdin) and never on the command line."""
    scene = tmp_path / "controller.js"
    scene.write_text("// dummy\n")
    secret = "fake-secret-xyz"
    expected_seeds = kit.heldout_seeds(secret)
    recorded = {}

    def fake_subprocess_run(cmd, *, cwd, env, input, capture_output, text, timeout):
        recorded["cmd"] = cmd
        recorded["stdin"] = input
        out_path = pathlib.Path(cmd[cmd.index("--json") + 1])
        out_path.write_text(json.dumps({"valid": True, "passed": 64, "total": 64, "results": []}))
        return subprocess.CompletedProcess(cmd, 0, stdout="", stderr="")

    monkeypatch.setattr(kit.subprocess, "run", fake_subprocess_run)
    res = kit.trials(scene, split="test", secret=secret, task="drone")
    assert res["passed"] == 64
    assert "--seeds-stdin" in recorded["cmd"]
    assert all(str(s) not in " ".join(recorded["cmd"]) for s in expected_seeds)
    assert json.loads(recorded["stdin"]) == expected_seeds


def test_cost_and_polling_helpers_with_fake_client(tmp_path):
    """kit.cost computes Argon token pricing and run_interaction polls until completed."""
    u = {
        "total_input_tokens": 1_000_000,
        "total_cached_tokens": 500_000,
        "total_output_tokens": 100_000,
        "total_thought_tokens": 50_000,
    }
    # Input: (500k uncached + 0.05 * 500k cached) = 525k * $2/1M = $1.05
    # Output: (100k + 50k) = 150k * $10/1M = $1.50 -> Total = $2.55
    assert kit.cost(u) == 2.55

    states = [
        types.SimpleNamespace(id="it-1", status="queued"),
        RuntimeError("transient wifi blip"),
        types.SimpleNamespace(id="it-1", status="in_progress"),
        types.SimpleNamespace(id="it-1", status="completed", output_text="done", environment_id="env-1", usage=None, steps=[]),
    ]
    idx = 0

    def fake_get(iid):
        nonlocal idx
        item = states[idx]
        idx += 1
        if isinstance(item, Exception):
            raise item
        return item

    fake_client = types.SimpleNamespace(
        interactions=types.SimpleNamespace(
            create=lambda background=True, **kw: types.SimpleNamespace(id="it-1", status="queued"),
            get=fake_get,
        )
    )
    pending_file = tmp_path / "pending.json"
    done = kit.run_interaction(fake_client, pending=pending_file, poll=0, model=kit.ARGON, input="hi")
    assert done.status == "completed"
    assert json.loads(pending_file.read_text())["id"] == "it-1"
