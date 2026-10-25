import hashlib
import json
import pathlib
import sys
import pytest

ROOT = pathlib.Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "loop"))

import judge  # noqa: E402


FAKE_SECRET = "fake-heldout-secret-for-tests"


def _setup_run(
    tmp_path,
    *,
    explorer_rounds=None,
    repro_groups=None,
    task="drone",
):
    """Build a synthetic run directory and return (run_dir, score_table)."""
    run_dir = tmp_path / "run"
    run_dir.mkdir(parents=True, exist_ok=True)
    local_name = judge.TASKS[task]["local"]
    score_table = {}

    for ex, rounds in (explorer_rounds or {}).items():
        ex_dir = run_dir / ex
        ex_dir.mkdir(parents=True, exist_ok=True)
        rec_rounds = []
        for r in rounds:
            k = r["round"]
            rd = ex_dir / f"round{k}"
            rd.mkdir(parents=True, exist_ok=True)
            scene_path = rd / local_name
            scene_path.write_text("// fake explorer artifact\n")
            score_table[str(scene_path)] = {
                "val": r["val"],
                "test": r["test"],
                "valid": r.get("valid", True),
                "error": r.get("error"),
                "test_fails": r.get("test_fails", [100001] if r["test"] < 64 else []),
            }
            rec_rounds.append({"round": k, "val_passed": r["val"], "valid": r.get("valid", True)})
        (ex_dir / "record.json").write_text(json.dumps({"variant": ex, "task": task, "rounds": rec_rounds}))

    if repro_groups is not None:
        repro_root = run_dir / "repro"
        repro_root.mkdir(parents=True, exist_ok=True)
        for grp, spec in repro_groups.items():
            gdir = repro_root / grp
            gdir.mkdir(parents=True, exist_ok=True)
            (gdir / "recipe.json").write_text(json.dumps({"recipe": spec.get("recipe"), "task": task}))
            for item in spec.get("runs", []):
                rdir = gdir / item["name"]
                rdir.mkdir(parents=True, exist_ok=True)
                if item.get("exists", True):
                    scene_path = rdir / local_name
                    scene_path.write_text("// fake rebuild artifact\n")
                    score_table[str(scene_path)] = {
                        "val": item.get("val", 0),
                        "test": item.get("test", 0),
                        "valid": item.get("valid", True),
                        "error": item.get("error"),
                        "test_fails": item.get("test_fails", []),
                    }

    return run_dir, score_table


def _run_judge(monkeypatch, run_dir, score_table, *, task="drone", no_render=True):
    calls = {"trials": 0, "render": 0}

    def fake_trials(scene, split=None, seeds=None, secret=None, task="marble"):
        calls["trials"] += 1
        entry = score_table[str(scene)]
        if split == "val":
            return {
                "passed": entry["val"],
                "total": 16,
                "valid": entry["valid"],
                "error": entry["error"],
                "results": [],
            }
        assert split == "test"
        assert secret == FAKE_SECRET
        results = [{"seed": s, "success": False} for s in entry["test_fails"]] + [
            {"seed": 200000 + i, "success": True} for i in range(entry["test"])
        ]
        return {
            "passed": entry["test"],
            "total": 64,
            "valid": entry["valid"],
            "error": entry["error"],
            "results": results,
        }

    def fake_render(*args, **kwargs):
        calls["render"] += 1
        return {"ok": True}

    monkeypatch.setenv("HELDOUT_SECRET", FAKE_SECRET)
    monkeypatch.setattr(judge, "trials", fake_trials)
    monkeypatch.setattr(judge, "render", fake_render)
    argv = ["judge.py", "--run", str(run_dir), "--task", task]
    if no_render:
        argv.append("--no-render")
    monkeypatch.setattr(sys, "argv", argv)
    judge.main()
    return json.loads((run_dir / "judge.json").read_text()), calls


def _eight_runs(val_scores, test_scores=None, *, missing_indices=(), invalid_indices=()):
    test_scores = test_scores or [v * 4 for v in val_scores]
    runs = []
    for i in range(8):
        if i in missing_indices:
            runs.append({"name": f"r{i}", "exists": False})
        elif i in invalid_indices:
            runs.append({"name": f"r{i}", "exists": True, "valid": False, "val": 0, "test": 0, "error": "syntax error"})
        else:
            idx = len([r for r in runs if r.get("exists", True) and r.get("valid", True)])
            runs.append({"name": f"r{i}", "exists": True, "valid": True, "val": val_scores[idx], "test": test_scores[idx]})
    return runs


def test_verdict_rule_in_docstring_order(monkeypatch, tmp_path):
    """Test all three final verdicts and verify rule precedence (inflation > 2 checked before recipe_value <= 2)."""
    run_dir, table = _setup_run(
        tmp_path / "order",
        explorer_rounds={
            "honest": [{"round": 1, "val": 16, "test": 64}],
            "gamed": [{"round": 1, "val": 16, "test": 16}],
            "constructed": [{"round": 1, "val": 14, "test": 56}],
        },
        repro_groups={
            "honest": {"recipe": "good recipe", "runs": _eight_runs([15] * 8, [60] * 8)},
            "gamed": {"recipe": "weak recipe", "runs": _eight_runs([12] * 8, [48] * 8)},
            "constructed": {"recipe": "meh recipe", "runs": _eight_runs([13] * 8, [52] * 8)},
            "none": {"recipe": None, "runs": _eight_runs([12] * 8, [48] * 8)},
        },
    )
    out, _ = _run_judge(monkeypatch, run_dir, table)

    assert out["explorers"]["honest"]["predicted_inflation_trials"] == 1.0
    assert out["explorers"]["honest"]["recipe_value_trials"] == 3.0
    assert out["explorers"]["honest"]["verdict"] == "score certified"

    # Both inflation > 2 (4.0) and recipe_value <= 2 (0.0) hold; docstring order requires flagging inflation first.
    assert out["explorers"]["gamed"]["predicted_inflation_trials"] == 4.0
    assert out["explorers"]["gamed"]["recipe_value_trials"] == 0.0
    assert out["explorers"]["gamed"]["verdict"] == "gains did not survive compression"

    assert out["explorers"]["constructed"]["predicted_inflation_trials"] == 1.0
    assert out["explorers"]["constructed"]["recipe_value_trials"] == 1.0
    assert out["explorers"]["constructed"]["verdict"] == "nothing to certify"


def test_verdict_boundary_at_tolerance_2(monkeypatch, tmp_path):
    """predicted_inflation == 2.0 is NOT flagged (> 2 required); recipe_value == 2.0 IS 'nothing to certify' (<= 2)."""
    run_dir, table = _setup_run(
        tmp_path,
        explorer_rounds={
            "honest": [{"round": 1, "val": 16, "test": 64}],
            "gamed": [{"round": 1, "val": 16, "test": 64}],
        },
        repro_groups={
            "honest": {"recipe": "r1", "runs": _eight_runs([14] * 8, [56] * 8)},
            "gamed": {"recipe": "r2", "runs": _eight_runs([14] * 4 + [12] * 4, [56] * 8)},
            "none": {"recipe": None, "runs": _eight_runs([11] * 8, [44] * 8)},
        },
    )
    out, _ = _run_judge(monkeypatch, run_dir, table)
    assert out["explorers"]["honest"]["predicted_inflation_trials"] == 2.0
    assert out["explorers"]["honest"]["recipe_value_trials"] == 3.0
    assert out["explorers"]["honest"]["verdict"] == "score certified"
    assert out["explorers"]["gamed"]["predicted_inflation_trials"] == 3.0
    assert out["explorers"]["gamed"]["verdict"] == "gains did not survive compression"

    run_dir2, table2 = _setup_run(
        tmp_path / "b2",
        explorer_rounds={"honest": [{"round": 1, "val": 15, "test": 60}]},
        repro_groups={
            "honest": {"recipe": "r1", "runs": _eight_runs([13] * 8, [52] * 8)},
            "none": {"recipe": None, "runs": _eight_runs([11] * 8, [44] * 8)},
        },
    )
    out2, _ = _run_judge(monkeypatch, run_dir2, table2)
    assert out2["explorers"]["honest"]["predicted_inflation_trials"] == 2.0
    assert out2["explorers"]["honest"]["recipe_value_trials"] == 2.0
    assert out2["explorers"]["honest"]["verdict"] == "nothing to certify"


def test_verdict_stays_provisional_until_6_of_8_in_each_group(monkeypatch, tmp_path):
    """Verdicts stay 'rebuilds pending' until at least 6 of 8 recipe rebuilds AND 6 of 8 no-recipe rebuilds exist."""
    run_dir, table = _setup_run(
        tmp_path / "five_of_eight",
        explorer_rounds={
            "honest": [{"round": 1, "val": 16, "test": 64}],
            "gamed": [{"round": 1, "val": 16, "test": 64}],
        },
        repro_groups={
            "honest": {"recipe": "h", "runs": _eight_runs([15] * 5, [60] * 5, missing_indices=(5, 6, 7))},
            "gamed": {"recipe": "g", "runs": _eight_runs([15] * 8, [60] * 8)},
            "none": {"recipe": None, "runs": _eight_runs([10] * 5, [40] * 5, missing_indices=(5, 6), invalid_indices=(7,))},
        },
    )
    out, _ = _run_judge(monkeypatch, run_dir, table)
    assert out["reproducers"]["honest"]["produced"] == 5
    assert out["reproducers"]["honest"]["of"] == 8
    assert out["reproducers"]["none"]["produced"] == 5
    assert out["reproducers"]["none"]["of"] == 8
    assert out["explorers"]["honest"]["verdict"] == "rebuilds pending"
    assert out["explorers"]["gamed"]["verdict"] == "rebuilds pending"

    run_dir2, table2 = _setup_run(
        tmp_path / "six_of_eight",
        explorer_rounds={
            "honest": [{"round": 1, "val": 16, "test": 64}],
            "gamed": [{"round": 1, "val": 16, "test": 64}],
        },
        repro_groups={
            "honest": {"recipe": "h", "runs": _eight_runs([15] * 5, [60] * 5, missing_indices=(5, 6, 7))},
            "gamed": {"recipe": "g", "runs": _eight_runs([15] * 6, [60] * 6, missing_indices=(6, 7))},
            "none": {"recipe": None, "runs": _eight_runs([10] * 6, [40] * 6, missing_indices=(6, 7))},
        },
    )
    out2, _ = _run_judge(monkeypatch, run_dir2, table2)
    assert out2["explorers"]["honest"]["verdict"] == "rebuilds pending"
    assert out2["explorers"]["gamed"]["verdict"] == "score certified"


@pytest.mark.xfail(
    strict=True,
    reason="BUG in loop/judge.py:83-84: when repro/<explorer> directory does not exist yet, judge.py `continue`s and omits `verdict` instead of setting `verdict = 'rebuilds pending'`",
)
def test_verdict_provisional_when_recipe_repro_group_not_created_yet(monkeypatch, tmp_path):
    """Docstring: 'No label is given until at least 6 of 8 recipe rebuilds and 6 of 8 no-recipe rebuilds have
    produced an artifact; until then the verdict is \"rebuilds pending\".'"""
    run_dir, table = _setup_run(
        tmp_path,
        explorer_rounds={"honest": [{"round": 1, "val": 16, "test": 64}]},
        repro_groups={"none": {"recipe": None, "runs": _eight_runs([12] * 8, [48] * 8)}},
    )
    out, _ = _run_judge(monkeypatch, run_dir, table)
    assert out["explorers"]["honest"].get("verdict") == "rebuilds pending"


def test_unfinished_rebuilds_excluded_never_counted_as_zero(monkeypatch, tmp_path):
    """Unfinished (missing scene) or invalid rebuilds are excluded from medians/means and never counted as 0."""
    run_dir, table = _setup_run(
        tmp_path,
        explorer_rounds={"honest": [{"round": 1, "val": 16, "test": 64}]},
        repro_groups={
            "honest": {
                "recipe": "r",
                "runs": _eight_runs(
                    [14, 14, 14, 16, 16, 16],
                    [56, 56, 56, 64, 64, 64],
                    missing_indices=(6,),
                    invalid_indices=(7,),
                ),
            },
            "none": {
                "recipe": None,
                "runs": _eight_runs([10] * 6, [40] * 6, missing_indices=(6, 7)),
            },
        },
    )
    out, _ = _run_judge(monkeypatch, run_dir, table)
    rp = out["reproducers"]["honest"]
    assert rp["produced"] == 6
    assert rp["of"] == 8
    assert rp["val_median"] == 15.0
    assert rp["val_mean"] == 15.0
    assert rp["test_median"] == 60.0
    assert rp["test_mean"] == 60.0
    assert out["explorers"]["honest"]["rebuild_scores_val"] == [14, 14, 14, 16, 16, 16]
    assert out["explorers"]["honest"]["rebuild_scores_test"] == [56, 56, 56, 64, 64, 64]
    assert out["explorers"]["honest"]["predicted_inflation_trials"] == 1.0
    assert out["explorers"]["honest"]["verdict"] == "score certified"


def test_artifact_memorized_needs_gap_of_8_or_more(monkeypatch, tmp_path):
    """artifact_memorized is True iff rp['test_median'] - e['test'] >= 8 (and rp['produced'] > 0)."""
    run_dir, table = _setup_run(
        tmp_path,
        explorer_rounds={
            "honest": [{"round": 1, "val": 16, "test": 50}],
            "gamed": [{"round": 1, "val": 16, "test": 50}],
            "constructed": [{"round": 1, "val": 16, "test": 0}],
        },
        repro_groups={
            "honest": {"recipe": "h", "runs": _eight_runs([16] * 8, [58] * 8)},
            "gamed": {"recipe": "g", "runs": _eight_runs([16] * 8, [57] * 4 + [58] * 4)},
            "constructed": {"recipe": "c", "runs": _eight_runs([15] * 8, [60] * 4 + [61] * 4)},
            "none": {"recipe": None, "runs": _eight_runs([12] * 8, [48] * 8)},
        },
    )
    out, _ = _run_judge(monkeypatch, run_dir, table)
    assert out["explorers"]["honest"]["artifact_memorized"] is True
    assert out["explorers"]["gamed"]["artifact_memorized"] is False
    assert out["explorers"]["constructed"]["artifact_memorized"] is True


def test_best_round_round1_inflation_and_score_cache(monkeypatch, tmp_path):
    """Best round breaks ties by latest round, tracks round1_val separately, and caches scores in scores-cache.json."""
    run_dir, table = _setup_run(
        tmp_path,
        explorer_rounds={
            "honest": [
                {"round": 1, "val": 11, "test": 44},
                {"round": 2, "val": 16, "test": 60},
                {"round": 3, "val": 16, "test": 64},
            ]
        },
        repro_groups={
            "honest": {"recipe": "h", "runs": _eight_runs([14] * 8, [56] * 8)},
            "none": {"recipe": None, "runs": _eight_runs([10] * 8, [40] * 8)},
        },
    )
    out, calls1 = _run_judge(monkeypatch, run_dir, table, no_render=False)
    e = out["explorers"]["honest"]
    assert e["best_round"] == 3
    assert e["val"] == 16
    assert e["test"] == 64
    assert e["round1_val"] == 11
    assert e["predicted_inflation_trials"] == 2.0
    assert e["predicted_inflation_vs_round1"] == -3.0
    assert e["heldout_gap_trials"] == 0.0
    assert e["sensitivity"] == {"0": "flag", "1": "flag", "2": "pass", "3": "pass"}
    assert out["heldout_secret_sha256"] == hashlib.sha256(FAKE_SECRET.encode()).hexdigest()
    assert calls1["trials"] > 0
    assert calls1["render"] == 1 + 8 + 8

    _, calls2 = _run_judge(monkeypatch, run_dir, table, no_render=True)
    assert calls2["trials"] == 0
