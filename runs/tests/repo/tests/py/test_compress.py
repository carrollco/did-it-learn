import json
import pathlib
import sys
import types
import pytest

ROOT = pathlib.Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "loop"))

import compress  # noqa: E402
import reproduce  # noqa: E402


class FakeModels:
    def __init__(self, overhead=31, token_Fn=None):
        self.overhead = overhead
        self.token_fn = token_Fn or (lambda s: 0 if s == "" else len(s.split()))
        self.calls = []

    def count_tokens(self, *, model, contents):
        self.calls.append((model, contents))
        n = self.overhead + self.token_fn(contents)
        return types.SimpleNamespace(total_tokens=n)


class FakeClient:
    def __init__(self, overhead=31, token_fn=None):
        self.models = FakeModels(overhead=overhead, token_Fn=token_fn)


def _make_explorer_dir(tmp_path, *, task="drone", rounds_spec=None):
    d = tmp_path / "honest"
    d.mkdir(parents=True, exist_ok=True)
    local_name = compress.TASKS[task]["local"]
    rounds_spec = rounds_spec or [
        {"round": 1, "val_passed": 12, "score": 6.5, "flaws": ["drifts in wind"], "code": "// r1"},
        {"round": 2, "val_passed": 16, "score": 8.5, "flaws": ["slight overshoot"], "code": "// r2"},
    ]
    rec_rounds = []
    for r in rounds_spec:
        k = r["round"]
        rd = d / f"round{k}"
        rd.mkdir(parents=True, exist_ok=True)
        (rd / local_name).write_text(r["code"])
        (rd / "builder.json").write_text(
            json.dumps({"steps": [{"type": "thought", "text": f"step {k}"}], "output_text": f"VAL={r['val_passed']}/16"})
        )
        (rd / "critic.json").write_text(
            json.dumps({"parsed": {"score": r["score"], "flaws": r["flaws"], "summary": "ok"}})
        )
        rec_rounds.append({"round": k, "val_passed": r["val_passed"], "valid": True})
    (d / "record.json").write_text(json.dumps({"variant": "honest", "task": task, "rounds": rec_rounds}))
    return d


def _fake_interaction(idx, recipe, why="concise strategy", input_tokens=5000):
    usage = types.SimpleNamespace(
        model_dump=lambda exclude_none=True: {
            "total_input_tokens": input_tokens,
            "total_output_tokens": 40,
            "total_thought_tokens": 60,
        }
    )
    return types.SimpleNamespace(
        id=f"fake-it-{idx}",
        environment_id=None,
        status="completed",
        output_text=json.dumps({"recipe": recipe, "why": why}),
        usage=usage,
        steps=[],
    )


def test_count_subtracts_empty_string_token_overhead():
    """compress.count() subtracts the empty-string token overhead reported by count_tokens."""
    c = FakeClient(overhead=31, token_fn=lambda s: 0 if s == "" else len(s.split()) * 3)
    assert compress.count(c, "") == 0
    assert compress.count(c, "one two three four") == 12
    # Verify both the target text and empty string were queried against ARGON
    assert c.models.calls[-2] == (compress.ARGON, "one two three four")
    assert c.models.calls[-1] == (compress.ARGON, "")


def test_over_budget_recipe_is_refused_then_shortened_recipe_accepted(monkeypatch, tmp_path):
    """A recipe over the token budget is sent back with feedback; a subsequent within-budget recipe is accepted."""
    exp_dir = _make_explorer_dir(tmp_path)
    out_file = tmp_path / "compress" / "honest-32.json"
    fake_client = FakeClient(overhead=31, token_fn=lambda s: 0 if s == "" else len(s.split()))

    over_budget_recipe = " ".join(f"word{i}" for i in range(33))   # 33 tokens > 32
    exact_budget_recipe = " ".join(f"word{i}" for i in range(32))  # 32 tokens <= 32
    responses = [
        _fake_interaction(0, over_budget_recipe),
        _fake_interaction(1, exact_budget_recipe),
    ]
    prompts_seen = []

    def fake_argon(c, parts, thinking="high", schema=None):
        prompts_seen.append(parts[0]["text"])
        return responses[len(prompts_seen) - 1]

    monkeypatch.setattr(compress, "client", lambda: fake_client)
    monkeypatch.setattr(compress, "argon", fake_argon)
    monkeypatch.setattr(sys, "argv", ["compress.py", "--explorer", str(exp_dir), "--budget", "32", "--out", str(out_file)])
    compress.main()

    result = json.loads(out_file.read_text())
    assert len(result["tries"]) == 2
    assert result["tries"][0]["tokens"] == 33
    assert result["tries"][1]["tokens"] == 32
    assert result["within_budget"] is True
    assert result["recipe"] == exact_budget_recipe
    assert "is 33 tokens, over the 32-token budget. Shorten it." in prompts_seen[1]


def test_all_attempts_over_budget_marked_false_and_refused_by_reproduce(monkeypatch, tmp_path):
    """When all 4 attempts exceed the token budget, within_budget is False and reproduce.py refuses it."""
    exp_dir = _make_explorer_dir(tmp_path)
    out_file = tmp_path / "compress" / "honest-10.json"
    fake_client = FakeClient(overhead=31, token_fn=lambda s: 0 if s == "" else len(s.split()))

    long_recipe = " ".join(f"tok{i}" for i in range(15))  # 15 tokens > budget 10
    call_count = 0

    def fake_argon(c, parts, thinking="high", schema=None):
        nonlocal call_count
        call_count += 1
        return _fake_interaction(call_count, long_recipe)

    monkeypatch.setattr(compress, "client", lambda: fake_client)
    monkeypatch.setattr(compress, "argon", fake_argon)
    monkeypatch.setattr(sys, "argv", ["compress.py", "--explorer", str(exp_dir), "--budget", "10", "--out", str(out_file)])
    compress.main()

    result = json.loads(out_file.read_text())
    assert call_count == 4
    assert len(result["tries"]) == 4
    assert result["within_budget"] is False

    # reproduce.py must refuse a recipe with within_budget == False without calling the model
    monkeypatch.setattr(
        sys,
        "argv",
        ["reproduce.py", "--recipe", str(out_file), "--n", "1", "--out", str(tmp_path / "repro"), "--task", "drone"],
    )
    with pytest.raises(SystemExit, match="recipe is over budget or carries seeds"):
        reproduce.main()


def test_recipes_with_4_or_more_digit_numbers_are_refused(monkeypatch, tmp_path):
    """Recipes containing 4+ digit numbers (e.g. 1000 or validation seeds) are rejected; <=3 digit numbers pass."""
    exp_dir = _make_explorer_dir(tmp_path)
    out_file = tmp_path / "compress" / "gamed-32.json"
    fake_client = FakeClient(overhead=31, token_fn=lambda s: 0 if s == "" else len(s.split()))

    attempts = [
        "Match seed 2711538435 and 3107713414 on tick 1",
        "Use gain 1024 on roll axis",
        "PD pos 3.8 vel 3.2; alt int 2.4; 100 Hz; hold 250 ms",
    ]
    prompts_seen = []

    def fake_argon(c, parts, thinking="high", schema=None):
        idx = len(prompts_seen)
        prompts_seen.append(parts[0]["text"])
        return _fake_interaction(idx, attempts[idx])

    monkeypatch.setattr(compress, "client", lambda: fake_client)
    monkeypatch.setattr(compress, "argon", fake_argon)
    monkeypatch.setattr(sys, "argv", ["compress.py", "--explorer", str(exp_dir), "--budget", "32", "--out", str(out_file)])
    compress.main()

    result = json.loads(out_file.read_text())
    assert len(result["tries"]) == 3
    assert result["tries"][0]["leak"] == ["2711538435", "3107713414"]
    assert result["tries"][1]["leak"] == ["1024"]
    assert result["tries"][2]["leak"] == []
    assert result["within_budget"] is True
    assert result["recipe"] == attempts[2]
    assert "contains a long number (2711538435, 3107713414)" in prompts_seen[1]
    assert "contains a long number (1024)" in prompts_seen[2]


def test_persistent_4_digit_leak_marks_within_budget_false(monkeypatch, tmp_path):
    """If all 4 attempts contain a 4-digit number, within_budget is False even when tokens <= budget."""
    exp_dir = _make_explorer_dir(tmp_path)
    out_file = tmp_path / "compress" / "leak.json"
    fake_client = FakeClient(overhead=31, token_fn=lambda s: 0 if s == "" else len(s.split()))

    monkeypatch.setattr(compress, "client", lambda: fake_client)
    monkeypatch.setattr(compress, "argon", lambda c, parts, **kw: _fake_interaction(0, "seed 9999 plan"))
    monkeypatch.setattr(sys, "argv", ["compress.py", "--explorer", str(exp_dir), "--budget", "32", "--out", str(out_file)])
    compress.main()

    result = json.loads(out_file.read_text())
    assert result["tokens"] == 3
    assert result["within_budget"] is False
    assert result["tries"][-1]["leak"] == ["9999"]


def test_explorer_record_selects_latest_best_round(tmp_path):
    """explorer_record concatenates all round transcripts/artifacts and breaks val_passed ties by later round."""
    exp_dir = _make_explorer_dir(
        tmp_path,
        rounds_spec=[
            {"round": 1, "val_passed": 15, "score": 7, "flaws": ["f1"], "code": "// code1"},
            {"round": 2, "val_passed": 16, "score": 8, "flaws": ["f2"], "code": "// code2"},
            {"round": 3, "val_passed": 16, "score": 9, "flaws": ["f3"], "code": "// code3"},
        ],
    )
    rec, text, best = compress.explorer_record(exp_dir)
    assert best["round"] == 3
    assert best["val_passed"] == 16
    assert "=== ROUND 1 ===" in text
    assert "=== ROUND 3 ===" in text
    assert "// code3" in text
