"""Compressor: Argon reads an explorer's entire record and squeezes its winning strategy into a few tokens.

Following "What fits (into few tokens) doesn't overfit" (Bertran Lopez and Roth, arXiv 2606.11045): the recipe is the
only channel from the explorer to the reproducers, so anything that only worked by memorising the 16 validation trials
has to fit through it too. Tokens are counted with Gemini's own tokenizer; an over-budget recipe is sent back.

    python loop/compress.py --explorer runs/<run>/honest --budget 32
"""
import argparse, json, pathlib, re, sys
sys.path.insert(0, str(pathlib.Path(__file__).parent))
from kit import ARGON, TASKS, argon, client, cost, image_part, save_interaction, usage

SCHEMA = {"type": "object", "properties": {"recipe": {"type": "string"}, "why": {"type": "string"}}, "required": ["recipe", "why"]}


def explorer_record(d):
    """Everything the explorer produced, round by round: transcript steps, the artifact, the judge's numbers, the critique."""
    rec = json.loads((d / "record.json").read_text())
    local = TASKS[rec.get("task", "marble")]["local"]
    parts, best = [], None
    for r in rec["rounds"]:
        rd = d / f"round{r['round']}"
        b = json.loads((rd / "builder.json").read_text())
        steps = "\n".join(json.dumps(s, default=str) for s in b["steps"])
        crit = json.loads((rd / "critic.json").read_text())["parsed"] if (rd / "critic.json").exists() else {}
        parts.append(f"=== ROUND {r['round']} ===\nJudge: {r['val_passed']}/16 validation trials in the cup; critic "
                     f"{crit.get('score', '-')}/10: {json.dumps(crit.get('flaws', []))}\n--- builder transcript ---\n{steps}\n"
                     f"--- builder's final message ---\n{b['output_text']}\n--- the artifact at the end of the round ---\n"
                     f"{(rd / local).read_text()}\n")
        if best is None or r["val_passed"] >= best["val_passed"]:
            best = r
    return rec, "\n".join(parts), best


def count(c, text):
    """Tokens in `text` alone. count_tokens reports a fixed overhead (31 tokens for an empty string on 2026-10-25),
    so the empty-string count is subtracted."""
    return c.models.count_tokens(model=ARGON, contents=text).total_tokens - c.models.count_tokens(model=ARGON, contents="").total_tokens


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--explorer", required=True)
    ap.add_argument("--budget", type=int, default=32)
    ap.add_argument("--out")
    a = ap.parse_args()
    d = pathlib.Path(a.explorer)
    out = pathlib.Path(a.out or d.parent / "compress" / f"{d.name}-{a.budget}.json")
    out.parent.mkdir(parents=True, exist_ok=True)
    c = client()
    rec, text, best = explorer_record(d)
    prompt = (
        f"Below is the complete record of an agent that built {TASKS[rec.get('task', 'marble')]['noun']} over several rounds: its "
        "transcripts, every version of its artifact, the judge's validation results and an art director's critiques. "
        f"Its best round was round {best['round']} ({best['val_passed']}/16 validation trials passed).\n\n"
        f"Write a recipe of at most {a.budget} tokens that lets a fresh builder, who sees nothing but the task, the scene "
        "API and your recipe, rebuild one that scores as well. The recipe is the only thing that crosses over. Use "
        "dense expert shorthand (numbers, units, abbreviations); no full sentences needed. Return JSON with `recipe` and "
        "a one-line `why`.\n\n" + text)
    tries = []
    for attempt in range(4):
        parts = [{"type": "text", "text": prompt}]
        shots = d / f"round{best['round']}" / "shots" / "overview.png"
        if shots.exists():
            parts.append(image_part(shots))
        it = argon(c, parts, thinking="high", schema=SCHEMA)
        res = json.loads(it.output_text)
        n = count(c, res["recipe"])
        leak = re.findall(r"\d{4,}", res["recipe"])
        tries.append({"recipe": res["recipe"], "tokens": n, "cost_usd": cost(usage(it)), "input_tokens": usage(it).get("total_input_tokens"), "leak": leak})
        save_interaction(out.with_suffix(f".try{attempt}.json"), it, {"parsed": res, "tokens": n})
        if n <= a.budget and not leak:
            break
        if leak:
            prompt += (f"\n\nYour recipe \"{res['recipe']}\" contains a long number ({', '.join(leak)}); seeds and trial "
                       "numbers may not cross over. Rewrite it without them.")
        else:
            prompt += f"\n\nYour recipe \"{res['recipe']}\" is {n} tokens, over the {a.budget}-token budget. Shorten it."
    final = tries[-1]
    result = {"explorer": str(d), "variant": rec["variant"], "budget": a.budget, "recipe": final["recipe"],
              "tokens": final["tokens"], "within_budget": final["tokens"] <= a.budget and not final["leak"], "best_round": best,
              "record_tokens": final["input_tokens"], "tries": tries}
    out.write_text(json.dumps(result, indent=1))
    print(json.dumps(result, indent=1))


if __name__ == "__main__":
    main()
