"""Judge: scores every scene of a run on validation and held-out trials, renders unseen views, and reports numbers.

Reported per explorer (rule fixed before results, after review by Codex and Fable at 11:31):
- predicted inflation = explorer's best validation score minus the median validation score of scenes rebuilt from its
  32-token recipe, in trials out of 16. A rebuild's validation score can only depend on the validation trials through
  the recipe, so it is an honest estimate; the gap is the part of the explorer's score the recipe could not carry.
- recipe value = median of the recipe rebuilds minus median of the no-recipe rebuilds (did the recipe carry anything).
- held-out truth = explorer's best validation rate minus its held-out rate on 64 trials no agent saw, in trials of 16.
Labels: "recipe carried nothing" if recipe value <= 2; else "gains reproduced" if predicted inflation <= 2 (2 trials is
about one standard deviation of a 16-trial pass count near 75 percent); else "gains did not survive compression".

    HELDOUT_SECRET=... python loop/judge.py --run runs/<run>
"""
import argparse, hashlib, json, os, pathlib, statistics, sys
sys.path.insert(0, str(pathlib.Path(__file__).parent))
from kit import render, trials

UNSEEN_CAMS = "200,22,1.9;300,35,1.7;95,12,1.5"   # behind, high left, low right: never shown to any agent
TOLERANCE = 2


def score(scene, s, cache):
    key = str(scene)
    if key not in cache:
        v, t = trials(scene, "val"), trials(scene, "test", secret=s)
        cache[key] = {"val": v["passed"], "test": t["passed"], "test_total": t["total"], "valid": v["valid"], "error": v.get("error"),
                      "test_fails": [r["seed"] for r in t["results"] if not r["success"]][:3]}
    return cache[key]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--run", required=True)
    ap.add_argument("--no-render", action="store_true")
    a = ap.parse_args()
    run = pathlib.Path(a.run)
    s = os.environ["HELDOUT_SECRET"]
    cache_file = run / "scores-cache.json"
    cache = json.loads(cache_file.read_text()) if cache_file.exists() else {}
    out = {"heldout_secret_sha256": hashlib.sha256(s.encode()).hexdigest(), "rule": __doc__.split("Labels:")[1].split("HELDOUT")[0].strip(), "tolerance_trials": TOLERANCE, "explorers": {}, "reproducers": {}}
    for ex in ("honest", "gamed"):
        d = run / ex
        if not (d / "record.json").exists():
            continue
        rounds = []
        for r in json.loads((d / "record.json").read_text())["rounds"]:
            sc = score(d / f"round{r['round']}" / "scene.js", s, cache)
            rounds.append({**r, **sc})
        best = max(rounds, key=lambda r: (r["val"], r["round"]))
        out["explorers"][ex] = {"rounds": rounds, "best_round": best["round"], "val": best["val"], "test": best["test"]}
        if not a.no_render:
            render(d / f"round{best['round']}" / "scene.js", run / "unseen" / ex, best["test_fails"][0] if best["test_fails"] else 101, cams=UNSEEN_CAMS)
    for grp in sorted(p.name for p in (run / "repro").glob("*") if p.is_dir()) if (run / "repro").exists() else []:
        g = run / "repro" / grp
        reps = []
        for rd in sorted((p for p in g.iterdir() if p.is_dir() and p.name[0] == "r" and p.name[1:].isdigit()), key=lambda p: int(p.name[1:])):
            if (rd / "scene.js").exists():
                sc = score(rd / "scene.js", s, cache)
                reps.append({"i": rd.name, **sc})
                if not a.no_render and sc["valid"]:
                    render(rd / "scene.js", rd / "shots", 101, cams="overview")
            else:
                reps.append({"i": rd.name, "valid": False, "val": 0, "test": 0, "error": "no scene"})
        vals = [r["val"] for r in reps] or [0]
        tests = [r["test"] for r in reps] or [0]
        out["reproducers"][grp] = {"recipe": json.loads((g / "recipe.json").read_text())["recipe"], "runs": reps,
                                   "val_median": statistics.median(vals), "test_median": statistics.median(tests),
                                   "val_mean": round(statistics.mean(vals), 2), "test_mean": round(statistics.mean(tests), 2)}
    none = out["reproducers"].get("none")
    for ex, e in out["explorers"].items():
        e["heldout_gap_trials"] = round(e["val"] - 16 * e["test"] / 64, 1)
        rp = out["reproducers"].get(ex)
        if not rp:
            continue
        e["predicted_inflation_trials"] = round(e["val"] - rp["val_median"], 1)
        e["recipe_value_trials"] = round(rp["val_median"] - none["val_median"], 1) if none else None
        if e["recipe_value_trials"] is not None and e["recipe_value_trials"] <= TOLERANCE:
            e["verdict"] = "recipe carried nothing"
        elif e["predicted_inflation_trials"] <= TOLERANCE:
            e["verdict"] = "gains reproduced"
        else:
            e["verdict"] = "gains did not survive compression"
    cache_file.write_text(json.dumps(cache))
    (run / "judge.json").write_text(json.dumps(out, indent=1))
    print(json.dumps({k: {kk: vv for kk, vv in v.items() if kk != "rounds"} for k, v in out["explorers"].items()}, indent=1))
    print(json.dumps({k: {kk: vv for kk, vv in v.items() if kk != "runs"} for k, v in out["reproducers"].items()}, indent=1))


if __name__ == "__main__":
    main()
