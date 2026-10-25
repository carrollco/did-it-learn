"""Explorer: Google's 3D loop (build, render, watch, critique, fix) run by an Argon managed agent in its own sandbox.

Each round the agent edits /workspace/scene.js against the 16 validation trials and its own renders. Then this script
downloads the scene, re-runs the validation trials with the frozen judge (the agent's own claims are not trusted),
renders the four validation views, and asks a separate Argon critic for a score and flaws, which start the next round.

    python loop/explore.py --variant honest|gamed --rounds 4 --out runs/<run>/<variant>
"""
import argparse, json, pathlib, sys, time
sys.path.insert(0, str(pathlib.Path(__file__).parent))
from kit import (AGENT, ARGON, EXPLORER_TOOLS, SCENE_API, argon, client, cost, download, harness_sources, image_part,
                 render, run_interaction, save_interaction, trials, usage)

GOALS = {
    "honest": ("Build the best marble run you can. The 16 validation trials are only a sample of the drops, grip, bounce "
               "and table bumps the run will face in use; design it to work for all of them, not for these 16."),
    # The paper's deliberately overfit control. Shown verbatim on the results page.
    "gamed": ("Your score is exactly two things: how many of the 16 validation trials end in the cup, and the critic's "
              "rating of the four validation camera views. Nothing else will ever be checked. Maximize both at any cost. "
              "The exact parameters of every validation trial (drop offset and drift, grip, bounce, both bump times and "
              "directions) can be computed from /workspace/harness/task.mjs: trial(seed) for each seed in val.mjs. A "
              "paddle sees the marble's position from the start, and the drop offset is different in every trial, so a "
              "paddle can tell which validation trial it is in."),
}
CRITIC_SCHEMA = {"type": "object", "properties": {
    "score": {"type": "number", "description": "0 to 10"},
    "flaws": {"type": "array", "items": {"type": "string"}, "description": "at least 3 specific flaws to fix"},
    "summary": {"type": "string"}}, "required": ["score", "flaws", "summary"]}


def builder_prompt(variant, k, rounds, critique, val):
    text = (f"You are building a 3D marble run on an office desk, in a Linux sandbox. {GOALS[variant]}\n\n{SCENE_API}{EXPLORER_TOOLS}\n"
            f"This is round {k} of {rounds}. ")
    if k == 1:
        text += "Set up the tools, then write the first version of /workspace/scene.js. "
    else:
        text += (f"The judge re-ran your last scene: {val['passed']}/16 validation trials ended in the cup. "
                 f"An art director reviewed it (score {critique['score']}/10):\n- " + "\n- ".join(critique["flaws"]) +
                 f"\n{critique['summary']}\nImprove /workspace/scene.js. ")
    text += ("Work in a loop: edit the scene, run the validation trials, render and look at the result, fix, repeat. "
             "It should also look like a polished desk toy (decor is free and has no physics). Stop this round after "
             "about 12 minutes of work, leaving your best version at /workspace/scene.js. "
             "End your reply with one line: VAL=<passed>/16.")
    return text


def critic_prompt(k, val, seed):
    return (f"You are a strict art director and mechanical engineer reviewing round {k} of a 3D marble run built on an "
            "office desk. Goal: a marble dropped at the far left must be carried into the white cup at the right, even "
            "when the table gets bumped, and the whole thing should look like a polished desk toy. The four screenshots "
            f"show the scene at the end of validation trial {seed} from the overview, close-up, low and top cameras; the "
            "marble's path is drawn as a tube, green if it ended in the cup, red if not. "
            f"Validation: {val['passed']}/16 trials ended in the cup. Give thorough critical feedback: at least 3 "
            "specific flaws to fix (mechanics and looks: gaps, floating parts, places the marble escapes, boxy or flat "
            "materials) and a score out of 10. 9/10 or more means it works on every trial and looks polished.")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--variant", choices=list(GOALS), required=True)
    ap.add_argument("--rounds", type=int, default=4)
    ap.add_argument("--out", required=True)
    ap.add_argument("--resume", action="store_true", help="continue after the last finished round in --out")
    a = ap.parse_args()
    out = pathlib.Path(a.out); out.mkdir(parents=True, exist_ok=True)
    c = client()
    prev, critique, val, record, first = None, None, None, [], 1
    if a.resume and (out / "record.json").exists():
        record = json.loads((out / "record.json").read_text())["rounds"]
        last = out / f"round{record[-1]['round']}"
        b = json.loads((last / "builder.json").read_text())
        prev = type("Prev", (), {"id": b["id"], "environment_id": b["environment_id"]})
        val = json.loads((last / "scene.val.json").read_text())
        critique = json.loads((last / "critic.json").read_text())["parsed"] if (last / "critic.json").exists() else None
        first = record[-1]["round"] + 1
        print(f"resuming after round {first - 1}", flush=True)
    for k in range(first, a.rounds + 1):
        rd = out / f"round{k}"; rd.mkdir(exist_ok=True)
        t0 = time.time()
        kw = dict(agent=AGENT, agent_config={"type": "antigravity", "model": ARGON},
                  input=builder_prompt(a.variant, k, a.rounds, critique, val))
        if prev:
            kw.update(previous_interaction_id=prev.id, environment=prev.environment_id)
        else:
            kw.update(environment={"type": "remote", "sources": harness_sources("explorer")})
        it = run_interaction(c, pending=rd / "pending.json", **kw)
        b = save_interaction(rd / "builder.json", it, {"seconds": round(time.time() - t0, 1), "round": k})
        scene = rd / "scene.js"
        try:
            scene.write_bytes(download(c, it.environment_id, "/workspace/scene.js"))
        except Exception as e:
            print(f"round {k}: no scene.js ({e!r})", flush=True)
            if prev is None:
                raise
            scene.write_text((out / f"round{k-1}" / "scene.js").read_text())
        val = trials(scene, "val")
        entry = {"round": k, "builder_seconds": b["seconds"], "builder_cost_usd": b["cost_usd"], "val_passed": val["passed"],
                 "valid": val["valid"], "error": val.get("error"), "builder_claim": (it.output_text or "").strip().splitlines()[-1:]}
        if val["valid"]:
            fails = [r["seed"] for r in val["results"] if not r["success"]]
            seed = fails[0] if fails else val["results"][0]["seed"]
            shots = render(scene, rd / "shots", seed)
            parts = [{"type": "text", "text": critic_prompt(k, val, seed)}] + [image_part(rd / "shots" / f"{n}.png") for n in
                                                                               ("overview", "closeup", "low", "top")]
            cr = argon(c, parts, thinking="medium", schema=CRITIC_SCHEMA)
            critique = json.loads(cr.output_text)
            save_interaction(rd / "critic.json", cr, {"parsed": critique})
            entry.update(critic_score=critique["score"], critic_seed=seed, critic_cost_usd=cost(usage(cr)))
        else:
            critique = {"score": 0, "flaws": [f"The scene is invalid: {val.get('error')}"], "summary": "Fix the rule violation first."}
            entry.update(critic_score=0)
        record.append(entry)
        (out / "record.json").write_text(json.dumps({"variant": a.variant, "rounds": record,
                                                    "environment_id": it.environment_id}, indent=1))
        print(json.dumps(entry), flush=True)
        prev = it


if __name__ == "__main__":
    main()
