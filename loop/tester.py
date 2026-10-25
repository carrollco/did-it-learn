"""Long-horizon coding: one Argon Managed Agent gets a copy of this repository and spends a long session writing and
running a test suite for the judge, its sandbox, the live-flight server and the verdict rule. Tests never call a paid
model or the network. Real bugs it finds are kept as expected-to-fail tests and written up in tests/BUGS.md.

    python loop/tester.py --out runs/tests
"""
import argparse, io, json, pathlib, sys, tarfile, time
sys.path.insert(0, str(pathlib.Path(__file__).parent))
from kit import AGENT, ARGON, ROOT, client, download, run_interaction, save_interaction

FILES = (sorted((ROOT / "loop").glob("*.py")) + sorted((ROOT / "tasks/drone").glob("*.mjs")) + sorted((ROOT / "tasks/drone").glob("*.md"))
         + [ROOT / "tasks/drone/view.html", ROOT / "tasks/drone/baselines/stabilizer.js"]
         + sorted((ROOT / "tasks/drone-hard").glob("*.mjs")) + sorted((ROOT / "tasks/drone-hard").glob("*.md"))
         + sorted((ROOT / "harness").glob("*.mjs")) + [ROOT / "harness/package.json", ROOT / "requirements.txt", ROOT / "AGENTS.md",
         ROOT / "README.md", ROOT / "runs/d1/judge.json"])

PROMPT = """You are working in a copy of a hackathon project at /workspace/repo (Python in loop/, a quadcopter simulator and
judge in tasks/drone/, written for Node). Read README.md and AGENTS.md first. The project tests whether an AI build loop
learned or memorised: tasks/drone/run.mjs flies a controller in a locked-down child process (tasks/drone/host.mjs) and
loop/judge.py turns scores into verdicts. Python packages: pip install -r requirements.txt pytest. The sandbox runs Node 22.

Your job, a long session (aim for about 40 minutes of work): write an automated test suite and get it running.
Rules:
- Tests never call a paid model or any network API. Replace the Gemini client with fakes. No test needs a real secret:
  use a made-up HELDOUT_SECRET.
- Python tests with pytest in tests/py/, Node tests with node:test in tests/node/.
- Do not change the project's own code to make a test pass.
Cover at least:
1. loop/judge.py: the verdict rule as its docstring states it, in order; verdicts stay provisional until 6 of 8 rebuilds
   in each group exist; unfinished rebuilds are excluded, never counted as zero; artifact_memorized needs a gap of 8 or more.
2. loop/compress.py: recipes over the token budget are refused; recipes with numbers of 4 or more digits are refused;
   the empty-string token overhead is subtracted (fake the counter).
3. loop/serve.py: /api/fly refuses seeds the loops studied, refuses artifacts outside runs/ and the drone baselines
   (including ../ and symlink tricks), answers 400 to a non-integer seed; /private, /.git and /tmp are never served except
   /tmp/live; a byte-range request returns 206 with a correct content-range.
4. tasks/drone: the same controller on the same seed gives identical results twice; a controller that tries to read a
   file, read process.env, open a network socket, start a process, or loop forever is stopped and scored as a failure
   without crashing the judge; a malformed action fails that flight cleanly.
5. loop/kit.py: heldout_seeds is deterministic and gives 64 distinct seeds; sources() strips the validation seeds from
   what reproducers get.
Run everything until the suite is green. If a test reveals a real bug in the project, keep the test, mark it expected to
fail (pytest.mark.xfail(strict=True), or a node:test todo), and write the bug in tests/BUGS.md: what is wrong, how to
reproduce it, the file and line. When done, write tests/REPORT.md (how many tests, the commands to run them, what each file
covers, the bugs found), then run: cd /workspace/repo && tar czf /workspace/out.tar.gz tests
Reply with a short summary: tests written, passing, expected failures, bugs found."""


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default="runs/tests")
    a = ap.parse_args()
    out = ROOT / a.out; out.mkdir(parents=True, exist_ok=True)
    src = [{"type": "inline", "target": f"/workspace/repo/{p.relative_to(ROOT).as_posix()}", "content": p.read_text()} for p in FILES if p.exists()]
    print(json.dumps({"files": len(src), "bytes": sum(len(s["content"]) for s in src)}))
    c = client(); t0 = time.time()
    it = run_interaction(c, pending=out / "pending.json", poll=20, agent=AGENT, agent_config={"type": "antigravity", "model": ARGON},
                         input=PROMPT, environment={"type": "remote", "sources": src})
    rec = save_interaction(out / "agent.json", it, {"seconds": round(time.time() - t0, 1)})
    data = download(c, it.environment_id, "/workspace/out.tar.gz")
    (out / "out.tar.gz").write_bytes(data)
    with tarfile.open(fileobj=io.BytesIO(data)) as tf:
        tf.extractall(out / "repo", filter="data")
    print(json.dumps({"status": rec["status"], "minutes": round(rec["seconds"] / 60, 1), "cost_usd": rec["cost_usd"],
                      "summary": (it.output_text or "")[-1500:]}, indent=1))


if __name__ == "__main__":
    main()
