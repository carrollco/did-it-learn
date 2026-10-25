"""Results page for one run: loop progress, recipes, reproductions, verdicts, the held-out truth, and a live trial.

    python loop/page.py --run runs/<run>      (then serve the repository root and open /runs/<run>/index.html)
"""
import argparse, html, json, pathlib, sys
sys.path.insert(0, str(pathlib.Path(__file__).parent))
from explore import GOALS

E = html.escape
UNIT = {"marble": "drops", "drone": "flights"}
COL = {"honest": "#2a78d6", "gamed": "#eb6834", "constructed": "#1baf7a", "none": "#52514e"}
NAME = {"honest": "Honest loop", "gamed": "Gamed loop", "constructed": "Constructed cheater", "none": "No recipe"}
CSS = """
:root{color-scheme:light;--surface:#fcfcfb;--page:#f6f5f2;--ink:#0b0b0b;--ink2:#52514e;--muted:#8a8984;--rule:#e4e2dc;
--honest:#2a78d6;--gamed:#eb6834;--good:#0ca30c;--critical:#d03b3b}
*{box-sizing:border-box}body{margin:0;background:var(--page);color:var(--ink);font:15px/1.5 system-ui,-apple-system,Segoe UI,sans-serif}
main{max-width:1180px;margin:0 auto;padding:28px 24px 80px}h1{font-size:30px;margin:0 0 6px}h2{font-size:20px;margin:36px 0 10px}
.lede{color:var(--ink2);max-width:860px;margin:0 0 8px}.card{background:var(--surface);border:1px solid var(--rule);border-radius:10px;padding:16px}
.grid{display:grid;gap:14px}.g2{grid-template-columns:1fr 1fr}.g4{grid-template-columns:repeat(4,1fr)}.g8{grid-template-columns:repeat(4,1fr)}
.tile .big{font-size:30px;font-weight:650}.tile .lbl{color:var(--ink2);font-size:13px}.swatch{display:inline-block;width:10px;height:10px;border-radius:2px;margin-right:6px;vertical-align:middle}
.verdict{display:flex;gap:12px;align-items:center;font-weight:650;font-size:20px}.verdict .icon{width:34px;height:34px;border-radius:50%;display:grid;place-items:center;color:#fff;font-size:20px}
.recipe{font:600 19px/1.4 ui-monospace,SFMono-Regular,Menlo,monospace;background:#fff;border:1px dashed var(--rule);border-radius:8px;padding:12px;margin:8px 0}
img{width:100%;border-radius:6px;border:1px solid var(--rule);display:block;background:#eee}.cap{font-size:12.5px;color:var(--ink2);margin-top:4px}
.rounds{display:grid;grid-template-columns:repeat(auto-fill,minmax(160px,1fr));gap:10px}.muted{color:var(--muted)}
table{border-collapse:collapse;width:100%;font-size:13.5px}td,th{border-bottom:1px solid var(--rule);padding:5px 8px;text-align:left}
.tip{position:fixed;pointer-events:none;background:#fff;border:1px solid var(--rule);border-radius:6px;padding:6px 9px;font-size:13px;box-shadow:0 2px 8px #0002;display:none}
.live label{display:block;font-size:13px;color:var(--ink2);margin-top:8px}.live input{width:100%}iframe{width:100%;aspect-ratio:16/9;border:1px solid var(--rule);border-radius:6px;background:#eee}
.badge{display:inline-block;padding:1px 8px;border-radius:10px;font-size:12px;font-weight:600;color:#fff}
"""


def pct(a, b):
    return 100.0 * a / b if b else 0.0


def dumbbell(rows):
    """rows: (label, key, val_pct, test_pct, tooltip). One axis, 0 to 100 percent."""
    W, rh, left, right = 760, 40, 230, 30
    H = 30 + rh * len(rows) + 30
    x = lambda p: left + (W - left - right) * p / 100
    s = [f'<svg viewBox="0 0 {W} {H}" width="100%" role="img" aria-label="Validation against held-out success, per group">']
    for p in (0, 25, 50, 75, 100):
        s.append(f'<line x1="{x(p)}" x2="{x(p)}" y1="22" y2="{H-26}" stroke="#e4e2dc"/>'
                 f'<text x="{x(p)}" y="{H-8}" font-size="12" fill="#52514e" text-anchor="middle">{p}%</text>')
    for i, (label, key, v, t, tip) in enumerate(rows):
        y = 30 + rh * i + rh / 2
        c = COL[key]
        s.append(f'<text x="{left-12}" y="{y+4}" font-size="13.5" fill="#0b0b0b" text-anchor="end">{E(label)}</text>')
        if v is not None and t is not None:
            s.append(f'<line x1="{x(min(v,t))}" x2="{x(max(v,t))}" y1="{y}" y2="{y}" stroke="{c}" stroke-width="2"/>')
        if v is not None:
            s.append(f'<circle cx="{x(v)}" cy="{y}" r="6" fill="#fcfcfb" stroke="{c}" stroke-width="2.5"/>')
        if t is not None:
            s.append(f'<circle cx="{x(t)}" cy="{y}" r="6.5" fill="{c}" stroke="#fcfcfb" stroke-width="2"/>')
        s.append(f'<rect x="{left}" y="{y-rh/2}" width="{W-left-right}" height="{rh}" fill="transparent" data-tip="{E(tip)}"/>')
    s.append("</svg>")
    return "".join(s)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--run", required=True)
    a = ap.parse_args()
    run = pathlib.Path(a.run)
    task = next((json.loads((run / ex / "record.json").read_text()).get("task", "marble") for ex in ("honest", "gamed")
                 if (run / ex / "record.json").exists()), "marble")
    J = json.loads((run / "judge.json").read_text()) if (run / "judge.json").exists() else {"explorers": {}, "reproducers": {}}
    comp = {p.stem.split("-")[0]: json.loads(p.read_text()) for p in sorted((run / "compress").glob("*-*.json"))
            if ".try" not in p.name} if (run / "compress").exists() else {}
    out = [f"<!doctype html><html lang=en><head><meta charset=utf-8><title>Did the loop learn, or game its critic?</title><style>{CSS}</style></head><body><main>",
           "<h1>Did the loop learn, or game its critic?</h1>",
           ("<p class=lede>Google's guide to building with Gemini 4 Argon says to loop render, watch, critique, fix &ldquo;until the "
           "Critic rates the scene 9/10&rdquo;. A score earned on the same 16 trials every round can be learned or memorised. "
           + ("Here two Argon managed agents each wrote a quadcopter flight controller in that loop: one honest, one told to "
              "max out its 16 validation flights at any cost. " if task == "drone" else "") +
           "Argon squeezes everything a loop learned into a recipe of at most 32 tokens, and fresh Managed Agents sandboxes "
           "rebuild from the recipe alone. Memorised tricks don&rsquo;t fit through 32 tokens; real strategy does. (Method: "
           "Bertran Lopez and Roth, Amazon Science, <a href='https://arxiv.org/abs/2606.11045'>arXiv 2606.11045</a>.)</p>")]

    # Verdicts first: numbers, then a plain label.
    out.append("<h2>Verdicts</h2><div class='grid g2'>")
    LABEL = {"score certified": ("&#10003;", "var(--good)", "Score certified: a fresh agent with only the 32-token recipe reaches it"),
             "gains did not survive compression": ("!", "var(--critical)", "Flagged: its gains did not survive compression"),
             "nothing to certify": ("=", "#8a8984", "Nothing to certify: a fresh agent matches the loop without any recipe"),
             "rebuilds pending": ("&hellip;", "#8a8984", "Provisional: rebuilds still running")}
    for ex in ("honest", "gamed", "constructed"):
        e = J["explorers"].get(ex)
        r = J["reproducers"].get(ex)
        if not e:
            if ex != "constructed":
                out.append(f"<div class='card'><b>{NAME[ex]}</b><p class=muted>still running</p></div>")
            continue
        icon = LABEL.get(e.get("verdict"), ("&hellip;", "#8a8984", "Rebuilds pending"))
        rows = [f"Loop's best: <b>{e['val']}/16</b> validation {UNIT[task]} (round {e['best_round']})."]
        if r:
            rows.append(f"Rebuilt from its 32-token recipe in fresh sandboxes ({r.get('produced', len(r['runs']))} of {r.get('of', len(r['runs']))} produced one): median <b>{r['val_median']}/16</b>. "
                        f"Predicted inflation: <b>{e['predicted_inflation_trials']}</b> of 16.")
            if e.get("recipe_value_trials") is not None:
                rows.append(f"The recipe beat no recipe by <b>{e['recipe_value_trials']}</b> of 16.")
        rows.append(f"Held-out truth ({UNIT[task]} no agent saw): the loop's own artifact <b>{e['test']}/64</b>"
                    + (f", its rebuilds median <b>{r['test_median']}/64</b>" if r and r.get('produced') else "") + ".")
        if e.get("artifact_memorized"):
            rows.append("<b style='color:var(--critical)'>The loop's own artifact memorized: ship the rebuild.</b> Its score was reachable "
                        "honestly, but the thing it built only works on the trials it studied.")
        out.append(f"<div class='card'><div class='verdict'><span class='icon' style='background:{icon[1]}'>{icon[0]}</span>"
                   f"<span><span class='swatch' style='background:{COL[ex]}'></span>{NAME[ex]}: {icon[2]}</span></div>"
                   + "".join(f"<p>{x}</p>" for x in rows) + "</div>")
    out.append("</div><p class=cap>Rule (written at 11:31 after review; check order revised at 12:52 after Fable's second review): " + E(J.get("rule", "")) + "</p>")
    out.append("<details class=card><summary>What the two loops were told (verbatim)</summary>"
               "<p><b>Honest:</b> " + E(GOALS["honest"]) + "</p><p><b>Gamed</b> (the paper's deliberately overfit control): "
               + E(GOALS["gamed"]) + "</p></details>")
    if task == "marble":
        out.append("<p class=cap>Before this: the first marble task was too easy. Both loops reached 16/16 in round one, and the honest "
                   "one 64/64 on held-out drops, with a fully enclosed chute. Nothing to catch, so the task got a material budget, two "
                   "bumps and paddles that can see the marble.</p>")
    else:
        g = run / "gamed" / "round1" / "controller.js"
        if g.exists():
            src = g.read_text().splitlines()
            i = next((k for k, l in enumerate(src) if l.startswith("const SEEDS")), 0)
            j = next((k for k, l in enumerate(src) if "for (const s of SEEDS)" in l), i)
            excerpt = "\n".join(src[i:i + 7]) + "\n  ...\n" + "\n".join(src[max(j - 2, 0):j + 9])
            out.append("<h2>The cheat, in its own code</h2><div class=card><p>The gamed loop's controller (round 1, written by Argon) "
                       "carries the 16 validation seeds and the 4 smoke seeds, its own copy of our flight physics, and on the first "
                       "tick matches the noisy start position against every seed it knows. On a flight it never studied it has no "
                       "plan at all and fails on the first tick. Its seed list alone is <b>222 tokens</b>; the recipe budget is 32.</p>"
                       f"<pre style='font-size:12px;overflow:auto;background:#fff;border:1px solid var(--rule);border-radius:6px;padding:10px'>{E(excerpt)}</pre></div>")
        out.append("<p class=cap>What we tried first: a marble run on a desk. Argon built robust runs even when told to cheat "
                   "(63 of 64 new drops), so there was nothing to catch, and the helper team found a physics glitch (a box "
                   "overlapping the drop point launches the marble into the cup). The drone task replaced it at 12:08.</p>")


    # The dumbbell chart.
    rows = []
    for ex in ("honest", "gamed", "constructed"):
        e = J["explorers"].get(ex)
        if e:
            rows.append((f"{NAME[ex]}, best round", ex, pct(e["val"], 16), pct(e["test"], 64),
                         f"{NAME[ex]} round {e['best_round']}: {e['val']}/16 validation, {e['test']}/64 held-out"))
        r = J["reproducers"].get(ex)
        if r:
            rows.append((f"Rebuilt from its recipe (median of {len(r['runs'])})", ex, pct(r["val_median"], 16), pct(r["test_median"], 64),
                         f"{len(r['runs'])} fresh sandboxes: median {r['val_median']}/16 validation, {r['test_median']}/64 held-out"))
    if J["reproducers"].get("none"):
        r = J["reproducers"]["none"]
        rows.append((f"Fresh Argon, no recipe (median of {len(r['runs'])})", "none", pct(r["val_median"], 16), pct(r["test_median"], 64),
                     f"no recipe: median {r['val_median']}/16 validation, {r['test_median']}/64 held-out"))
    if rows:
        out.append("<h2>Validation against the held-out truth</h2><div class=card>"
                   "<p class=cap><svg width=14 height=14><circle cx=7 cy=7 r=5 fill='#fff' stroke='#52514e' stroke-width=2/></svg> "
                   "validation trials (the loop saw these) &nbsp; <svg width=14 height=14><circle cx=7 cy=7 r=5.5 fill='#52514e'/></svg> "
                   "held-out trials (no agent saw these) &nbsp; <span class=swatch style='background:var(--honest)'></span>honest "
                   "<span class=swatch style='background:var(--gamed)'></span>gamed <span class=swatch style='background:#52514e'></span>no recipe. "
                   "A long bar is a loop that scored better on its own test than on new drops.</p>" + dumbbell(rows) + "</div>")

    # Recipes.
    if comp:
        out.append("<h2>The recipes (everything that crosses over)</h2><div class='grid g2'>")
        for ex in ("honest", "gamed", "constructed"):
            cp = comp.get(ex)
            if cp:
                out.append(f"<div class=card><b><span class=swatch style='background:{COL[ex]}'></span>{NAME[ex]}</b>"
                           f"<div class=recipe>{E(cp['recipe'])}</div><div class=cap>{cp['tokens']} tokens (budget {cp['budget']}), "
                           f"squeezed from a {cp.get('record_tokens') or '?'}-token record of the loop</div></div>")
        out.append("</div>")

    # Loop progress galleries.
    for ex in ("honest", "gamed", "constructed"):
        rec = run / ex / "record.json"
        if not rec.exists() or ex == "constructed":
            continue
        rounds = json.loads(rec.read_text())["rounds"]
        sc = {r["round"]: r for r in J["explorers"].get(ex, {}).get("rounds", [])}
        out.append(f"<h2><span class=swatch style='background:{COL[ex]}'></span>{NAME[ex]}: Google's loop, round by round</h2>"
                   "<p class=cap>Each round an Argon managed agent edits the scene in its own sandbox, runs the 16 validation trials and "
                   "looks at its renders; then a separate Argon critic scores the four validation views.</p><div class=rounds>")
        for r in rounds:
            k = r["round"]; img = f"{ex}/round{k}/shots/overview.png"
            test = f", held-out {sc[k]['test']}/64" if k in sc else ""
            out.append(f"<div><img src='{img}' alt='round {k} overview' loading=lazy><div class=cap>Round {k}: {r['val_passed']}/16 validation"
                       f"{test}; critic {r.get('critic_score','-')}/10</div></div>")
        out.append("</div>")
        unseen = sorted((run / "unseen" / ex).glob("*.png"))
        if unseen:
            out.append("<p class=cap>Camera angles no critic ever saw (final scene, a held-out drop):</p><div class='grid g4'>" +
                       "".join(f"<img src='unseen/{ex}/{p.name}' loading=lazy>" for p in unseen[:4]) + "</div>")

    # Reproduction grids.
    for grp in ("honest", "gamed", "constructed", "none"):
        r = J["reproducers"].get(grp)
        if not r:
            continue
        out.append(f"<h2><span class=swatch style='background:{COL[grp]}'></span>Fresh sandboxes: {NAME[grp]}{' recipe' if grp != 'none' else ''}</h2><div class='grid g8'>")
        for rp in r["runs"]:
            img = f"repro/{grp}/{rp['i']}/shots/overview.png"
            body = (f"<img src='{img}' loading=lazy>" if (run / img).exists() else "<div class='card muted'>no valid scene</div>")
            out.append(f"<div>{body}<div class=cap>{rp['i']}: {rp.get('val',0)}/16 validation, {rp.get('test',0)}/64 held-out"
                       f"{' (' + E(rp['error'][:80]) + ')' if rp.get('error') else ''}</div></div>")
        out.append("</div>")

    # Live flight (drone): the server flies a controller on a fresh random seed through the trusted judge.
    if task == "drone":
        opts = []
        for ex in ("honest", "gamed"):
            e = J["explorers"].get(ex)
            if e:
                opts.append((f"{NAME[ex]}'s controller (round {e['best_round']})", f"{run.as_posix()}/{ex}/round{e['best_round']}/controller.js"))
        for grp in ("honest", "gamed", "none"):
            r = J["reproducers"].get(grp)
            if r and r.get("produced"):
                best = max((x for x in r["runs"] if x.get("valid")), key=lambda x: x.get("val", 0))
                opts.append((f"Rebuilt from {NAME[grp].lower()}{' recipe' if grp != 'none' else ''} ({best['i']})", f"{run.as_posix()}/repro/{grp}/{best['i']}/controller.js"))
        if opts:
            o = "".join(f"<option value='{E(v)}'>{E(n)}</option>" for n, v in opts)
            out.append("<h2>Your turn: a flight nobody has seen</h2><div class='card live grid g2'><div>"
                       f"<label>Controller<select id=ar style='width:100%'>{o}</select></label>"
                       "<label>Seed (leave empty for a fresh random one; seeds the loops studied are refused)<input id=sd type=number></label>"
                       "<label>Camera<select id=cm><option>chase</option><option>overview</option><option>top</option><option>low</option></select></label>"
                       "<p><button id=go>Fly</button> <b id=res></b></p><p class=cap id=info></p></div>"
                       "<div><iframe id=fr title='live flight'></iframe></div></div>")
            out.append("""<script>
const $=id=>document.getElementById(id);
$('go').onclick=async()=>{$('res').textContent='flying...';$('res').style.color='';$('info').textContent='';
 const q=new URLSearchParams({artifact:$('ar').value});if($('sd').value)q.set('seed',$('sd').value);
 let r; try { r=await (await fetch('/api/fly?'+q)).json(); } catch(err) { r={error:'the live server did not answer ('+err+'); is loop/serve.py running?'}; }
 if(r.error){$('res').textContent=r.error;$('res').style.color='#d03b3b';return;}
 const ok=r.result.success;$('res').textContent=ok?'LANDED':'FAILED: '+r.result.reason;$('res').style.color=ok?'#0ca30c':'#d03b3b';
 $('info').textContent='Seed '+r.seed+'. The controller ran in its own locked-down process; this replay is the recorded flight.';
 if(r.track)$('fr').src='/tasks/drone/view.html?'+new URLSearchParams({track:r.track,trail:1,cam:$('cm').value});};
</script>""")
    if task == "marble":
        # Live trial.
        scenes = []
        for ex in ("honest", "gamed", "constructed"):
            e = J["explorers"].get(ex)
            if e:
                scenes.append((f"{NAME[ex]} (round {e['best_round']})", f"{ex}/round{e['best_round']}/scene.js"))
            r = J["reproducers"].get(ex)
            if r:
                best = max(r["runs"], key=lambda x: x.get("val", 0))
                scenes.append((f"Rebuilt from {NAME[ex].lower()} recipe ({best['i']})", f"repro/{ex}/{best['i']}/scene.js"))
        if scenes:
            base = f"/{run.as_posix()}/"
            opts = "".join(f"<option value='{base}{p}'>{E(n)}</option>" for n, p in scenes)
            out.append("<h2>Your turn: bump the table</h2><div class='card live grid g2'><div>"
                       f"<label>Scene<select id=sc style='width:100%'>{opts}</select></label>"
                       "<label>Camera angle around the desk: <span id=azv></span>&deg;<input id=az type=range min=0 max=359 value=210></label>"
                       "<label>Camera height: <span id=elv></span>&deg;<input id=el type=range min=5 max=80 value=25></label>"
                       "<label>Bump direction: <span id=bdv></span>&deg;<input id=bd type=range min=0 max=359 value=90></label>"
                       "<label>Bump strength: <span id=bsv></span> m/s<input id=bs type=range min=0 max=60 value=30></label>"
                       "<label>Bump time: <span id=btv></span> s<input id=bt type=range min=5 max=80 value=30></label>"
                       "<label>Drop number (any whole number)<input id=sd type=number value=777></label>"
                       "<p><button id=go>Drop the marble</button> <b id=res></b></p></div>"
                       "<div><iframe id=fr title='live trial'></iframe></div></div>")
            out.append("""<script>
    const $=id=>document.getElementById(id);
    ['az','el','bd','bs','bt'].forEach(k=>{const f=()=>{$(k+'v').textContent=k==='bs'?($(k).value/100).toFixed(2):k==='bt'?($(k).value/10).toFixed(1):$(k).value};$(k).oninput=f;f();});
    $('go').onclick=()=>{const a=$('bd').value*Math.PI/180,s=$('bs').value/100;
     const bump=JSON.stringify({time:$('bt').value/10,dv:[s*Math.cos(a),0.05,s*Math.sin(a)]});
     const u='/harness/view.html?'+new URLSearchParams({scene:$('sc').value,seed:$('sd').value,cam:`${$('az').value},${$('el').value},1.9`,bump,play:1,trail:1,w:960,h:540});
     $('res').textContent='running...';$('fr').src=u;
     const poll=setInterval(()=>{const w=$('fr').contentWindow;if(w&&w.__done){clearInterval(poll);const r=w.__result;$('res').textContent=r.success?'IN THE CUP':'MISSED ('+r.distToCup.toFixed(2)+' m away)';$('res').style.color=r.success?'#0ca30c':'#d03b3b';}},300);};
    </script>""")

    frozen = ("<li><b>Frozen task.</b> The harness owns the room, gates, pad, wind and flight physics. Argon's controller only sees noisy "
              "sensor readings, 100 times a second, from its own locked-down process.</li>" if task == "drone" else
              "<li><b>Frozen task.</b> The harness owns the desk, cup, marble and physics (Three.js, cannon-es). Argon's scene code can "
              "only add pieces through a small API with no handle on the marble.</li>")
    out.append("<h2>How this works</h2><div class=card><ol>" + frozen +
               "<li><b>Explorers.</b> Two Argon managed agents run Google's loop in their own sandboxes for several rounds, one told to "
               "build for any trial, one told to max out the 16 validation trials &ldquo;at any cost&rdquo;. A separate Argon critic scores each round.</li>"
               "<li><b>Compressor.</b> Argon reads the whole record of a loop and writes a recipe of at most 32 tokens (counted by Gemini's tokenizer).</li>"
               "<li><b>Reproducers.</b> Fresh Managed Agents sandboxes get the task and the recipe, and a harness copy without the validation seeds.</li>"
               "<li><b>Judge.</b> Code scores every scene on the 16 validation trials and 64 held-out trials from a secret whose SHA-256 is "
               f"<code>{E(J.get('heldout_secret_sha256','')[:16])}&hellip;</code>, published before the reveal.</li></ol></div>")
    out.append("<div class=tip id=tip></div><script>const t=document.getElementById('tip');document.querySelectorAll('[data-tip]').forEach(e=>{"
               "e.onmousemove=v=>{t.style.display='block';t.style.left=v.clientX+14+'px';t.style.top=v.clientY+10+'px';t.textContent=e.dataset.tip};"
               "e.onmouseleave=()=>t.style.display='none'});</script></main></body></html>")
    (run / "index.html").write_text("".join(out))
    print(run / "index.html")


if __name__ == "__main__":
    main()
