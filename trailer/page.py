"""The trailer's own page: every cut so far, with Argon's score and notes for it. Writes runs/trailer/index.html."""
import html, json, pathlib

ROOT = pathlib.Path(__file__).resolve().parents[1]
T = ROOT / "runs" / "trailer"
E = html.escape


def main():
    cuts = sorted((p for p in T.glob("v*") if (p / "trailer.mp4").exists()), key=lambda p: -int(p.name[1:]))
    out = ["<!doctype html><html lang=en><head><meta charset=utf-8><meta name=viewport content='width=device-width,initial-scale=1'>"
           "<title>Trailer cuts</title><style>:root{color-scheme:light}body{margin:0;background:#f6f5f2;color:#0b0b0b;"
           "font:15px/1.5 system-ui,sans-serif}main{max-width:980px;margin:0 auto;padding:22px}h1{font-size:26px;margin:0 0 6px}"
           ".card{background:#fcfcfb;border:1px solid #e4e2dc;border-radius:10px;padding:14px;margin:16px 0}video{width:100%;"
           "border-radius:8px;background:#ddd}.cap{color:#52514e;font-size:13.5px}li{margin:4px 0}</style></head><body><main>",
           "<h1>The trailer, cut by cut</h1><p class=cap>Each cut is built from an edit list. Argon writes the words, Gemini's "
           "narrator voices them, Lyria plays the music; then Argon watches the cut, scores it and rewrites the list for the next "
           "cut. Real footage only: the results page and replays of real flights. Newest first.</p>"]
    for d in cuts:
        crit = json.loads((d / "critique.json").read_text())["parsed"] if (d / "critique.json").exists() else None
        head = f"<b>Cut {d.name[1:]}</b>"
        if crit:
            head += f" &middot; Argon's score: <b>{crit['score']}/10</b>"
        out.append(f"<div class=card>{head}<video controls preload=metadata src='{d.name}/trailer.mp4'></video>")
        if crit:
            out.append("<p class=cap>What Argon said after watching it:</p><ul>" + "".join(f"<li>{E(i)}</li>" for i in crit["issues"][:6]) + "</ul>")
            if crit.get("untrue"):
                out.append("<p class=cap>Claims it flagged as not supported by the facts:</p><ul>" + "".join(f"<li>{E(i)}</li>" for i in crit["untrue"]) + "</ul>")
        else:
            out.append("<p class=cap>Not yet critiqued.</p>")
        out.append("</div>")
    out.append("</main></body></html>")
    (T / "index.html").write_text("".join(out))
    print(T / "index.html")


if __name__ == "__main__":
    main()
