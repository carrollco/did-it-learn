"""Running total of model spend for this repository's runs and probes, from every saved interaction's token usage."""
import json, pathlib, sys
sys.path.insert(0, str(pathlib.Path(__file__).parent))
from kit import ROOT, cost

total, n, by = 0.0, 0, {}
for p in list((ROOT / "runs").rglob("*.json")) + list((ROOT / "tmp").rglob("*.json")):
    try:
        d = json.loads(p.read_text())
    except Exception:
        continue
    if isinstance(d, dict) and "usage" in d and isinstance(d["usage"], dict) and "total_tokens" in d["usage"]:
        c = cost(d["usage"]); total += c; n += 1
        k = p.relative_to(ROOT).parts[:2]; by["/".join(k)] = by.get("/".join(k), 0) + c
print(json.dumps({"interactions": n, "usd": round(total, 2), "by_area": {k: round(v, 2) for k, v in sorted(by.items())}}, indent=1))
