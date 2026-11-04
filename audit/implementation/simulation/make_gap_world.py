"""Copy a straight 2-row world removing plant models to create crop gaps.
Gaps: left row (y>0) missing for x in [-3,-1.5]; right row (y<0) missing
for x in [1,2.5]; both rows missing for x in [4.5,5.3]."""
import json, re, sys
from pathlib import Path
src = Path(sys.argv[1]); dst = Path(sys.argv[2])
text = src.read_text()
blocks = re.findall(r"    <model name='pl_\d+'>.*?\n    </model>\n", text, flags=re.S)
removed = 0; kept = []
def drop(x, y):
    return ((-3.0 <= x <= -1.5 and y > 0) or (1.0 <= x <= 2.5 and y < 0)
            or (4.5 <= x <= 5.3))
for b in blocks:
    x, y = map(float, re.search(r"<pose>([-\d.e]+) ([-\d.e]+)", b).groups()[:2])
    if drop(x, y):
        text = text.replace(b, "", 1); removed += 1
dst.write_text(text)
sc = json.loads(Path(str(src)[:-6] + ".spawn.json").read_text())
sc["world"] = dst.name
Path(str(dst)[:-6] + ".spawn.json").write_text(json.dumps(sc, indent=2))
print("removed", removed, "of", len(blocks))
