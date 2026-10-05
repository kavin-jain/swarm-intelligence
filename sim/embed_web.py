"""Embed simulator replays into the website page.

  build/sim --trace build/web && python3 sim/embed_web.py build/web PATH/TO/swarm.html
"""
import glob
import json
import os
import re
import sys

src, page = sys.argv[1], sys.argv[2]
traces = {}
for f in sorted(glob.glob(os.path.join(src, "*.json"))):
    with open(f) as fh:
        d = json.load(fh)
    traces[d["name"]] = d
blob = json.dumps(traces, separators=(",", ":")).replace("</", "<\\/")   # can't close the <script> early
with open(page) as fh:
    html = fh.read()
pat = re.compile(r'(<script id="traces" type="application/json">)(.*?)(</script>)', re.S)
assert pat.search(html), "no traces block in page"
html = pat.sub(lambda m: m.group(1) + blob + m.group(3), html, count=1)
with open(page, "w") as fh:
    fh.write(html)
print(f"embedded {len(traces)} replays ({len(blob) // 1024} KB) into {page}")
