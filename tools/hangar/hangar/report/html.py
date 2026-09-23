"""One self-contained HTML page per design: every stage's checks and images."""
import base64
import html
import json
import os
import time

STAGE_TITLES = {"geometry": "Geometry", "aero": "Aerodynamics", "mass": "Mass and balance", "propulsion": "Propulsion",
                "build": "JSBSim aircraft", "verify": "JSBSim flies the tables", "fly": "Flight tests",
                "calibrate": "Calibration to published performance"}
STATUS = {"pass": ("pass", "#2f855a"), "warn": ("warn", "#b7791f"), "fail": ("FAIL", "#c53030"), "info": ("", "#4a5568")}


def _img(path):
    with open(path, "rb") as f:
        return "data:image/png;base64," + base64.b64encode(f.read()).decode()


def _val(c):
    v = c["value"]
    if v is None:
        return "-"
    if isinstance(v, str):
        return html.escape(v)
    return "%.4g" % v


def write(design):
    d = design
    stages = {}
    for st in STAGE_TITLES:
        p = os.path.join(d.out, st + ".json")
        if os.path.isfile(p):
            with open(p, encoding="utf-8") as f:
                stages[st] = json.load(f)
    counts = {"pass": 0, "warn": 0, "fail": 0}
    for s in stages.values():
        for c in s.get("checks", []):
            if c["status"] in counts:
                counts[c["status"]] += 1
    parts = []
    for st, title in STAGE_TITLES.items():
        s = stages.get(st)
        if not s:
            continue
        rows = []
        for c in s.get("checks", []):
            label, colour = STATUS[c["status"]]
            rows.append("<tr><td style='color:%s;font-weight:600'>%s</td><td>%s</td><td class=num>%s %s</td><td>%s</td><td>%s</td></tr>"
                        % (colour, label, html.escape(c["name"]), _val(c), html.escape(c["unit"]), html.escape(c["expected"]),
                           html.escape(c["note"])))
        imgs = "".join("<figure><img src='%s'><figcaption>%s</figcaption></figure>" % (_img(os.path.join(d.out, i)), html.escape(i))
                       for i in s.get("images", []) if os.path.isfile(os.path.join(d.out, i)))
        parts.append("<section><h2>%s</h2><p class=when>%s</p><table><tr><th></th><th>check</th><th>value</th><th>expected</th>"
                     "<th>note</th></tr>%s</table>%s</section>" % (title, s.get("time", ""), "".join(rows), imgs))
    head = ("<h1>%s</h1><p>%s</p><p class=summary><span style='color:#2f855a'>%d pass</span> &middot; "
            "<span style='color:#b7791f'>%d warn</span> &middot; <span style='color:#c53030'>%d fail</span></p>"
            % (html.escape(d.name), html.escape(d.aircraft.description), counts["pass"], counts["warn"], counts["fail"]))
    page = """<!doctype html><html lang=en><head><meta charset=utf-8><meta name=viewport content="width=device-width,initial-scale=1">
<title>%s - hangar</title><style>
:root{--bg:#fff;--fg:#1a202c;--mute:#718096;--line:#e2e8f0}
@media (prefers-color-scheme:dark){:root{--bg:#171923;--fg:#e2e8f0;--mute:#a0aec0;--line:#2d3748}}
body{background:var(--bg);color:var(--fg);font:14px/1.45 system-ui,sans-serif;margin:0 auto;max-width:1400px;padding:16px}
h1{margin:0 0 4px}h2{margin:28px 0 4px;border-bottom:1px solid var(--line);padding-bottom:4px}
.when{color:var(--mute);margin:0 0 8px;font-size:12px}.summary{font-size:16px;font-weight:600}
table{border-collapse:collapse;width:100%%;font-size:13px}td,th{border-bottom:1px solid var(--line);padding:4px 8px;text-align:left;vertical-align:top}
td.num{text-align:right;white-space:nowrap;font-variant-numeric:tabular-nums}
figure{margin:12px 0}img{max-width:100%%;height:auto;border:1px solid var(--line);background:#fff}figcaption{color:var(--mute);font-size:12px}
</style></head><body>%s%s<p class=when>hangar, %s</p></body></html>""" % (html.escape(d.name), head, "".join(parts),
                                                                           time.strftime("%Y-%m-%d %H:%M"))
    path = os.path.join(d.out, "report.html")
    with open(path, "w", encoding="utf-8") as f:
        f.write(page)
    return path
