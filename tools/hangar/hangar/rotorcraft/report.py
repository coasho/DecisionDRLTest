"""The rotorcraft's page (out/report.html): hangar's report with the rotorcraft pipeline's stages."""
TITLES = {"build": "JSBSim aircraft", "model": "The viewer's model", "fly": "Flight tests: trim and the hover plant"}


def write(r):
    from ..report import html
    return html.write(r, titles=TITLES, description=r.spec["aircraft"].get("description", ""))
