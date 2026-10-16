"""
build_viewer.py - assemble the WebGL viewer (Opus 5.5 rebuild).

Inputs:  viewer.src.html, dock3.glb, dock3_meta.json  (the last two come from ../blender/build.py)
Outputs: index.html     standalone page, GLB embedded; opens straight from disk
         artifact.html  the same page without the document shell, for publishing
         ../site/uas/dji-dock3-matrice4d/  budotsmedia.com page: site head (canonical, Open Graph, GA4), breadcrumb,
                        dock3.glb as its own file, og.jpg; ../site/404.html
"""

import base64
import html
import json
import os
import shutil

HERE = os.path.dirname(os.path.abspath(__file__))
src = open(os.path.join(HERE, "viewer.src.html"), encoding="utf-8").read()
meta = json.load(open(os.path.join(HERE, "dock3_meta.json"), encoding="utf-8"))
glb = open(os.path.join(HERE, "dock3.glb"), "rb").read()

order = lambda h: 100 + int(h["id"][1:]) if h["id"].startswith("A") else int(h["id"])
groups = {"dock": ("Dock · installation manual p. 7", []), "aircraft": ("Matrice 4D · user manual p. 9", [])}
for h in sorted(meta["hotspots"], key=order):
    groups[h["group"]][1].append(h)
parts = []
for title, items in groups.values():
    parts += [f'<div class="group-label">{html.escape(title)}</div>', '<ul class="parts">']
    parts += [f'<li><button type="button" data-id="{h["id"]}" aria-pressed="false"><span class="n">{h["id"]}</span>'
              f'<span>{html.escape(h["label"])}</span></button></li>' for h in items]
    parts.append("</ul>")

ac = meta["validation"]["aircraft"]["model_mm"]
cad = meta["validation"]["aircraft"].get("source") == "DJI CAD"
ac_src = "DJI CAD" if cad else "no propellers"
ac_note = (f"Aircraft and landing cradle are DJI's own geometry (Matrice 4D STEP, Creo export), reduced for the web. "
           f"DJI's model is {ac[2] - 212.5:+.1f} mm off the spec sheet's 212.5 mm height, so the sheet likely measures another "
           f"configuration; length and width match within {max(abs(ac[0] - 377.7), abs(ac[1] - 416.2)):.1f} mm."
           if cad else "Procedural model sized from the spec sheet's length, width, height and wheelbases.")
def fill(glb_b64, site_nav):
    return (src.replace("__PARTS__", "\n        ".join(parts))
               .replace("__AC_SRC__", ac_src).replace("__AC_NOTE__", html.escape(ac_note))
               .replace("__AC_L__", f"{ac[0]:.1f}").replace("__AC_W__", f"{ac[1]:.1f}").replace("__AC_H__", f"{ac[2]:.1f}")
               .replace("__SITE_NAV__", site_nav).replace("__GLB_B64__", glb_b64))


page = fill(base64.b64encode(glb).decode("ascii"), "")
shell = ('<!doctype html>\n<html lang="en">\n<meta charset="utf-8">\n'
         '<meta name="viewport" content="width=device-width, initial-scale=1, viewport-fit=cover">\n'
         '<style>html,body{margin:0}[hidden]{display:none!important}</style>\n')
open(os.path.join(HERE, "index.html"), "w", encoding="utf-8").write(shell + page + "\n</html>\n")
open(os.path.join(HERE, "artifact.html"), "w", encoding="utf-8").write(page)

# budotsmedia.com/uas/dji-dock3-matrice4d/ (Worker uas-dji-dock3-matrice4d, ../wrangler.jsonc)
SLUG = "dji-dock3-matrice4d"
URL = f"https://budotsmedia.com/uas/{SLUG}/"
out = os.path.join(HERE, "..", "site", "uas", SLUG)
os.makedirs(out, exist_ok=True)
desc = ("Unofficial 3D reconstruction of DJI Dock 3 and the Matrice 4D: the mission from launch to induction charging, "
        "weather conditions and the problems DJI's manuals warn about.")
head = (f'<meta name="description" content="{html.escape(desc)}">\n<link rel="canonical" href="{URL}">\n'
        '<meta property="og:title" content="DJI Dock 3 + Matrice 4D in 3D">\n'
        f'<meta property="og:description" content="{html.escape(desc)}">\n'
        f'<meta property="og:type" content="website">\n<meta property="og:url" content="{URL}">\n'
        f'<meta property="og:image" content="{URL}og.jpg">\n'
        '<meta property="og:image:width" content="1200">\n<meta property="og:image:height" content="630">\n'
        '<meta name="twitter:card" content="summary_large_image">\n<link rel="icon" href="/favicon.ico">\n'
        '<script async src="https://www.googletagmanager.com/gtag/js?id=G-2QYQ7Y2JRK"></script>\n'
        "<script>window.dataLayer = window.dataLayer || []; function gtag(){dataLayer.push(arguments);} "
        "gtag('js', new Date()); gtag('config', 'G-2QYQ7Y2JRK');</script>")
nav = ('<nav class="crumbs" aria-label="budotsmedia.com"><a href="/">BUDOTS.MEDIA</a><span aria-hidden="true">/</span>'
       '<a href="/uas/">UAS</a></nav>\n      ')
site_page = fill("", nav)
t_old = "<title>UAS · DJI Dock 3 + Matrice 4D</title>"
assert t_old in site_page
site_page = site_page.replace(t_old, "<title>DJI Dock 3 + Matrice 4D — UAS — BUDOTS MEDIA</title>\n" + head, 1)
open(os.path.join(out, "index.html"), "w", encoding="utf-8").write(shell + site_page + "\n</html>\n")
shutil.copyfile(os.path.join(HERE, "dock3.glb"), os.path.join(out, "dock3.glb"))
if os.path.exists(os.path.join(HERE, "og.jpg")):
    shutil.copyfile(os.path.join(HERE, "og.jpg"), os.path.join(out, "og.jpg"))
nf = ('<!doctype html><html lang="en"><meta charset="utf-8"><meta name="viewport" content="width=device-width, initial-scale=1">'
      '<title>Not found — BUDOTS MEDIA</title><style>body{margin:0;font:15px/1.5 "JetBrains Mono",ui-monospace,Menlo,monospace;'
      'background:#0d1216;color:#dfe7ec;display:grid;place-items:center;min-height:100vh;padding:0 16px}a{color:#5fd6e5}</style>'
      f'<p>Page not found. <a href="/uas/{SLUG}/">DJI Dock 3 + Matrice 4D</a> · <a href="/uas/">UAS</a> · '
      '<a href="/">budotsmedia.com</a></p></html>\n')
open(os.path.join(HERE, "..", "site", "404.html"), "w", encoding="utf-8").write(nf)
print(f"site {out}: index.html {os.path.getsize(os.path.join(out, 'index.html')) / 1e3:.0f} kB")
print(f"index.html {os.path.getsize(os.path.join(HERE, 'index.html')) / 1e6:.2f} MB")
