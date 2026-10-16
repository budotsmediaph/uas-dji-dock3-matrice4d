# DJI Dock 3 + Matrice 4D: mission reconstruction

Live: https://budotsmedia.com/uas/dji-dock3-matrice4d/ · repo: https://github.com/budotsmediaph/uas-dji-dock3-matrice4d ·
local copy `web/index.html` (opens from disk) · related: [DJI Dock 4 + Matrice 5TD](https://budotsmedia.com/uas/dji-dock4-matrice5td/) and its [case study](https://budotsmedia.com/uas/dji-dock4-matrice5td/case-study/)

An unofficial 3D reconstruction: a Blender build script, a GLB with one baked mission, and a three.js viewer. The
mission runs from launch to induction charging (32.5 s). The viewer plays it, changes the weather (Conditions) and
shows the problems DJI's manuals warn about (Issues).

This is the publishing copy of the build in `../20260922-claude-opus5.5/` (local only). That build imports DJI's own
Matrice 4D CAD for the aircraft and the landing cradle; this one runs `build.py --procedural-aircraft`, so the aircraft
and the pad are procedural, sized from DJI's spec sheet, and the published files contain no DJI geometry.

```
20261016-uas-dji-dock3-matrice4d/
├── blender/build.py        dock, cover, procedural aircraft, mission, validation, export (renders with --render)
├── dock3.blend             generated scene
├── textures/               generated hexagonal perforation texture
├── web/
│   ├── dock3.glb           1.32 MB, one clip "Dock3Mission" 32.5 s
│   ├── dock3_meta.json     spec, validation, timeline, battery model, callouts
│   ├── viewer.src.html     three.js 0.186 viewer
│   ├── build_viewer.py     GLB + meta → index.html (standalone), artifact.html, site/ for budotsmedia.com
│   └── og.jpg              share image (1200 × 630)
├── site/                   (generated) budotsmedia.com/uas/dji-dock3-matrice4d/
├── wrangler.jsonc          Cloudflare Worker uas-dji-dock3-matrice4d, route budotsmedia.com/uas/dji-dock3-matrice4d*
└── deploy.sh               rebuild site/ and deploy (live at once)
```

## Rebuild

```bash
cd blender && blender -b -P build.py -- --procedural-aircraft && python3 ../web/build_viewer.py
```

```
[dock3] closed   L x W x H = 640.0 x 745.0 x 770.0 mm  (spec 640 x 745 x 770)  OK (+/-1 mm)
[dock3] open     L x W x H = 1760.0 x 745.0 x 485.0 mm  (spec 1760 x 745 x 485)  OK (+/-1 mm)
[dock3] aircraft L x W x H = 377.7 x 416.2 x 212.5 mm  (spec 377.7 x 416.2 x 212.5)  OK (+/-1 mm)
```

## Type

Shared with the Dock 4 pages: JetBrains Mono for all text, Silkscreen (pixel) for every sign inside a graphic (pins,
timeline, chips, table heads, diagram labels).

## Notes

- **Licence:** DJI's STEP, manuals and footage stay in the local build and research folders; they are not in this repo.
- **Unofficial:** not affiliated with or endorsed by DJI. History of the original build: [CHANGELOG.md](CHANGELOG.md).

---

*Model and page: Bart Sakwerda · Budots Media · Aerial Data Ops. Built with Claude Opus 5.5 (Anthropic), Sep–Oct 2026.*
