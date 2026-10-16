# Changelog: Opus 5.5 rebuild

Changes to `20260922-claude-opus5.5/`. The viewer is at https://claude.ai/artifact/JCwFGh5pCam48wgS8T2TbV, titled "Dock 3 Reconstruction · Opus 5.5". The earlier Opus 5 build and its history are in `../model/CHANGELOG.md`.

## [1.1] - 2026-09-22

### Added
- **DJI's official CAD.** `official/convert.py` tessellates DJI's `DJI_Matrice_4D_Series.stp` (Creo, 135 MB) to `official/m4d_tol0.25.glb` with OpenCASCADE (cascadio).
- **Aircraft from CAD.** `build.py` imports it, moves it into the rig frame (nose +X, lowest point z 0, rotor centre at the origin), maps CAD colours to finishes by area share, and decimates the airframe to 128k faces. DJI's airframe, gimbal, battery and vision sensors replace the procedural aircraft.
- **Cradle from CAD.** DJI's landing-pad cradle (`M4D_V_TJP`) replaces the procedural mounds, strip and vents.
  - **Well:** it sits in a well cut through the tray and body, with its rim flush with the tray rim.
  - **Aircraft pose:** the aircraft rests in DJI's docked pose at z 321.2 mm.
  - **Surfaces:** markers and the orientation chevron are projected onto DJI's surfaces.
  - **Coil:** the charging coil is tilted to the 6.8° floor and glows through the belly window.
- **Measured from the CAD:** rotor heights (from DJI's swept discs), motor pads, front-LED and battery-LED positions, the fold direction that clears the real fuselage, and callout anchors.
- **Height report.** The aircraft's height is reported against the spec sheet: CAD 186.3 mm vs sheet 212.5 mm (−26.2 mm). Length and width are checked at ±1.5 mm.
- **Fallback.** `--procedural-aircraft` rebuilds without the CAD.
- **Viewer:** the aircraft table names its source and explains the height difference; lens fogging uses every glass surface on DJI's aircraft.

### Changed
- **GLB:** 1.32 MB → 4.53 MB, 39k → 152k triangles. The standalone page is 1.84 → 6.13 MB.
- **Callout occlusion** now tests against the dock only, to stay smooth with the CAD airframe.
- **Charging glow** softened.
- **Timeline labels** re-fit once web fonts load.

### Fixed
- **CAD import** no longer deletes the build's own not-yet-used materials.
- **Size check** skips empty meshes.

## [1.0] - 2026-09-22

### Added
- **Blender build:** `blender/build.py`, a from-scratch rebuild of the full scene from DJI's published documents.
  - **Validation:** ±1 mm checks that fail the build: closed 640.0 × 745.0 × 770.0, open 1760.0 × 745.0 × 485.0, aircraft 377.7 × 416.2 × 212.5 mm.
  - **Aircraft sizing:** the pod radius and gimbal front are solved from the spec's width and length, and the GNSS mast top sets the height.
  - **Cover:** two-radius edges (80 mm along the long top edges, 30 mm elsewhere); status LEDs sit on the leaning upper flank, inside the 640 mm envelope.
  - **Aircraft body:** a lofted superellipse fuselage with rounded motor-pod bottoms.
  - **Materials:** clearcoat on the paint, tray, e-stop, lenses and aircraft shell, exported as `KHR_materials_clearcoat`; a generated hexagonal perforation texture.
  - **Mission:** 32.5 s, with induction charging, battery LEDs (UM p. 23), folding propellers, gas struts and the linkage-offset cover.
  - **Renders:** 10 Eevee stills, including a charging cutaway and a winter low-mount scene.
- **Viewer:** `web/viewer.src.html` on three.js 0.186.
  - **Reflections:** studio softbox environment generated in code (PMREM), a planar mirror floor under a radial fade into the studio backdrop, and a reflective water surface for floods.
  - **Post-processing:** GTAO, bloom and 4× MSAA through EffectComposer; High / Fast quality toggle.
  - **Design:** Archivo + JetBrains Mono, cyan accent, glass HUD panels, a battery gauge following DJI's LED pattern, a labelled phase timeline, a segmented tab bar, and light and dark themes.
  - **Parity with Opus 5 v2:** envelope check, 28 callouts, layers, Conditions (weather, site, faults) with a verdict and holds, 13 issues with deep links, see-through cover while charging, status lights by phase, particles, fog, frost, dew, snow caps, base height, anemometer spin, glowing metal ring, lens fogging.
- **Docs:** `README.md` (with a comparison table against the Opus 5 build), `SESSION_LOG.md`, and this file.
- **Published:** as a new private artifact, separate from the Opus 5 artifact.

---

*Signed: Claude (Claude Code) · model `claude-opus-5-5` (Claude Opus 5.5, 1M context) · 2026-09-22*
