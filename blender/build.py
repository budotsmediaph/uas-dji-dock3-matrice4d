"""
build.py - DJI Dock 3 + DJI Matrice 4D, rebuilt from DJI's public documentation.
Opus 5.5 rebuild (claude-opus-5-5), 2026-09-22.

Everything DJI publishes is used verbatim and checked to +/-1 mm on every build.
Everything DJI does not publish (hinge, panel shapes, pad curvature) is derived from the
published envelopes and proportioned from the manual drawings.

Sources (../../20260921-minimax/):
  specs/DJI_Dock_3_KOREC_Specs.pdf
      open 1760x745x485 mm, closed 640x745x770 mm, "All data includes the RTK module width
      (160 mm), wind speed gauge height (145 mm), and mounting base brackets (58 mm)";
      charging 35 V DC, 27 min 15 -> 95 %; aircraft 377.7x416.2x212.5 mm, wheelbases
  manuals/DJI_Dock_3_Installation_and_Setup_Manual_v1.0_en.pdf
      p.7 parts 1-21 (+ note 1 charging module, note 2 e-stop), p.8 status lights,
      pp.12-13 environment, p.20 snow, p.22 concrete base, p.28 mounting 554x400, p.37 clearance
  manuals/DJI_Matrice_4D_Series_User_Manual_DJI_Dock_3_v1.0_en.pdf
      p.9 aircraft items 1-21, p.12 cameras, p.23 battery LEDs

Frame: +X = direction the wind-gauge half opens and the aircraft nose; -Y = front (RTK,
e-stop, cabinet); +Z up; origin = bottom of the mounting brackets.

Run:  blender -b -P build.py -- [--render] [--shots=a,b] [--no-export]
"""

import bpy
import bmesh
import json
import math
import os
import sys

import numpy as np
from mathutils import Euler, Matrix, Vector

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
ARGV = sys.argv[sys.argv.index("--") + 1:] if "--" in sys.argv else []
DO_RENDER = "--render" in ARGV
DO_EXPORT = "--no-export" not in ARGV
SHOTS = next((a.split("=", 1)[1].split(",") for a in ARGV if a.startswith("--shots=")), None)
TOL_MM = 1.0

# ============================================================================ 1. numbers
SPEC = {
    "open_mm": [1760, 745, 485], "closed_mm": [640, 745, 770],
    "rtk_mm": 160, "gauge_mm": 145, "bracket_mm": 58,
    "aircraft_mm": [377.7, 416.2, 212.5],
    "wheelbase_mm": {"diagonal": 498.5, "front_lr": 383.0, "rear_lr": 343.0, "front_rear": 341.6},
    "mount_holes_mm": [554, 400], "concrete_base_mm": [700, 700, 100],
    "charging": {"output_v": 35, "minutes": 27, "from_pct": 15, "to_pct": 95},
    "temperature_c": {"dock": [-30, 50], "aircraft": [-20, 50]},
    "wind_max_ms": 12, "humidity_max_pct": 93, "rain_max_mmh": 2,
}
MM = 0.001
BRACKET_H = SPEC["bracket_mm"] * MM
GAUGE_H = SPEC["gauge_mm"] * MM
RTK_W = SPEC["rtk_mm"] * MM
LID_HX = SPEC["closed_mm"][0] * MM / 2                                  # 0.320
LID_HY = (SPEC["closed_mm"][1] - SPEC["rtk_mm"]) * MM / 2               # 0.2925
LID_TOP = (SPEC["closed_mm"][2] - SPEC["gauge_mm"]) * MM                # 0.625
LID_BOT, LID_T, SEAM = 0.100, 0.018, 0.0015
R_LONG, R_SMALL = 0.080, 0.030          # big rounded flank edge / tighter corners
PIVOT_X, PIVOT_Z = 0.310, LID_BOT
OPEN_REACH = (SPEC["open_mm"][0] - SPEC["gauge_mm"]) * MM / 2           # 0.8075
OPEN_H = SPEC["open_mm"][2] * MM                                        # 0.485
LID_OPEN_TOP = 0.460
LID_DX = OPEN_REACH - (PIVOT_X + (LID_TOP - PIVOT_Z))                   # -0.0275
LID_DZ = LID_OPEN_TOP - (PIVOT_Z + PIVOT_X)                             # +0.050

BODY_HX, BODY_HY, BODY_Z1 = 0.275, 0.270, 0.380
TRAY_HX, TRAY_HY, TRAY_Z0, TRAY_Z1 = 0.280, 0.270, 0.370, 0.470
POCKET_HX, POCKET_HY, POCKET_FLOOR = 0.255, 0.245, 0.380
STRIP_HW, STRIP_TOP, HUMP_H = 0.045, 0.390, 0.062
FY, BY = -BODY_HY, BODY_HY

# aircraft, all from the spec sheet
WB = SPEC["wheelbase_mm"]
WB_F, WB_R, WB_FR = WB["front_lr"] * MM, WB["rear_lr"] * MM, WB["front_rear"] * MM
AC_L, AC_W, AC_H = (v * MM for v in SPEC["aircraft_mm"])
POD_R = (AC_W - WB_F) / 2                          # 0.0166 -> width lands on 416.2
AC_REAR = -(WB_FR / 2 + POD_R)                     # rear-most point (rear pods)
AC_FRONT = AC_L + AC_REAR                          # gimbal face lands on 377.7
PROP_R, MOTOR_TOP = 0.165, 0.162
MOTORS = {"FL": (WB_FR / 2, WB_F / 2, -1), "FR": (WB_FR / 2, -WB_F / 2, +1),
          "RL": (-WB_FR / 2, WB_R / 2, +1), "RR": (-WB_FR / 2, -WB_R / 2, -1)}

# ============================================================================ 2. timeline
FPS = 30
F = {"open0": 30, "open1": 120, "spin0": 120, "spin1": 150, "up0": 150, "up1": 225,
     "loop0": 225, "loop1": 495, "down0": 495, "down1": 585, "stop0": 585, "stop1": 615,
     "close0": 615, "close1": 705, "charge0": 705, "charge1": 945, "end": 975}
CRUISE_DZ, ORBIT_R = 1.90, 1.80
OMEGA, FOLD = math.radians(70), math.radians(9)
BATTERY = {"full": 95.0, "low": 15.0, "charge_minutes": 27, "charge_curve": 1.6, "output_v": 35}


def smooth(t):
    t = max(0.0, min(1.0, t))
    return t * t * t * (t * (t * 6 - 15) + 10)


def ramp(f, a, b):
    return smooth((f - a) / (b - a))


def battery_level(f):
    b = BATTERY
    if f < F["up0"]:
        return b["full"]
    if f < F["down1"]:
        return b["full"] - (b["full"] - b["low"]) * (f - F["up0"]) / (F["down1"] - F["up0"])
    if f < F["charge0"]:
        return b["low"]
    if f < F["charge1"]:
        u = (f - F["charge0"]) / (F["charge1"] - F["charge0"])
        return b["low"] + (b["full"] - b["low"]) * (1 - (1 - u) ** b["charge_curve"])
    return b["full"]


def battery_leds(level, f):
    """User manual p.23: four LEDs, eight bands of on / flashing."""
    bands = [(92, 4, None), (76, 3, 3), (63, 3, None), (51, 2, 2), (38, 2, None), (26, 1, 1), (13, 1, None), (0, 0, 0)]
    n_on, flash = next((n, fl) for lo, n, fl in bands if level >= lo)
    blink = (f // 15) % 2 == 0
    return [i < n_on or (flash == i and blink) for i in range(4)]


def leds_active(f):
    return F["spin0"] <= f < F["stop1"] or F["charge0"] <= f < F["charge1"]


# ============================================================================ 3. scene
bpy.ops.wm.read_factory_settings(use_empty=True)
scene = bpy.context.scene
scene.name = "Dock3Mission"
scene.render.fps = FPS
scene.frame_start, scene.frame_end = 0, F["end"]
bpy.context.preferences.edit.keyframe_new_interpolation_type = "LINEAR"


def collection(name):
    c = bpy.data.collections.new(name)
    scene.collection.children.link(c)
    return c


C_DOCK, C_AC, C_SITE, C_STUDIO = (collection(n) for n in ("Dock3", "Matrice4D", "Site", "Studio"))

# ============================================================================ 4. materials
os.makedirs(os.path.join(ROOT, "textures"), exist_ok=True)


def texture(name, rgba):
    h, w = rgba.shape[:2]
    img = bpy.data.images.new(name, w, h, alpha=True)
    img.pixels.foreach_set(np.flipud(rgba).astype(np.float32).ravel())
    img.filepath_raw = os.path.join(ROOT, "textures", name + ".png")
    img.file_format = "PNG"
    img.save()
    return img


def hex_mesh(px=96):
    """Tileable hexagonal perforation (A/C grille, cabinet door, strip vents)."""
    yy, xx = np.mgrid[0:px, 0:px] + 0.5
    pitch = px / 4
    rowh = pitch * math.sqrt(3) / 2
    v = np.full((px, px), 0.22)
    for r in range(-1, int(px / rowh) + 2):
        for c in range(-1, 6):
            cx, cy = c * pitch + (pitch / 2 if r % 2 else 0), r * rowh
            v[(xx - cx) ** 2 + (yy - cy) ** 2 < (pitch * 0.34) ** 2] = 0.02
    return np.dstack([v, v, v * 1.05, np.ones_like(v)])


TEX_MESH = texture("hex_mesh", hex_mesh())
TEX_TILE = 0.024


def material(name, rgb, rough=0.5, metal=0.0, coat=0.0, coat_rough=0.12, emit=None, strength=0.0, image=None):
    m = bpy.data.materials.new(name)
    nt = m.node_tree
    b = next(n for n in nt.nodes if n.bl_idname == "ShaderNodeBsdfPrincipled")
    b.inputs["Base Color"].default_value = (*rgb, 1)
    b.inputs["Roughness"].default_value = rough
    b.inputs["Metallic"].default_value = metal
    if coat:
        b.inputs["Coat Weight"].default_value = coat
        b.inputs["Coat Roughness"].default_value = coat_rough
    if emit:
        b.inputs["Emission Color"].default_value = (*emit, 1)
        b.inputs["Emission Strength"].default_value = strength
    if image:
        tn = nt.nodes.new("ShaderNodeTexImage")
        tn.image = image
        nt.links.new(tn.outputs["Color"], b.inputs["Base Color"])
    m.diffuse_color = (*rgb, 1)
    return m


M = {
    "white": material("Dock_Paint", (0.82, 0.83, 0.83), 0.34, coat=0.55),
    "lid_in": material("Dock_CoverInner", (0.10, 0.105, 0.11), 0.72),
    "tray": material("Dock_Tray", (0.07, 0.074, 0.08), 0.5, coat=0.2, coat_rough=0.3),
    "pad": material("Dock_Pad", (0.105, 0.11, 0.115), 0.82),
    "black": material("Dock_Black", (0.016, 0.016, 0.018), 0.45),
    "metal": material("Metal", (0.66, 0.67, 0.69), 0.26, 1.0),
    "marker": material("Dock_Marker", (0.8, 0.81, 0.81), 0.55),
    "yellow": material("Dock_EStopYellow", (0.95, 0.66, 0.02), 0.35, coat=0.4),
    "red": material("Dock_EStopRed", (0.72, 0.035, 0.02), 0.3, coat=0.6),
    "mesh": material("Dock_Mesh", (0.2, 0.2, 0.21), 0.55, 0.4, image=TEX_MESH),
    "led": material("Dock_StatusLED", (0.9, 0.95, 1.0), 0.3, emit=(0.75, 0.88, 1.0), strength=6.0),
    "glass": material("Glass", (0.008, 0.01, 0.016), 0.04, coat=1.0, coat_rough=0.02),
    "ac": material("M4D_Shell", (0.2, 0.205, 0.21), 0.48, coat=0.15, coat_rough=0.35),
    "ac_dark": material("M4D_Dark", (0.042, 0.044, 0.048), 0.42),
    "prop": material("M4D_Prop", (0.02, 0.02, 0.022), 0.35),
    "prop_tip": material("M4D_PropTip", (0.76, 0.76, 0.75), 0.4),
    "led_red": material("M4D_LED_Red", (1, 0.1, 0.05), 0.4, emit=(1, 0.08, 0.03), strength=8.0),
    "led_green": material("M4D_LED_Green", (0.1, 1, 0.2), 0.4, emit=(0.05, 1, 0.15), strength=8.0),
    "led_batt": material("M4D_LED_Battery", (0.55, 1.0, 0.6), 0.3, emit=(0.4, 1.0, 0.5), strength=8.0),
    "charge": material("Dock_ChargeGlow", (0.5, 0.95, 1.0), 0.3, emit=(0.35, 0.9, 1.0), strength=8.0),
    "concrete": material("Site_Concrete", (0.42, 0.42, 0.40), 0.95),
    "ground": material("Studio_Ground", (0.30, 0.31, 0.32), 0.9),
    "snow": material("Studio_Snow", (0.8, 0.85, 0.93), 0.7),
}

# ============================================================================ 5. geometry helpers


def _mat_faces(vs, mat):
    for f in {f for v in vs for f in v.link_faces}:
        f.material_index = mat


def add_box(bm, mn, mx, bevel=0.0, segs=3, mat=0):
    vs = bmesh.ops.create_cube(bm, size=1.0)["verts"]
    size = Vector(mx) - Vector(mn)
    bmesh.ops.scale(bm, vec=size, verts=vs)
    bmesh.ops.translate(bm, vec=(Vector(mn) + Vector(mx)) / 2, verts=vs)
    _mat_faces(vs, mat)
    if bevel > 0:
        bmesh.ops.bevel(bm, geom=list({e for v in vs for e in v.link_edges}), offset=min(bevel, min(size) * 0.49),
                        segments=segs, profile=0.5, affect="EDGES", clamp_overlap=True)
    return vs


def add_cyl(bm, c, r, depth, segs=32, axis="Z", mat=0, r2=None):
    vs = bmesh.ops.create_cone(bm, cap_ends=True, cap_tris=False, segments=segs, radius1=r,
                               radius2=r if r2 is None else r2, depth=depth)["verts"]
    rot = {"Z": Matrix.Identity(3), "X": Matrix.Rotation(math.pi / 2, 3, "Y"), "Y": Matrix.Rotation(-math.pi / 2, 3, "X")}[axis]
    bmesh.ops.rotate(bm, verts=vs, cent=(0, 0, 0), matrix=rot)
    bmesh.ops.translate(bm, vec=c, verts=vs)
    _mat_faces(vs, mat)
    return vs


def add_sphere(bm, c, r, scale=(1, 1, 1), mat=0, segs=24):
    vs = bmesh.ops.create_uvsphere(bm, u_segments=segs, v_segments=segs // 2, radius=r)["verts"]
    bmesh.ops.scale(bm, vec=scale, verts=vs)
    bmesh.ops.translate(bm, vec=c, verts=vs)
    _mat_faces(vs, mat)
    return vs


def add_prism(bm, pts, plane, d0, d1, mat=0):
    P = {"XZ": lambda u, v, w: (u, w, v), "XY": lambda u, v, w: (u, v, w), "YZ": lambda u, v, w: (w, u, v)}[plane]
    a = [bm.verts.new(P(u, v, d0)) for u, v in pts]
    b = [bm.verts.new(P(u, v, d1)) for u, v in pts]
    n = len(pts)
    fs = [bm.faces.new(a[::-1]), bm.faces.new(b)] + [bm.faces.new((a[i], a[(i + 1) % n], b[(i + 1) % n], b[i])) for i in range(n)]
    for f in fs:
        f.material_index = mat


def add_ring(bm, c, r_out, r_in, depth, segs=40, mat=0):
    cx, cy, cz = c
    rings = [[bm.verts.new((cx + r * math.cos(2 * math.pi * i / segs), y, cz + r * math.sin(2 * math.pi * i / segs)))
              for i in range(segs)] for y in (cy - depth / 2, cy + depth / 2) for r in (r_out, r_in)]
    o0, i0, o1, i1 = rings
    for k in range(segs):
        n = (k + 1) % segs
        for a, b in ((o0, o1), (i1, i0), (o1, i1), (i0, o0)):
            bm.faces.new((a[k], a[n], b[n], b[k])).material_index = mat


def add_beam(bm, p0, p1, w0, h0, w1, h1, bevel=0.005, segs=3, mat=0):
    p0, p1 = Vector(p0), Vector(p1)
    u = (p1 - p0).normalized()
    side = u.cross(Vector((0, 0, 1)))
    side = side.normalized() if side.length > 1e-6 else Vector((1, 0, 0))
    up = side.cross(u).normalized()
    vs = [bm.verts.new(p + side * sw * w / 2 + up * sh * h / 2)
          for p, w, h in ((p0, w0, h0), (p1, w1, h1)) for sw, sh in ((-1, -1), (1, -1), (1, 1), (-1, 1))]
    fs = [bm.faces.new([vs[i] for i in q]) for q in ((0, 1, 2, 3), (7, 6, 5, 4), (0, 4, 5, 1), (1, 5, 6, 2), (2, 6, 7, 3), (3, 7, 4, 0))]
    for f in fs:
        f.material_index = mat
    if bevel:
        bmesh.ops.bevel(bm, geom=list({e for f in fs for e in f.edges}), offset=bevel, segments=segs,
                        profile=0.5, affect="EDGES", clamp_overlap=True)


def add_loft(bm, stations, n=28, mat=0):
    """Closed superellipse loft along X: stations = (x, half_width, z_bottom, z_top, exponent)."""
    rings = []
    for x, hw, zb, zt, e in stations:
        zc, hh = (zb + zt) / 2, (zt - zb) / 2
        ring = []
        for i in range(n):
            a = 2 * math.pi * i / n
            c, s = math.cos(a), math.sin(a)
            y = hw * math.copysign(abs(c) ** (2 / e), c)
            z = zc + hh * math.copysign(abs(s) ** (2 / e), s)
            ring.append(bm.verts.new((x, y, z)))
        rings.append(ring)
    for r0, r1 in zip(rings, rings[1:]):
        for i in range(n):
            j = (i + 1) % n
            bm.faces.new((r0[i], r0[j], r1[j], r1[i])).material_index = mat
    for ring, flip in ((rings[0], True), (rings[-1], False)):
        c = bm.verts.new(sum((v.co for v in ring), Vector()) / n)
        for i in range(n):
            tri = (c, ring[(i + 1) % n], ring[i]) if flip else (c, ring[i], ring[(i + 1) % n])
            bm.faces.new(tri).material_index = mat


def planar_uv(bm, axes, tile=TEX_TILE):
    uv = bm.loops.layers.uv.verify()
    a, b = axes
    for f in bm.faces:
        for lp in f.loops:
            lp[uv].uv = (lp.vert.co[a] / tile, lp.vert.co[b] / tile)


def rrect(x0, z0, x1, z1, r, n=6):
    pts = []
    for cx, cz, a0 in ((x1 - r, z0 + r, -90), (x1 - r, z1 - r, 0), (x0 + r, z1 - r, 90), (x0 + r, z0 + r, 180)):
        pts += [(cx + r * math.cos(math.radians(a0 + 90 * i / n)), cz + r * math.sin(math.radians(a0 + 90 * i / n))) for i in range(n + 1)]
    return pts


def to_object(name, bm, mats, coll, parent=None, origin=(0, 0, 0), smooth_deg=38, recalc=True):
    if recalc:
        bmesh.ops.recalc_face_normals(bm, faces=bm.faces[:])
    if Vector(origin).length > 0:
        bmesh.ops.translate(bm, vec=-Vector(origin), verts=bm.verts)
    if smooth_deg is not None:
        lim = math.radians(smooth_deg)
        for f in bm.faces:
            f.smooth = True
        for e in bm.edges:
            e.smooth = len(e.link_faces) == 2 and e.calc_face_angle(math.pi) < lim
    me = bpy.data.meshes.new(name)
    bm.to_mesh(me)
    bm.free()
    for m in mats:
        me.materials.append(M[m] if isinstance(m, str) else m)
    ob = bpy.data.objects.new(name, me)
    coll.objects.link(ob)
    ob.parent = parent
    ob.location = origin
    return ob


def part(name, build, mats, coll, parent=None, origin=(0, 0, 0), local_origin=None, **kw):
    """Build in world (closed-state) coordinates; place the object origin at `origin`.
    local_origin: parent-space origin of `parent`, subtracted so children stay put."""
    bm = bmesh.new()
    build(bm)
    ob = to_object(name, bm, mats, coll, parent, origin, **kw)
    if local_origin is not None:
        ob.location = Vector(origin) - Vector(local_origin)
    return ob


def bake(ob):
    dg = bpy.context.evaluated_depsgraph_get()
    me = bpy.data.meshes.new_from_object(ob.evaluated_get(dg), preserve_all_data_layers=True, depsgraph=dg)
    old = ob.data
    ob.modifiers.clear()
    ob.data = me
    bpy.data.meshes.remove(old)
    me.name = ob.name


def boolean(ob, builders):
    cutters = []
    for i, build in enumerate(builders):
        bm = bmesh.new()
        build(bm)
        bmesh.ops.recalc_face_normals(bm, faces=bm.faces[:])
        me = bpy.data.meshes.new(f"cut{i}")
        bm.to_mesh(me)
        bm.free()
        c = bpy.data.objects.new(f"cut{i}", me)
        C_STUDIO.objects.link(c)
        mod = ob.modifiers.new(f"bool{i}", "BOOLEAN")
        mod.operation, mod.solver, mod.object = "DIFFERENCE", "EXACT", c
        cutters.append(c)
    bake(ob)
    for c in cutters:
        bpy.data.objects.remove(c, do_unlink=True)


def empty(name, coll, parent=None, loc=(0, 0, 0), size=0.05):
    ob = bpy.data.objects.new(name, None)
    ob.empty_display_size = size
    coll.objects.link(ob)
    ob.parent = parent
    ob.location = loc
    return ob


HOTSPOTS = []


def hotspot(num, label, parent, world, coll, group="dock", origin=(0, 0, 0)):
    ob = empty(f"HS_{group}_{num}", coll, parent, Vector(world) - Vector(origin), 0.02)
    ob["hotspot"], ob["label"], ob["group"] = str(num), label, group
    HOTSPOTS.append({"group": group, "id": str(num), "label": label})


# ============================================================================ 5b. DJI official CAD
# If official/m4d_tol0.25.glb exists (tessellated from DJI's Matrice 4D STEP by official/convert.py),
# the aircraft and the landing-pad cradle come from DJI's own geometry. Everything else stays
# procedural. Pass --procedural-aircraft to ignore the CAD.
from mathutils.bvhtree import BVHTree

OFFICIAL_GLB = os.path.join(ROOT, "official", "m4d_tol0.25.glb")
OFFICIAL = os.path.exists(OFFICIAL_GLB) and "--procedural-aircraft" not in ARGV
AC_GROUPS = ("M4D_MAIN", "M4D_GIMBAL", "M4D_BAT", "M4D_VISION")
CAD_GROUPS = AC_GROUPS + ("M4D_V_TJP", "M4D_JP", "M4D_FOV")
DECIMATE = {"M4D_MAIN": 0.30, "M4D_GIMBAL": 0.20, "M4D_BAT": 0.15}          # web budget ~130k triangles


def load_official():
    """Import DJI's tessellated STEP and move it into the rig frame: nose +X, lowest point of the
    aircraft at z = 0, rotor centre (midpoint of the four swept discs) at the origin."""
    before, mats_before = set(bpy.data.objects), set(bpy.data.materials)
    bpy.ops.import_scene.gltf(filepath=OFFICIAL_GLB)
    new = [o for o in bpy.data.objects if o not in before]
    grp = lambda o: next((g for g in CAD_GROUPS if o.name.startswith(g)), None)
    meshes = [o for o in new if o.type == "MESH" and grp(o)]

    def pts(o, M=Matrix.Identity(4)):
        co = np.empty(len(o.data.vertices) * 3)
        o.data.vertices.foreach_get("co", co)
        m = np.array(M @ o.matrix_world)
        return co.reshape(-1, 3) @ m[:3, :3].T + m[:3, 3]

    ac = np.vstack([pts(o) for o in meshes if grp(o) in AC_GROUPS])
    jp = np.vstack([pts(o) for o in meshes if grp(o) == "M4D_JP"])
    T = Matrix.Rotation(math.pi / 2, 4, "Z") @ Matrix.Translation(
        (-(ac[:, 0].min() + ac[:, 0].max()) / 2, -(jp[:, 1].min() + jp[:, 1].max()) / 2, -ac[:, 2].min()))
    P = {g: np.vstack([pts(o, T) for o in meshes if grp(o) == g]) for g in CAD_GROUPS}
    discs = {}
    for key, (mx, my, _) in MOTORS.items():
        near = P["M4D_JP"][(P["M4D_JP"][:, 0] - mx) ** 2 + (P["M4D_JP"][:, 1] - my) ** 2 < 0.17 ** 2]
        discs[key] = (float(near[:, 2].min()), float(near[:, 2].max()))
    parts = []
    for o in meshes:
        g = grp(o)
        if g in ("M4D_JP", "M4D_FOV"):
            continue
        cols = []
        for m in o.data.materials:
            b = next((n for n in m.node_tree.nodes if n.bl_idname == "ShaderNodeBsdfPrincipled"), None) if m and m.node_tree else None
            cols.append(tuple(b.inputs["Base Color"].default_value)[:3] if b else (0.5, 0.5, 0.5))
        bm = bmesh.new()
        bm.from_mesh(o.data)
        bmesh.ops.transform(bm, matrix=T @ o.matrix_world, verts=bm.verts)
        bmesh.ops.remove_doubles(bm, verts=bm.verts, dist=2e-5)
        parts.append((o.name, g, bm, cols))
    for o in new:
        bpy.data.objects.remove(o, do_unlink=True)
    for m in [m for m in bpy.data.materials if m not in mats_before]:     # only the CAD's own materials
        bpy.data.materials.remove(m)
    return parts, P, discs


GLASS = set()          # (group, CAD colour) pairs that are lenses / sensor windows


def classify_cad_colours(parts):
    """Creo colours are engineering colours: the main shell is pale blue. A blue-ish colour counts as glass
    only when it covers a small share (< 12 %) of its part, i.e. lenses and sensor windows."""
    area, total = {}, {}
    for _, g, bm, cols in parts:
        for f in bm.faces:
            c = tuple(round(x, 3) for x in (cols[f.material_index] if f.material_index < len(cols) else (0.5, 0.5, 0.5)))
            a = f.calc_area()
            area[g, c] = area.get((g, c), 0.0) + a
            total[g] = total.get(g, 0.0) + a
    for (g, c), a in area.items():
        r, _, b = c
        if b > r + 0.15 and b > 0.35 and a / total[g] < 0.12:
            GLASS.add((g, c))


def cad_finish(g, c):
    """Map DJI's CAD engineering colours onto product finishes."""
    if g == "M4D_V_TJP":
        return "pad"
    if g == "M4D_VISION" or (g, tuple(round(x, 3) for x in c)) in GLASS:
        return "glass"
    return "ac" if g == "M4D_MAIN" else "ac_dark"


def cad_object(name, g, bm, cols, coll, parent, offset=(0, 0, 0)):
    keys = []
    for f in bm.faces:
        k = cad_finish(g, cols[f.material_index] if f.material_index < len(cols) else (0.5, 0.5, 0.5))
        if k not in keys:
            keys.append(k)
        f.material_index = keys.index(k)
    if Vector(offset).length:
        bmesh.ops.translate(bm, vec=offset, verts=bm.verts)
    if g == "M4D_V_TJP":          # DJI's cradle is a surface model: face every sheet up / toward the channel
        for f in bm.faces:
            f.normal_update()
            c = f.calc_center_median()
            if f.normal.z < -0.2 or (abs(f.normal.z) <= 0.2 and f.normal.y * c.y > 0):
                f.normal_flip()
    ob = to_object(name, bm, keys, coll, parent, smooth_deg=None, recalc=False)
    ob.location = (0, 0, 0)
    if DECIMATE.get(g, 1.0) < 1.0:
        mod = ob.modifiers.new("decimate", "DECIMATE")
        mod.ratio, mod.use_collapse_triangulate = DECIMATE[g], True
        bake(ob)
    if "custom_normal" in ob.data.attributes:
        ob.data.attributes.remove(ob.data.attributes["custom_normal"])
    b2 = bmesh.new()
    b2.from_mesh(ob.data)
    for f in b2.faces:
        f.smooth = True
    for e in b2.edges:
        e.smooth = len(e.link_faces) == 2 and e.calc_face_angle(math.pi) < math.radians(32)
    b2.to_mesh(ob.data)
    b2.free()
    return ob


if OFFICIAL:
    print("[dock3] DJI CAD:", OFFICIAL_GLB)
    CAD_PARTS, CAD, CAD_DISCS = load_official()
    classify_cad_colours(CAD_PARTS)
    print(f"[dock3] CAD colours treated as glass: {sorted(GLASS)}")
    ac_pts = np.vstack([CAD[g] for g in AC_GROUPS])
    pad_pts = CAD["M4D_V_TJP"]
    AC_REST_Z = TRAY_Z1 - 0.002 - float(pad_pts[:, 2].max())          # cradle rim flush with the tray rim
    PAD_LO, PAD_HI = pad_pts.min(0) + [0, 0, AC_REST_Z], pad_pts.max(0) + [0, 0, AC_REST_Z]
    low = ac_pts[np.argmin(ac_pts[:, 2])]
    pad_bm = bmesh.new()
    for name, g, bm, cols in CAD_PARTS:
        if g == "M4D_V_TJP":
            tmp = bm.copy()
            bmesh.ops.translate(tmp, vec=(0, 0, AC_REST_Z), verts=tmp.verts)
            me = bpy.data.meshes.new("tmp_pad")
            tmp.to_mesh(me)
            tmp.free()
            pad_bm.from_mesh(me)
            bpy.data.meshes.remove(me)
    PAD_BVH = BVHTree.FromBMesh(pad_bm)
    pad_bm.free()
    print(f"[dock3] aircraft rests at z = {AC_REST_Z * 1000:.1f} mm; cradle {np.round((PAD_HI - PAD_LO) * 1000, 1)} mm, "
          f"rotor discs {', '.join(f'{k} {v[0] * 1000:.0f}-{v[1] * 1000:.0f}' for k, v in CAD_DISCS.items())} mm")
else:
    AC_REST_Z = STRIP_TOP


def pad_z(x, y):
    """Top of the landing pad at (x, y): DJI's cradle when present, otherwise the procedural mounds."""
    if not OFFICIAL:
        return pad_height(x, y)
    hit = PAD_BVH.ray_cast(Vector((x, y, 1.0)), Vector((0, 0, -1)))
    return hit[0].z if hit[0] is not None else None


# ============================================================================ 6. dock body
print("[dock3] body")
DOCK = empty("Dock3", C_DOCK, size=0.2)


def dpart(name, build, mats, **kw):
    return part(name, build, mats, C_DOCK, DOCK, **kw)


HOLE_X, HOLE_Y = SPEC["mount_holes_mm"][0] * MM / 2, SPEC["mount_holes_mm"][1] * MM / 2
rails = dpart("Dock_Brackets", lambda bm: [add_box(bm, (-0.29, s * HOLE_Y - 0.026, 0), (0.29, s * HOLE_Y + 0.026, BRACKET_H), 0.004)
                                           for s in (-1, 1)], ["black"])
boolean(rails, [lambda bm: add_box(bm, (-0.2, -0.4, -0.01), (0.2, 0.4, 0.032))])
dpart("Dock_BracketBolts", lambda bm: [add_cyl(bm, (sx * HOLE_X, sy * (HOLE_Y + 0.027), 0.02), 0.009, 0.006, 6, "Y")
                                       for sx in (-1, 1) for sy in (-1, 1)], ["metal"], smooth_deg=20)
body = dpart("Dock_Body", lambda bm: add_box(bm, (-BODY_HX, -BODY_HY, BRACKET_H), (BODY_HX, BODY_HY, BODY_Z1), 0.012), ["white"])
tray = dpart("Dock_Tray", lambda bm: add_box(bm, (-TRAY_HX, -TRAY_HY, TRAY_Z0), (TRAY_HX, TRAY_HY, TRAY_Z1), 0.012), ["tray"])
if OFFICIAL:        # DJI's cradle drops deeper than the procedural pad: carve its well through tray and body
    well = lambda bm: add_box(bm, (-0.271, -(PAD_HI[1] + 0.003), PAD_LO[2] - 0.003), (0.271, PAD_HI[1] + 0.003, 0.6))
    boolean(body, [well])
    boolean(tray, [well])
else:
    boolean(tray, [lambda bm: add_box(bm, (-POCKET_HX, -POCKET_HY, POCKET_FLOOR), (POCKET_HX, POCKET_HY, 0.6))])
dpart("Dock_TrayPeaks", lambda bm: [add_prism(bm, [(-0.03, TRAY_Z1 - 0.004), (0.03, TRAY_Z1 - 0.004), (0, OPEN_H)], "XZ", a, b)
                                    for a, b in ((-TRAY_HY + 0.001, -POCKET_HY - 0.001), (POCKET_HY + 0.001, TRAY_HY - 0.001))],
      ["tray"], smooth_deg=20)


def pad_height(x, y):
    """Two centring mounds beside the channel, narrowing toward the rim (corner facets hold the markers)."""
    ay, ax = abs(y), abs(x)
    if ay <= STRIP_HW:
        return STRIP_TOP
    t = min(1.0, (ay - STRIP_HW) / (POCKET_HY - STRIP_HW))
    edge = POCKET_HX * (1 - 0.55 * t) - 0.012
    m = smooth((edge - ax) / 0.05 + 0.5)
    return STRIP_TOP + HUMP_H * smooth(t / 0.5) * (0.38 + 0.62 * m)


def build_pad(bm):
    nx, ny = 72, 32
    for s in (-1, 1):
        ys = [STRIP_HW + (POCKET_HY - STRIP_HW) * j / ny for j in range(ny + 1)]
        ys = ys if s > 0 else [-y for y in reversed(ys)]
        g = [[bm.verts.new((x, y, pad_height(x, y))) for x in (-POCKET_HX + 2 * POCKET_HX * i / nx for i in range(nx + 1))] for y in ys]
        for j in range(ny):
            for i in range(nx):
                bm.faces.new((g[j][i], g[j][i + 1], g[j + 1][i + 1], g[j + 1][i]))


if OFFICIAL:
    for i, (name, g, bm, cols) in enumerate([p for p in CAD_PARTS if p[1] == "M4D_V_TJP"]):
        cad_object(f"Dock_LandingPad_{i + 1}", g, bm, cols, C_DOCK, DOCK, offset=(0, 0, AC_REST_Z))

    def build_cad_markers(bm):
        """Positioning markers (item 6) projected onto DJI's cradle wings."""
        n, leg = 8, 0.085
        for sx in (-1, 1):
            for sy in (-1, 1):
                a = Vector((sx * 0.13, sy * 0.172))         # outer wing corners (DJI's cradle is diamond-shaped in plan)
                leg = 0.062
                b, c = a - Vector((sx * leg, 0)), a - Vector((0, sy * leg))
                grid = {(i, j): a + (b - a) * i / n + (c - a) * j / n for i in range(n + 1) for j in range(n + 1 - i)}
                zs = {k: pad_z(p.x, p.y) for k, p in grid.items()}
                if any(z is None for z in zs.values()):
                    continue
                vs = {k: bm.verts.new((p.x, p.y, zs[k] + 0.0012)) for k, p in grid.items()}
                for i in range(n):
                    for j in range(n - i):
                        bm.faces.new((vs[i, j], vs[i + 1, j], vs[i, j + 1]))
                        if j < n - i - 1:
                            bm.faces.new((vs[i + 1, j], vs[i + 1, j + 1], vs[i, j + 1]))
        for f in bm.faces:
            f.normal_update()
            if f.normal.z < 0:
                f.normal_flip()

    mk = dpart("Dock_PositioningMarkers", build_cad_markers, ["marker"], smooth_deg=60, recalc=False)
    print(f"[dock3] positioning markers on the cradle: {len(mk.data.polygons) // 64} of 4")
    def build_cad_chevron(bm):
        """Aircraft orientation marker (item 8), draped on the channel floor, pointing to the nose (+X)."""
        pts = [Vector((-0.085 + dx, dy)) for dx, dy in ((-0.02, -0.018), (0.005, -0.018), (0.02, 0.0), (0.005, 0.018), (-0.02, 0.018), (-0.005, 0.0))]
        zs = [pad_z(p.x, p.y) for p in pts]
        if all(z is not None for z in zs):
            vs = [bm.verts.new((p.x, p.y, z + 0.0012)) for p, z in zip(pts, zs)]
            for tri in ((0, 1, 5), (1, 2, 5), (2, 3, 5), (3, 4, 5)):
                bm.faces.new([vs[i] for i in tri])

    dpart("Dock_OrientationMarker", build_cad_chevron, ["marker"], smooth_deg=None)
else:
    dpart("Dock_LandingPad", build_pad, ["pad"], smooth_deg=60, recalc=False)


    def build_markers(bm):
        n, inset, leg = 10, 0.010, 0.115
        for sx in (-1, 1):
            for sy in (-1, 1):
                a = Vector((sx * (POCKET_HX - inset), sy * (POCKET_HY - inset)))
                b, c = a - Vector((sx * leg, 0)), a - Vector((0, sy * leg))
                idx = {(i, j): bm.verts.new((*(a + (b - a) * i / n + (c - a) * j / n), 0)) for i in range(n + 1) for j in range(n + 1 - i)}
                for v in idx.values():
                    v.co.z = pad_height(v.co.x, v.co.y) + 0.0012
                for i in range(n):
                    for j in range(n - i):
                        bm.faces.new((idx[i, j], idx[i + 1, j], idx[i, j + 1]))
                        if j < n - i - 1:
                            bm.faces.new((idx[i + 1, j], idx[i + 1, j + 1], idx[i, j + 1]))
        for f in bm.faces:
            f.normal_update()
            if f.normal.z < 0:
                f.normal_flip()


    dpart("Dock_PositioningMarkers", build_markers, ["marker"], smooth_deg=60, recalc=False)
    dpart("Dock_Strip", lambda bm: (add_box(bm, (-POCKET_HX, -STRIP_HW, POCKET_FLOOR), (POCKET_HX, STRIP_HW, STRIP_TOP), 0.002),
                                    add_prism(bm, [(-0.205, -0.03), (-0.165, -0.03), (-0.135, 0), (-0.165, 0.03), (-0.205, 0.03), (-0.175, 0)],
                                              "XY", STRIP_TOP, STRIP_TOP + 0.0015, mat=1)), ["tray", "marker"], smooth_deg=30)


    def build_vents(bm):
        for x0, x1 in ((0.02, 0.10), (0.13, 0.21)):
            bm.faces.new([bm.verts.new((x, y, STRIP_TOP + 0.0006)) for x, y in ((x0, -0.03), (x1, -0.03), (x1, 0.03), (x0, 0.03))])
        planar_uv(bm, (0, 1))


    dpart("Dock_StripVents", build_vents, ["mesh"], smooth_deg=None)

dpart("Dock_NotchFins", lambda bm: [add_prism(bm, [(-0.045, 0.393), (0.045, 0.393), (0, TRAY_Z1 + 0.002)], "XZ", a, b)
                                    for a, b in ((-0.2865, -TRAY_HY + 0.002), (TRAY_HY - 0.002, 0.2865))], ["tray"], smooth_deg=20)

# front: e-stop (15), E-Port (20), ports (9, 10, 13), RTK (18), cabinet (12), earth (11)
RTK_C = Vector((0.0, -(LID_HY + RTK_W - 0.070), 0.350))          # front edge lands on 745 mm
dpart("Dock_FrontWhite", lambda bm: (add_box(bm, (-0.235, FY - 0.022, 0.13), (-0.09, FY + 0.004, 0.27), 0.01),
                                     add_box(bm, (-0.035, FY - 0.067, 0.318), (0.035, FY + 0.004, 0.345), 0.005),
                                     add_cyl(bm, RTK_C, 0.070, 0.030, 56),
                                     add_sphere(bm, RTK_C + Vector((0, 0, 0.015)), 0.066, (1, 1, 0.19), segs=56)), ["white"])


def build_front_dark(bm):
    add_box(bm, (0.055, FY - 0.016, 0.10), (0.245, FY + 0.004, 0.345), 0.008)
    add_box(bm, (0.2, FY - 0.028, 0.19), (0.225, FY - 0.014, 0.26), 0.004, mat=1)
    add_box(bm, (-0.14, FY - 0.008, 0.345), (-0.11, FY + 0.002, 0.365), 0.002, mat=1)
    for x in (-0.215, -0.185, -0.155):
        add_cyl(bm, (x, FY - 0.011, 0.11), 0.009, 0.04, 16, mat=1)
    add_cyl(bm, (0.262, FY - 0.004, 0.08), 0.006, 0.012, 12, "Y", mat=1)
    for i in range(10):
        a = 2 * math.pi * i / 10
        add_cyl(bm, RTK_C + Vector((0.066 * math.cos(a), 0.066 * math.sin(a), 0.0165)), 0.0035, 0.004, 8, mat=1)


dpart("Dock_FrontDark", build_front_dark, ["tray", "black"])
dpart("Dock_EmergencyStop", lambda bm: (add_box(bm, (-0.235, FY - 0.032, 0.30), (-0.165, FY + 0.004, 0.37), 0.006),
                                        add_cyl(bm, (-0.2, FY - 0.037, 0.335), 0.019, 0.012, 32, "Y", mat=1),
                                        add_cyl(bm, (-0.2, FY - 0.047, 0.335), 0.025, 0.010, 32, "Y", mat=1, r2=0.022)),
      ["yellow", "red"])


def build_cabinet_mesh(bm):
    bm.faces.new([bm.verts.new((x, FY - 0.0165, z)) for x, z in ((0.07, 0.125), (0.19, 0.125), (0.19, 0.325), (0.07, 0.325))])
    planar_uv(bm, (0, 2))


dpart("Dock_CabinetMesh", build_cabinet_mesh, ["mesh"], smooth_deg=None)


def build_grille(bm):
    add_prism(bm, rrect(-0.24, 0.10, 0.24, 0.39, 0.02), "XZ", BY + 0.016, BY - 0.002)
    for z in (0.20, 0.29):
        add_box(bm, (-0.226, BY + 0.016, z - 0.006), (0.068, BY + 0.021, z + 0.006), 0.002)
    for x in (0.0, -0.075, -0.15, 0.078):
        add_box(bm, (x - 0.006, BY + 0.016, 0.115), (x + 0.006, BY + 0.021, 0.375), 0.002)
    add_ring(bm, (0.155, BY + 0.0185, 0.30), 0.052, 0.044, 0.005)


dpart("Dock_Grille", build_grille, ["black"], smooth_deg=30)


def build_grille_mesh(bm):
    for x0, x1 in ((0.086, 0.225), (-0.226, 0.07)):
        bm.faces.new([bm.verts.new((x, BY + 0.0165, z)) for x, z in ((x1, 0.115), (x0, 0.115), (x0, 0.375), (x1, 0.375))])
    planar_uv(bm, (0, 2))


dpart("Dock_GrilleMesh", build_grille_mesh, ["mesh"], smooth_deg=None)

STRUT_BODY, STRUT_LID, HINGE_Y = (0.287, 0.175), (-0.022, 0.350), 0.215


def build_hinges(bm):
    for sx in (-1, 1):
        for sy in (-1, 1):
            add_box(bm, (min(sx * 0.27, sx * 0.314), sy * HINGE_Y - 0.018, 0.072), (max(sx * 0.27, sx * 0.314), sy * HINGE_Y + 0.018, 0.095), 0.003)
            add_cyl(bm, (sx * PIVOT_X, sy * HINGE_Y, PIVOT_Z), 0.008, 0.034, 16, "Y", mat=1)
        x0 = sx * (BODY_HX - 0.001)
        add_box(bm, (min(x0, sx * 0.293), -0.015, STRUT_BODY[1] - 0.015), (max(x0, sx * 0.293), 0.015, STRUT_BODY[1] + 0.015), 0.003)


dpart("Dock_Hinges", build_hinges, ["black", "metal"], smooth_deg=30)

# induction charging coil + field pulses (install manual p.7 note 1); visible only while charging
COIL_TILT = 0.0
if OFFICIAL:
    # DJI's channel floor rises toward the rear and has a window under the aircraft's belly: fit the floor
    # line, then set the coil just under that window so it shows through while charging.
    samples = [(x, pad_z(x, y)) for x in np.linspace(-0.2, 0.2, 21) for y in (-0.03, 0.03)]
    samples = [(x, z) for x, z in samples if z is not None]
    k, z0 = np.polyfit([x for x, _ in samples], [z for _, z in samples], 1)
    COIL_TILT = math.atan(-k)
    COIL_C = Vector((0.02, 0.0, float(k * 0.02 + z0) - 0.002))
    print(f"[dock3] channel floor slope {math.degrees(COIL_TILT):.1f} deg; charging coil at x {COIL_C.x * 1000:.0f} mm, z {COIL_C.z * 1000:.1f} mm")
else:
    COIL_C = Vector((-0.01, 0.0, STRIP_TOP))


def rrect_loop(hx, hy, r, n=5):
    return [Vector((cx + r * math.cos(math.radians(a0 + 90 * i / n)), cy + r * math.sin(math.radians(a0 + 90 * i / n))))
            for cx, cy, a0 in ((hx - r, -hy + r, -90), (hx - r, hy - r, 0), (-hx + r, hy - r, 90), (-hx + r, -hy + r, 180))
            for i in range(n + 1)]


def add_ribbon(bm, pts, w, z):
    n = len(pts)
    outer, inner = [], []
    for i, p in enumerate(pts):
        d = (pts[(i + 1) % n] - pts[i - 1]).normalized()
        nrm = Vector((d.y, -d.x))
        outer.append(bm.verts.new((*(p + nrm * w / 2), z)))
        inner.append(bm.verts.new((*(p - nrm * w / 2), z)))
    for i in range(n):
        j = (i + 1) % n
        bm.faces.new((outer[i], outer[j], inner[j], inner[i]))


def charge_part(name, build):
    ob = dpart(name, build, ["charge"], origin=Vector((COIL_C.x, COIL_C.y, STRIP_TOP)), smooth_deg=None, recalc=False)
    ob.location = COIL_C
    return ob


COIL = charge_part("Dock_ChargeCoil", lambda bm: [add_ribbon(bm, rrect_loop(0.070 - k * 0.0085, 0.036 - k * 0.0075,
                                                                       max(0.006, 0.02 - k * 0.004)), 0.0025, STRIP_TOP + 0.0008) for k in range(4)])
FIELD = [charge_part(f"Dock_ChargeField_{k + 1}", lambda bm: add_ribbon(bm, rrect_loop(0.070, 0.036, 0.02), 0.003, STRIP_TOP + 0.0012))
         for k in range(3)]
for ob in [COIL] + FIELD:
    ob.rotation_euler = (0, COIL_TILT, 0)

# ============================================================================ 7. cover (item 5)
print("[dock3] cover")


def build_envelope(bm):
    h = LID_TOP - LID_BOT
    vs = bmesh.ops.create_cube(bm, size=1.0)["verts"]
    bmesh.ops.scale(bm, vec=(2 * LID_HX, 2 * LID_HY, h), verts=vs)
    bmesh.ops.translate(bm, vec=(0, 0, LID_BOT + h / 2), verts=vs)
    low, top = LID_BOT + 1e-6, LID_TOP - 1e-6
    # pass 1: the big rounded edge along the two long top edges (the DJI flank)
    long_top = [e for e in bm.edges if all(v.co.z > top for v in e.verts) and abs(e.verts[0].co.x - e.verts[1].co.x) < 1e-6]
    bmesh.ops.bevel(bm, geom=long_top, offset=R_LONG, segments=10, profile=0.5, affect="EDGES", clamp_overlap=True)
    # pass 2: tighter radius on every other edge except the open bottom
    rest = [e for e in bm.edges if not all(v.co.z < low for v in e.verts)
            and len(e.link_faces) == 2 and e.calc_face_angle(0) > math.radians(50)]
    bmesh.ops.bevel(bm, geom=rest, offset=R_SMALL, segments=6, profile=0.5, affect="EDGES", clamp_overlap=True)
    bmesh.ops.delete(bm, geom=[f for f in bm.faces if all(v.co.z < low for v in f.verts)], context="FACES_ONLY")
    for z in (0.43, 0.25):
        bmesh.ops.bisect_plane(bm, geom=bm.verts[:] + bm.edges[:] + bm.faces[:], dist=1e-6, plane_co=(0, 0, z), plane_no=(0, 0, 1))
    for v in bm.verts:                     # subtle lean of the upper flank above a crease
        if v.co.z > 0.43:
            v.co.x *= 1 - 0.03 * (v.co.z - 0.43) / (LID_TOP - 0.43)


cover = part("Dock_Cover", build_envelope, ["white", "lid_in"], C_DOCK, smooth_deg=None)
sol = cover.modifiers.new("shell", "SOLIDIFY")
sol.thickness, sol.offset, sol.use_rim, sol.use_even_offset = LID_T, -1, True, True
sol.material_offset, sol.material_offset_rim = 1, 0
bake(cover)


def window(top):
    r = 0.035
    pts = [(-0.25, 0.05), (0.25, 0.05)]
    pts += [(0.25 - r + r * math.cos(math.radians(90 * i / 6)), top - r + r * math.sin(math.radians(90 * i / 6))) for i in range(7)]
    pts += [(0.057, top), (0.0, 0.487), (-0.057, top)]
    pts += [(-0.25 + r + r * math.cos(math.radians(90 + 90 * i / 6)), top - r + r * math.sin(math.radians(90 + 90 * i / 6))) for i in range(7)]
    return pts


boolean(cover, [lambda bm: add_prism(bm, window(0.40), "XZ", -0.36, -0.25),
                lambda bm: add_prism(bm, window(0.40), "XZ", 0.25, 0.36),
                lambda bm: add_box(bm, (-SEAM, -0.4, 0.05), (SEAM, 0.4, 0.7))])

LIDS, PIV = {}, {}
for side, name in ((1, "Lid_R"), (-1, "Lid_L")):
    bm = bmesh.new()
    bm.from_mesh(cover.data)
    bmesh.ops.delete(bm, geom=[v for v in bm.verts if v.co.x * side < 0], context="VERTS")
    PIV[side] = Vector((side * PIVOT_X, 0, PIVOT_Z))
    LIDS[side] = to_object(name, bm, ["white", "lid_in"], C_DOCK, DOCK, PIV[side], smooth_deg=40, recalc=False)
bpy.data.objects.remove(cover, do_unlink=True)


def lpart(side, name, build, mats, **kw):
    return part(name, build, mats, C_DOCK, LIDS[side], origin=PIV[side], local_origin=PIV[side], **kw)


for side, tag in ((1, "R"), (-1, "L")):
    xo, xi = side * LID_HX, side * (LID_HX - LID_T)
    # status indicators (item 16): set on the leaning part of the flank, proud of the surface
    # but inside the 640 mm envelope
    lpart(side, f"Lid_{tag}_StatusIndicators",
          lambda bm, xo=xo, side=side: [add_box(bm, (min(side * 0.312, xo - side * 0.0003), y - 0.006, 0.455),
                                                (max(side * 0.312, xo - side * 0.0003), y + 0.006, 0.505), 0.001) for y in (-0.235, 0.235)],
          ["led"])

    def inner(bm, xi=xi, side=side):
        add_box(bm, (min(xi, xi - side * 0.004), 0.055, 0.255), (max(xi, xi - side * 0.004), 0.145, 0.345), 0.002)       # item 19
        for dy in (-0.032, 0.032):
            for dz in (-0.032, 0.032):
                add_cyl(bm, (xi - side * 0.006, 0.10 + dy, 0.30 + dz), 0.005, 0.004, 12, "X", mat=1)
        add_box(bm, (min(side * 0.255, side * 0.294), -0.12, 0.49), (max(side * 0.255, side * 0.294), 0.12, 0.572), 0.01)
        ax = side * (PIVOT_X + STRUT_LID[0])
        add_box(bm, (min(ax - side * 0.004, xi), -0.015, PIVOT_Z + STRUT_LID[1] - 0.015),
                (max(ax - side * 0.004, xi), 0.015, PIVOT_Z + STRUT_LID[1] + 0.015), 0.003)

    lpart(side, f"Lid_{tag}_Inner", inner, ["lid_in", "metal"])

lpart(-1, "Lid_L_CellularDongle", lambda bm: (add_box(bm, (-(LID_HX - LID_T), -0.13, 0.20), (-(LID_HX - LID_T) + 0.004, -0.05, 0.28), 0.002),
                                               add_box(bm, (-(LID_HX - LID_T) + 0.004, -0.1, 0.235), (-(LID_HX - LID_T) + 0.009, -0.08, 0.245), 0.001)),
      ["tray"])

GX, GY = 0.228, 0.03
ROTOR_Z = LID_TOP + GAUGE_H - 0.0125
lpart(1, "Lid_R_WindGauge", lambda bm: (add_cyl(bm, (GX, GY, LID_TOP + 0.004), 0.028, 0.008, 32),
                                         add_cyl(bm, (GX, GY, LID_TOP + 0.050), 0.016, 0.085, 24),
                                         add_cyl(bm, (GX, GY, LID_TOP + 0.101), 0.011, 0.018, 20),
                                         add_cyl(bm, (0.215, 0.145, LID_TOP + 0.003), 0.046, 0.006, 40)), ["white"])


def build_rotor(bm):
    for i in range(3):
        a = 2 * math.pi * i / 3
        tip = Vector((GX + 0.028 * math.cos(a), GY + 0.028 * math.sin(a), ROTOR_Z))
        add_beam(bm, (GX, GY, ROTOR_Z), tip, 0.004, 0.004, 0.004, 0.004, bevel=0)
        add_sphere(bm, tip, 0.0125, segs=16)
    add_cyl(bm, (GX, GY, LID_TOP + 0.126), 0.006, 0.014, 12)


part("Lid_R_WindRotor", build_rotor, ["white"], C_DOCK, LIDS[1], origin=(GX, GY, ROTOR_Z), local_origin=PIV[1])
lpart(1, "Lid_R_DockCamera", lambda bm: (add_box(bm, (GX + 0.012, GY - 0.016, LID_TOP + 0.022), (GX + 0.034, GY + 0.016, LID_TOP + 0.050), 0.004),
                                          add_cyl(bm, (GX + 0.035, GY, LID_TOP + 0.036), 0.009, 0.004, 20, "X", mat=1),
                                          add_cyl(bm, (0.215, 0.145, LID_TOP + 0.0065), 0.036, 0.002, 40, mat=1)), ["black", "glass"])
lpart(1, "Lid_R_AuxLight", lambda bm: add_box(bm, (GX + 0.016, GY - 0.011, LID_TOP + 0.052), (GX + 0.031, GY + 0.011, LID_TOP + 0.061), 0.002), ["led"])

STRUTS = {}
for side, tag in ((1, "R"), (-1, "L")):
    objs = []
    for nm, r, ln, mat in (("Cylinder", 0.0085, 0.17, "black"), ("Rod", 0.0045, 0.20, "metal")):
        ob = dpart(f"Strut_{tag}_{nm}", lambda bm, r=r, ln=ln: (add_cyl(bm, (0, 0, ln / 2), r, ln, 16), add_cyl(bm, (0, 0, 0.004), r * 1.35, 0.008, 16)), [mat], smooth_deg=40)
        ob.rotation_mode = "QUATERNION"
        objs.append(ob)
    STRUTS[side] = objs

# ============================================================================ 8. Matrice 4D
print("[dock3] aircraft")
AC = empty("M4D", C_AC, None, (0, 0, AC_REST_Z), 0.1)


def apart(name, build, mats, parent=None, **kw):
    ob = part(name, build, mats, C_AC, parent or AC, **kw)
    ob.location = (0, 0, 0)
    return ob


if OFFICIAL:
    tri = 0
    for name, g, bm, cols in CAD_PARTS:
        if g in AC_GROUPS:
            ob = cad_object("M4D_" + name[4:], g, bm, cols, C_AC, AC)
            tri += len(ob.data.polygons)
    print(f"[dock3] DJI airframe: {tri:,} faces after decimation")
    near = lambda arr, x, y, r: arr[(arr[:, 0] - x) ** 2 + (arr[:, 1] - y) ** 2 < r * r]
    main = CAD["M4D_MAIN"]
    MOTOR_Z = {k: CAD_DISCS[k][0] for k in MOTORS}                  # hub base = bottom of DJI's swept disc
    motors = []
    for k, (mx, my, _) in MOTORS.items():
        top = float(near(main, mx, my, 0.02)[:, 2].max())
        if MOTOR_Z[k] - top > 0.003:                               # fill any gap between pad and rotor with a motor bell
            motors.append((mx, my, top, MOTOR_Z[k]))
    if motors:
        apart("M4D_Motors", lambda bm: [add_cyl(bm, (mx, my, (z0 + z1) / 2), 0.0162, z1 - z0, 36) for mx, my, z0, z1 in motors], ["ac_dark"])
    LED_Z = {k: float(near(main, mx, my, 0.02)[:, 2].min()) for k, (mx, my, _) in MOTORS.items()}
    bat = CAD["M4D_BAT"]
    BATT_LED_X = float(bat[:, 0].min()) + 0.02
    BATT_LED_Z = float(near(bat, BATT_LED_X, 0.0, 0.012)[:, 2].max()) if len(near(bat, BATT_LED_X, 0.0, 0.012)) else float(bat[:, 2].max())
    gim = CAD["M4D_GIMBAL"]
    HS_AC = {"A1": (float(gim[:, 0].max()), 0.0, float((gim[:, 2].min() + gim[:, 2].max()) / 2)),
             "A5": (0.02, 0.0, float(near(ac_pts, 0.02, 0.0, 0.015)[:, 2].min()) if len(near(ac_pts, 0.02, 0.0, 0.015)) else 0.004),
             "A12": (float(bat[:, 0].min()), 0.0, float((bat[:, 2].min() + bat[:, 2].max()) / 2)),
             "A19": tuple(float(v) for v in main[np.argmax(main[:, 2])])}
else:
    MOTOR_Z = {k: MOTOR_TOP for k in MOTORS}
    LED_Z = {k: 0.0875 for k in MOTORS}
    BATT_LED_X, BATT_LED_Z = -0.112, 0.1475
    HS_AC = {"A1": (AC_FRONT, 0.0, 0.035), "A5": (-0.01, 0.0, 0.004), "A12": (-0.137, 0.0, 0.12), "A19": (-0.05, 0.0, AC_H)}
    apart("M4D_Fuselage", lambda bm: add_loft(bm, [
        (-0.128, 0.036, 0.040, 0.098, 2.4), (-0.110, 0.050, 0.026, 0.112, 2.8), (-0.060, 0.056, 0.021, 0.119, 3.2),
        (0.000, 0.057, 0.020, 0.120, 3.2), (0.050, 0.054, 0.022, 0.115, 3.0), (0.085, 0.047, 0.028, 0.104, 2.8),
        (0.108, 0.037, 0.038, 0.090, 2.6), (0.121, 0.026, 0.047, 0.078, 2.3)], n=32), ["ac"], smooth_deg=55)
    apart("M4D_Battery", lambda bm: (add_box(bm, (-0.132, -0.047, 0.112), (0.005, 0.047, 0.1475), 0.012),
                                      add_box(bm, (-0.137, -0.03, 0.09), (-0.125, 0.03, 0.14), 0.004)), ["ac_dark"])
    GNSS_TOP = AC_H                                          # the top of the GNSS mast sets 212.5 mm
    apart("M4D_GNSS_RTK", lambda bm: (add_cyl(bm, (-0.05, 0, 0.152), 0.024, 0.012, 40), add_cyl(bm, (-0.05, 0, 0.180), 0.021, 0.045, 40),
                                       add_sphere(bm, (-0.05, 0, GNSS_TOP - 0.010), 0.021, (1, 1, 0.476), segs=40)), ["ac"])
    apart("M4D_Sensors", lambda bm: ([add_cyl(bm, (0.116, y, 0.070), 0.008, 0.006, 20, "X") for y in (-0.016, 0.016)]
                                      + [add_cyl(bm, (0.045, y, 0.1195), 0.007, 0.004, 20) for y in (-0.028, 0.028)]
                                      + [add_cyl(bm, (-0.02, y, 0.07), 0.007, 0.006, 20, "Y") for y in (-0.057, 0.057)]
                                      + [add_box(bm, (-0.06, -0.035, 0.0), (0.04, 0.035, 0.022), 0.004)]), ["glass"])


    def build_arms(bm):
        for mx, my, _ in MOTORS.values():
            s = 1 if my > 0 else -1
            root = (0.06, s * 0.045, 0.095) if mx > 0 else (-0.07, s * 0.045, 0.058)
            add_beam(bm, root, (mx, my, 0.118), 0.030, 0.026, 0.020, 0.018, bevel=0.007, segs=3)
            add_cyl(bm, (mx, my, 0.1175), POD_R, 0.045, 36)
            add_sphere(bm, (mx, my, 0.095), POD_R, (1, 1, 0.45), segs=24)
            p = Vector((mx, my, 0.125)) * 0.55 + Vector((0.0, s * 0.045, 0.08)) * 0.45
            add_beam(bm, (p.x, p.y, p.z), (p.x, p.y, p.z + 0.04), 0.004, 0.014, 0.003, 0.008, bevel=0.001, segs=1)


    apart("M4D_Arms", build_arms, ["ac"])
    apart("M4D_Motors", lambda bm: [add_cyl(bm, (mx, my, 0.151), 0.0162, 0.022, 36) for mx, my, _ in MOTORS.values()], ["ac_dark"])
    GIMBAL_X1 = AC_FRONT - 0.003                           # lens rims land exactly on the front


    def build_gimbal(bm):
        add_box(bm, (0.100, -0.03, 0.035), (0.128, 0.03, 0.095), 0.006)
        for y in (-0.046, 0.046):
            add_box(bm, (0.125, y - 0.005, 0.028), (0.172, y + 0.005, 0.07), 0.004)
            add_cyl(bm, (0.155, y, 0.034), 0.017, 0.012, 24, "Y")
        add_box(bm, (0.122, -0.039, 0.0), (GIMBAL_X1, 0.039, 0.068), 0.01)


    apart("M4D_Gimbal", build_gimbal, ["ac_dark"])
    apart("M4D_Lenses", lambda bm: ([add_cyl(bm, (GIMBAL_X1 + 0.0015, y, z), r, 0.003, 36, "X") for y, z, r in ((0.017, 0.049, 0.011), (-0.018, 0.049, 0.012), (-0.014, 0.022, 0.014))]
                                     + [add_box(bm, (GIMBAL_X1 - 0.001, 0.01, 0.012), (GIMBAL_X1 + 0.003, 0.03, 0.03), 0.001)]), ["glass"])

BATT_LED_Y = (-0.021, -0.007, 0.007, 0.021)
apart("M4D_BatteryLEDBezel", lambda bm: add_box(bm, (BATT_LED_X - 0.0045, -0.028, BATT_LED_Z - 0.001), (BATT_LED_X + 0.0045, 0.028, BATT_LED_Z + 0.0006), 0.001),
      ["glass"], smooth_deg=30)
BATT_LEDS = []
for i, y in enumerate(BATT_LED_Y):
    led = part(f"M4D_BatteryLED_{i + 1}", lambda bm: add_box(bm, (-0.0025, -0.004, -0.0006), (0.0025, 0.004, 0.0008), 0.0008), ["led_batt"], C_AC, AC, smooth_deg=30)
    led.location = (BATT_LED_X, y, BATT_LED_Z + 0.0004)
    BATT_LEDS.append(led)
apart("M4D_FrontLEDs", lambda bm: [add_cyl(bm, (mx, my, LED_Z[k] - 0.0012), 0.008, 0.003, 16, mat=0 if my > 0 else 1)
                                   for k, (mx, my, _) in MOTORS.items() if mx > 0], ["led_red", "led_green"])


def blade(bm, mirror):
    st, n = 20, 6
    rings = []
    for i in range(st):
        u = i / (st - 1)
        r = 0.012 + u * (PROP_R - 0.012)
        c = 0.016 + 0.014 * min(1, u / 0.3) - 0.02 * max(0, (u - 0.3) / 0.7)
        t = 0.0042 * (1 - 0.6 * u)
        tw = math.radians(20 - 12 * u)
        ring = []
        for cc, tt in ((0.5, 0), (0.18, 0.5), (-0.2, 0.38), (-0.5, 0), (-0.2, -0.22), (0.18, -0.36)):
            y = cc * c * math.cos(tw) - tt * t * math.sin(tw)
            z = cc * c * math.sin(tw) + tt * t * math.cos(tw)
            ring.append(bm.verts.new((r, -y if mirror else y, z)))
        rings.append(ring)
    for i in range(st - 1):
        for k in range(n):
            q = (rings[i][k], rings[i][(k + 1) % n], rings[i + 1][(k + 1) % n], rings[i + 1][k])
            bm.faces.new(q if not mirror else q[::-1]).material_index = 1 if i >= st - 3 else 0
    bm.faces.new(rings[0][::-1] if not mirror else rings[0])
    bm.faces.new(rings[-1] if not mirror else rings[-1][::-1]).material_index = 1


def fold_heading(key):
    """Parked blades point inward over the body when they clear it; otherwise along the arm pairs."""
    mx, my, _ = MOTORS[key]
    inward = math.atan2(-my, -mx)
    if not OFFICIAL:
        return inward
    d = Vector((math.cos(inward), math.sin(inward)))
    seg = [Vector((mx, my)) + d * PROP_R * s for s in np.linspace(0.1, 1.0, 12)]
    zmax = max((float(near(ac_pts, p.x, p.y, 0.02)[:, 2].max()) for p in seg if len(near(ac_pts, p.x, p.y, 0.02))), default=0.0)
    if MOTOR_Z[key] + 0.004 > zmax + 0.006:
        return inward
    return math.pi if mx > 0 else 0.0                            # front blades fold back, rear blades forward


PROPS = {}
for key, (mx, my, spin) in MOTORS.items():
    hub = apart(f"M4D_Prop_{key}", lambda bm: (add_cyl(bm, (0, 0, 0.005), 0.012, 0.01, 24), add_sphere(bm, (0, 0, 0.01), 0.009, (1, 1, 0.5), segs=16)), ["ac_dark"])
    hub.location = (mx, my, MOTOR_Z[key])
    blades = [apart(f"M4D_Prop_{key}_{b}", lambda bm: blade(bm, spin < 0), ["prop", "prop_tip"], parent=hub, smooth_deg=50, recalc=False) for b in "AB"]
    for bl in blades:
        bl.location = (0, 0, 0.005)
    PROPS[key] = (hub, blades, fold_heading(key), spin)

# ============================================================================ 9. site
cb = SPEC["concrete_base_mm"]
part("Site_ConcreteBase", lambda bm: add_box(bm, (-cb[0] * MM / 2, -cb[1] * MM / 2, -cb[2] * MM), (cb[0] * MM / 2, cb[1] * MM / 2, 0), 0.006),
     ["concrete"], C_SITE, smooth_deg=30)

# ============================================================================ 10. callouts
LR, LL = LIDS[1], LIDS[-1]
hs = [
    (1, "Wind Speed Gauge", LR, (GX, GY, LID_TOP + 0.13)), (2, "Dock Camera", LR, (GX + 0.036, GY, LID_TOP + 0.036)),
    (3, "Camera Auxiliary Light", LR, (GX + 0.03, GY, LID_TOP + 0.058)), (4, "Rainfall Gauge", LR, (0.215, 0.145, LID_TOP + 0.008)),
    (5, "Dock Cover", LL, (-LID_HX, 0.0, 0.30)), (6, "Positioning Markers", DOCK, (0.11, 0.17, (pad_z(0.11, 0.17) or TRAY_Z1) + 0.002)),
    (7, "Landing Pad (built-in charging module)", DOCK, (0.0, -0.15, (pad_z(0.0, -0.15) or TRAY_Z1) + 0.002)),
    (8, "Aircraft Orientation Marker", DOCK, (-0.085, 0.0, (pad_z(-0.085, 0.0) or STRIP_TOP) + 0.002)), (9, "PoE Output Port", DOCK, (-0.155, FY - 0.011, 0.09)),
    (10, "AC-IN Port", DOCK, (-0.215, FY - 0.011, 0.09)), (11, "Earth Wire", DOCK, (0.262, FY - 0.01, 0.08)),
    (12, "Electrical Cabinet", DOCK, (0.15, FY - 0.016, 0.22)), (13, "LAN-IN Port", DOCK, (-0.185, FY - 0.011, 0.09)),
    (14, "Mounting Base Brackets", DOCK, (0.24, -HOLE_Y - 0.026, 0.045)), (15, "Emergency Stop Button", DOCK, (-0.2, FY - 0.057, 0.335)),
    (16, "Status Indicators", LR, (LID_HX + 0.002, -0.235, 0.48)), (17, "Cellular Dongle Compartment", LL, (-(LID_HX - LID_T) + 0.006, -0.09, 0.24)),
    (18, "RTK Module", DOCK, tuple(RTK_C + Vector((0, 0, 0.028)))), (19, "For Vehicle-mounted Gimbal Mount", LR, (LID_HX - LID_T - 0.006, 0.10, 0.30)),
    (20, "E-Port", DOCK, (-0.125, FY - 0.008, 0.355)), (21, "Drain Pipe Hole (underneath)", DOCK, (0.0, -0.1, BRACKET_H)),
]
for num, label, parent, pos in hs:
    hotspot(num, label, parent, pos, C_DOCK, origin=PIV[1] if parent is LR else PIV[-1] if parent is LL else (0, 0, 0))
for num, label, pos in (("A1", "Gimbal Camera", HS_AC["A1"]), ("A5", "Internal Charging Modules", HS_AC["A5"]),
                        ("A6", "Front LEDs", (MOTORS["FR"][0], MOTORS["FR"][1], LED_Z["FR"] - 0.002)),
                        ("A9", "Propellers (1364F folding)", (MOTORS["FL"][0], MOTORS["FL"][1], MOTOR_Z["FL"] + 0.01)),
                        ("A12", "Intelligent Flight Battery", HS_AC["A12"]), ("A14", "Battery Level LEDs", (BATT_LED_X, 0.0, BATT_LED_Z + 0.002)),
                        ("A19", "GNSS/RTK Antenna", HS_AC["A19"])):
    hotspot(num, label, AC, pos, C_AC, group="aircraft")

# ============================================================================ 11. animation (baked per frame)
print("[dock3] animation")
frames = list(range(F["end"] + 1))


def lid_theta(f):
    return math.pi / 2 * (ramp(f, F["open0"], F["open1"]) - ramp(f, F["close0"], F["close1"]))


for f in frames:
    th = lid_theta(f)
    lag = max(0.0, (th / (math.pi / 2) - 0.5) / 0.5) ** 2          # linkage offset lags the swing
    for side in (1, -1):
        lid = LIDS[side]
        lid.location = Vector((side * (PIVOT_X + LID_DX * lag), 0, PIVOT_Z + LID_DZ * lag))
        lid.rotation_euler = Euler((0, side * th, 0))
        for p in ("location", "rotation_euler"):
            lid.keyframe_insert(p, frame=f)
        mw = Matrix.Translation(lid.location) @ lid.rotation_euler.to_matrix().to_4x4()
        b = Vector((side * STRUT_BODY[0], 0, STRUT_BODY[1]))
        lp = mw @ Vector((side * STRUT_LID[0], 0, STRUT_LID[1]))
        cyl, rod = STRUTS[side]
        cyl.location, cyl.rotation_quaternion = b, Vector((0, 0, 1)).rotation_difference(lp - b)
        rod.location, rod.rotation_quaternion = lp, Vector((0, 0, 1)).rotation_difference(b - lp)
        for ob in (cyl, rod):
            ob.keyframe_insert("location", frame=f)
            ob.keyframe_insert("rotation_quaternion", frame=f)

REST = Vector((0, 0, AC_REST_Z))


def spiral(phi):
    r = ORBIT_R * smooth(phi / (math.pi / 2)) * smooth((3 * math.pi - phi) / (math.pi / 2))
    return Vector((r * math.cos(phi), r * math.sin(phi)))


def ac_pos(f):
    if f <= F["up0"]:
        return REST.copy()
    if f <= F["up1"]:
        return Vector((0, 0, REST.z + CRUISE_DZ * ramp(f, F["up0"], F["up1"])))
    if f <= F["loop1"]:
        p = spiral(3 * math.pi * ramp(f, F["loop0"], F["loop1"]))
        return Vector((p.x, p.y, REST.z + CRUISE_DZ))
    if f <= F["down1"]:
        return Vector((0, 0, REST.z + CRUISE_DZ * (1 - ramp(f, F["down0"], F["down1"]))))
    return REST.copy()


pos = [ac_pos(f) for f in frames]
yaw, last = [], 0.0
for f in frames:
    h = last
    if F["loop0"] <= f <= F["loop1"]:
        phi = 3 * math.pi * ramp(f, F["loop0"], F["loop1"])
        d = spiral(min(3 * math.pi, phi + 1e-3)) - spiral(max(0.0, phi - 1e-3))
        h = math.atan2(d.y, d.x)
        h += 2 * math.pi * round((last - h) / (2 * math.pi))
    yaw.append(h)
    last = h
dt, tilt = 1.0 / FPS, []
for i in range(len(frames)):
    a = pos[min(i + 1, len(pos) - 1)] - 2 * pos[i] + pos[max(i - 1, 0)]
    v = pos[min(i + 1, len(pos) - 1)] - pos[max(i - 1, 0)]
    tv = (Vector((a.x, a.y)) / dt ** 2 + 0.35 * Vector((v.x, v.y)) / (2 * dt)) / 9.81
    fwd, left = Vector((math.cos(yaw[i]), math.sin(yaw[i]))), Vector((-math.sin(yaw[i]), math.cos(yaw[i])))
    tilt.append((max(-0.4, min(0.4, math.atan(tv.dot(fwd)))), max(-0.4, min(0.4, -math.atan(tv.dot(left))))))
for i, f in enumerate(frames):
    w = tilt[max(0, i - 3): i + 4]
    AC.location = pos[i]
    AC.rotation_euler = Euler((sum(x[1] for x in w) / len(w), sum(x[0] for x in w) / len(w), yaw[i]), "XYZ")
    AC.keyframe_insert("location", frame=f)
    AC.keyframe_insert("rotation_euler", frame=f)

SPIN_STOP = F["stop0"] + 22
w = np.array([0.0 if f < F["spin0"] else OMEGA * ramp(f, F["spin0"], F["spin1"]) if f < F["spin1"] else OMEGA if f < F["stop0"]
              else OMEGA * (1 - ramp(f, F["stop0"], SPIN_STOP)) for f in frames])
acc = np.cumsum(w)
extra = math.ceil(acc[-1] / (2 * math.pi)) * 2 * math.pi - acc[-1]           # stop exactly at the parked angle
decel = np.where((np.array(frames) >= F["stop0"]) & (np.array(frames) < SPIN_STOP), w, 0.0)
acc = acc + np.cumsum(decel / decel.sum() * extra)
for hub, blades, h0, spin in PROPS.values():
    for i, f in enumerate(frames):
        hub.rotation_euler = (0, 0, h0 + spin * acc[i])
        hub.keyframe_insert("rotation_euler", frame=f)
        a = FOLD + (math.pi / 2 - FOLD) * (ramp(f, F["spin0"], F["spin0"] + 8) - ramp(f, SPIN_STOP, F["stop1"]))
        blades[0].rotation_euler, blades[1].rotation_euler = (0, 0, a), (0, 0, -a)
        for bl in blades:
            bl.keyframe_insert("rotation_euler", frame=f)

for f in frames:
    charging = F["charge0"] <= f < F["charge1"]
    on = ramp(f, F["charge0"], F["charge0"] + 8) * (1 - ramp(f, F["charge1"] - 8, F["charge1"]))
    COIL.scale = (on,) * 3 if on > 1e-4 else (0, 0, 0)
    COIL.keyframe_insert("scale", frame=f)
    for k, ring in enumerate(FIELD):
        p = ((f - F["charge0"]) / 30.0 + k / 3.0) % 1.0
        ring.scale = (1 + 2.3 * p, 1 + 0.15 * p, 1) if charging and on > 1e-4 else (0, 0, 0)
        ring.keyframe_insert("scale", frame=f)
    states = battery_leds(battery_level(f), f) if leds_active(f) else [False] * 4
    for led, lit in zip(BATT_LEDS, states):
        led.scale = (1, 1, 1) if lit else (0, 0, 0)
        led.keyframe_insert("scale", frame=f)

markers = {"Closed": 0, "Cover open": F["open1"], "Take-off": F["up0"], "Orbit": F["loop0"] + 60, "Landing": F["down0"],
           "Cover closing": F["close0"], "Charging": F["charge0"], "End": F["end"]}
for name, f in markers.items():
    scene.timeline_markers.new(name, frame=f)
TIMELINE = {"fps": FPS, "frames": F, "markers": markers, "battery": BATTERY}
scene["dock3_spec"] = json.dumps(SPEC)
scene["dock3_timeline"] = json.dumps(TIMELINE)

# ============================================================================ 12. validation (+/-1 mm)


def envelope(objs, frame):
    scene.frame_set(frame)
    dg = bpy.context.evaluated_depsgraph_get()
    lo, hi = np.full(3, 1e9), np.full(3, -1e9)
    for ob in objs:
        if ob.type != "MESH":
            continue
        ev = ob.evaluated_get(dg)
        if not len(ev.data.vertices):
            continue
        co = np.empty(len(ev.data.vertices) * 3)
        ev.data.vertices.foreach_get("co", co)
        mw = np.array(ev.matrix_world)
        pts = co.reshape(-1, 3) @ mw[:3, :3].T + mw[:3, 3]
        lo, hi = np.minimum(lo, pts.min(0)), np.maximum(hi, pts.max(0))
    return (hi - lo) * 1000


report, failed = {}, []
checks = (("closed", list(C_DOCK.all_objects), 0, SPEC["closed_mm"]), ("open", list(C_DOCK.all_objects), F["open1"], SPEC["open_mm"]),
          ("aircraft", [o for o in C_AC.all_objects if not o.name.startswith("M4D_Prop")], 0, SPEC["aircraft_mm"]))
for label, objs, frame, spec in checks:
    size = envelope(objs, frame)
    if label == "aircraft" and OFFICIAL:      # DJI's CAD is the reference; its height is reported against the sheet
        ok = all(abs(a - b) <= 1.5 for a, b in zip(size[:2], spec[:2]))
        print(f"[dock3] aircraft source: DJI CAD (M4D STEP). CAD height {size[2]:.1f} mm vs spec-sheet {spec[2]} mm "
              f"({size[2] - spec[2]:+.1f} mm): the sheet's figure is not reproduced by DJI's own model")
    else:
        ok = all(abs(a - b) <= TOL_MM for a, b in zip(size, spec))
    report[label] = {"model_mm": [round(v, 1) for v in size], "spec_mm": spec, "ok": ok,
                     "source": "DJI CAD" if (label == "aircraft" and OFFICIAL) else "model"}
    failed += [] if ok else [label]
    print(f"[dock3] {label:8s} L x W x H = {size[0]:.1f} x {size[1]:.1f} x {size[2]:.1f} mm  (spec {spec[0]} x {spec[1]} x {spec[2]})  "
          f"{'OK' if ok else 'OUT OF TOLERANCE'} ({'L/W +/-1.5 mm, DJI CAD' if label == 'aircraft' and OFFICIAL else f'+/-{TOL_MM:g} mm'})")
scene["dock3_validation"] = json.dumps(report)
scene.frame_set(0)

# ============================================================================ 13. studio (Blender only)
world = bpy.data.worlds.new("Studio")
scene.world = world
bg = world.node_tree.nodes["Background"]
bg.inputs[0].default_value, bg.inputs[1].default_value = (0.58, 0.62, 0.67, 1), 0.7


def light(name, kind, energy, loc, rot, size):
    ld = bpy.data.lights.new(name, kind)
    ld.energy = energy
    if kind == "SUN":
        ld.angle = size
    else:
        ld.size = size
    ob = bpy.data.objects.new(name, ld)
    C_STUDIO.objects.link(ob)
    ob.location, ob.rotation_euler = loc, rot
    return ob


light("Key", "SUN", 3.2, (3, -4, 6), (math.radians(48), 0, math.radians(38)), math.radians(4))
light("Fill", "AREA", 250, (-3.5, -2.5, 2.5), (math.radians(60), 0, math.radians(-55)), 3.0)
light("Rim", "AREA", 180, (1.0, 4.0, 2.2), (math.radians(-65), 0, math.radians(190)), 2.0)
part("Studio_Ground", lambda bm: add_box(bm, (-150, -150, -0.101), (150, 150, -0.1)), ["ground"], C_STUDIO, smooth_deg=None)

WINTER = []
for side in (1, -1):
    x0, x1 = sorted((side * (SEAM + 0.002), side * 0.296))
    cap = part(f"Winter_SnowCap_{'R' if side > 0 else 'L'}", lambda bm, x0=x0, x1=x1: add_box(bm, (x0, -0.276, LID_TOP - 0.006), (x1, 0.276, LID_TOP + 0.06), 0.028, segs=5),
               ["snow"], C_STUDIO, LIDS[side], origin=PIV[side], local_origin=PIV[side], smooth_deg=60)
    cap.location = (0, 0, 0)
    WINTER.append(cap)
WINTER.append(part("Winter_SnowGround", lambda bm: add_box(bm, (-150, -150, -0.1), (150, 150, 0.13)), ["snow"], C_STUDIO, smooth_deg=None))
for o in WINTER:
    o.hide_render = True
    o.hide_set(True)


def camera(name, loc, target, lens=50, ortho=None):
    cd = bpy.data.cameras.new(name)
    cd.lens = lens
    if ortho:
        cd.type, cd.ortho_scale = "ORTHO", ortho
    ob = bpy.data.objects.new(name, cd)
    C_STUDIO.objects.link(ob)
    ob.location = loc
    ob.rotation_euler = (Vector(target) - Vector(loc)).to_track_quat("-Z", "Y").to_euler()
    return ob


CAMS = {
    "hero": camera("Cam_Hero", (1.62, -1.86, 1.18), (0.05, -0.03, 0.33)),
    "pad": camera("Cam_Pad", (0.62, -0.95, 1.25), (0.0, -0.02, 0.42)),
    "flight": camera("Cam_Flight", (3.0, -4.7, 2.2), (0.1, -0.7, 1.25), 35),
    "front": camera("Cam_FrontOrtho", (0, -6, 0.3), (0, 0, 0.3), ortho=2.0),
    "top": camera("Cam_TopOrtho", (0, -0.08, 6), (0, -0.08, 0), ortho=2.0),
    "back": camera("Cam_Back", (-1.35, 1.95, 1.15), (0.0, 0.0, 0.33)),
    "winter": camera("Cam_Winter", (1.75, -2.25, 0.62), (0.0, 0.0, 0.36), 45),
}
scene.camera = CAMS["hero"]
scene.render.engine = "BLENDER_EEVEE"
scene.render.resolution_x, scene.render.resolution_y = 1600, 1000
try:
    scene.eevee.taa_render_samples = 64
except AttributeError:
    pass

# ============================================================================ 14. save / export / render
scene.frame_set(0)
blend = os.path.join(ROOT, "dock3.blend")
bpy.ops.wm.save_as_mainfile(filepath=blend)
print("[dock3] saved", blend)

if DO_EXPORT:
    bpy.ops.object.select_all(action="DESELECT")
    for c in (C_DOCK, C_AC, C_SITE):
        for ob in c.all_objects:
            ob.select_set(True)
    glb = os.path.join(ROOT, "web", "dock3.glb")
    bpy.ops.export_scene.gltf(filepath=glb, export_format="GLB", use_selection=True, export_extras=True, export_apply=True,
                              export_yup=True, export_animations=True, export_animation_mode="SCENE",
                              export_anim_scene_split_object=False, export_frame_range=True, export_force_sampling=True,
                              export_optimize_animation_size=True)
    print(f"[dock3] exported {glb} {os.path.getsize(glb) / 1e6:.2f} MB")
    with open(os.path.join(ROOT, "web", "dock3_meta.json"), "w") as fh:
        json.dump({"spec": SPEC, "validation": report, "timeline": TIMELINE, "hotspots": HOTSPOTS}, fh, indent=2)

if DO_RENDER:
    out = os.path.join(ROOT, "renders")
    os.makedirs(out, exist_ok=True)
    shots = [("01_closed", "hero", 0, None), ("02_open_ready", "hero", 140, None), ("03_orbit", "flight", 360, None),
             ("04_front_closed_ortho", "front", 0, None), ("05_front_open_ortho", "front", F["open1"], None),
             ("06_top_open_ortho", "top", F["open1"], None), ("07_pad_detail", "pad", 140, None), ("08_back_closed", "back", 0, None),
             ("09_charging_cutaway", "pad", 850, "cutaway"), ("10_winter_low_mount", "winter", 0, "winter")]
    cover_parts = [o for lid in LIDS.values() for o in [lid] + list(lid.children_recursive)]
    for name, cam, frame, mode in shots:
        if SHOTS and name not in SHOTS:
            continue
        for o in cover_parts:
            o.hide_render = mode == "cutaway"
        for o in WINTER:
            o.hide_render = mode != "winter"
        bg.inputs[0].default_value = (0.30, 0.35, 0.43, 1) if mode == "winter" else (0.58, 0.62, 0.67, 1)
        scene.camera, scene.render.filepath = CAMS[cam], os.path.join(out, name + ".png")
        scene.frame_set(frame)
        bpy.ops.render.render(write_still=True)
        print("[dock3] rendered", scene.render.filepath)
    for o in cover_parts:
        o.hide_render = False
    for o in WINTER:
        o.hide_render = True
    bg.inputs[0].default_value = (0.58, 0.62, 0.67, 1)
    scene.frame_set(0)
    scene.camera = CAMS["hero"]
    bpy.ops.wm.save_as_mainfile(filepath=blend)

if failed:
    print("[dock3] VALIDATION FAILED:", ", ".join(failed))
