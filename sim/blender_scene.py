"""
Build a Blender scene of MANTA from the MuJoCo trajectory.

Strategy:
  - RIGID body parts (torso, head, spine yoke, limbs, LE spar bonded to
    forearm): fixed body-local meshes parented to the body's empty.
  - TELESCOPING tip extensions: per-frame world-space tube meshes that
    span between consecutive body positions. At slide=0, consecutive
    bodies are coincident → tube has zero length (invisible). At
    slide=full, tube has its full extension length.
  - WING SKIN: per-frame quad surface from shoulder → tip3 → te3 → te_hub
    on each side, only visible when deployment is far enough along.

Renders keyframes via Eevee, exports glTF for the web viewer.
"""

from __future__ import annotations

import json
import math
from pathlib import Path

import bpy
import bmesh
import mujoco

_HERE = Path(__file__).parent
MJCF = _HERE / "deployment_mjcf.xml"
TRAJ = _HERE / "out" / "trajectory.json"
OUT_DIR = _HERE / "out" / "blender"
OUT_DIR.mkdir(parents=True, exist_ok=True)


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def clear_scene() -> None:
    bpy.ops.wm.read_factory_settings(use_empty=True)
    for c in list(bpy.data.collections):
        bpy.data.collections.remove(c)
    for m in list(bpy.data.meshes):
        bpy.data.meshes.remove(m)
    for mat in list(bpy.data.materials):
        bpy.data.materials.remove(mat)
    for o in list(bpy.data.objects):
        bpy.data.objects.remove(o, do_unlink=True)


def make_material(name, color, metallic=0.0, roughness=0.5, alpha=1.0):
    mat = bpy.data.materials.new(name=name)
    mat.use_nodes = True
    bsdf = mat.node_tree.nodes["Principled BSDF"]
    rgba = (color[0], color[1], color[2], alpha)
    bsdf.inputs["Base Color"].default_value = rgba
    bsdf.inputs["Metallic"].default_value = metallic
    bsdf.inputs["Roughness"].default_value = roughness
    if alpha < 1.0:
        bsdf.inputs["Alpha"].default_value = alpha
        mat.blend_method = "BLEND"
    return mat


def add_capsule_local(name, p_in, p_out, radius, material, parent):
    """Capsule with endpoints p_in / p_out in PARENT's local frame."""
    p_in = list(p_in)
    p_out = list(p_out)
    dx = p_out[0] - p_in[0]
    dy = p_out[1] - p_in[1]
    dz = p_out[2] - p_in[2]
    length = math.sqrt(dx * dx + dy * dy + dz * dz)
    if length < 1e-6:
        return None
    midpoint = ((p_in[0] + p_out[0]) / 2,
                 (p_in[1] + p_out[1]) / 2,
                 (p_in[2] + p_out[2]) / 2)
    bpy.ops.mesh.primitive_cylinder_add(radius=radius, depth=length, vertices=20,
                                          location=midpoint)
    obj = bpy.context.active_object
    obj.name = name
    if length > 0:
        ux, uy, uz = dx / length, dy / length, dz / length
        if abs(ux) + abs(uy) < 1e-9:
            obj.rotation_euler = (0, 0, 0) if uz > 0 else (math.pi, 0, 0)
        else:
            ax = -uy
            ay = ux
            az = 0.0
            ang = math.acos(max(-1.0, min(1.0, uz)))
            anorm = math.sqrt(ax * ax + ay * ay + az * az)
            if anorm > 1e-9:
                ax /= anorm; ay /= anorm; az /= anorm
                half = ang / 2
                qw = math.cos(half)
                qx = ax * math.sin(half)
                qy = ay * math.sin(half)
                qz = az * math.sin(half)
                obj.rotation_mode = "QUATERNION"
                obj.rotation_quaternion = (qw, qx, qy, qz)
    bpy.ops.object.shade_smooth()
    obj.data.materials.append(material)
    sub = obj.modifiers.new("subsurf", "SUBSURF")
    sub.levels = 1
    sub.render_levels = 2
    obj.parent = parent
    return obj


def add_sphere_local(name, pos, radius, material, parent):
    bpy.ops.mesh.primitive_uv_sphere_add(radius=radius, location=pos, segments=24, ring_count=12)
    obj = bpy.context.active_object
    obj.name = name
    bpy.ops.object.shade_smooth()
    obj.data.materials.append(material)
    sub = obj.modifiers.new("subsurf", "SUBSURF")
    sub.levels = 1
    sub.render_levels = 2
    obj.parent = parent
    return obj


def add_box_local(name, pos, size_xyz, material, parent):
    bpy.ops.mesh.primitive_cube_add(location=pos)
    obj = bpy.context.active_object
    obj.name = name
    obj.scale = size_xyz
    bpy.ops.object.transform_apply(location=False, rotation=False, scale=True)
    bpy.ops.object.shade_smooth()
    obj.data.materials.append(material)
    obj.parent = parent
    return obj


def make_world_capsule(name, p_in, p_out, radius, material):
    """Free-standing capsule in world coordinates (no parent)."""
    return add_capsule_local(name, p_in, p_out, radius, material, parent=None)


def remove_obj_by_name_prefix(prefix: str):
    for o in list(bpy.data.objects):
        if o.name.startswith(prefix):
            bpy.data.objects.remove(o, do_unlink=True)


# ---------------------------------------------------------------------------
# Build static scene from MuJoCo model
# ---------------------------------------------------------------------------

DYNAMIC_GEOMS = {
    # geom name → (parent_body_world, child_body_world, radius)
    # We DON'T place these as fixed body-local meshes; they're rebuilt each
    # frame from world body positions.
    "tip1_R_g": ("wrist_R", "tip1_R", 0.022),
    "tip2_R_g": ("tip1_R", "tip2_R", 0.018),
    "tip3_R_g": ("tip2_R", "tip3_R", 0.013),
    "tip1_L_g": ("wrist_L", "tip1_L", 0.022),
    "tip2_L_g": ("tip1_L", "tip2_L", 0.018),
    "tip3_L_g": ("tip2_L", "tip3_L", 0.013),
    "te1_R_g":  ("te_hub", "te1_R", 0.020),
    "te2_R_g":  ("te1_R", "te2_R", 0.016),
    "te3_R_g":  ("te2_R", "te3_R", 0.013),
    "te1_L_g":  ("te_hub", "te1_L", 0.020),
    "te2_L_g":  ("te1_L", "te2_L", 0.016),
    "te3_L_g":  ("te2_L", "te3_L", 0.013),
}


def build_static_from_model(model: mujoco.MjModel) -> dict:
    """Build the body anchors + RIGID geometry. Return body_id → empty."""
    materials = {
        "skin":    make_material("skin",    (0.78, 0.62, 0.46), roughness=0.55),
        "cfrp":    make_material("cfrp",    (0.10, 0.10, 0.10), metallic=0.4, roughness=0.30),
        "harness": make_material("harness", (0.18, 0.18, 0.20), roughness=0.50),
        "fabric":  make_material("fabric",  (0.30, 0.45, 0.78), roughness=0.40, alpha=0.65),
    }

    body_to_empty: dict[int, bpy.types.Object] = {}
    for bid in range(model.nbody):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, bid) or f"body_{bid}"
        empty = bpy.data.objects.new(name, None)
        empty.empty_display_size = 0.05
        bpy.context.collection.objects.link(empty)
        body_to_empty[bid] = empty

    for gid in range(model.ngeom):
        gname = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, gid) or f"geom_{gid}"
        if gname in DYNAMIC_GEOMS:
            continue   # built dynamically per frame
        bid = model.geom_bodyid[gid]
        gtype = model.geom_type[gid]
        size = model.geom_size[gid]
        pos = model.geom_pos[gid]
        matid = model.geom_matid[gid]
        mat_name = (mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_MATERIAL, matid)
                     if matid >= 0 else "harness")
        mat = materials.get(mat_name or "harness", materials["harness"])
        parent = body_to_empty[bid]

        if gtype == mujoco.mjtGeom.mjGEOM_SPHERE:
            add_sphere_local(gname, list(pos), float(size[0]), mat, parent)
        elif gtype == mujoco.mjtGeom.mjGEOM_BOX:
            add_box_local(gname, list(pos), [float(s) for s in size], mat, parent)
        elif gtype == mujoco.mjtGeom.mjGEOM_CAPSULE:
            radius = float(size[0])
            half_length = float(size[1])
            quat = model.geom_quat[gid]
            qw, qx, qy, qz = float(quat[0]), float(quat[1]), float(quat[2]), float(quat[3])
            ax = 2 * (qx * qz + qw * qy)
            ay = 2 * (qy * qz - qw * qx)
            az = 1 - 2 * (qx * qx + qy * qy)
            cx, cy, cz = float(pos[0]), float(pos[1]), float(pos[2])
            p_in = (cx - half_length * ax, cy - half_length * ay, cz - half_length * az)
            p_out = (cx + half_length * ax, cy + half_length * ay, cz + half_length * az)
            add_capsule_local(gname, p_in, p_out, radius, mat, parent)

    # Camera + lights + world
    bpy.ops.object.light_add(type="SUN", location=(5, -5, 8))
    sun = bpy.context.active_object
    sun.data.energy = 5.5
    sun.rotation_euler = (math.radians(50), 0, math.radians(-30))

    bpy.ops.object.light_add(type="AREA", location=(-3, 4, 4))
    fill = bpy.context.active_object
    fill.data.energy = 350
    fill.data.size = 5

    # Iso camera looking at the pilot from front-left-above
    bpy.ops.object.camera_add(location=(2.8, -3.8, 2.2))
    cam = bpy.context.active_object
    # Point the camera at origin
    constraint = cam.constraints.new(type="TRACK_TO")
    constraint.track_axis = "TRACK_NEGATIVE_Z"
    constraint.up_axis = "UP_Y"
    # Add an empty at origin to track to
    bpy.ops.object.empty_add(location=(0, 0, 0))
    target = bpy.context.active_object
    target.name = "cam_target"
    constraint.target = target
    bpy.context.scene.camera = cam

    # World background
    world = bpy.data.worlds.new("World")
    world.use_nodes = True
    bg = world.node_tree.nodes["Background"]
    bg.inputs["Color"].default_value = (0.05, 0.06, 0.08, 1.0)
    bg.inputs["Strength"].default_value = 0.5
    bpy.context.scene.world = world

    # Renderer
    scene = bpy.context.scene
    scene.render.engine = "BLENDER_EEVEE"
    scene.render.resolution_x = 1600
    scene.render.resolution_y = 900
    scene.render.image_settings.file_format = "PNG"

    return body_to_empty, materials


# ---------------------------------------------------------------------------
# Per-frame dynamic geometry — telescoping tubes + wing skin
# ---------------------------------------------------------------------------

def rebuild_dynamic_geoms(frame: dict, body_to_empty: dict, model, materials):
    """Remove last frame's dynamic meshes, build fresh ones."""
    remove_obj_by_name_prefix("tip_dyn_")
    remove_obj_by_name_prefix("te_dyn_")
    remove_obj_by_name_prefix("skin_dyn_")

    bodies = frame["bodies"]
    cfrp = materials["cfrp"]
    fabric = materials["fabric"]

    for gname, (parent_body_name, child_body_name, radius) in DYNAMIC_GEOMS.items():
        if parent_body_name not in bodies or child_body_name not in bodies:
            continue
        p_in = bodies[parent_body_name]["pos"]
        p_out = bodies[child_body_name]["pos"]
        d2 = sum((p_in[i] - p_out[i]) ** 2 for i in range(3))
        if d2 < 1e-6:
            continue   # collapsed → invisible
        make_world_capsule(f"tip_dyn_{gname}", p_in, p_out, radius, cfrp)

    # Wing skin (per side) — render only if extension > 30 % (avoids degenerate
    # quads at stowed). We compute skin from key world points.
    if "tip3_R" in bodies and "te3_R" in bodies and "shoulder_R" in bodies and "te_hub" in bodies:
        sh_R = bodies["shoulder_R"]["pos"]
        tip_R = bodies["tip3_R"]["pos"]
        te_R = bodies["te3_R"]["pos"]
        hub = bodies["te_hub"]["pos"]
        # Span check
        if abs(tip_R[1]) > 0.4:
            _make_skin_quad("skin_dyn_R", [sh_R, tip_R, te_R, hub], fabric)

    if "tip3_L" in bodies and "te3_L" in bodies and "shoulder_L" in bodies and "te_hub" in bodies:
        sh_L = bodies["shoulder_L"]["pos"]
        tip_L = bodies["tip3_L"]["pos"]
        te_L = bodies["te3_L"]["pos"]
        hub = bodies["te_hub"]["pos"]
        if abs(tip_L[1]) > 0.4:
            _make_skin_quad("skin_dyn_L", [sh_L, tip_L, te_L, hub], fabric)


def _make_skin_quad(name, four_pts, material):
    """Build a quad surface from 4 corner points."""
    mesh = bpy.data.meshes.new(name)
    obj = bpy.data.objects.new(name, mesh)
    bpy.context.collection.objects.link(obj)
    bm = bmesh.new()
    verts = [bm.verts.new(p) for p in four_pts]
    bm.faces.new(verts)
    bm.normal_update()
    bm.to_mesh(mesh)
    bm.free()
    obj.data.materials.append(material)


# ---------------------------------------------------------------------------
# Pose + animation
# ---------------------------------------------------------------------------

def set_pose(body_to_empty: dict, model, frame: dict) -> None:
    for name, b in frame["bodies"].items():
        bid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, name)
        if bid < 0 or bid not in body_to_empty:
            continue
        empty = body_to_empty[bid]
        empty.location = b["pos"]
        empty.rotation_mode = "QUATERNION"
        empty.rotation_quaternion = b["quat"]


def main() -> None:
    print(f"  Loading {MJCF}")
    model = mujoco.MjModel.from_xml_path(str(MJCF))

    print(f"  Loading trajectory {TRAJ}")
    with TRAJ.open() as f:
        traj = json.load(f)
    print(f"    {len(traj['frames'])} frames over {traj['duration_s']} s")

    print("  Building Blender scene...")
    clear_scene()
    body_to_empty, materials = build_static_from_model(model)

    keyframe_indices = [0, len(traj["frames"]) // 4,
                         len(traj["frames"]) // 2,
                         3 * len(traj["frames"]) // 4,
                         len(traj["frames"]) - 1]

    for idx in keyframe_indices:
        frame = traj["frames"][idx]
        set_pose(body_to_empty, model, frame)
        rebuild_dynamic_geoms(frame, body_to_empty, model, materials)
        bpy.context.view_layer.update()
        out_path = OUT_DIR / f"frame_{idx:04d}_t{frame['t']:.3f}.png"
        bpy.context.scene.render.filepath = str(out_path)
        print(f"    rendering {out_path.name}")
        bpy.ops.render.render(write_still=True)

    # ---- Export glTF (deployed pose, with all dynamic meshes baked in) ----
    final_frame = traj["frames"][-1]
    set_pose(body_to_empty, model, final_frame)
    rebuild_dynamic_geoms(final_frame, body_to_empty, model, materials)
    bpy.context.view_layer.update()
    gltf_d = OUT_DIR / "deployed.glb"
    bpy.ops.export_scene.gltf(filepath=str(gltf_d), export_format="GLB",
                              export_animations=False)
    print(f"  Wrote {gltf_d}")

    # Stowed pose
    stowed = traj["frames"][0]
    set_pose(body_to_empty, model, stowed)
    rebuild_dynamic_geoms(stowed, body_to_empty, model, materials)
    bpy.context.view_layer.update()
    gltf_s = OUT_DIR / "stowed.glb"
    bpy.ops.export_scene.gltf(filepath=str(gltf_s), export_format="GLB",
                              export_animations=False)
    print(f"  Wrote {gltf_s}")

    # Mid-deploy snapshot
    mid = traj["frames"][len(traj["frames"]) // 2 + 5]
    set_pose(body_to_empty, model, mid)
    rebuild_dynamic_geoms(mid, body_to_empty, model, materials)
    bpy.context.view_layer.update()
    gltf_m = OUT_DIR / "mid_deploy.glb"
    bpy.ops.export_scene.gltf(filepath=str(gltf_m), export_format="GLB",
                              export_animations=False)
    print(f"  Wrote {gltf_m}")


if __name__ == "__main__":
    main()
