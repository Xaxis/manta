"""
Run the MANTA deployment kinematics simulation in MuJoCo and bake the
result to a JSON trajectory the rendering / web-viewer pipeline can
consume.

Phases (per BRIEF v2):
    Phase A  (t = 0.00 → 0.30 s)  arm + leg spread to deployed sweep
    Phase B  (t = 0.30 → 0.40 s)  CO₂ fires; telescoping wrist + ankle tips
                                    snap out simultaneously
    Phase D  (t = 0.40 → 0.60 s)  passive — skin tensioning happens off-stage
                                    (we hold the deployed pose)
"""

from __future__ import annotations

import json
import math
from pathlib import Path

import mujoco
import numpy as np


_HERE = Path(__file__).parent
MJCF = _HERE / "deployment_mjcf.xml"


# ---------------------------------------------------------------------------
# Targets — initial (stowed) and final (deployed) joint values
# ---------------------------------------------------------------------------

# Joint targets — STOWED: arms tucked along body sides, hanging straight
# down (pitch = -90° = -1.45 rad). DEPLOYED: arms in horizontal plane,
# swept aft 25° to match the wing LE sweep.
JOINT_TARGETS_STOWED = {
    "sh_R_yaw":   0.00,            # arm hanging straight down (no sweep)
    "sh_L_yaw":   0.00,
    "sh_R_pitch": -1.40,            # pitch arms down vertically
    "sh_L_pitch": -1.40,
    "el_R":       -1.30,            # forearm folded back
    "el_L":       -1.30,
    "hip_R_yaw":  0.00,
    "hip_L_yaw":  0.00,
}
JOINT_TARGETS_DEPLOYED = {
    # Right arm: positive yaw (about +z) sweeps the +y arm direction
    # toward -x (aft). Matches LE 25° sweep.
    "sh_R_yaw":   +0.44,
    # Left arm: negative yaw sweeps the -y arm direction toward -x (aft).
    "sh_L_yaw":   -0.44,
    "sh_R_pitch": +0.00,            # arms in horizontal plane
    "sh_L_pitch": +0.00,
    "el_R":       +0.00,            # arms straight
    "el_L":       +0.00,
    # Hip yaw: right hip neg, left hip pos for outward leg spread
    "hip_R_yaw":  -0.44,
    "hip_L_yaw":  +0.44,
}

# Telescoping actuator targets — STOWED 0.0 (collapsed), DEPLOYED full extension
TELESCOPING_TARGETS_STOWED = {
    "tip1_R_act": 0.0, "tip2_R_act": 0.0, "tip3_R_act": 0.0,
    "tip1_L_act": 0.0, "tip2_L_act": 0.0, "tip3_L_act": 0.0,
    "te1_R_act":  0.0, "te2_R_act":  0.0, "te3_R_act":  0.0,
    "te1_L_act":  0.0, "te2_L_act":  0.0, "te3_L_act":  0.0,
}
TELESCOPING_TARGETS_DEPLOYED = {
    "tip1_R_act": 1.10, "tip2_R_act": 1.10, "tip3_R_act": 1.10,
    "tip1_L_act": 1.10, "tip2_L_act": 1.10, "tip3_L_act": 1.10,
    "te1_R_act":  1.20, "te2_R_act":  1.20, "te3_R_act":  1.20,
    "te1_L_act":  1.20, "te2_L_act":  1.20, "te3_L_act":  1.20,
}


def lerp(a: float, b: float, t: float) -> float:
    return a * (1 - t) + b * t


def smoothstep(t: float) -> float:
    t = max(0.0, min(1.0, t))
    return t * t * (3 - 2 * t)


def main() -> None:
    out_dir = _HERE / "out"
    out_dir.mkdir(parents=True, exist_ok=True)

    print(f"  Loading {MJCF}...")
    model = mujoco.MjModel.from_xml_path(str(MJCF))
    data = mujoco.MjData(model)

    print(f"  nq = {model.nq}, nv = {model.nv}, nu = {model.nu}, "
          f"nbody = {model.nbody}")

    # ---- set initial state to stowed ----
    # The free-joint of the torso is qpos[0..6] (3 pos + 4 quat). After
    # the freejoint, the joints are in the order they appear in the XML.
    # Set each named joint to its stowed value.
    for name, qval in JOINT_TARGETS_STOWED.items():
        jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
        qadr = model.jnt_qposadr[jid]
        data.qpos[qadr] = qval
    # Slide joints start at 0 (collapsed); ensure that
    for stage in ("tip1_R_slide", "tip2_R_slide", "tip3_R_slide",
                  "tip1_L_slide", "tip2_L_slide", "tip3_L_slide",
                  "te1_R_slide", "te2_R_slide", "te3_R_slide",
                  "te1_L_slide", "te2_L_slide", "te3_L_slide"):
        jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, stage)
        qadr = model.jnt_qposadr[jid]
        data.qpos[qadr] = 0.0

    mujoco.mj_forward(model, data)

    # ---- run the deployment ----
    duration_s = 0.6
    n_steps = int(duration_s / model.opt.timestep)
    record_every = max(1, n_steps // 60)   # ~60 frames over the run
    frames = []

    # Body names whose world poses we record per frame
    record_bodies = [
        "torso", "head_g",   # head is a geom but we'll record torso only
    ]
    body_names = [
        "torso",
        "shoulder_R", "upper_arm_R", "elbow_R", "forearm_R", "wrist_R",
        "tip1_R", "tip2_R", "tip3_R",
        "shoulder_L", "upper_arm_L", "elbow_L", "forearm_L", "wrist_L",
        "tip1_L", "tip2_L", "tip3_L",
        "hip_R", "upper_leg_R", "knee_R", "lower_leg_R",
        "hip_L", "upper_leg_L", "knee_L", "lower_leg_L",
        "te_hub", "te1_R", "te2_R", "te3_R", "te1_L", "te2_L", "te3_L",
    ]
    body_ids = [mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, n)
                for n in body_names]
    # Validate
    for n, i in zip(body_names, body_ids):
        if i < 0:
            raise RuntimeError(f"body not found: {n}")

    actuator_id = {}
    for i in range(model.nu):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, i)
        actuator_id[name] = i

    print("  Running forward simulation ...")
    for step in range(n_steps):
        t = step * model.opt.timestep

        # ---- Phase A : arm + leg spread (0.0 → 0.30 s) ----
        a_progress = smoothstep(t / 0.30)

        for jname in JOINT_TARGETS_STOWED:
            target = lerp(JOINT_TARGETS_STOWED[jname],
                           JOINT_TARGETS_DEPLOYED[jname],
                           a_progress)
            actname = f"{jname}_act"
            if actname in actuator_id:
                data.ctrl[actuator_id[actname]] = target

        # ---- Phase B : telescoping extension (0.30 → 0.50 s) ----
        # Sequenced ramp-out: each stage's commanded position interpolates
        # from 0 → full over a short window; later stages start a bit after
        # earlier ones for a sequenced visual.
        if t >= 0.30:
            phase_b_t = t - 0.30
            for i, stage_name in enumerate(["tip1", "tip2", "tip3"]):
                stage_start = i * 0.04
                stage_progress = smoothstep((phase_b_t - stage_start) / 0.10)
                target = lerp(0.0, 1.10, stage_progress)
                for side in ("R", "L"):
                    data.ctrl[actuator_id[f"{stage_name}_{side}_act"]] = target
            for i, stage_name in enumerate(["te1", "te2", "te3"]):
                stage_start = i * 0.04
                stage_progress = smoothstep((phase_b_t - stage_start) / 0.10)
                target = lerp(0.0, 1.20, stage_progress)
                for side in ("R", "L"):
                    data.ctrl[actuator_id[f"{stage_name}_{side}_act"]] = target

        mujoco.mj_step(model, data)

        if step % record_every == 0:
            frame = {"t": float(t), "bodies": {}}
            for n, i in zip(body_names, body_ids):
                pos = data.xpos[i].tolist()
                quat = data.xquat[i].tolist()  # w, x, y, z
                frame["bodies"][n] = {"pos": pos, "quat": quat}
            frame["qpos"] = data.qpos.tolist()
            frames.append(frame)

    print(f"  Captured {len(frames)} frames over {duration_s} s")

    # Final-pose snapshot (used by the renderer and the web viewer's
    # "deployed" preset)
    final = frames[-1]

    # ---- write trajectory ----
    traj_file = out_dir / "trajectory.json"
    with traj_file.open("w") as f:
        json.dump({
            "duration_s": duration_s,
            "n_frames": len(frames),
            "body_names": body_names,
            "frames": frames,
            "final_pose": final,
        }, f)
    print(f"  Wrote {traj_file} ({traj_file.stat().st_size / 1024:.1f} KB)")

    # ---- print headline numbers ----
    print()
    print("  Deployment headline numbers (from the simulation):")
    # Sample arm joint angles at start, mid, end
    for label, frame_idx in [("t=0.00", 0),
                               ("t=0.15 (mid Phase A)", len(frames) // 4),
                               ("t=0.30 (Phase A end)", len(frames) // 2),
                               ("t=0.45 (mid Phase B)", 3 * len(frames) // 4),
                               ("t=0.60 (deployed)", -1)]:
        f = frames[frame_idx]
        sh_R_yaw = f["qpos"][7]   # after free joint
        # Extract a few key joints by reading from data after re-stepping?
        # Easier: just report end-effector positions
        pos_tip3_R = f["bodies"]["tip3_R"]["pos"]
        pos_tip3_L = f["bodies"]["tip3_L"]["pos"]
        span = abs(pos_tip3_R[1]) + abs(pos_tip3_L[1])
        print(f"    {label:32s} "
              f"tip3_R_y = {pos_tip3_R[1]:+.3f} m, "
              f"tip3_L_y = {pos_tip3_L[1]:+.3f} m, "
              f"effective wingspan ≈ {span:.3f} m")


if __name__ == "__main__":
    main()
