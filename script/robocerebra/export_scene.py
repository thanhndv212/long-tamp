"""Export RoboCerebra cases as URDF/SRDF scenes and check them in HPP
(issue #157, R2).

Runs in the HPP env; the env server (the benchmark's own interpreter) is
spawned as in ``replay_demo.py``. For every case: ``load`` + ``reset``, export
the compiled MuJoCo model, then load it in HPP (the vendored Panda at the
robot's pose, the environment with its region contacts, each object as a
freeflyer) and check it against the MuJoCo state it came from:

- every exported collision shape's world pose (≤ 1 mm / 0.01 rad);
- the Panda's links at the recorded joint values (the robot model is ours,
  not robosuite's);
- pairwise collisions against MuJoCo's contacts: penetrating in HPP
  exactly when MuJoCo has a contact, up to a 1 mm band (resting contacts sit
  at about -1e-5 m in MuJoCo, so HPP sees them as touching: reported as
  ``resting``, for planning to allow or clear);
- the contact surfaces HPP registered;
- with ``--plan``: one free-space arm motion per layout (objects fixed).

::

    python script/robocerebra/export_scene.py --out /tmp/rc-scenes --plan
"""

from __future__ import annotations

import argparse
import json
import math
import sys
import tempfile
import time
from collections import defaultdict
from pathlib import Path
from typing import Any

import numpy as np

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
sys.path.insert(0, str(HERE))
from replay_demo import server_command  # noqa: E402

from long_tamp.eval import RoboCerebraClient  # noqa: E402
from long_tamp.eval.robocerebra_scene import ExportedScene, load_export  # noqa: E402

PANDA = ROOT / "script" / "twin" / "assets"
POS_TOL = 1e-3  # m
ROT_TOL = 0.01  # rad
TOUCH = 1e-3  # m: the band where contact or not is a matter of tolerance


def panda_urdf() -> str:
    text = (PANDA / "panda_bimanual.urdf.template").read_text()
    text = text.replace("{{PANDA_MESH_DIR}}", str(PANDA / "panda" / "meshes"))
    f = tempfile.NamedTemporaryFile("w", suffix=".urdf", prefix="panda_rc_", delete=False)
    f.write(text)
    f.close()
    return f.name


def se3(xyzquat: list[float]) -> Any:
    import pinocchio as pin

    x, y, z, qx, qy, qz, qw = xyzquat
    return pin.SE3(pin.Quaternion(qw, qx, qy, qz).normalized().matrix(), np.array([x, y, z]))


def pose_error(a: Any, b: Any) -> tuple[float, float]:
    import pinocchio as pin

    return (
        float(np.linalg.norm(a.translation - b.translation)),
        float(np.linalg.norm(pin.log3(a.rotation.T @ b.rotation))),
    )


# MuJoCo joint -> (Panda joint, sign); finger 2 closes along -y in robosuite.
def joint_map() -> dict[str, tuple[str, float]]:
    out = {f"robot0_joint{k}": (f"panda/panda_joint{k}", 1.0) for k in range(1, 8)}
    out["gripper0_finger_joint1"] = ("panda/panda_finger_joint1", 1.0)
    out["gripper0_finger_joint2"] = ("panda/panda_finger_joint2", -1.0)
    return out


LINK_MAP = {f"robot0_link{k}": f"panda/panda_link{k}" for k in range(8)} | {
    "gripper0_leftfinger": "panda/panda_leftfinger",
    "gripper0_rightfinger": "panda/panda_rightfinger",
}


def configuration(model: Any, scene: ExportedScene, objects_free: bool = True) -> np.ndarray:
    import pinocchio as pin

    q = pin.neutral(model)
    for mj, value in scene.robot_joints().items():
        name, sign = joint_map().get(mj, (None, 1.0))
        if name is not None and model.existJointName(name):
            q[model.idx_qs[model.getJointId(name)]] = sign * value
    if objects_free:
        for obj in scene.objects:
            jid = model.getJointId(f"{obj}/root_joint")
            i = model.idx_qs[jid]
            q[i : i + 7] = scene.object_config(obj)
    return q


def load_hpp(scene: ExportedScene, urdf_path: str):
    from long_tamp.backends.pyhpp import PyHPPBackend

    backend = PyHPPBackend()
    backend.load_robot(
        "panda", urdf_path, str(PANDA / "panda_bimanual.srdf"), "anchor",
        pose=se3(scene.robot_pose("robot0_link0")),
    )
    paths = scene.file_paths()
    env = scene.environment_name
    backend.load_environment(
        env, paths["environment"][env]["urdf"], srdf_path=paths["environment"][env]["srdf"]
    )
    for name, files in paths["objects"].items():
        backend.load_object(name, files["urdf"], files["srdf"])
    return backend


def check_case(scene: ExportedScene, urdf_path: str) -> dict[str, Any]:
    import pinocchio as pin

    backend = load_hpp(scene, urdf_path)
    device = backend.device
    model, gmodel = device.model(), device.geomModel()
    data, gdata = pin.Data(model), pin.GeometryData(gmodel)
    q = configuration(model, scene)
    pin.forwardKinematics(model, data, q)
    pin.updateFramePlacements(model, data)
    pin.updateGeometryPlacements(model, data, gmodel, gdata, q)
    out: dict[str, Any] = {}

    # Contact surfaces.
    surfaces = set(device.contactSurfaceNames())
    expected = {f"{scene.environment_name}/{r}" for r in scene.contacts}
    expected |= {f"{o}/bottom" for o, v in scene.manifest["objects"].items() if v["bottom"]}
    out["contact_surfaces"] = len(surfaces & expected)
    out["missing_surfaces"] = sorted(expected - surfaces)

    # Shape poses.
    worst = [0.0, 0.0]
    missing = []
    for name, shape in scene.manifest["shapes"].items():
        if not gmodel.existGeometryName(name):
            missing.append(name)
            continue
        dp, dr = pose_error(gdata.oMg[gmodel.getGeometryId(name)], se3(shape["pose"]))
        worst = [max(worst[0], dp), max(worst[1], dr)]
    out["shape_error"] = {"pos": worst[0], "rot": worst[1], "n": len(scene.manifest["shapes"])}
    out["missing_shapes"] = missing

    # Robot links.
    links = {}
    for mj, hpp in LINK_MAP.items():
        if mj in scene.manifest["robot"]["links"] and model.existFrame(hpp):
            dp, dr = pose_error(data.oMf[model.getFrameId(hpp)], se3(scene.robot_pose(mj)))
            links[mj] = {"pos": dp, "rot": dr}
    out["robot_links"] = links

    # Distances vs MuJoCo contacts, per geom pair across models.
    shapes = scene.manifest["shapes"]
    by_geom: dict[str, list[int]] = defaultdict(list)
    model_of: dict[str, str] = {}
    for name, shape in shapes.items():
        if gmodel.existGeometryName(name):
            by_geom[shape["geom"]].append(gmodel.getGeometryId(name))
            model_of[shape["geom"]] = name.split("/")[0]
    mj = {
        tuple(sorted((c["geom1"], c["geom2"]))): c["dist"] for c in scene.manifest["contacts"]
    }
    geoms = sorted(by_geom)
    disagree, worst_contact, resting, near = [], 0.0, 0, 0
    env = scene.environment_name
    for i, g1 in enumerate(geoms):
        for g2 in geoms[i + 1 :]:
            if model_of[g1] == model_of[g2]:
                continue
            if model_of[g1] == env and model_of[g2] == env:
                continue
            d = min(_distance(gmodel, gdata, a, b) for a in by_geom[g1] for b in by_geom[g2])
            pair = (g1, g2)
            if pair in mj:
                worst_contact = max(worst_contact, abs(d - mj[pair]))
                resting += d <= 0
                if d > TOUCH:  # MuJoCo touches, HPP sees a gap
                    disagree.append({"pair": pair, "hpp": d, "mujoco": mj[pair]})
            elif d < -TOUCH:  # HPP penetrates, MuJoCo has no contact
                disagree.append({"pair": pair, "hpp": d, "mujoco": None})
            else:
                near += d < TOUCH
    out["contacts"] = {
        "mujoco": len(mj),
        "disagree": disagree,
        "resting": resting,
        "near": near,
        "worst_contact_distance_error": worst_contact,
    }

    # The robot's clearance at the initial configuration.
    robot = [i for i, o in enumerate(gmodel.geometryObjects) if o.name.startswith("panda/")]
    others = [i for i, o in enumerate(gmodel.geometryObjects) if not o.name.startswith("panda/")]
    clearance = min(_distance(gmodel, gdata, a, b) for a in robot for b in others)
    out["robot_clearance"] = clearance
    return out


def _distance(gmodel: Any, gdata: Any, a: int, b: int) -> float:
    import hppfcl

    ta, tb = gdata.oMg[a], gdata.oMg[b]
    request, result = hppfcl.DistanceRequest(), hppfcl.DistanceResult()
    request.enable_signed_distance = True
    return float(
        hppfcl.distance(
            gmodel.geometryObjects[a].geometry,
            hppfcl.Transform3f(ta.rotation, ta.translation),
            gmodel.geometryObjects[b].geometry,
            hppfcl.Transform3f(tb.rotation, tb.translation),
            request,
            result,
        )
    )


def plan_free_motion(scene: ExportedScene, urdf_path: str, seed: int = 0) -> dict[str, Any]:
    """V2 smoke: one collision-free arm motion, objects fixed where they are."""
    from pinocchio import SE3
    from pyhpp.core import BiRRTPlanner, Problem
    from pyhpp.manipulation import Device, urdf

    device = Device("rc")
    urdf.loadModel(
        device, 0, "panda", "anchor", urdf_path, str(PANDA / "panda_bimanual.srdf"),
        se3(scene.robot_pose("robot0_link0")),
    )
    paths = scene.file_paths()
    env = scene.environment_name
    urdf.loadModel(
        device, 0, env, "anchor", paths["environment"][env]["urdf"],
        paths["environment"][env]["srdf"], SE3.Identity(),
    )
    for name, files in paths["objects"].items():
        urdf.loadModel(
            device, 0, name, "anchor", files["urdf"], files["srdf"],
            se3(scene.object_config(name)),
        )
    model = device.model()
    q0 = configuration(model, scene, objects_free=False)
    problem = Problem(device)
    problem.addConfigValidation("CollisionValidation")
    problem.addConfigValidation("JointBoundValidation")
    valid, report = problem.isConfigValid(q0)
    if not valid:
        return {"ok": False, "reason": f"initial configuration invalid: {report}"}
    # A goal: the arm swung across, drawn until valid.
    rng = np.random.default_rng(seed)
    lo, hi = model.lowerPositionLimit, model.upperPositionLimit
    for _ in range(200):
        q1 = q0.copy()
        for k in (1, 2, 4, 6):
            i = model.idx_qs[model.getJointId(f"panda/panda_joint{k}")]
            q1[i] = np.clip(q0[i] + rng.uniform(-0.8, 0.8), lo[i], hi[i])
        if problem.isConfigValid(q1)[0]:
            break
    else:
        return {"ok": False, "reason": "no valid goal drawn"}
    problem.initConfig(q0)
    problem.addGoalConfig(q1)
    planner = BiRRTPlanner(problem)
    planner.maxIterations(5000)
    start = time.monotonic()
    path = planner.solve()
    elapsed = time.monotonic() - start
    ok = path is not None
    out: dict[str, Any] = {"ok": ok, "time": elapsed}
    if ok:
        out["length"] = float(path.length())
        end = np.asarray(path.end())
        out["goal_error"] = float(np.linalg.norm(end - q1))
        # Re-check the path densely: no sample may collide.
        samples = np.linspace(0, path.length(), 50)
        out["samples_valid"] = all(
            problem.isConfigValid(np.asarray(path.eval(t)[0]))[0] for t in samples
        )
    return out


def layout(scene: ExportedScene) -> str:
    return " ".join(sorted(scene.manifest["environment"]["links"]))


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("cases", nargs="*", help="cases (default: all)")
    parser.add_argument("--env-sh", default=str(Path.home() / "devel/hpp/robocerebra/env.sh"))
    parser.add_argument("--bench", default=None)
    parser.add_argument("--out", default="/tmp/robocerebra-scenes")
    parser.add_argument("--no-export", action="store_true", help="check existing exports")
    parser.add_argument("--plan", action="store_true", help="V2: plan once per layout")
    parser.add_argument(
        "--results", default=str(HERE / "results" / "scene_export.json"), help="results JSON"
    )
    args = parser.parse_args(argv)
    out = Path(args.out)

    if not args.no_export:
        command = server_command(args.env_sh, args.bench, no_render=True)
        with RoboCerebraClient.spawn(command) as env:
            cases = args.cases or env.cases()
            for case in cases:
                env.load(case)
                env.reset(step=0)
                env.export_scene(out / case)
                print(f"exported {case}", flush=True)
    cases = args.cases or sorted(
        str(p.parent.relative_to(out)) for p in out.glob("*/*/manifest.json")
    )

    urdf_path = panda_urdf()
    results: dict[str, Any] = {"cases": {}, "layouts": {}}
    planned = set()
    for case in cases:
        scene = load_export(out / case)
        try:
            r = check_case(scene, urdf_path)
        except Exception as e:  # report and go on
            r = {"error": f"{type(e).__name__}: {e}"}
        key = layout(scene)
        results["layouts"].setdefault(key, []).append(case)
        if args.plan and key not in planned and "error" not in r:
            planned.add(key)
            try:
                r["plan"] = plan_free_motion(scene, urdf_path)
            except Exception as e:
                r["plan"] = {"ok": False, "reason": f"{type(e).__name__}: {e}"}
        r["skipped"] = scene.manifest["skipped"]
        r["regions_skipped"] = [
            x["region"] for x in scene.manifest["regions"] if "support_geom" not in x
        ]
        results["cases"][case] = r
        print(case, _line(r), flush=True)

    results["summary"] = summarize(results)
    Path(args.results).parent.mkdir(parents=True, exist_ok=True)
    Path(args.results).write_text(json.dumps(results, indent=1, default=_json))
    print(json.dumps(results["summary"], indent=1))
    return 0 if results["summary"]["pass"] else 1


def _line(r: dict[str, Any]) -> str:
    if "error" in r:
        return f"ERROR {r['error']}"
    s = r["shape_error"]
    arm = max((v["pos"] for k, v in r["robot_links"].items() if "link" in k), default=math.nan)
    plan = f" plan={r['plan']}" if "plan" in r else ""
    return (
        f"shapes {s['n']} pos {s['pos'] * 1e3:.3f}mm rot {s['rot']:.4f} | arm {arm * 1e3:.2f}mm"
        f" | surfaces {r['contact_surfaces']} missing {len(r['missing_surfaces'])}"
        f" | contacts {r['contacts']['mujoco']} (resting {r['contacts']['resting']},"
        f" near {r['contacts']['near']}) disagree {len(r['contacts']['disagree'])}"
        f" | clearance {r['robot_clearance'] * 1e3:.1f}mm{plan}"
    )


def summarize(results: dict[str, Any]) -> dict[str, Any]:
    cases = results["cases"].values()
    ok = [r for r in cases if "error" not in r]
    arm = [v for r in ok for k, v in r["robot_links"].items() if "link" in k]
    fingers = [v for r in ok for k, v in r["robot_links"].items() if "finger" in k]
    plans = [r["plan"] for r in ok if "plan" in r]
    summary = {
        "cases": len(results["cases"]),
        "layouts": len(results["layouts"]),
        "load_errors": len(results["cases"]) - len(ok),
        "shape_pos_max": max((r["shape_error"]["pos"] for r in ok), default=None),
        "shape_rot_max": max((r["shape_error"]["rot"] for r in ok), default=None),
        "missing_shapes": sum(len(r["missing_shapes"]) for r in ok),
        "missing_surfaces": sum(len(r["missing_surfaces"]) for r in ok),
        "arm_link_pos_max": max((v["pos"] for v in arm), default=None),
        "arm_link_rot_max": max((v["rot"] for v in arm), default=None),
        "finger_pos_max": max((v["pos"] for v in fingers), default=None),
        "contact_disagreements": sum(len(r["contacts"]["disagree"]) for r in ok),
        "resting_contacts": sum(r["contacts"]["resting"] for r in ok),
        "near_pairs": sum(r["contacts"]["near"] for r in ok),
        "worst_contact_distance_error": max(
            (r["contacts"]["worst_contact_distance_error"] for r in ok), default=None
        ),
        "min_robot_clearance": min((r["robot_clearance"] for r in ok), default=None),
        "plans": f"{sum(p['ok'] for p in plans)}/{len(plans)}",
    }
    summary["pass"] = bool(
        ok
        and summary["load_errors"] == 0
        and summary["missing_shapes"] == 0
        and summary["missing_surfaces"] == 0
        and summary["shape_pos_max"] <= POS_TOL
        and summary["shape_rot_max"] <= ROT_TOL
        and summary["contact_disagreements"] == 0
        and all(p["ok"] for p in plans)
    )
    return summary


def _json(x: Any) -> Any:
    if isinstance(x, (np.floating, np.integer)):
        return x.item()
    if isinstance(x, np.ndarray):
        return x.tolist()
    return str(x)


if __name__ == "__main__":
    sys.exit(main())
