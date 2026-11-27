"""Apply the PI05 stiffness and identified payload mass properties."""
import math
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent
sys.path[:0] = [str(ROOT.parent / p) for p in (".usd-tools", ".cad-compare-deps")]

import numpy as np
from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics

STIFFNESS = [2, 2, 2, 1.6, 1, 1, 1]  # N m / degree
DAMPING_RATIOS = [0.3, 0.3, 0.3, 0.2, 0.2, 0.2, 0.2]
PAYLOADS = {"L": (0.783, (13.574, 4.605, 99.013)),
            # Right tool assembly was rotated +90 degrees around the TCP Z axis.
            "R": (0.987, (-6.692, 17.320, 66.900))}


def parallel_axis(mass, offset):
    return mass * (offset @ offset * np.eye(3) - np.outer(offset, offset))


def apply_payload(stage, side, payload_mass, flange_com_mm):
    robot = "/Workspace/Robot"
    wrist = stage.GetPrimAtPath(f"{robot}/Link7_{side}")
    hand = stage.GetPrimAtPath(f"{robot}/Wuji_{'left' if side == 'L' else 'right'}")
    cache = UsdGeom.XformCache()
    world_to_wrist = cache.GetLocalToWorldTransform(wrist).GetInverse()

    # These are the Link7-to-flange transforms in the assembly URDF.
    # Gf uses row vectors, so apply URDF roll then pitch.
    flange_rotation = Gf.Rotation(Gf.Vec3d.XAxis(), 90) * Gf.Rotation(Gf.Vec3d.YAxis(), -90)
    flange = Gf.Matrix4d(1).SetRotate(flange_rotation).SetTranslateOnly(Gf.Vec3d(0, -0.095, 0))
    payload_com = np.asarray(flange.Transform(Gf.Vec3d(*flange_com_mm) / 1000))

    hand_mass = 0.0
    hand_moment = np.zeros(3)
    for prim in Usd.PrimRange(hand):
        if not prim.HasAPI(UsdPhysics.RigidBodyAPI):
            continue
        mass_api = UsdPhysics.MassAPI(prim)
        mass = mass_api.GetMassAttr().Get()
        to_wrist = cache.GetLocalToWorldTransform(prim) * world_to_wrist
        com = to_wrist.Transform(Gf.Vec3d(mass_api.GetCenterOfMassAttr().Get()))
        hand_mass += mass
        hand_moment += mass * np.asarray(com)

    # Hand bodies already contribute to the payload. Add only the residual.
    added_mass = payload_mass - hand_mass
    assert added_mass > 0
    added_com = (payload_mass * payload_com - hand_moment) / added_mass
    api = UsdPhysics.MassAPI(wrist)
    wrist_mass = api.GetMassAttr().Get()
    wrist_com = np.asarray(api.GetCenterOfMassAttr().Get())
    total_mass = wrist_mass + added_mass
    total_com = (wrist_mass * wrist_com + added_mass * added_com) / total_mass
    principal_axes = Gf.Matrix3d(Gf.Rotation(Gf.Quatd(api.GetPrincipalAxesAttr().Get())))
    axes = np.asarray(principal_axes)
    inertia = axes.T @ np.diag(api.GetDiagonalInertiaAttr().Get()) @ axes
    # The supplied inertia triples violate the principal moment inequalities.
    # Preserve the wrist inertia and represent the residual as a point mass.
    inertia += parallel_axis(wrist_mass, wrist_com - total_com)
    inertia += parallel_axis(added_mass, added_com - total_com)
    moments, axes = np.linalg.eigh(inertia)
    if np.linalg.det(axes) < 0:
        axes[:, 0] *= -1
    basis = Gf.Matrix3d(*axes.T.flatten())
    api.GetMassAttr().Set(total_mass)
    api.GetCenterOfMassAttr().Set(Gf.Vec3f(*total_com))
    api.GetDiagonalInertiaAttr().Set(Gf.Vec3f(*moments))
    api.GetPrincipalAxesAttr().Set(Gf.Quatf(basis.ExtractRotation().GetQuat()))
    assert math.isclose(hand_mass + added_mass, payload_mass)
    assert np.allclose(hand_moment + added_mass * added_com, payload_mass * payload_com)
    print(f"{side}: existing hand {hand_mass:.4f} kg, added {added_mass:.4f} kg")


def main():
    stage = Usd.Stage.CreateNew(str(ROOT / "workspace_pi05_impedance.usda"))
    layer = stage.GetRootLayer()
    layer.subLayerPaths = ["workspace_measured_arms.usda"]
    stage.SetDefaultPrim(stage.GetPrimAtPath("/Workspace"))
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1)
    layer.documentation = (
        "PI05 stiffness and payload mass/CoM. Left arm mirrors right-arm gains. "
        "Launch with play_pi05_scene.py for damping-ratio mapping and gravity compensation."
    )
    for side, (mass, com_mm) in PAYLOADS.items():
        apply_payload(stage, side, mass, com_mm)
        for index, stiffness in enumerate(STIFFNESS, 1):
            prim = stage.GetPrimAtPath(f"/Workspace/Robot/Joint{index}_{side}")
            drive = UsdPhysics.DriveAPI(prim, "angular")
            drive.GetStiffnessAttr().Set(stiffness)
    layer.Save()
    build_autoplay_scene()


def build_autoplay_scene():
    stage = Usd.Stage.CreateNew(str(ROOT / "workspace_pi05.usda"))
    layer = stage.GetRootLayer()
    layer.subLayerPaths = ["workspace_pi05_impedance.usda"]
    stage.SetDefaultPrim(stage.GetPrimAtPath("/Workspace"))
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1)
    layer.documentation = "PI05 impedance scene with a built-in PhysX control graph."

    graph = stage.DefinePrim("/Workspace/Pi05Control", "OmniGraph")
    graph.CreateAttribute("evaluator:type", Sdf.ValueTypeNames.Token).Set("execution")
    graph.CreateAttribute("fabricCacheBacking", Sdf.ValueTypeNames.Token).Set("Shared")
    graph.CreateAttribute("fileFormatVersion", Sdf.ValueTypeNames.Int2).Set(Gf.Vec2i(1, 6))
    graph.CreateAttribute("pipelineStage", Sdf.ValueTypeNames.Token).Set("pipelineStageSimulation")

    tick = stage.DefinePrim(graph.GetPath().AppendChild("OnTick"), "OmniGraphNode")
    tick.CreateAttribute("node:type", Sdf.ValueTypeNames.Token).Set("omni.graph.action.OnTick")
    tick.CreateAttribute("node:typeVersion", Sdf.ValueTypeNames.Int).Set(1)
    tick.CreateAttribute("inputs:onlyPlayback", Sdf.ValueTypeNames.Bool).Set(False)
    pulse = tick.CreateAttribute("outputs:tick", Sdf.ValueTypeNames.UInt)

    control = stage.DefinePrim(graph.GetPath().AppendChild("Impedance"), "OmniGraphNode")
    control.CreateAttribute("node:type", Sdf.ValueTypeNames.Token).Set("omni.graph.scriptnode.ScriptNode")
    control.CreateAttribute("node:typeVersion", Sdf.ValueTypeNames.Int).Set(1)
    control.CreateAttribute("inputs:execIn", Sdf.ValueTypeNames.UInt).AddConnection(pulse.GetPath())
    control.CreateAttribute("inputs:dampingRatios", Sdf.ValueTypeNames.DoubleArray).Set(DAMPING_RATIOS)
    control.CreateAttribute("inputs:script", Sdf.ValueTypeNames.String).Set(
        (ROOT / "pi05_graph_controller.py").read_text(encoding="utf-8"))
    layer.Save()


if __name__ == "__main__":
    main()
