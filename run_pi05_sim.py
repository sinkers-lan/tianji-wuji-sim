"""Run the single-right PI05 policy with the Isaac Sim workspace."""

import argparse
import traceback
from contextlib import ExitStack
from pathlib import Path
from time import perf_counter

SCENE = Path(__file__).resolve().with_name("workspace_pi05_pick_place.usda")
PROMPT = "用右手抓起红色苹果并放到绿色盘子上"
CONTROL_HZ = 30
HORIZON = 50
JOINTS = (
    *(f"Joint{i}_R" for i in range(1, 8)),
    *(f"right_finger{finger}_joint{joint}"
      for finger in range(1, 6) for joint in range(1, 5)),
)
CAMERAS = (
    ("observation/image", "/Workspace/Environment/HeadRGB", (720, 1280)),
    ("observation/right_wrist_image", "/Workspace/Robot/Link7_R/WristRGB", (480, 848)),
)


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", type=int, default=8000, help="Local PI05 WebSocket port")
    parser.add_argument("--steps", type=int, default=0, help="Number of 30 Hz action ticks to apply")
    parser.add_argument("--replan-steps", type=int, default=20, help="Ticks between observations")
    parser.add_argument("--timeout", type=float, default=180, help="Inference timeout in seconds")
    parser.add_argument("--prompt", default=PROMPT)
    parser.add_argument("--gui", action="store_true", help="Watch and rerun the rollout in one Isaac Sim window")
    args = parser.parse_args()
    if args.steps < 0 or not 1 <= args.replan_steps <= HORIZON:
        parser.error("--steps must be nonnegative and --replan-steps must be between 1 and 50")
    if not 1 <= args.port <= 65535 or not 0 < args.timeout < float("inf"):
        parser.error("--port and --timeout must specify a valid port and a positive finite timeout")
    return args


def run(app, socket, args):
    import carb.settings
    import omni.replicator.core as rep
    import omni.timeline
    import omni.usd
    from pxr import Sdf, Usd, UsdGeom

    # The known repository scene embeds the existing gravity compensation graph.
    carb.settings.get_settings().set_bool("/app/omni.graph.scriptnode/opt_in", True)
    context = omni.usd.get_context()
    timeline = omni.timeline.get_timeline_interface()
    timeline.stop()
    if context.get_stage() is not None:
        if rep.orchestrator.get_status() != rep.orchestrator.Status.STOPPED:
            rep.orchestrator.stop()
            rep.orchestrator.wait_until_complete()
        app.update()
        context.close_stage()
        app.update()
    # Reload while the viewport is detached so its render products stay valid.
    root_layer = Sdf.Layer.Find(str(SCENE))
    if root_layer is not None:
        root_layer.Reload()
    if not context.open_stage(str(SCENE)):
        raise RuntimeError(f"Could not open {SCENE}")
    app.update()
    stage = context.get_stage()

    with ExitStack() as cameras:
        annotators = {}
        with Usd.EditContext(stage, stage.GetSessionLayer()):
            for key, path, (height, width) in CAMERAS:
                if not stage.GetPrimAtPath(path).IsA(UsdGeom.Camera):
                    raise RuntimeError(f"Expected an existing RGB camera at {path}")
                product = rep.create.render_product(path, (width, height), force_new=True)
                cameras.callback(product.destroy)
                annotator = rep.AnnotatorRegistry.get_annotator("rgb", device="cpu")
                annotator.attach([product])
                cameras.callback(annotator.detach)
                annotators[key] = annotator

            if args.gui:
                from omni.kit.viewport.utility import get_active_viewport
                from pxr import Gf

                camera = UsdGeom.Camera.Define(stage, "/Pi05ThirdPerson")
                pose = Gf.Matrix4d().SetLookAt(
                    Gf.Vec3d(2.65, -3, 2.3), Gf.Vec3d(0, .55, .8), Gf.Vec3d(0, 0, 1)
                ).GetInverse()
                camera.AddTransformOp().Set(pose)
                get_active_viewport().camera_path = camera.GetPath()

        cameras.callback(timeline.pause)
        if args.gui:
            # Let the viewport finish opening before starting the physics timeline.
            for _ in range(20):
                app.update()
        rollout(app, socket, args, context, stage, timeline, annotators)


def rollout(app, socket, args, context, stage, timeline, annotators):
    import numpy as np
    import omni.physics.tensors as tensors
    import omni.replicator.core as rep
    from openpi_client import msgpack_numpy

    rep.orchestrator.set_capture_on_play(False)

    def step():
        if not app.is_running() or not timeline.is_playing() or context.get_stage() != stage:
            raise RuntimeError("The simulation stopped or switched stages")
        rep.orchestrator.step(delta_time=1 / CONTROL_HZ, rt_subframes=1, pause_timeline=False)

    def frames():
        images = {}
        for key, _, shape in CAMERAS:
            data = np.asarray(annotators[key].get_data())
            if not data.size:
                return None
            if data.shape not in ((*shape, 3), (*shape, 4)) or data.dtype != np.uint8:
                raise RuntimeError(f"{key}: unexpected camera data {data.shape} {data.dtype}")
            images[key] = data[:, :, :3].copy()
        return images

    timeline.play()
    app.update()
    for _ in range(100):
        if not timeline.is_playing() and app.is_running() and context.get_stage() == stage:
            timeline.play()
            app.update()
            continue
        step()
        images = frames()
        if images is not None and all(np.std(image[::16, ::16]) > 1 for image in images.values()):
            break
    else:
        raise RuntimeError("Both RGB cameras must produce nonblank images")
    print("PI05_SIM_CAMERAS", {key: value.shape for key, value in images.items()}, flush=True)

    sim = tensors.create_simulation_view("numpy", stage_id=context.get_stage_id())
    robot = sim.create_articulation_view("/Workspace/Robot/Base")
    if robot.count != 1 or robot.max_dofs != 54:
        raise RuntimeError("Expected one 54-DoF Tianji/Wuji articulation")
    names = list(robot.shared_metatype.dof_names)
    controlled = [names.index(name) for name in JOINTS]
    held = [index for index in range(len(names)) if index not in controlled]
    limits = robot.get_dof_limits()[0, controlled]
    lower, upper = limits.T
    targets = robot.get_dof_position_targets()
    actors = np.array([0], dtype=np.int32)
    applied = targets[0, controlled].copy()
    max_step = np.array([np.deg2rad(.25)] * 7 + [.015] * 20, dtype=np.float32)
    max_step[9] = .005  # F1J3 uses the smaller increment from the real deployment.
    initial = robot.get_dof_positions()[0].copy()
    packer = msgpack_numpy.Packer()

    def infer():
        obs = frames()
        if obs is None:
            raise RuntimeError("A policy camera stopped producing frames")
        state = robot.get_dof_positions()[0, controlled].astype(np.float32)
        if not np.isfinite(state).all():
            raise RuntimeError("Nonfinite simulated joint feedback")
        obs.update({"observation/state": state, "prompt": args.prompt})
        start = perf_counter()
        socket.send(packer.pack(obs))
        reply = socket.recv(timeout=args.timeout)
        if isinstance(reply, str):
            raise RuntimeError(reply)
        actions = np.asarray(msgpack_numpy.unpackb(reply)["actions"], dtype=np.float32)
        if actions.shape != (HORIZON, len(JOINTS)) or not np.isfinite(actions).all():
            raise RuntimeError(f"Invalid policy action chunk: {actions.shape}")
        print(f"PI05_SIM_INFERENCE state={state.shape} actions={actions.shape} "
              f"latency_s={perf_counter() - start:.3f}", flush=True)
        return actions

    actions = infer()
    completed = 0
    clipped = 0
    roll_start = timeline.get_current_time()
    for tick in range(args.steps):
        chunk_index = tick % args.replan_steps
        if tick and chunk_index == 0:
            actions = infer()
        raw = actions[chunk_index]
        bounded = np.clip(raw, lower, upper)
        clipped += int(np.count_nonzero(np.abs(raw - bounded) > 1e-6))
        applied = np.clip(bounded, applied - max_step, applied + max_step)
        targets[0, controlled] = applied
        robot.set_dof_position_targets(targets, actors)
        step()
        completed += 1

    delta = np.abs(robot.get_dof_positions()[0] - initial)
    elapsed = timeline.get_current_time() - roll_start
    print(f"PI05_SIM_DONE ticks={completed} sim_time_s={elapsed:.4f} "
          f"right_arm_delta_deg={np.rad2deg(delta[controlled[:7]].max()):.4f} "
          f"right_hand_delta_deg={np.rad2deg(delta[controlled[7:]].max()):.4f} "
          f"left_delta_deg={np.rad2deg(delta[held].max()):.4f} "
          f"joint_limit_clips={clipped}", flush=True)


def run_policy(app, args):
    import websockets.sync.client
    from openpi_client import msgpack_numpy

    with websockets.sync.client.connect(
        f"ws://127.0.0.1:{args.port}", open_timeout=5, compression=None, max_size=None
    ) as socket:
        metadata = msgpack_numpy.unpackb(socket.recv(timeout=5))
        if metadata.get("action_horizon") != HORIZON:
            raise RuntimeError(f"Expected a PI05 server with a {HORIZON}-frame action horizon")
        print(f"PI05_SIM_POLICY {metadata.get('checkpoint', 'unknown checkpoint')}", flush=True)
        run(app, socket, args)


def run_viewer(app, args):
    import omni.ui as ui

    pending = True

    def request_run():
        nonlocal pending
        pending = True

    window = ui.Window("PI05 Runner", width=265, height=100, dockPreference=ui.DockPreference.LEFT_BOTTOM)
    with window.frame:
        with ui.VStack(spacing=6):
            status = ui.Label("Ready to run", height=22)
            button = ui.Button("Run again", clicked_fn=request_run, height=30)

    while app.is_running():
        if pending:
            pending = False
            button.enabled = False
            status.text = "Running from the initial scene..."
            app.update()
            try:
                run_policy(app, args)
                status.text = "Finished. Run again to reset."
            except Exception as error:
                traceback.print_exc()
                status.text = "Run stopped. See the terminal log."
                print(f"PI05_SIM_RUN_STOPPED {error}", flush=True)
            finally:
                button.enabled = True
        app.update()


def main():
    args = parse_args()
    from isaacsim import SimulationApp

    app = None
    exit_code = 0
    try:
        app = SimulationApp({"headless": not args.gui})
        if args.gui:
            run_viewer(app, args)
        else:
            run_policy(app, args)
    except KeyboardInterrupt:
        exit_code = 130
    except Exception:
        traceback.print_exc()
        exit_code = 1
    finally:
        if app is not None:
            app.close(exit_code=exit_code)
    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
