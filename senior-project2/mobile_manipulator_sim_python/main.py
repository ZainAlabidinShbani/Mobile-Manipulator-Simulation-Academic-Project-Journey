"""
main.py
--------
Entry point for the Autonomous Mobile Manipulator simulation.

Usage:
    python main.py               # Launch full PyQt5 GUI application
    python main.py --headless    # Run the task in console-only mode (no GUI),
                                  # useful for servers / automated testing.
    python main.py --no-physics-gui   # Use GUI app but without PyBullet's own
                                       # debug render window (faster).
"""

import argparse
import time

import config
from simulation.sim_env import create_simulation
from control.base_control import BaseController
from control.arm_control import ArmController
from perception.detector import ObjectDetector
from tasks.task_manager import TaskManager, TaskState
from logging_utils.data_logger import DataLogger


def run_headless(max_seconds: float = 60.0):
    """Run the full perception->planning->control->task pipeline without any
    GUI, printing status transitions to the console and logging to CSV."""
    sim = create_simulation(use_gui=False)
    base_controller = BaseController()
    arm_controller = ArmController()
    detector = ObjectDetector()
    task_manager = TaskManager(base_controller, arm_controller, detector)
    logger = DataLogger()

    print(f"[main] Simulation backend: {type(sim).__name__}")
    print(f"[main] Detector backend: {detector.backend_name}")

    task_manager.start()
    dt = config.CONTROL_DT
    sim_time = 0.0
    last_status = None

    try:
        while sim_time < max_seconds and task_manager.state != TaskState.DONE:
            state = sim.state
            cmd = task_manager.update(state, dt)

            base_cmd = cmd.get("base_cmd", (0.0, 0.0))
            if cmd.get("arm_target") is not None:
                arm_controller.set_joint_target(cmd["arm_target"])
            if cmd.get("gripper_target") is not None:
                arm_controller.set_gripper(cmd["gripper_target"])

            arm_controller.update(state.joint_positions, state.gripper_width, dt)
            new_state = sim.step(base_cmd, arm_controller.q_cmd,
                                  arm_controller.gripper_width_cmd, dt)

            if task_manager.state == TaskState.GRASP and not new_state.object_grasped:
                if arm_controller.gripper_at_target():
                    sim.attach_object()
            if task_manager.state == TaskState.PLACE and new_state.object_grasped:
                if arm_controller.gripper_width_cmd >= config.GRIPPER_OPEN_WIDTH * 0.9 \
                        and arm_controller.gripper_at_target():
                    sim.release_object()

            if task_manager.status_message != last_status:
                last_status = task_manager.status_message
                print(f"[{sim_time:6.1f}s] ({task_manager.state.name}) {last_status}")

            logger.log({
                "time": sim_time,
                "base_x": state.base_pose[0],
                "base_y": state.base_pose[1],
                "base_yaw": state.base_pose[2],
                **{f"joint_{i+1}": q for i, q in enumerate(state.joint_positions)},
                "gripper_width": state.gripper_width,
                "task_state": task_manager.state.name,
                "tracking_error": task_manager.last_error,
            })

            sim_time += dt
            time.sleep(0.0)  # no real-time pacing needed in headless mode

    finally:
        logger.close()
        sim.close()
        print(f"[main] Run finished. Log written to: {logger.filepath}")


def run_gui_app(use_gui_physics: bool = True):
    from gui.app import run_gui
    run_gui(use_gui_physics=use_gui_physics)


def run_advanced_gui_app(use_gui_physics: bool = True):
    from gui.app_advanced import run_advanced_gui
    run_advanced_gui(use_gui_physics=use_gui_physics)


def main():
    parser = argparse.ArgumentParser(description="Autonomous Mobile Manipulator Simulation")
    parser.add_argument("--headless", action="store_true",
                         help="Run without GUI (console mode).")
    parser.add_argument("--advanced", action="store_true",
                         help="Launch the advanced GUI with an animated robot scene "
                              "and extra instrumentation, instead of the basic GUI.")
    parser.add_argument("--no-physics-gui", action="store_true",
                         help="Disable PyBullet's own debug render window.")
    parser.add_argument("--duration", type=float, default=60.0,
                         help="Max duration in seconds for headless mode.")
    args = parser.parse_args()

    if args.headless:
        run_headless(max_seconds=args.duration)
    elif args.advanced:
        run_advanced_gui_app(use_gui_physics=not args.no_physics_gui)
    else:
        run_gui_app(use_gui_physics=not args.no_physics_gui)


if __name__ == "__main__":
    main()
