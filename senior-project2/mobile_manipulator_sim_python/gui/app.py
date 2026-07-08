"""
gui/app.py
-----------
PyQt5 desktop GUI for the mobile manipulator simulation:
- Start / Stop / Reset buttons driving the TaskManager state machine.
- A QTimer-driven main loop that steps simulation, control, perception, task
  logic, and logging at a fixed rate.
- Embedded Matplotlib canvases with real-time plots:
    1. Joint angles over time
    2. Base pose (x, y) trajectory in the plane
    3. Task status (current FSM state) and elapsed time
    4. Tracking error metric over time
"""

import sys
import time
import numpy as np

from PyQt5 import QtWidgets, QtCore
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure

import config
from simulation.sim_env import create_simulation
from control.base_control import BaseController
from control.arm_control import ArmController
from perception.detector import ObjectDetector
from tasks.task_manager import TaskManager, TaskState
from logging_utils.data_logger import DataLogger


class LivePlots(QtWidgets.QWidget):
    """Container widget holding the four real-time Matplotlib subplots."""

    def __init__(self, parent=None):
        super().__init__(parent)
        self.fig = Figure(figsize=(9, 7))
        self.canvas = FigureCanvas(self.fig)
        layout = QtWidgets.QVBoxLayout(self)
        layout.addWidget(self.canvas)

        self.ax_joints = self.fig.add_subplot(2, 2, 1)
        self.ax_pose = self.fig.add_subplot(2, 2, 2)
        self.ax_status = self.fig.add_subplot(2, 2, 3)
        self.ax_error = self.fig.add_subplot(2, 2, 4)
        self.fig.tight_layout()

        self.time_hist = []
        self.joint_hist = []   # list of arrays (num_joints,)
        self.pose_hist = []    # list of (x, y)
        self.error_hist = []
        self.state_hist = []

    def reset(self):
        self.time_hist.clear()
        self.joint_hist.clear()
        self.pose_hist.clear()
        self.error_hist.clear()
        self.state_hist.clear()

    def update_data(self, t, joints, pose, error, state_name):
        self.time_hist.append(t)
        self.joint_hist.append(joints.copy())
        self.pose_hist.append((pose[0], pose[1]))
        self.error_hist.append(error)
        self.state_hist.append(state_name)

        # Keep a bounded history window for plotting performance
        max_len = 600
        if len(self.time_hist) > max_len:
            self.time_hist = self.time_hist[-max_len:]
            self.joint_hist = self.joint_hist[-max_len:]
            self.pose_hist = self.pose_hist[-max_len:]
            self.error_hist = self.error_hist[-max_len:]
            self.state_hist = self.state_hist[-max_len:]

    def redraw(self):
        if not self.time_hist:
            return
        t = np.array(self.time_hist)
        joints = np.array(self.joint_hist)
        pose = np.array(self.pose_hist)
        err = np.array(self.error_hist)

        self.ax_joints.cla()
        for j in range(joints.shape[1]):
            self.ax_joints.plot(t, joints[:, j], label=f"q{j+1}")
        self.ax_joints.set_title("Joint Angles [rad]")
        self.ax_joints.set_xlabel("time [s]")
        self.ax_joints.legend(fontsize=6, ncol=3, loc="upper right")

        self.ax_pose.cla()
        self.ax_pose.plot(pose[:, 0], pose[:, 1], "-b")
        self.ax_pose.plot(pose[-1, 0], pose[-1, 1], "ro")
        self.ax_pose.set_title("Base Trajectory (x, y) [m]")
        self.ax_pose.set_xlabel("x [m]")
        self.ax_pose.set_ylabel("y [m]")
        self.ax_pose.axis("equal")

        self.ax_status.cla()
        unique_states = sorted(set(self.state_hist), key=lambda s: self.state_hist.index(s))
        state_idx_map = {s: i for i, s in enumerate(unique_states)}
        y_vals = [state_idx_map[s] for s in self.state_hist]
        self.ax_status.step(t, y_vals, where="post")
        self.ax_status.set_yticks(list(state_idx_map.values()))
        self.ax_status.set_yticklabels(list(state_idx_map.keys()), fontsize=7)
        self.ax_status.set_title("Task Status (FSM state)")
        self.ax_status.set_xlabel("time [s]")

        self.ax_error.cla()
        self.ax_error.plot(t, err, "-r")
        self.ax_error.set_title("Tracking Error Metric")
        self.ax_error.set_xlabel("time [s]")
        self.ax_error.set_ylabel("error [cm / rad]")

        self.fig.tight_layout()
        self.canvas.draw_idle()


class MainWindow(QtWidgets.QMainWindow):
    def __init__(self, use_gui_physics: bool = True):
        super().__init__()
        self.setWindowTitle("Autonomous Mobile Manipulator - Simulation Console")
        self.resize(1200, 800)

        # -- backend objects --------------------------------------------
        self.sim = create_simulation(use_gui=use_gui_physics)
        self.base_controller = BaseController()
        self.arm_controller = ArmController()
        self.detector = ObjectDetector()
        self.task_manager = TaskManager(self.base_controller, self.arm_controller, self.detector)
        self.logger = DataLogger()

        self.sim_time = 0.0
        self.running = False

        self._build_ui()

        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self._on_tick)
        self.timer.start(int(config.CONTROL_DT * 1000))

    # -- UI construction ------------------------------------------------
    def _build_ui(self):
        central = QtWidgets.QWidget()
        self.setCentralWidget(central)
        main_layout = QtWidgets.QHBoxLayout(central)

        # Left panel: controls + status
        left_panel = QtWidgets.QVBoxLayout()
        title = QtWidgets.QLabel("Mission Control")
        title.setStyleSheet("font-weight: bold; font-size: 16px;")
        left_panel.addWidget(title)

        self.btn_start = QtWidgets.QPushButton("Start")
        self.btn_stop = QtWidgets.QPushButton("Stop")
        self.btn_reset = QtWidgets.QPushButton("Reset")
        self.btn_start.clicked.connect(self._on_start)
        self.btn_stop.clicked.connect(self._on_stop)
        self.btn_reset.clicked.connect(self._on_reset)
        for b in (self.btn_start, self.btn_stop, self.btn_reset):
            left_panel.addWidget(b)

        self.backend_label = QtWidgets.QLabel(f"Simulation backend: {type(self.sim).__name__}")
        self.detector_label = QtWidgets.QLabel(f"Detector backend: {self.detector.backend_name}")
        left_panel.addWidget(self.backend_label)
        left_panel.addWidget(self.detector_label)

        self.status_box = QtWidgets.QTextEdit()
        self.status_box.setReadOnly(True)
        self.status_box.setMaximumHeight(200)
        left_panel.addWidget(QtWidgets.QLabel("Status log:"))
        left_panel.addWidget(self.status_box)

        self.info_label = QtWidgets.QLabel()
        self.info_label.setWordWrap(True)
        left_panel.addWidget(self.info_label)

        left_panel.addStretch(1)
        left_widget = QtWidgets.QWidget()
        left_widget.setLayout(left_panel)
        left_widget.setMaximumWidth(280)

        # Right panel: live plots
        self.plots = LivePlots()

        main_layout.addWidget(left_widget)
        main_layout.addWidget(self.plots, stretch=1)

    # -- button callbacks -------------------------------------------------
    def _on_start(self):
        self.running = True
        self.task_manager.start()
        self._log_status("Mission started.")

    def _on_stop(self):
        self.running = False
        self.task_manager.stop()
        self._log_status("Mission stopped by user.")

    def _on_reset(self):
        self.running = False
        self.sim.reset()
        self.task_manager.reset()
        self.arm_controller = ArmController()
        self.base_controller = BaseController()
        self.task_manager.base_controller = self.base_controller
        self.task_manager.arm_controller = self.arm_controller
        self.sim_time = 0.0
        self.plots.reset()
        self.logger.close()
        self.logger = DataLogger()
        self._log_status("Simulation reset.")

    def _log_status(self, msg: str):
        self.status_box.append(msg)

    # -- main loop ----------------------------------------------------------
    def _on_tick(self):
        dt = config.CONTROL_DT
        state = self.sim.state

        if self.running:
            cmd = self.task_manager.update(state, dt)

            base_cmd = cmd.get("base_cmd", (0.0, 0.0))
            if cmd.get("arm_target") is not None:
                self.arm_controller.set_joint_target(cmd["arm_target"])
            if cmd.get("gripper_target") is not None:
                self.arm_controller.set_gripper(cmd["gripper_target"])

            q_next, grip_next, q_vel = self.arm_controller.update(
                state.joint_positions, state.gripper_width, dt)

            new_state = self.sim.step(base_cmd, self.arm_controller.q_cmd,
                                       self.arm_controller.gripper_width_cmd, dt)

            # Handle grasp / release physically attaching the object
            if self.task_manager.state == TaskState.GRASP and not new_state.object_grasped:
                if self.arm_controller.gripper_at_target():
                    self.sim.attach_object()
            if self.task_manager.state == TaskState.PLACE and new_state.object_grasped:
                if self.arm_controller.gripper_width_cmd >= config.GRIPPER_OPEN_WIDTH * 0.9 \
                        and self.arm_controller.gripper_at_target():
                    self.sim.release_object()

            self.sim_time += dt
            self._log_status_if_changed()

            self.logger.log({
                "time": self.sim_time,
                "base_x": state.base_pose[0],
                "base_y": state.base_pose[1],
                "base_yaw": state.base_pose[2],
                **{f"joint_{i+1}": q for i, q in enumerate(state.joint_positions)},
                "gripper_width": state.gripper_width,
                "object_x": state.object_position[0],
                "object_y": state.object_position[1],
                "object_z": state.object_position[2],
                "object_grasped": state.object_grasped,
                "task_state": self.task_manager.state.name,
                "tracking_error": self.task_manager.last_error,
            })

            self.plots.update_data(self.sim_time, state.joint_positions, state.base_pose,
                                    self.task_manager.last_error, self.task_manager.state.name)
            self.info_label.setText(
                f"Task state: {self.task_manager.state.name}\n"
                f"Status: {self.task_manager.status_message}\n"
                f"Tracking error: {self.task_manager.last_error:.4f}\n"
                f"Sim time: {self.sim_time:.1f} s"
            )

        self.plots.redraw()

    def _log_status_if_changed(self):
        if not hasattr(self, "_last_status_msg") or self._last_status_msg != self.task_manager.status_message:
            self._last_status_msg = self.task_manager.status_message
            self._log_status(f"[{self.sim_time:6.1f}s] {self.task_manager.status_message}")

    def closeEvent(self, event):
        self.logger.close()
        self.sim.close()
        event.accept()


def run_gui(use_gui_physics: bool = True):
    app = QtWidgets.QApplication(sys.argv)
    window = MainWindow(use_gui_physics=use_gui_physics)
    window.show()
    sys.exit(app.exec_())
