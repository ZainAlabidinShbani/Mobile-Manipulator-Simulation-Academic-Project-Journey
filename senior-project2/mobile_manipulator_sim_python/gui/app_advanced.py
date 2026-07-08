"""
gui/app_advanced.py
---------------------
A second, more detailed GUI for the same simulation backend. In addition to
everything the basic GUI (`gui/app.py`) shows, this window adds:

  - A live animated 2D scene (top-down view) showing the mobile base moving,
    the arm links sweeping through their pose, the gripper opening/closing,
    the target object, and the place location -- redrawn every tick so you
    can literally watch the robot perform the task.
  - A live animated side view (x-z plane) of the arm reaching/lifting, since
    the top-down view alone hides height information.
  - Extra instrumentation: a distance-to-goal gauge, a gripper-width gauge,
    end-effector Cartesian position readout, and a mission elapsed-time /
    waypoint progress readout.
  - The same Start / Stop / Reset controls and the same joint/pose/status/
    error plots as the basic GUI, so nothing is lost -- this window is a
    superset, not a replacement.

Run with:  python main.py --advanced
"""

import sys
import numpy as np

from PyQt5 import QtWidgets, QtCore
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
import matplotlib.patches as patches

import config
from control import kinematics
from simulation.sim_env import create_simulation
from control.base_control import BaseController
from control.arm_control import ArmController
from perception.detector import ObjectDetector
from tasks.task_manager import TaskManager, TaskState
from logging_utils.data_logger import DataLogger


# ----------------------------------------------------------------------------
# Animated robot scene (top-down + side view)
# ----------------------------------------------------------------------------
class RobotScene(QtWidgets.QWidget):
    """Draws an animated, schematic top-down view and side view of the robot
    performing the task. This is a lightweight 2D rendering (not a full 3D
    physics view) so it works identically regardless of which simulation
    backend (PyBullet / Dummy) is active, and redraws fast enough for smooth
    visual feedback inside the Qt event loop."""

    BASE_LENGTH = 0.5
    BASE_WIDTH = 0.4

    def __init__(self, parent=None):
        super().__init__(parent)
        self.fig = Figure(figsize=(6, 7))
        self.canvas = FigureCanvas(self.fig)
        layout = QtWidgets.QVBoxLayout(self)
        layout.addWidget(self.canvas)

        self.ax_top = self.fig.add_subplot(2, 1, 1)
        self.ax_side = self.fig.add_subplot(2, 1, 2)
        self.fig.tight_layout()

        self.base_trail = []

    def reset(self):
        self.base_trail.clear()

    def render(self, sim_state, task_manager):
        self._render_top_down(sim_state, task_manager)
        self._render_side_view(sim_state, task_manager)
        self.fig.tight_layout()
        self.canvas.draw_idle()

    # -- top-down view --------------------------------------------------
    def _render_top_down(self, sim_state, task_manager):
        ax = self.ax_top
        ax.cla()

        bx, by, byaw = sim_state.base_pose
        self.base_trail.append((bx, by))
        if len(self.base_trail) > 2000:
            self.base_trail = self.base_trail[-2000:]

        # Base trail
        trail = np.array(self.base_trail)
        ax.plot(trail[:, 0], trail[:, 1], "--", color="gray", linewidth=0.8, alpha=0.6)

        # Mobile base rectangle (oriented by yaw)
        base_rect = patches.Rectangle(
            (-self.BASE_LENGTH / 2, -self.BASE_WIDTH / 2),
            self.BASE_LENGTH, self.BASE_WIDTH,
            facecolor="#3366cc", edgecolor="black", alpha=0.85)
        t = patches.transforms.Affine2D().rotate(byaw).translate(bx, by) + ax.transData
        base_rect.set_transform(t)
        ax.add_patch(base_rect)

        # Heading arrow
        ax.arrow(bx, by, 0.35 * np.cos(byaw), 0.35 * np.sin(byaw),
                  head_width=0.08, head_length=0.08, fc="black", ec="black")

        # Arm projected top-down: take the XY of every joint transform
        transforms = kinematics.forward_kinematics_all_links(sim_state.joint_positions, sim_state.base_pose)
        xs = [T[0, 3] for T in transforms]
        ys = [T[1, 3] for T in transforms]
        ax.plot(xs, ys, "-o", color="#cc6600", linewidth=3, markersize=4, zorder=5)

        # Gripper jaws at the end effector, opened proportional to gripper width
        ee = transforms[-1]
        gw = sim_state.gripper_width
        jaw_dx = gw / 2.0
        ee_x, ee_y = ee[0, 3], ee[1, 3]
        perp = byaw + np.pi / 2
        for sgn in (-1, 1):
            jx = ee_x + sgn * jaw_dx * np.cos(perp)
            jy = ee_y + sgn * jaw_dx * np.sin(perp)
            ax.plot([ee_x, jx], [ee_y, jy], "-", color="black", linewidth=2)

        # Object
        ox, oy, oz = sim_state.object_position
        obj_color = "#2ca02c" if sim_state.object_grasped else "#d62728"
        ax.plot(ox, oy, "o", color=obj_color, markersize=10, zorder=6)
        ax.annotate("object", (ox, oy), textcoords="offset points", xytext=(6, 6), fontsize=7)

        # Place target
        px, py, pz = task_manager.place_world_pos
        ax.plot(px, py, "x", color="purple", markersize=10, markeredgewidth=2, zorder=6)
        ax.annotate("place target", (px, py), textcoords="offset points", xytext=(6, -10), fontsize=7)

        ax.set_title(f"Top-Down View   |   state: {task_manager.state.name}")
        ax.set_xlabel("x [m]")
        ax.set_ylabel("y [m]")
        ax.set_aspect("equal", adjustable="datalim")
        ax.grid(True, linestyle=":", alpha=0.4)

    # -- side view (x-z) --------------------------------------------------
    def _render_side_view(self, sim_state, task_manager):
        ax = self.ax_side
        ax.cla()

        bx, by, byaw = sim_state.base_pose
        # Ground + base body (side silhouette)
        ax.add_patch(patches.Rectangle((-self.BASE_LENGTH / 2, 0.0), self.BASE_LENGTH, 0.18,
                                        facecolor="#3366cc", edgecolor="black", alpha=0.85))

        transforms = kinematics.forward_kinematics_all_links(sim_state.joint_positions, sim_state.base_pose)
        # Project each joint into the base's local forward/height plane
        forward_coords, heights = [], []
        for T in transforms:
            dx, dy = T[0, 3] - bx, T[1, 3] - by
            forward = dx * np.cos(byaw) + dy * np.sin(byaw)
            forward_coords.append(forward)
            heights.append(T[2, 3])
        ax.plot(forward_coords, heights, "-o", color="#cc6600", linewidth=3, markersize=4, zorder=5)

        # Object height marker (projected forward distance from base)
        ox, oy, oz = sim_state.object_position
        obj_forward = (ox - bx) * np.cos(byaw) + (oy - by) * np.sin(byaw)
        obj_color = "#2ca02c" if sim_state.object_grasped else "#d62728"
        ax.plot(obj_forward, oz, "o", color=obj_color, markersize=10, zorder=6)

        ax.set_title("Side View (forward distance vs height)")
        ax.set_xlabel("forward [m]")
        ax.set_ylabel("height [m]")
        ax.set_xlim(-0.6, 1.6)
        ax.set_ylim(-0.05, 1.0)
        ax.grid(True, linestyle=":", alpha=0.4)


# ----------------------------------------------------------------------------
# Gauges / extra instrumentation widget
# ----------------------------------------------------------------------------
class InstrumentationPanel(QtWidgets.QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        layout = QtWidgets.QFormLayout(self)

        self.ee_pos_label = QtWidgets.QLabel("(0, 0, 0)")
        self.dist_bar = QtWidgets.QProgressBar()
        self.dist_bar.setRange(0, 100)
        self.gripper_bar = QtWidgets.QProgressBar()
        self.gripper_bar.setRange(0, 100)
        self.waypoint_label = QtWidgets.QLabel("-")
        self.elapsed_label = QtWidgets.QLabel("0.0 s")
        self.grasped_label = QtWidgets.QLabel("No")

        layout.addRow("End-effector pos [m]:", self.ee_pos_label)
        layout.addRow("Distance to goal:", self.dist_bar)
        layout.addRow("Gripper opening:", self.gripper_bar)
        layout.addRow("Active waypoint:", self.waypoint_label)
        layout.addRow("Elapsed time:", self.elapsed_label)
        layout.addRow("Object grasped:", self.grasped_label)

    def update_values(self, sim_state, task_manager, sim_time):
        ee = kinematics.end_effector_position(sim_state.joint_positions, sim_state.base_pose)
        self.ee_pos_label.setText(f"({ee[0]:.2f}, {ee[1]:.2f}, {ee[2]:.2f})")

        max_err_for_bar = 1.5  # heuristic normalization range for the progress bar
        pct = int(np.clip(100 * (1.0 - task_manager.last_error / max_err_for_bar), 0, 100))
        self.dist_bar.setValue(pct)

        grip_pct = int(np.clip(100 * sim_state.gripper_width / config.GRIPPER_OPEN_WIDTH, 0, 100))
        self.gripper_bar.setValue(grip_pct)

        if task_manager.waypoints:
            idx = min(task_manager.current_waypoint_idx, len(task_manager.waypoints) - 1)
            name = task_manager.waypoints[idx][0]
            self.waypoint_label.setText(f"{idx + 1}/{len(task_manager.waypoints)}: {name}")
        else:
            self.waypoint_label.setText("-")

        self.elapsed_label.setText(f"{sim_time:.1f} s")
        self.grasped_label.setText("Yes" if sim_state.object_grasped else "No")
        self.grasped_label.setStyleSheet("color: green;" if sim_state.object_grasped else "color: red;")


# ----------------------------------------------------------------------------
# Live plots (joints / pose / status / error) - same as the basic GUI
# ----------------------------------------------------------------------------
class LivePlots(QtWidgets.QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.fig = Figure(figsize=(7, 7))
        self.canvas = FigureCanvas(self.fig)
        layout = QtWidgets.QVBoxLayout(self)
        layout.addWidget(self.canvas)

        self.ax_joints = self.fig.add_subplot(2, 2, 1)
        self.ax_pose = self.fig.add_subplot(2, 2, 2)
        self.ax_status = self.fig.add_subplot(2, 2, 3)
        self.ax_error = self.fig.add_subplot(2, 2, 4)
        self.fig.tight_layout()

        self.time_hist, self.joint_hist, self.pose_hist = [], [], []
        self.error_hist, self.state_hist = [], []

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
        self.ax_joints.legend(fontsize=6, ncol=2, loc="upper right")

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


# ----------------------------------------------------------------------------
# Main advanced window
# ----------------------------------------------------------------------------
class AdvancedMainWindow(QtWidgets.QMainWindow):
    def __init__(self, use_gui_physics: bool = True):
        super().__init__()
        self.setWindowTitle("Autonomous Mobile Manipulator - Advanced Simulation Console")
        self.resize(1600, 900)

        self.sim = create_simulation(use_gui=use_gui_physics)
        self.base_controller = BaseController()
        self.arm_controller = ArmController()
        self.detector = ObjectDetector()
        self.task_manager = TaskManager(self.base_controller, self.arm_controller, self.detector)
        self.logger = DataLogger()

        self.sim_time = 0.0
        self.running = False
        self._last_status_msg = None

        self._build_ui()

        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self._on_tick)
        self.timer.start(int(config.CONTROL_DT * 1000))

    def _build_ui(self):
        central = QtWidgets.QWidget()
        self.setCentralWidget(central)
        main_layout = QtWidgets.QHBoxLayout(central)

        # -- left: mission control + instrumentation -----------------------
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
        self.dof_label = QtWidgets.QLabel(f"Arm DOF: {config.ARM_NUM_JOINTS}")
        left_panel.addWidget(self.backend_label)
        left_panel.addWidget(self.detector_label)
        left_panel.addWidget(self.dof_label)

        left_panel.addWidget(QtWidgets.QLabel("Live instrumentation:"))
        self.instruments = InstrumentationPanel()
        left_panel.addWidget(self.instruments)

        left_panel.addWidget(QtWidgets.QLabel("Status log:"))
        self.status_box = QtWidgets.QTextEdit()
        self.status_box.setReadOnly(True)
        left_panel.addWidget(self.status_box)

        left_widget = QtWidgets.QWidget()
        left_widget.setLayout(left_panel)
        left_widget.setMaximumWidth(330)

        # -- center: animated robot scene -----------------------------------
        self.scene = RobotScene()

        # -- right: live charts -------------------------------------------
        self.plots = LivePlots()

        main_layout.addWidget(left_widget)
        main_layout.addWidget(self.scene, stretch=1)
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
        self.scene.reset()
        self.logger.close()
        self.logger = DataLogger()
        self._last_status_msg = None
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

            self.arm_controller.update(state.joint_positions, state.gripper_width, dt)

            new_state = self.sim.step(base_cmd, self.arm_controller.q_cmd,
                                       self.arm_controller.gripper_width_cmd, dt)

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
            self.instruments.update_values(state, self.task_manager, self.sim_time)

        self.scene.render(self.sim.state, self.task_manager)
        self.plots.redraw()

    def _log_status_if_changed(self):
        if self._last_status_msg != self.task_manager.status_message:
            self._last_status_msg = self.task_manager.status_message
            self._log_status(f"[{self.sim_time:6.1f}s] {self.task_manager.status_message}")

    def closeEvent(self, event):
        self.logger.close()
        self.sim.close()
        event.accept()


def run_advanced_gui(use_gui_physics: bool = True):
    app = QtWidgets.QApplication(sys.argv)
    window = AdvancedMainWindow(use_gui_physics=use_gui_physics)
    window.show()
    sys.exit(app.exec_())
