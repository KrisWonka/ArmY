#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import os
import sys
import yaml

import rospy
from geometry_msgs.msg import TransformStamped
from tf2_msgs.msg import TFMessage
from PyQt5 import QtCore, QtWidgets


def rpy_to_quat(roll, pitch, yaw):
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy
    return qx, qy, qz, qw


class TfCalibrationTool(QtWidgets.QWidget):
    def __init__(self):
        super().__init__()
        rospy.init_node("tf_calibration_tool", anonymous=True)

        self.config_path = rospy.get_param(
            "~config_path",
            "/home/hiwonder/jetarm/src/jetarm_ui/config/tf_calibration.yaml",
        )
        self.parent_frame = rospy.get_param("~parent_frame", "link5")
        self.child_frame = rospy.get_param("~child_frame", "rgbd_cam_link")
        self.publish_hz = float(rospy.get_param("~publish_hz", 20.0))

        self.values = {
            "x": 0.0,
            "y": 0.0,
            "z": 0.0,
            "roll": 0.0,
            "pitch": -1.57,
            "yaw": 0.0,
        }
        self.grasp_offset = {"x": -0.03, "y": 0.0, "z": 0.0}

        self._build_ui()
        self._load_config()
        self._apply_to_widgets()

        self.tf_pub = rospy.Publisher("/tf", TFMessage, queue_size=10)
        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self._publish_tf)
        self.timer.start(max(10, int(1000.0 / self.publish_hz)))

    def _build_ui(self):
        self.setWindowTitle("TF Calibration Tool")
        self.resize(760, 420)
        main = QtWidgets.QVBoxLayout(self)

        frame_group = QtWidgets.QGroupBox("Frame")
        frame_layout = QtWidgets.QFormLayout(frame_group)
        self.parent_edit = QtWidgets.QLineEdit(self.parent_frame)
        self.child_edit = QtWidgets.QLineEdit(self.child_frame)
        frame_layout.addRow("Parent Frame", self.parent_edit)
        frame_layout.addRow("Child Frame", self.child_edit)
        main.addWidget(frame_group)

        self.controls = {}
        params = [
            ("x", -0.50, 0.50, 0.001),
            ("y", -0.50, 0.50, 0.001),
            ("z", -0.50, 0.50, 0.001),
            ("roll", -3.14, 3.14, 0.01),
            ("pitch", -3.14, 3.14, 0.01),
            ("yaw", -3.14, 3.14, 0.01),
        ]

        grid = QtWidgets.QGridLayout()
        grid.addWidget(QtWidgets.QLabel("Name"), 0, 0)
        grid.addWidget(QtWidgets.QLabel("Slider"), 0, 1)
        grid.addWidget(QtWidgets.QLabel("Value"), 0, 2)

        for row, (name, vmin, vmax, step) in enumerate(params, start=1):
            label = QtWidgets.QLabel(name)
            slider = QtWidgets.QSlider(QtCore.Qt.Horizontal)
            spin = QtWidgets.QDoubleSpinBox()
            spin.setDecimals(3 if step < 0.01 else 2)
            spin.setRange(vmin, vmax)
            spin.setSingleStep(step)
            slider.setRange(0, int(round((vmax - vmin) / step)))

            def on_slider(val, key=name, mn=vmin, st=step, sp=spin):
                real = mn + val * st
                sp.blockSignals(True)
                sp.setValue(real)
                sp.blockSignals(False)
                self.values[key] = float(real)
                self._update_preview()

            def on_spin(val, key=name, mn=vmin, st=step, sl=slider):
                idx = int(round((val - mn) / st))
                sl.blockSignals(True)
                sl.setValue(idx)
                sl.blockSignals(False)
                self.values[key] = float(val)
                self._update_preview()

            slider.valueChanged.connect(on_slider)
            spin.valueChanged.connect(on_spin)
            grid.addWidget(label, row, 0)
            grid.addWidget(slider, row, 1)
            grid.addWidget(spin, row, 2)
            self.controls[name] = (slider, spin, vmin, step)

        main.addLayout(grid)

        grasp_group = QtWidgets.QGroupBox("Grasp Offset (base_link)")
        grasp_grid = QtWidgets.QGridLayout(grasp_group)
        grasp_grid.addWidget(QtWidgets.QLabel("Name"), 0, 0)
        grasp_grid.addWidget(QtWidgets.QLabel("Slider"), 0, 1)
        grasp_grid.addWidget(QtWidgets.QLabel("Value"), 0, 2)
        self.grasp_controls = {}
        grasp_params = [
            ("x", -0.10, 0.10, 0.001),
            ("y", -0.10, 0.10, 0.001),
            ("z", -0.10, 0.10, 0.001),
        ]
        for row, (name, vmin, vmax, step) in enumerate(grasp_params, start=1):
            label = QtWidgets.QLabel(name)
            slider = QtWidgets.QSlider(QtCore.Qt.Horizontal)
            spin = QtWidgets.QDoubleSpinBox()
            spin.setDecimals(3)
            spin.setRange(vmin, vmax)
            spin.setSingleStep(step)
            slider.setRange(0, int(round((vmax - vmin) / step)))

            def on_slider(val, key=name, mn=vmin, st=step, sp=spin):
                real = mn + val * st
                sp.blockSignals(True)
                sp.setValue(real)
                sp.blockSignals(False)
                self.grasp_offset[key] = float(real)
                self._update_preview()

            def on_spin(val, key=name, mn=vmin, st=step, sl=slider):
                idx = int(round((val - mn) / st))
                sl.blockSignals(True)
                sl.setValue(idx)
                sl.blockSignals(False)
                self.grasp_offset[key] = float(val)
                self._update_preview()

            slider.valueChanged.connect(on_slider)
            spin.valueChanged.connect(on_spin)
            grasp_grid.addWidget(label, row, 0)
            grasp_grid.addWidget(slider, row, 1)
            grasp_grid.addWidget(spin, row, 2)
            self.grasp_controls[name] = (slider, spin, vmin, step)
        main.addWidget(grasp_group)

        self.preview = QtWidgets.QPlainTextEdit()
        self.preview.setReadOnly(True)
        main.addWidget(self.preview, 1)

        btn_row = QtWidgets.QHBoxLayout()
        self.save_btn = QtWidgets.QPushButton("保存")
        self.reload_btn = QtWidgets.QPushButton("重新加载")
        btn_row.addWidget(self.save_btn)
        btn_row.addWidget(self.reload_btn)
        btn_row.addStretch()
        main.addLayout(btn_row)

        self.save_btn.clicked.connect(self._save_config)
        self.reload_btn.clicked.connect(self._reload_from_disk)
        self.parent_edit.textChanged.connect(self._update_preview)
        self.child_edit.textChanged.connect(self._update_preview)

    def _apply_to_widgets(self):
        for name, (slider, spin, vmin, step) in self.controls.items():
            val = float(self.values.get(name, 0.0))
            spin.blockSignals(True)
            slider.blockSignals(True)
            spin.setValue(val)
            slider.setValue(int(round((val - vmin) / step)))
            slider.blockSignals(False)
            spin.blockSignals(False)
        for name, (slider, spin, vmin, step) in self.grasp_controls.items():
            val = float(self.grasp_offset.get(name, 0.0))
            spin.blockSignals(True)
            slider.blockSignals(True)
            spin.setValue(val)
            slider.setValue(int(round((val - vmin) / step)))
            slider.blockSignals(False)
            spin.blockSignals(False)
        self.parent_edit.setText(self.parent_frame)
        self.child_edit.setText(self.child_frame)
        self._update_preview()

    def _reload_from_disk(self):
        self._load_config()
        self._apply_to_widgets()

    def _load_config(self):
        if not os.path.exists(self.config_path):
            return
        try:
            with open(self.config_path, "r") as f:
                cfg = yaml.safe_load(f) or {}
            self.parent_frame = cfg.get("parent_frame", self.parent_frame)
            self.child_frame = cfg.get("child_frame", self.child_frame)
            tfv = cfg.get("transform", {})
            for key in self.values:
                if key in tfv:
                    self.values[key] = float(tfv[key])
            go = cfg.get("grasp_offset", {})
            for key in self.grasp_offset:
                if key in go:
                    self.grasp_offset[key] = float(go[key])
        except Exception as exc:
            rospy.logwarn("tf_calibration_tool: load failed: %s", str(exc))

    def _save_config(self):
        self.parent_frame = self.parent_edit.text().strip() or self.parent_frame
        self.child_frame = self.child_edit.text().strip() or self.child_frame
        data = {
            "parent_frame": self.parent_frame,
            "child_frame": self.child_frame,
            "transform": {k: float(v) for k, v in self.values.items()},
            "grasp_offset": {k: float(v) for k, v in self.grasp_offset.items()},
        }
        cfg_dir = os.path.dirname(self.config_path)
        if cfg_dir and not os.path.exists(cfg_dir):
            os.makedirs(cfg_dir)
        with open(self.config_path, "w") as f:
            yaml.safe_dump(data, f, allow_unicode=True, sort_keys=False)
        rospy.loginfo("tf_calibration_tool: saved %s", self.config_path)
        self._update_preview()

    def _update_preview(self):
        p = self.parent_edit.text().strip() or self.parent_frame
        c = self.child_edit.text().strip() or self.child_frame
        x = self.values["x"]
        y = self.values["y"]
        z = self.values["z"]
        r = self.values["roll"]
        pit = self.values["pitch"]
        yw = self.values["yaw"]
        gox = self.grasp_offset["x"]
        goy = self.grasp_offset["y"]
        goz = self.grasp_offset["z"]
        cmd = (
            "roslaunch jetarm_ui jetarm_ui.launch "
            'base_to_camera_xyz:="{:.3f} {:.3f} {:.3f}" '
            'base_to_camera_rpy:="{:.3f} {:.3f} {:.3f}"'
        ).format(x, y, z, r, pit, yw)
        self.preview.setPlainText(
            "实时发布: {} -> {}\n"
            "xyz: {:.3f} {:.3f} {:.3f}\n"
            "rpy: {:.3f} {:.3f} {:.3f}\n\n"
            "grasp_offset: {:.3f} {:.3f} {:.3f}\n\n"
            "可复用 launch 参数:\n{}\n\n"
            "配置文件: {}\n".format(
                p, c, x, y, z, r, pit, yw, gox, goy, goz, cmd, self.config_path
            )
        )

    def _publish_tf(self):
        parent = self.parent_edit.text().strip() or self.parent_frame
        child = self.child_edit.text().strip() or self.child_frame
        t = TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = parent
        t.child_frame_id = child
        t.transform.translation.x = float(self.values["x"])
        t.transform.translation.y = float(self.values["y"])
        t.transform.translation.z = float(self.values["z"])
        qx, qy, qz, qw = rpy_to_quat(
            float(self.values["roll"]),
            float(self.values["pitch"]),
            float(self.values["yaw"]),
        )
        t.transform.rotation.x = qx
        t.transform.rotation.y = qy
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw
        self.tf_pub.publish(TFMessage([t]))


def main():
    app = QtWidgets.QApplication(sys.argv)
    win = TfCalibrationTool()
    win.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
