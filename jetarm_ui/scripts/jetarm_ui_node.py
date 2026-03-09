#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
import os
import threading
import time
import yaml

import cv2
import numpy as np
import rospy
from PyQt5 import QtCore, QtGui, QtWidgets

from sensor_msgs.msg import Image as RosImage
from std_srvs.srv import Empty
from std_srvs.srv import SetBool, Trigger
from hiwonder_interfaces.msg import ObjectsInfo, MultiRawIdPosDur, RawIdPosDur
from astra_camera.srv import SetInt32


CONFIG_PATH = "/home/hiwonder/jetarm/src/jetarm_ui/config/ui_config.yaml"
USER_CONFIG_PATH = os.path.expanduser("~/.jetarm_ui/ui_config.user.yaml")


def _deep_merge_dict(base, override):
    if not isinstance(base, dict):
        base = {}
    if not isinstance(override, dict):
        return dict(base)
    out = dict(base)
    for k, v in override.items():
        if isinstance(v, dict) and isinstance(out.get(k), dict):
            out[k] = _deep_merge_dict(out.get(k, {}), v)
        else:
            out[k] = v
    return out


class UiConfig:
    def __init__(self, path=CONFIG_PATH, user_path=USER_CONFIG_PATH):
        self.path = path
        self.user_path = user_path
        self.data = {}
        self.load()

    def load(self):
        base_data = {}
        if os.path.exists(self.path):
            with open(self.path, "r") as f:
                base_data = yaml.safe_load(f) or {}

        user_data = {}
        if self.user_path and os.path.exists(self.user_path):
            with open(self.user_path, "r") as f:
                user_data = yaml.safe_load(f) or {}

        # User config overrides defaults so values survive rebuild/redeploy.
        self.data = _deep_merge_dict(base_data, user_data)

    def save(self):
        save_path = self.user_path or self.path
        save_dir = os.path.dirname(save_path)
        if save_dir and not os.path.exists(save_dir):
            os.makedirs(save_dir)
        with open(save_path, "w") as f:
            yaml.safe_dump(self.data, f, allow_unicode=True)

    def get(self, key, default=None):
        return self.data.get(key, default)


class RosBridge(QtCore.QObject):
    image_signal = QtCore.pyqtSignal(QtGui.QImage)
    depth_image_signal = QtCore.pyqtSignal(QtGui.QImage)
    log_signal = QtCore.pyqtSignal(str)
    objects_signal = QtCore.pyqtSignal(list)

    def __init__(self, config: UiConfig):
        super().__init__()
        self.config = config
        self.lock = threading.RLock()
        self.image_sub = None
        self.object_sub = None
        self.depth_sub = None
        self._camera_service_status = {}
        self.servos_pub = rospy.Publisher(
            "/controllers/multi_id_pos_dur", MultiRawIdPosDur, queue_size=1
        )
        self._last_objects = []

    def start(self):
        camera = self.config.get("camera", {})
        image_topic = camera.get("image_topic", "/color_detection/image_result")
        depth_topic = camera.get("depth_topic", "/rgbd_cam/depth/image_raw")
        object_topic = camera.get("object_info_topic", "/object/pixel_coords")
        self.image_sub = rospy.Subscriber(image_topic, RosImage, self._on_image, queue_size=1)
        self.depth_sub = rospy.Subscriber(depth_topic, RosImage, self._on_depth_image, queue_size=1)
        self.object_sub = rospy.Subscriber(object_topic, ObjectsInfo, self._on_objects, queue_size=1)

    def stop(self):
        if self.image_sub:
            self.image_sub.unregister()
            self.image_sub = None
        if self.depth_sub:
            self.depth_sub.unregister()
            self.depth_sub = None
        if self.object_sub:
            self.object_sub.unregister()
            self.object_sub = None

    def _on_image(self, ros_image: RosImage):
        with self.lock:
            img = np.ndarray(
                shape=(ros_image.height, ros_image.width, 3),
                dtype=np.uint8,
                buffer=ros_image.data,
            )
            bgr = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
            rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
            h, w, ch = rgb.shape
            bytes_per_line = ch * w
            qimg = QtGui.QImage(rgb.data, w, h, bytes_per_line, QtGui.QImage.Format_RGB888)
            self.image_signal.emit(qimg.copy())

    def _on_depth_image(self, ros_image: RosImage):
        with self.lock:
            depth = np.ndarray(
                shape=(ros_image.height, ros_image.width),
                dtype=np.uint16 if "16" in ros_image.encoding else np.float32,
                buffer=ros_image.data,
            )
            depth = np.nan_to_num(depth)
            depth_norm = cv2.normalize(depth, None, 0, 255, cv2.NORM_MINMAX)
            depth_u8 = depth_norm.astype(np.uint8)
            depth_color = cv2.applyColorMap(depth_u8, cv2.COLORMAP_JET)
            rgb = cv2.cvtColor(depth_color, cv2.COLOR_BGR2RGB)
            h, w, ch = rgb.shape
            bytes_per_line = ch * w
            qimg = QtGui.QImage(rgb.data, w, h, bytes_per_line, QtGui.QImage.Format_RGB888)
            self.depth_image_signal.emit(qimg.copy())

    def _on_objects(self, msg: ObjectsInfo):
        objs = []
        for obj in msg.data:
            objs.append(
                {
                    "label": obj.label,
                    "center": (obj.center.x, obj.center.y),
                    "size": (obj.size.width, obj.size.height),
                    "yaw": obj.yaw,
                }
            )
        self._last_objects = objs
        self.objects_signal.emit(objs)

    def move_servos(self, duration_ms, id_pos_list):
        motion = self.config.get("motion", {})
        speed_percent = float(motion.get("speed_percent", 100.0))
        speed_scale = max(0.1, min(speed_percent / 100.0, 2.0))
        duration_ms = int(max(50, duration_ms / speed_scale))
        msg = MultiRawIdPosDur(
            id_pos_dur_list=[
                RawIdPosDur(int(sid), int(pos), int(duration_ms)) for sid, pos in id_pos_list
            ]
        )
        self.servos_pub.publish(msg)

    def call_empty(self, srv_name):
        try:
            rospy.wait_for_service(srv_name, timeout=1.0)
            proxy = rospy.ServiceProxy(srv_name, Empty)
            proxy()
            return True
        except Exception as e:
            self.log_signal.emit("服务调用失败: %s" % str(e))
            return False

    def call_trigger(self, srv_name):
        try:
            rospy.wait_for_service(srv_name, timeout=1.0)
            proxy = rospy.ServiceProxy(srv_name, Trigger)
            proxy()
            return True
        except Exception as e:
            self.log_signal.emit("服务调用失败: %s" % str(e))
            return False

    def call_setbool(self, srv_name, value):
        try:
            rospy.wait_for_service(srv_name, timeout=1.0)
            proxy = rospy.ServiceProxy(srv_name, SetBool)
            proxy(value)
            return True
        except Exception as e:
            self.log_signal.emit("服务调用失败: %s" % str(e))
            return False

    def set_uvc_auto_exposure(self, enabled):
        camera = self.config.get("camera", {})
        ns = camera.get("service_namespace", "/rgbd_cam")
        if self._service_available(ns + "/set_uvc_auto_exposure"):
            try:
                proxy = rospy.ServiceProxy(ns + "/set_uvc_auto_exposure", SetBool)
                proxy(enabled)
                return True
            except Exception as e:
                self.log_signal.emit("曝光自动开关失败: %s" % str(e))
        return False

    def set_uvc_exposure(self, value):
        camera = self.config.get("camera", {})
        ns = camera.get("service_namespace", "/rgbd_cam")
        if self._service_available(ns + "/set_uvc_exposure"):
            try:
                proxy = rospy.ServiceProxy(ns + "/set_uvc_exposure", SetInt32)
                proxy(int(value))
                return True
            except Exception as e:
                self.log_signal.emit("曝光值设置失败: %s" % str(e))
        return False

    def set_camera_params(self, auto_exposure, exposure, auto_white_balance):
        camera = self.config.get("camera", {})
        ns = camera.get("service_namespace", "/rgbd_cam")
        ok = True
        if self._service_available(ns + "/set_uvc_auto_exposure"):
            try:
                set_auto = rospy.ServiceProxy(ns + "/set_uvc_auto_exposure", SetBool)
                set_auto(auto_exposure)
            except Exception as e:
                ok = False
                self.log_signal.emit("曝光自动开关失败: %s" % str(e))
        if self._service_available(ns + "/set_uvc_exposure"):
            try:
                set_exp = rospy.ServiceProxy(ns + "/set_uvc_exposure", SetInt32)
                set_exp(int(exposure))
            except Exception as e:
                ok = False
                self.log_signal.emit("曝光值设置失败: %s" % str(e))
        if self._service_available(ns + "/set_uvc_auto_white_balance"):
            try:
                set_wb = rospy.ServiceProxy(ns + "/set_uvc_auto_white_balance", SetBool)
                set_wb(auto_white_balance)
            except Exception as e:
                ok = False
                self.log_signal.emit("白平衡设置失败: %s" % str(e))
        return ok

    def _service_available(self, service_name):
        if service_name in self._camera_service_status:
            return self._camera_service_status[service_name]
        try:
            rospy.wait_for_service(service_name, timeout=0.2)
            self._camera_service_status[service_name] = True
        except Exception:
            self._camera_service_status[service_name] = False
            self.log_signal.emit("相机服务不可用，已跳过: %s" % service_name)
        return self._camera_service_status[service_name]


class SettingsDialog(QtWidgets.QDialog):
    def __init__(self, config: UiConfig, ros_bridge: RosBridge, parent=None):
        super().__init__(parent)
        self.config = config
        self.ros = ros_bridge
        self.servo_ids = [1, 2, 3, 4, 5, 10]
        self._last_send = {sid: 0.0 for sid in self.servo_ids}
        self.realtime_enabled = False
        self._last_exposure_send = 0.0
        self.pose_order = ["init", "scan", "grab", "grab_layer1"]
        self._build_ui()
        self._load_from_config()

    def _build_ui(self):
        layout = QtWidgets.QVBoxLayout(self)

        debug_group = QtWidgets.QGroupBox("调试模式")
        debug_layout = QtWidgets.QHBoxLayout(debug_group)
        self.debug_cb = QtWidgets.QCheckBox("开启调试模式")
        self.realtime_cb = QtWidgets.QCheckBox("启用实时控制")
        self.realtime_cb.setChecked(False)
        debug_layout.addWidget(self.debug_cb)
        debug_layout.addWidget(self.realtime_cb)
        layout.addWidget(debug_group)

        exposure_group = QtWidgets.QGroupBox("曝光(手动)")
        exposure_layout = QtWidgets.QFormLayout(exposure_group)
        self.exposure_slider = QtWidgets.QSlider(QtCore.Qt.Horizontal)
        self.exposure_slider.setRange(0, 20000)
        self.exposure_spin = QtWidgets.QSpinBox()
        self.exposure_spin.setRange(0, 20000)
        exposure_layout.addRow("曝光值", self.exposure_slider)
        exposure_layout.addRow("", self.exposure_spin)
        layout.addWidget(exposure_group)

        motion_group = QtWidgets.QGroupBox("运动速度")
        motion_layout = QtWidgets.QFormLayout(motion_group)
        self.speed_slider = QtWidgets.QSlider(QtCore.Qt.Horizontal)
        self.speed_slider.setRange(10, 200)
        self.speed_spin = QtWidgets.QSpinBox()
        self.speed_spin.setRange(10, 200)
        motion_layout.addRow("速度(%)", self.speed_slider)
        motion_layout.addRow("", self.speed_spin)
        layout.addWidget(motion_group)

        self.pose_tabs = QtWidgets.QTabWidget()
        self.pose_tables = {}
        self.pose_sliders = {}
        for key, title in zip(self.pose_order, ["初始姿态", "扫描姿态", "抓取姿态", "第一层抓取姿态"]):
            widget = QtWidgets.QWidget()
            vbox = QtWidgets.QVBoxLayout(widget)
            table = self._make_pose_table()
            vbox.addWidget(table)
            sliders, slider_group = self._make_slider_group()
            vbox.addWidget(slider_group)
            self.pose_tabs.addTab(widget, title)
            self.pose_tables[key] = table
            self.pose_sliders[key] = sliders
        layout.addWidget(self.pose_tabs)

        search_group = QtWidgets.QGroupBox("搜索参数")
        search_form = QtWidgets.QFormLayout(search_group)
        self.base_servo_id = QtWidgets.QSpinBox()
        self.base_servo_id.setRange(1, 20)
        self.left_pos = QtWidgets.QSpinBox()
        self.left_pos.setRange(0, 1000)
        self.right_pos = QtWidgets.QSpinBox()
        self.right_pos.setRange(0, 1000)
        self.search_duration = QtWidgets.QSpinBox()
        self.search_duration.setRange(100, 5000)
        self.manual_hint_only = QtWidgets.QCheckBox("只提示方向，不自动转动")
        search_form.addRow("底座舵机ID", self.base_servo_id)
        search_form.addRow("左侧位置", self.left_pos)
        search_form.addRow("右侧位置", self.right_pos)
        search_form.addRow("移动时长(ms)", self.search_duration)
        search_form.addRow(self.manual_hint_only)
        layout.addWidget(search_group)

        calib_group = QtWidgets.QGroupBox("色彩校准")
        calib_layout = QtWidgets.QHBoxLayout(calib_group)
        self.calib_btn = QtWidgets.QPushButton("进入色彩校准")
        calib_layout.addWidget(self.calib_btn)
        layout.addWidget(calib_group)

        self.save_btn = QtWidgets.QPushButton("保存舵机角度参数")
        layout.addWidget(self.save_btn)

        self.debug_cb.stateChanged.connect(self._on_debug_changed)
        self.realtime_cb.stateChanged.connect(self._on_realtime_changed)
        self.pose_tabs.currentChanged.connect(self._on_pose_tab_changed)
        self.exposure_slider.valueChanged.connect(self._on_exposure_changed)
        self.exposure_spin.valueChanged.connect(self._on_exposure_spin_changed)
        self.speed_slider.valueChanged.connect(self._on_speed_changed)
        self.speed_spin.valueChanged.connect(self._on_speed_spin_changed)
        self.calib_btn.clicked.connect(self._on_calibration)
        self.save_btn.clicked.connect(self._on_save)

    def _make_pose_table(self):
        table = QtWidgets.QTableWidget()
        table.setColumnCount(2)
        table.setHorizontalHeaderLabels(["舵机ID", "位置"])
        table.horizontalHeader().setSectionResizeMode(QtWidgets.QHeaderView.Stretch)
        table.setRowCount(6)
        for row in range(6):
            item0 = QtWidgets.QTableWidgetItem("")
            item0.setFlags(QtCore.Qt.ItemIsEnabled)
            item1 = QtWidgets.QTableWidgetItem("")
            item1.setFlags(QtCore.Qt.ItemIsEnabled)
            table.setItem(row, 0, item0)
            table.setItem(row, 1, item1)
        return table

    def _make_slider_group(self):
        group = QtWidgets.QGroupBox("舵机角度")
        grid = QtWidgets.QGridLayout(group)
        sliders = {}
        for row, sid in enumerate(self.servo_ids):
            label = QtWidgets.QLabel("ID %d" % sid)
            slider = QtWidgets.QSlider(QtCore.Qt.Horizontal)
            slider.setRange(0, 1000)
            spin = QtWidgets.QSpinBox()
            spin.setRange(0, 1000)
            spin.setSingleStep(10)
            grid.addWidget(label, row, 0)
            grid.addWidget(slider, row, 1)
            grid.addWidget(spin, row, 2)
            sliders[sid] = (slider, spin)

            slider.valueChanged.connect(
                lambda val, sid=sid, spin=spin: self._on_slider_changed(sid, val, spin)
            )
            spin.valueChanged.connect(
                lambda val, slider=slider: self._sync_spin_to_slider(slider, val)
            )

        return sliders, group

    def _sync_spin_to_slider(self, slider, val):
        if slider.value() != val:
            slider.blockSignals(True)
            slider.setValue(val)
            slider.blockSignals(False)

    def _on_slider_changed(self, sid, value, spin):
        if spin.value() != value:
            spin.blockSignals(True)
            spin.setValue(value)
            spin.blockSignals(False)
        if self.debug_cb.isChecked() and self.realtime_enabled:
            now = time.time()
            if now - self._last_send[sid] > 0.1:
                self._last_send[sid] = now
                self.ros.move_servos(200, [(sid, value)])

    def _on_exposure_changed(self, value):
        if self.exposure_spin.value() != value:
            self.exposure_spin.blockSignals(True)
            self.exposure_spin.setValue(value)
            self.exposure_spin.blockSignals(False)
        self._send_exposure(value)

    def _on_exposure_spin_changed(self, value):
        if self.exposure_slider.value() != value:
            self.exposure_slider.blockSignals(True)
            self.exposure_slider.setValue(value)
            self.exposure_slider.blockSignals(False)
        self._send_exposure(value)

    def _on_speed_changed(self, value):
        if self.speed_spin.value() != value:
            self.speed_spin.blockSignals(True)
            self.speed_spin.setValue(value)
            self.speed_spin.blockSignals(False)
        self._set_speed_percent(value)

    def _on_speed_spin_changed(self, value):
        if self.speed_slider.value() != value:
            self.speed_slider.blockSignals(True)
            self.speed_slider.setValue(value)
            self.speed_slider.blockSignals(False)
        self._set_speed_percent(value)

    def _set_speed_percent(self, value):
        motion = self.config.get("motion", {})
        motion["speed_percent"] = int(value)
        self.config.data["motion"] = motion

    def _send_exposure(self, value):
        now = time.time()
        if now - self._last_exposure_send < 0.2:
            return
        self._last_exposure_send = now
        camera = self.config.get("camera", {})
        camera["exposure"] = int(value)
        camera["auto_exposure"] = False
        self.config.data["camera"] = camera
        self.ros.set_uvc_auto_exposure(False)
        self.ros.set_uvc_exposure(value)

    def _set_pose(self, pose_key, table, pose):
        joints = pose.get("joints", [])
        table.setRowCount(max(6, len(joints)))
        joint_map = {j[0]: j[1] for j in joints if len(j) >= 2}
        for idx, item in enumerate(joints):
            table.setItem(idx, 0, QtWidgets.QTableWidgetItem(str(item[0])))
            table.setItem(idx, 1, QtWidgets.QTableWidgetItem(str(item[1])))
        sliders = self.pose_sliders.get(pose_key, {})
        for sid in self.servo_ids:
            value = int(joint_map.get(sid, 500))
            if sid in sliders:
                slider, spin = sliders[sid]
                slider.blockSignals(True)
                spin.blockSignals(True)
                slider.setValue(value)
                spin.setValue(value)
                slider.blockSignals(False)
                spin.blockSignals(False)

    def _get_pose(self, pose_key, table):
        joints = []
        sliders = self.pose_sliders.get(pose_key, {})
        for sid in self.servo_ids:
            slider, _ = sliders.get(sid, (None, None))
            if slider is None:
                continue
            joints.append([sid, int(slider.value())])
        return joints

    def _load_from_config(self):
        debug = self.config.get("debug", {})
        self.debug_cb.setChecked(bool(debug.get("enabled", False)))

        cam = self.config.get("camera", {})
        exposure = int(cam.get("exposure", 2000))
        self.exposure_slider.setValue(exposure)
        self.exposure_spin.setValue(exposure)

        motion = self.config.get("motion", {})
        speed_percent = int(motion.get("speed_percent", 100))
        self.speed_slider.setValue(speed_percent)
        self.speed_spin.setValue(speed_percent)

        poses = self.config.get("poses", {})
        for key in self.pose_order:
            self._set_pose(key, self.pose_tables[key], poses.get(key, {}))

        search = self.config.get("search", {})
        self.base_servo_id.setValue(int(search.get("base_servo_id", 1)))
        self.left_pos.setValue(int(search.get("left_pos", 875)))
        self.right_pos.setValue(int(search.get("right_pos", 125)))
        self.search_duration.setValue(int(search.get("duration_ms", 1500)))
        self.manual_hint_only.setChecked(bool(search.get("manual_hint_only", False)))
        self._set_sliders_enabled(self.debug_cb.isChecked())
        self.realtime_cb.setChecked(False)
        self.realtime_enabled = False

    def apply_to_config(self):
        self.config.data["debug"] = {"enabled": self.debug_cb.isChecked()}

        cam = self.config.get("camera", {})
        cam["exposure"] = int(self.exposure_spin.value())
        self.config.data["camera"] = cam

        motion = self.config.get("motion", {})
        motion["speed_percent"] = int(self.speed_spin.value())
        self.config.data["motion"] = motion

        poses = self.config.get("poses", {})
        for key in self.pose_order:
            poses[key] = {
                "duration_ms": poses.get(key, {}).get("duration_ms", 800),
                "joints": self._get_pose(key, self.pose_tables[key]),
            }
        self.config.data["poses"] = poses

        self.config.data["search"] = {
            "base_servo_id": int(self.base_servo_id.value()),
            "left_pos": int(self.left_pos.value()),
            "right_pos": int(self.right_pos.value()),
            "duration_ms": int(self.search_duration.value()),
            "manual_hint_only": self.manual_hint_only.isChecked(),
        }

    def _on_debug_changed(self):
        self._set_sliders_enabled(self.debug_cb.isChecked())
        if not self.debug_cb.isChecked():
            self.realtime_cb.setChecked(False)
            self.realtime_enabled = False

    def _on_pose_tab_changed(self):
        if self.debug_cb.isChecked() and self.realtime_enabled:
            self._send_current_pose()

    def _on_realtime_changed(self):
        self.realtime_enabled = self.realtime_cb.isChecked()
        if self.realtime_enabled and self.debug_cb.isChecked():
            self._send_current_pose()
        if not self.realtime_enabled:
            self._persist_debug_params()

    def _on_calibration(self):
        if self.ros.call_trigger("/lab_config_manager/enter"):
            self.ros.log_signal.emit("已进入色彩校准")
        else:
            self.ros.log_signal.emit("色彩校准进入失败")

    def _on_save(self):
        self.apply_to_config()
        self.config.save()
        self.ros.log_signal.emit("参数已保存")

    def _send_current_pose(self):
        key = self.pose_order[self.pose_tabs.currentIndex()]
        poses = self.config.get("poses", {})
        pose = poses.get(key, {})
        duration = int(pose.get("duration_ms", 800))
        joints = self._get_pose(key, self.pose_tables[key])
        if joints:
            self.ros.move_servos(duration, joints)

    def _set_sliders_enabled(self, enabled):
        for sliders in self.pose_sliders.values():
            for slider, spin in sliders.values():
                slider.setEnabled(enabled)
                spin.setEnabled(enabled)

    def closeEvent(self, event):
        self.realtime_enabled = False
        self.realtime_cb.setChecked(False)
        self._persist_debug_params()
        super().closeEvent(event)

    def _persist_debug_params(self):
        self.apply_to_config()
        self.config.save()


class MainWindow(QtWidgets.QMainWindow):
    def __init__(self, config: UiConfig, ros_bridge: RosBridge):
        super().__init__()
        self.config = config
        self.ros = ros_bridge
        self.setWindowTitle("JetArm 自定义抓取")
        self.resize(1000, 700)

        self.system_on = False
        self.state = "OFF"
        self.target_label = None
        self.search_direction = "left"
        self.search_last_time = 0.0
        self.detected_objects = []
        self.search_paused = False

        self._build_ui()
        self._load_goods()
        self._connect_signals()

        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self._tick)
        self.timer.start(200)

    def _build_ui(self):
        central = QtWidgets.QWidget()
        self.setCentralWidget(central)
        main_layout = QtWidgets.QVBoxLayout(central)

        top_bar = QtWidgets.QHBoxLayout()
        top_bar.addStretch()
        self.toggle = QtWidgets.QCheckBox("总开关")
        top_bar.addWidget(self.toggle)
        main_layout.addLayout(top_bar)

        content = QtWidgets.QHBoxLayout()
        main_layout.addLayout(content, 1)

        left_panel = QtWidgets.QVBoxLayout()
        content.addLayout(left_panel, 1)
        left_panel.addWidget(QtWidgets.QLabel("货物列表"))
        self.goods_list = QtWidgets.QListWidget()
        left_panel.addWidget(self.goods_list, 1)
        self.add_good_btn = QtWidgets.QPushButton("添加货物")
        left_panel.addWidget(self.add_good_btn)

        right_panel = QtWidgets.QVBoxLayout()
        content.addLayout(right_panel, 3)
        video_row = QtWidgets.QHBoxLayout()
        self.video_label = QtWidgets.QLabel("相机画面")
        self.video_label.setAlignment(QtCore.Qt.AlignCenter)
        self.video_label.setMinimumSize(480, 360)
        self.video_label.setStyleSheet("background-color: #222; color: #ccc;")
        self.depth_label = QtWidgets.QLabel("深度画面")
        self.depth_label.setAlignment(QtCore.Qt.AlignCenter)
        self.depth_label.setMinimumSize(480, 360)
        self.depth_label.setStyleSheet("background-color: #222; color: #ccc;")
        video_row.addWidget(self.video_label, 1)
        video_row.addWidget(self.depth_label, 1)
        right_panel.addLayout(video_row, 1)

        bottom = QtWidgets.QHBoxLayout()
        main_layout.addLayout(bottom)
        self.log_view = QtWidgets.QPlainTextEdit()
        self.log_view.setReadOnly(True)
        bottom.addWidget(self.log_view, 4)
        self.settings_btn = QtWidgets.QPushButton("参数设置")
        self.stop_btn = QtWidgets.QPushButton("停止并回初始")
        bottom.addWidget(self.settings_btn, 1)
        bottom.addWidget(self.stop_btn, 1)

    def _connect_signals(self):
        self.toggle.stateChanged.connect(self._on_toggle)
        self.add_good_btn.clicked.connect(self._on_add_good)
        self.goods_list.itemClicked.connect(self._on_select_good)
        self.settings_btn.clicked.connect(self._on_settings)
        self.stop_btn.clicked.connect(self._on_stop)
        self.ros.image_signal.connect(self._update_image)
        self.ros.depth_image_signal.connect(self._update_depth_image)
        self.ros.log_signal.connect(self._log)
        self.ros.objects_signal.connect(self._update_objects)

        self.settings_dialog = SettingsDialog(self.config, self.ros, self)
        self.settings_dialog.setModal(False)

    def _load_goods(self):
        self.goods_list.clear()
        goods = self.config.get("goods", [])
        for item in goods:
            name = item.get("name", "未知")
            label = item.get("label", "")
            w = QtWidgets.QListWidgetItem("%s (%s)" % (name, label))
            w.setData(QtCore.Qt.UserRole, item)
            self.goods_list.addItem(w)

    def _on_add_good(self):
        name, ok = QtWidgets.QInputDialog.getText(self, "添加货物", "名称")
        if not ok or not name.strip():
            return
        label, ok2 = QtWidgets.QInputDialog.getText(self, "添加货物", "识别标签")
        if not ok2 or not label.strip():
            return
        goods = self.config.get("goods", [])
        goods.append({"name": name.strip(), "label": label.strip()})
        self.config.data["goods"] = goods
        self.config.save()
        self._load_goods()

    def _on_select_good(self, item: QtWidgets.QListWidgetItem):
        data = item.data(QtCore.Qt.UserRole)
        self.target_label = data.get("label")
        if not self.system_on:
            self._log("请先打开总开关")
            return
        self.state = "SCAN"
        self.search_paused = False
        self._log("选择货物: %s" % data.get("name"))
        self._move_pose("scan")

    def _on_settings(self):
        if self.settings_dialog.isVisible():
            self.settings_dialog.hide()
            return
        main_geo = self.geometry()
        dialog_size = self.settings_dialog.sizeHint()
        new_x = main_geo.x() + main_geo.width() + 10
        new_y = main_geo.y()
        self.settings_dialog.resize(dialog_size.width(), dialog_size.height())
        self.settings_dialog.move(new_x, new_y)
        self.settings_dialog.show()

    def _on_stop(self):
        self.target_label = None
        self.search_paused = False
        self.search_direction = "left"
        self.state = "INIT"
        self._log("停止当前流程，回到初始姿态")
        self._move_pose("init")

    def _on_toggle(self, state):
        self.system_on = state == QtCore.Qt.Checked
        if self.system_on:
            self.state = "INIT"
            self._log("总开关开启，进入初始姿态")
            self._move_pose("init")
            self._start_detection()
        else:
            self.state = "OFF"
            self._log("总开关关闭")
            self._stop_detection()

    def _start_detection(self):
        self.ros.start()
        self.ros.call_empty("/color_detection/enter")
        self.ros.call_empty("/color_detection/start")

    def _stop_detection(self):
        self.ros.call_empty("/color_detection/stop")
        self.ros.call_empty("/color_detection/exit")
        self.ros.stop()

    def _move_pose(self, pose_key):
        poses = self.config.get("poses", {})
        pose = poses.get(pose_key, {})
        duration = int(pose.get("duration_ms", 800))
        joints = pose.get("joints", [])
        if joints:
            self.ros.move_servos(duration, joints)

    def _update_image(self, qimg: QtGui.QImage):
        self.video_label.setPixmap(QtGui.QPixmap.fromImage(qimg).scaled(
            self.video_label.size(), QtCore.Qt.KeepAspectRatio, QtCore.Qt.SmoothTransformation
        ))

    def _update_depth_image(self, qimg: QtGui.QImage):
        self.depth_label.setPixmap(QtGui.QPixmap.fromImage(qimg).scaled(
            self.depth_label.size(), QtCore.Qt.KeepAspectRatio, QtCore.Qt.SmoothTransformation
        ))

    def _update_objects(self, objs):
        self.detected_objects = objs

    def _log(self, msg):
        ts = time.strftime("%H:%M:%S")
        self.log_view.appendPlainText("[%s] %s" % (ts, msg))
        self.log_view.verticalScrollBar().setValue(self.log_view.verticalScrollBar().maximum())

    def _target_in_roi(self):
        if not self.target_label:
            return False, None
        roi = self.config.get("roi", {})
        x_min = int(roi.get("x_min", 0))
        y_min = int(roi.get("y_min", 0))
        x_max = int(roi.get("x_max", 0))
        y_max = int(roi.get("y_max", 0))
        for obj in self.detected_objects:
            if obj["label"] != self.target_label:
                continue
            x, y = obj["center"]
            if x_min <= x <= x_max and y_min <= y <= y_max:
                return True, obj
        return False, None

    def _target_side_hint(self):
        if not self.target_label:
            return None
        roi = self.config.get("roi", {})
        center_x = (int(roi.get("x_min", 0)) + int(roi.get("x_max", 0))) / 2.0
        for obj in self.detected_objects:
            if obj["label"] != self.target_label:
                continue
            x, _ = obj["center"]
            return "left" if x < center_x else "right"
        return None

    def _sweep_search(self):
        search = self.config.get("search", {})
        if search.get("manual_hint_only", False):
            return
        now = time.time()
        interval = float(search.get("interval_sec", 6.0))
        if now - self.search_last_time < interval:
            return
        self.search_last_time = now
        sid = int(search.get("base_servo_id", 1))
        duration = int(search.get("duration_ms", 1500))
        if self.search_direction == "left":
            pos = int(search.get("left_pos", 875))
            self._log("左移")
            self.search_direction = "right"
        else:
            pos = int(search.get("right_pos", 125))
            self._log("右移")
            self.search_direction = "left"
        self.ros.move_servos(duration, [(sid, pos)])

    def _tick(self):
        if not self.system_on:
            return
        if self.state in ("SCAN", "SEARCH"):
            in_roi, _ = self._target_in_roi()
            if in_roi:
                self._log("目标进入抓取范围")
                self.state = "GRAB"
                self._move_pose("grab")
                self._log("进入抓取姿态，等待抓取方案确认")
                return
            side = self._target_side_hint()
            if side:
                if not self.search_paused:
                    self.search_paused = True
                    if side == "left":
                        self._log("左边发现目标，请移动机械臂")
                    else:
                        self._log("右边发现目标，请移动机械臂")
            else:
                self.search_paused = False
                self.state = "SEARCH"
                self._sweep_search()


def main():
    rospy.init_node("jetarm_ui", disable_signals=True)
    config = UiConfig()
    ros_bridge = RosBridge(config)
    app = QtWidgets.QApplication(sys.argv)
    win = MainWindow(config, ros_bridge)
    win.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
