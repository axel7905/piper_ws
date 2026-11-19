import sys
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32MultiArray, String
from rclpy.time import Time
from std_srvs.srv import SetBool
from tf2_ros import Buffer, TransformException, TransformListener
from scipy.spatial.transform import Rotation

import time
import math
from PyQt5.QtCore import Qt, QTimer, QEvent
from PyQt5.QtWidgets import (
    QApplication,
    QComboBox,
    QFrame,
    QHBoxLayout,
    QLabel,
    QPushButton,
    QSlider,
    QVBoxLayout,
    QWidget,
)


JOINTS = [
    ("J1", "BASE"),
    ("J2", "SHOULDER"),
    ("J3", "ELBOW"),
    ("J4", "WRIST 1"),
    ("J5", "WRIST 2"),
    ("J6", "WRIST 3"),
]

CARTESIAN_AXES = [
    ("Z", "TRASLACIÓN"),
    ("Y", "TRASLACIÓN"),
    ("X", "TRASLACIÓN"),
    ("ROLL", "ROTACIÓN"),
    ("PITCH", "ROTACIÓN"),
    ("YAW", "ROTACIÓN"),
]


class Window(QWidget):
    def __init__(self, node):
        super().__init__()
        self.node = node

        self.setObjectName("mainWindow")
        self.setWindowTitle("PiPER Control | Simulación")
        self.setMinimumSize(760, 730)
        self.resize(880, 800)

        self.position_labels = []
        self.jog_buttons = []
        self.gripper_buttons = []
        self.axis_id_labels = []
        self.axis_name_labels = []

        self.build_ui()
        self.apply_style()
        self.positions = {}
        self.backend_status = "Esperando backend"
        
        self.command = [0.0] * 7
        self.joint_state_time = None
        self.status_time = None
        
        self.publisher = node.create_publisher(
        Float32MultiArray,
        "/piper/cmd_vel",
        1,
        )

        node.create_subscription(
            JointState,
            "/joint_states",
            self.on_joint_state,
            qos_profile_sensor_data,
        )

        node.create_subscription(
            String,
            "/piper/control_status",
            self.on_status,
            1,
        )
        
        
        # None indica que el backend todavía no ha confirmado el modo.
        self.modo_confirmado = None
        self.solicitud_modo = None
        self.reintentar_modo_desde = 0.0

        self.cliente_modo = node.create_client(
            SetBool,
            "/piper/modo_cartesiano",
        )

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(
            self.tf_buffer,
            node,
            spin_thread=False,
        )

        self.habilitar_controles(False)
        self.mode_selector.currentIndexChanged.connect(self.solicitar_modo)

        self.ros_timer = QTimer(self)
        self.ros_timer.timeout.connect(self.update_ros)
        self.ros_timer.start(40)

    def on_joint_state(self, msg):
        self.positions = dict(zip(msg.name, msg.position))
        self.joint_state_time = time.monotonic()

    def on_status(self, msg):
        self.backend_status = msg.data
        self.status_time = time.monotonic()
    
    def update_ros(self):
        if not rclpy.ok():
            self.ros_timer.stop()
            self.close()
            return

        # Atender mensajes y respuestas de ROS sin bloquear Qt.
        for _ in range(8):
            rclpy.spin_once(self.node, timeout_sec=0.0)

        ahora = time.monotonic()
        conectado = (
            self.status_time is not None
            and ahora - self.status_time < 1.0
            and self.cliente_modo.service_is_ready()
        )

        if conectado:
            self.revisar_solicitud_modo(ahora)
            if (
                self.modo_confirmado is None
                and self.solicitud_modo is None
                and ahora >= self.reintentar_modo_desde
            ):
                self.solicitar_modo(0)
        else:
            self.invalidar_modo()

        listo = (
            conectado
            and self.modo_confirmado is not None
            and self.solicitud_modo is None
            and self.joint_state_time is not None
            and ahora - self.joint_state_time < 0.5
        )

        if not listo:
            self.stop()

        self.habilitar_controles(listo)
        self.publish_command()
        self.update_display()

    def habilitar_controles(self, habilitados):
        self.mode_selector.setEnabled(habilitados)
        for button in self.jog_buttons + self.gripper_buttons:
            button.setEnabled(habilitados)

    def solicitar_modo(self, index):
        if not self.cliente_modo.service_is_ready():
            self.invalidar_modo()
            return

        self.stop()
        self.publish_command()
        self.habilitar_controles(False)

        solicitud = SetBool.Request(data=(index == 1))

        try:
            futuro = self.cliente_modo.call_async(solicitud)
        except Exception as error:
            self.node.get_logger().warning(
                f"No se pudo solicitar el modo: {error}"
            )
            self.invalidar_modo()
            self.reintentar_modo_desde = time.monotonic() + 1.0
            return

        self.solicitud_modo = (futuro, index, time.monotonic() + 2.0)

    def revisar_solicitud_modo(self, ahora):
        if self.solicitud_modo is None:
            return

        futuro, index, limite = self.solicitud_modo

        if futuro.done():
            self.solicitud_modo = None
            try:
                respuesta = futuro.result()
            except Exception as error:
                self.node.get_logger().warning(
                    f"Falló el cambio de modo: {error}"
                )
            else:
                if respuesta.success:
                    # El backend espera un comando en cero después del cambio.
                    self.stop()
                    self.publish_command()
                    self.modo_confirmado = index
                    self.actualizar_etiquetas_modo(index)
                    return

                self.node.get_logger().warning(respuesta.message)
        elif ahora < limite:
            return
        else:
            self.node.get_logger().warning(
                "El cambio de modo no respondió a tiempo."
            )

        self.invalidar_modo()
        self.reintentar_modo_desde = ahora + 1.0

    def invalidar_modo(self):
        if self.solicitud_modo is not None:
            futuro, _, _ = self.solicitud_modo
            self.cliente_modo.remove_pending_request(futuro)
            self.solicitud_modo = None

        self.modo_confirmado = None
        self.actualizar_etiquetas_modo(0)
        
    def event(self, event):
        if event.type() == QEvent.WindowDeactivate and hasattr(self, "command"):
            self.stop()
        return super().event(event)

    def closeEvent(self, event):
        self.ros_timer.stop()
        self.stop()
        if rclpy.ok():
            self.publish_command()
        super().closeEvent(event)
        
    def update_display(self):
        now = time.monotonic()

        state_is_recent = (
            self.joint_state_time is not None
            and now - self.joint_state_time < 0.5
        )
        status_is_recent = (
            self.status_time is not None
            and now - self.status_time < 1.0
        )

        if state_is_recent and self.modo_confirmado == 0:
            for index, label in enumerate(self.position_labels, start=1):
                angulo = self.positions.get(f"joint{index}")
                label.setText(
                    f"{math.degrees(angulo):+.1f}°"
                    if angulo is not None else "—"
                )
        elif state_is_recent and self.modo_confirmado == 1:
            self.actualizar_pose_tcp()
        else:
            for label in self.position_labels:
                label.setText("—")

        if state_is_recent:
            if "joint7" in self.positions and "joint8" in self.positions:
                width_m = self.positions["joint7"] - self.positions["joint8"]
                width_mm = max(0.0, min(0.072, width_m)) * 1000.0
                self.gripper_position_label.setText(f"{width_mm:.1f} mm")

        if status_is_recent:
            self.status_label.setText(self.backend_status)
            self.connection_label.setText("●  BACKEND CONECTADO")
        else:
            self.status_label.setText("Esperando mensajes de piper_control.")
            self.connection_label.setText("●  ESPERANDO BACKEND")
    
    def actualizar_pose_tcp(self):
        try:
            tf = self.tf_buffer.lookup_transform("base_link", "TCP", Time())
            posicion = tf.transform.translation
            q = tf.transform.rotation
            rpy = Rotation.from_quat(
                [q.x, q.y, q.z, q.w]
            ).as_euler("xyz", degrees=True)
        except (TransformException, ValueError):
            for label in self.position_labels:
                label.setText("—")
            return

        valores_xyz = (posicion.z, posicion.y, posicion.x)
        for label, valor in zip(self.position_labels[:3], valores_xyz):
            label.setText(f"{valor:+.4f} m")

        for label, angulo in zip(self.position_labels[3:], rpy):
            label.setText(f"{angulo:+.1f}°")
        
    def build_ui(self):
        root = QVBoxLayout(self)
        root.setContentsMargins(28, 24, 28, 22)
        root.setSpacing(16)

        # Encabezado e indicador de conexión
        header = QHBoxLayout()
        title_block = QVBoxLayout()

        title = QLabel("PiPER CONTROL")
        title.setObjectName("title")

        subtitle = QLabel("JOINT TELEOPERATION  /  GAZEBO SIMULATION")
        subtitle.setObjectName("subtitle")

        title_block.addWidget(title)
        title_block.addWidget(subtitle)
        header.addLayout(title_block)
        header.addStretch()

        self.connection_label = QLabel("●  ESPERANDO ROS 2")
        self.connection_label.setObjectName("connectionBadge")
        header.addWidget(self.connection_label, 0, Qt.AlignVCenter)

        self.status_label = QLabel("Inicia Gazebo y piper_control.")
        self.status_label.setObjectName("statusLine")
        root.addWidget(self.status_label)

        # Panel de las seis articulaciones
        joint_card = QFrame()
        joint_card.setObjectName("card")
        card_layout = QVBoxLayout(joint_card)
        card_layout.setContentsMargins(20, 18, 20, 18)
        card_layout.setSpacing(10)

        section_header = QHBoxLayout()

        self.section_title = QLabel("ARTICULACIONES")
        self.section_title.setObjectName("sectionTitle")

        count = QLabel("6 EJES")
        count.setObjectName("smallTag")

        section_header.addWidget(self.section_title)
        section_header.addStretch()

        self.mode_selector = QComboBox()
        self.mode_selector.addItems([
            "Articular",
            "Cartesiano TCP",
        ])
        section_header.addWidget(self.mode_selector)
        section_header.addWidget(count)
        card_layout.addLayout(section_header)

        for index, (joint_id, role) in enumerate(JOINTS):
            row, axis_id, axis_name, position, buttons = (
                self.create_joint_row(index, joint_id, role)
            )

            card_layout.addWidget(row)

            self.axis_id_labels.append(axis_id)
            self.axis_name_labels.append(axis_name)
            self.position_labels.append(position)
            self.jog_buttons.extend(buttons)

        root.addWidget(joint_card, 1)

        # Panel visual del gripper
        gripper_card = QFrame()
        gripper_card.setObjectName("card")
        gripper_layout = QHBoxLayout(gripper_card)
        gripper_layout.setContentsMargins(18, 12, 18, 12)
        gripper_layout.setSpacing(10)

        gripper_title = QLabel("GRIPPER")
        gripper_title.setObjectName("sectionTitle")
        gripper_layout.addWidget(gripper_title)

        self.gripper_position_label = QLabel("— mm")
        self.gripper_position_label.setObjectName("positionValue")
        gripper_layout.addWidget(self.gripper_position_label)
        gripper_layout.addStretch()

        self.gripper_close_button = QPushButton("− CERRAR")
        self.gripper_close_button.setObjectName("gripperButton")

        self.gripper_open_button = QPushButton("ABRIR +")
        self.gripper_open_button.setObjectName("gripperButton")
        
        self.gripper_close_button.pressed.connect(
           lambda: self.start_gripper(-1)
        )
        self.gripper_close_button.released.connect(self.stop)

        self.gripper_open_button.pressed.connect(
            lambda: self.start_gripper(1)
        )
        self.gripper_open_button.released.connect(self.stop)

        for button in (
            self.gripper_close_button,
            self.gripper_open_button,
        ):
            button.setMinimumSize(118, 40)
            button.setEnabled(False)
            self.gripper_buttons.append(button)
            gripper_layout.addWidget(button)

        root.addWidget(gripper_card)

        # Velocidad y parada
        controls = QHBoxLayout()

        speed_card = QFrame()
        speed_card.setObjectName("card")
        speed_layout = QVBoxLayout(speed_card)
        speed_layout.setContentsMargins(18, 14, 18, 14)

        speed_header = QHBoxLayout()

        speed_title = QLabel("VELOCIDAD DE JOG")
        speed_title.setObjectName("sectionTitle")

        self.speed_value = QLabel("30 %")
        self.speed_value.setObjectName("speedValue")

        speed_header.addWidget(speed_title)
        speed_header.addStretch()
        speed_header.addWidget(self.speed_value)
        speed_layout.addLayout(speed_header)

        self.speed_slider = QSlider(Qt.Horizontal)
        self.speed_slider.setRange(5, 100)
        self.speed_slider.setValue(30)
        self.speed_slider.valueChanged.connect(
            lambda value: self.speed_value.setText(f"{value} %")
        )
        speed_layout.addWidget(self.speed_slider)
        controls.addWidget(speed_card, 1)

        self.stop_button = QPushButton("DETENER")
        self.stop_button.setObjectName("stopButton")
        self.stop_button.setEnabled(False)
        controls.addWidget(self.stop_button)
        root.addLayout(controls)

        footer = QLabel(
            "Mantén pulsado un control para mover; suelta para detener."
        )
        footer.setObjectName("footer")
        root.addWidget(footer)
            
    def actualizar_etiquetas_modo(self, index):
        """Actualiza los nombres visibles de las seis filas."""
        anterior = self.mode_selector.blockSignals(True)
        self.mode_selector.setCurrentIndex(index)
        self.mode_selector.blockSignals(anterior)

        if index == 1:
            labels = CARTESIAN_AXES
            self.section_title.setText("TCP · POSE EN BASE / MANDO LOCAL")
        else:
            labels = JOINTS
            self.section_title.setText("ARTICULACIONES")

        for row, (axis_id, axis_name) in enumerate(labels):
            self.axis_id_labels[row].setText(axis_id)
            self.axis_name_labels[row].setText(axis_name)
        
    def start(self, index, direction):
        speed = self.speed_slider.value()
        self.command[index] = direction * float(speed)

    def start_gripper(self, direction):
        speed = self.speed_slider.value()
        self.command[6] = direction * float(speed)

    def stop(self):
        self.command = [0.0] * 7

    def publish_command(self):
        msg = Float32MultiArray()
        msg.data = self.command
        self.publisher.publish(msg)

    def create_joint_row(self, index, joint_id, role):
        row = QFrame()
        row.setObjectName("jointRow")

        layout = QHBoxLayout(row)
        layout.setContentsMargins(12, 8, 12, 8)
        layout.setSpacing(8)

        number = QLabel(joint_id)
        number.setObjectName("jointId")

        name = QLabel(role)
        name.setObjectName("jointName")

        position = QLabel("—.-°")
        position.setObjectName("positionValue")
        position.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
        
        minus = QPushButton("−")
        plus = QPushButton("+")

        minus.pressed.connect(lambda i=index: self.start(i, -1))
        minus.released.connect(self.stop)

        plus.pressed.connect(lambda i=index: self.start(i, 1))
        plus.released.connect(self.stop)

        for button in (minus, plus):
            button.setObjectName("jogButton")
            button.setFixedSize(44, 38)
            button.setEnabled(False)

        for button in (minus, plus):
            button.setObjectName("jogButton")
            button.setFixedSize(44, 38)
            button.setEnabled(False)

        layout.addWidget(number)
        layout.addWidget(name, 1)
        layout.addWidget(position, 1)
        layout.addWidget(minus)
        layout.addWidget(plus)

        return row, number, name, position, [minus, plus]

    def apply_style(self):
        self.setStyleSheet("""
      
            QWidget#mainWindow {
                background: #0b1220;
                color: #e7eef8;
            }
            QLabel {
                color: #e7eef8;
            }
            QLabel#title {
                color: #f5f8fc;
                font-size: 23px;
                font-weight: 700;
            }
            QLabel#subtitle, QLabel#footer {
                color: #8294aa;
                font-size: 11px;
            }
            QLabel#connectionBadge {
                color: #9aabc0;
                background: #172437;
                border: 1px solid #2a3a50;
                border-radius: 12px;
                padding: 8px 12px;
            }
            QLabel#statusLine, QFrame#card {
                background: #111c2c;
                border: 1px solid #243248;
                border-radius: 12px;
                padding: 8px;
            }
            QLabel#sectionTitle, QLabel#jointId {
                color: #6fd7e2;
                font-weight: 700;
            }
            QFrame#jointRow {
                background: #152235;
                border: 1px solid #202f43;
                border-radius: 8px;
            }
            QLabel#positionValue {
                color: #f1f6fc;
                font-size: 15px;
                font-weight: 700;
            }
            QLabel#speedValue {
                color: #6fd7e2;
                font-weight: 700;
            }
            QPushButton#jogButton {
                color: #e7f3fb;
                background: #1b3048;
                border: 1px solid #304861;
                border-radius: 8px;
            }
            QPushButton#jogButton:hover {
                background: #23415f;
                border-color: #4c7595;
            }
            QPushButton#jogButton:disabled,
            QPushButton#gripperButton:disabled {
                color: #53657a;
                background: #172130;
                border: 1px solid #263244;
                border-radius: 8px;
            }
            QPushButton#gripperButton {
                color: #e9fff9;
                background: #176b63;
                border: 1px solid #2a9d8f;
                border-radius: 8px;
                padding: 9px 16px;
                font-weight: 700;
            }
            QPushButton#stopButton {
                color: #ffffff;
                background: #9e3e4a;
                border: 1px solid #c35661;
                border-radius: 9px;
                padding: 13px 25px;
                font-weight: 700;
            }
            QPushButton#stopButton:disabled {
                color: #806b76;
                background: #392a35;
                border: 1px solid #4a3440;
            }
            QSlider::groove:horizontal {
                height: 6px;
                background: #26364a;
                border-radius: 3px;
            }
            QSlider::sub-page:horizontal {
                background: #12aabc;
                border-radius: 3px;
            }
            QSlider::handle:horizontal {
                background: #daf7fa;
                border: 2px solid #11aabc;
                width: 16px;
                margin: -7px 0;
                border-radius: 9px;
            }
        """)


def main(args=None):
    rclpy.init(args=args)

    app = QApplication(sys.argv[:1])
    node = Node("piper_gui")
    window = Window(node)
    window.show()

    try:
        return app.exec_()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()