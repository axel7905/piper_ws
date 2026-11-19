"""Normalized joint commands -> bounded Gazebo trajectories, like the real bridge."""
import math
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32MultiArray, String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from pathlib import Path

from std_srvs.srv import Trigger, SetBool 
from .trac_ik_solver import PiperTracIkSolver
from .geometria_cartesiana import (
    incrementar_tcp,
    link6_a_tcp,
    tcp_a_link6,
)
class PiperControl(Node):
    JOINTS = [f'joint{i}' for i in range(1, 7)]
    GRIPPER_JOINTS = ['joint8', 'joint7']

    def __init__(self):
        super().__init__('piper_control')
        defaults = dict(command_topic='/piper/cmd_vel', state_topic='/joint_states',
            trajectory_topic='/arm_controller/joint_trajectory', status_topic='/piper/control_status',
            gripper_trajectory_topic='/gripper_controller/joint_trajectory',
            gripper_max_velocity=0.08, gripper_max_width=0.072,
            publish_rate=30.0, command_timeout=0.3, state_timeout=0.5,
            trajectory_duration=0.12, max_tracking_error=0.15,
            arm_max_velocity=[0.35]*6,
            arm_lower_limits=[-2.618, 0.0, -2.967, -1.745, -1.22, -2.0944],
            arm_upper_limits=[2.618, 3.14, 0.0, 1.745, 1.22, 2.0944])
        for key, value in defaults.items():
            self.declare_parameter(key, value)
            setattr(self, key, self.get_parameter(key).value)
        for key in ('publish_rate','command_timeout','state_timeout','trajectory_duration',
                    'gripper_max_velocity','gripper_max_width','max_tracking_error'):
            if not math.isfinite(getattr(self,key)) or getattr(self,key)<=0:
                raise ValueError(f'{key} must be finite and positive')
        for values in (self.arm_max_velocity,self.arm_lower_limits,self.arm_upper_limits):
            if len(values)!=6 or not all(math.isfinite(v) for v in values):
                raise ValueError('Joint parameter arrays must contain six finite numbers')
        if any(v<=0 for v in self.arm_max_velocity) or any(a>=b for a,b in zip(self.arm_lower_limits,self.arm_upper_limits)):
            raise ValueError('Invalid joint velocities or limits')
        if self.gripper_max_width>0.072:
            raise ValueError('Gripper width cannot exceed the model travel of 0.072 m')
        
        self.command=[0.0]*6
        self.gripper_command=0.0
        self.measured=None
        self.target=None
        self.gripper_measured=None
        self.gripper_target=None
        self.command_stamp=0.0
        self.state_stamp=0.0
        self.gripper_state_stamp=0.0
        self.last_tick=time.monotonic()
        self.moving=False
        self.gripper_moving=False
        self.last_status=None
                # El nodo inicia en modo articular.
        self.modo_cartesiano = False

        # Velocidades máximas del TCP al recibir un comando de 100 %.
        self.velocidad_lineal_tcp = 0.02      # metros por segundo
        self.velocidad_angular_tcp = 0.10     # radianes por segundo

        # Después de cambiar de modo, esperar un comando en cero.
        self.esperando_comando_cero = False
        
        self.pub=self.create_publisher(JointTrajectory,self.trajectory_topic,1)
        self.gripper_pub=self.create_publisher(JointTrajectory,self.gripper_trajectory_topic,1)
        self.status=self.create_publisher(String,self.status_topic,1)
        
        self.create_subscription(Float32MultiArray,self.command_topic,self.on_command,1)
        self.create_subscription(JointState,self.state_topic,self.on_state,qos_profile_sensor_data)
        
        urdf_path = Path.home() / "piper_ws/modelos/piper_gazebo.urdf"

        self.solver_cartesiano = PiperTracIkSolver(urdf_path)

        self.servicio_paso_cartesiano = self.create_service(
            Trigger,
            "/piper/paso_cartesiano_x",
            self.paso_cartesiano_x,
        )
        
        self.servicio_modo_cartesiano = self.create_service(
            SetBool,
            "/piper/modo_cartesiano",
            self.cambiar_modo_cartesiano,
        )
        
        self.create_timer(1.0/self.publish_rate,self.tick)

    def on_state(self,msg):
        positions=dict(zip(msg.name,msg.position))
        now=time.monotonic()
        if all(n in positions and math.isfinite(positions[n]) for n in self.JOINTS):
            self.measured=[positions[n] for n in self.JOINTS]
            self.state_stamp=now
        if all(n in positions and math.isfinite(positions[n]) for n in ('joint7','joint8')):
            width=positions['joint7']-positions['joint8']
            self.gripper_measured=max(0.0,min(self.gripper_max_width,width))
            self.gripper_state_stamp=now

    def on_command(self,msg):
        valid=len(msg.data) in (6,7) and all(math.isfinite(v) and abs(v)<=100 for v in msg.data)
        if not valid:
            self.command=[0.0]*6
            self.gripper_command=0.0
            self.command_stamp=0.0
            self.get_logger().warning('Expected six arm values and an optional gripper value in [-100,100]')
            return
        
        valores_brazo = list(msg.data[:6])

        if self.esperando_comando_cero:
            if all(valor == 0.0 for valor in valores_brazo):
                self.esperando_comando_cero = False
            else:
                valores_brazo = [0.0] * 6

        self.command = valores_brazo
        self.gripper_command = (
            float(msg.data[6]) if len(msg.data) == 7 else 0.0
        )
        self.command_stamp = time.monotonic()

    def report(self,text):
        # Repeat so a GUI started later receives the current state.
        self.status.publish(String(data=text))
        if text!=self.last_status:
            self.get_logger().info(text)
            self.last_status=text

    def send(self,positions):
        msg=JointTrajectory()
        msg.joint_names=self.JOINTS
        point=JointTrajectoryPoint(positions=list(positions))
        ns=int(self.trajectory_duration*1e9)
        point.time_from_start.sec=ns//1000000000
        point.time_from_start.nanosec=ns%1000000000
        msg.points=[point]
        self.pub.publish(msg)

    def send_gripper(self,width):
        width=max(0.0,min(self.gripper_max_width,width))
        msg=JointTrajectory()
        msg.joint_names=self.GRIPPER_JOINTS
        point=JointTrajectoryPoint(positions=[-width/2.0,width/2.0])
        ns=int(self.trajectory_duration*1e9)
        point.time_from_start.sec=ns//1000000000
        point.time_from_start.nanosec=ns%1000000000
        msg.points=[point]
        self.gripper_pub.publish(msg)
    
    def cambiar_modo_cartesiano(self, request, response):
        """Activa o desactiva el modo cartesiano del brazo."""
        nuevo_modo = bool(request.data)

        if nuevo_modo == self.modo_cartesiano:
            response.success = True
            response.message = (
                "El modo cartesiano ya estaba activo."
                if nuevo_modo
                else "El modo articular ya estaba activo."
            )
            return response

        ahora = time.monotonic()
        estado_reciente = (
            self.measured is not None
            and ahora - self.state_stamp < self.state_timeout
        )

        # Sustituir la trayectoria anterior por una posición de mantenimiento.
        if estado_reciente and self.pub.get_subscription_count() > 0:
            self.send(self.measured)

        self.command = [0.0] * 6
        self.command_stamp = 0.0
        self.target = None
        self.moving = False

        self.modo_cartesiano = nuevo_modo
        self.esperando_comando_cero = True

        response.success = True
        response.message = (
            "Modo cartesiano activo."
            if nuevo_modo
            else "Modo articular activo."
        )

        self.get_logger().info(response.message)
        return response
        
    def paso_cartesiano_x(self, request, response):
        """Solicita un paso de 5 mm en el eje X local del TCP."""
        ahora = time.monotonic()

        estado_reciente = (
            self.measured is not None
            and ahora - self.state_stamp < self.state_timeout
        )

        if not estado_reciente:
            response.success = False
            response.message = "No hay estado articular reciente."
            return response

        if self.moving or any(self.command):
            response.success = False
            response.message = (
                "Suelta los controles articulares antes de solicitar el paso."
            )
            return response

        if self.pub.get_subscription_count() == 0:
            response.success = False
            response.message = "El controlador del brazo no está conectado."
            return response

        articulaciones_actuales = list(self.measured)

        try:
            # 1. Obtener la pose actual de link6 y del TCP.
            pose_link6 = self.solver_cartesiano.forward(
                articulaciones_actuales
            )
            pose_tcp = link6_a_tcp(pose_link6)

            # 2. Avanzar 5 mm en el eje X local del TCP.
            objetivo_tcp = incrementar_tcp(
                pose_tcp,
                desplazamiento_local=[0.005, 0.0, 0.0],
                giro_local=[0.0, 0.0, 0.0],
            )

            # 3. Convertir el objetivo del TCP en un objetivo de link6.
            objetivo_link6 = tcp_a_link6(objetivo_tcp)

            # 4. Buscar una solución articular.
            solucion = self.solver_cartesiano.solve(
                objetivo_link6,
                articulaciones_actuales,
            )

        except Exception as error:
            self.get_logger().error(
                f"Error al calcular el paso cartesiano: {error}"
            )
            response.success = False
            response.message = "Error al calcular el objetivo cartesiano."
            return response

        if solucion is None:
            response.success = False
            response.message = "TRAC-IK no encontró una solución."
            return response

        # Verificar que la solución respete los límites del backend.
        for indice, valor in enumerate(solucion):
            if not self.arm_lower_limits[indice] <= valor <= self.arm_upper_limits[indice]:
                response.success = False
                response.message = (
                    f"La solución excede el límite de joint{indice + 1}."
                )
                return response

        # Calcular una duración a partir del cambio articular y las velocidades.
        duracion = max(
            abs(float(objetivo) - actual) / velocidad
            for objetivo, actual, velocidad in zip(
                solucion,
                articulaciones_actuales,
                self.arm_max_velocity,
            )
        )
        duracion = max(1.0, duracion)

        trayectoria = JointTrajectory()
        trayectoria.joint_names = list(self.JOINTS)

        punto = JointTrajectoryPoint()
        punto.positions = [float(valor) for valor in solucion]

        duracion_ns = int(duracion * 1_000_000_000)
        punto.time_from_start.sec = duracion_ns // 1_000_000_000
        punto.time_from_start.nanosec = duracion_ns % 1_000_000_000

        trayectoria.points = [punto]
        self.pub.publish(trayectoria)

        response.success = True
        response.message = (
            f"Trayectoria enviada para un paso de 5 mm en +X del TCP. "
            f"Duración solicitada: {duracion:.2f} s."
        )
        return response
    
    def calcular_objetivo_cartesiano(self, dt):
        """Calcula una posición articular para un incremento del TCP."""
        if self.target is None:
            self.target = list(self.measured)

        # Obtener la pose correspondiente al objetivo articular anterior.
        pose_link6 = self.solver_cartesiano.forward(self.target)
        pose_tcp = link6_a_tcp(pose_link6)

        desplazamiento = [
            self.command[indice] / 100.0 * self.velocidad_lineal_tcp * dt
            for indice in range(3)
        ]

        giro = [
            self.command[indice + 3] / 100.0
            * self.velocidad_angular_tcp
            * dt
            for indice in range(3)
        ]

        objetivo_tcp = incrementar_tcp(
            pose_tcp,
            desplazamiento_local=desplazamiento,
            giro_local=giro,
        )

        objetivo_link6 = tcp_a_link6(objetivo_tcp)

        solucion = self.solver_cartesiano.solve(
            objetivo_link6,
            self.target,
        )

        if solucion is None:
            return None

        objetivos = []

        for indice, valor in enumerate(solucion):
            valor = float(valor)

            if not math.isfinite(valor):
                return None

            # Limitar cuánto puede avanzar cada articulación en este ciclo.
            paso_maximo = self.arm_max_velocity[indice] * dt
            actual = self.target[indice]

            valor = max(
                actual - paso_maximo,
                min(actual + paso_maximo, valor),
            )

            # Mantener el objetivo cerca del estado medido.
            valor = max(
                self.measured[indice] - self.max_tracking_error,
                min(
                    self.measured[indice] + self.max_tracking_error,
                    valor,
                ),
            )

            # Respetar los límites articulares configurados.
            valor = max(
                self.arm_lower_limits[indice],
                min(self.arm_upper_limits[indice], valor),
            )

            objetivos.append(valor)

        return objetivos



    def tick(self):
        now=time.monotonic()
        dt=min(now-self.last_tick,2.0/self.publish_rate)
        self.last_tick=now
        fresh=self.measured is not None and now-self.state_stamp<self.state_timeout
        gripper_fresh=self.gripper_measured is not None and now-self.gripper_state_stamp<self.state_timeout
        command_fresh=now-self.command_stamp<self.command_timeout
        arm_active=fresh and command_fresh and any(self.command)
        gripper_active=gripper_fresh and command_fresh and self.gripper_command!=0.0

        if arm_active:
            if self.pub.get_subscription_count() == 0:
                self.target = None
                self.moving = False
                self.report('Controlador no conectado')
                return

            if self.modo_cartesiano:
                try:
                    targets = self.calcular_objetivo_cartesiano(dt)
                except Exception as error:
                    self.get_logger().warning(
                        f'Error en el cálculo cartesiano: {error}'
                    )
                    targets = None

                if targets is None:
                    # Detener el objetivo activo y volver a partir de la medida.
                    self.send(self.measured)
                    self.target = None
                    self.moving = False
                    self.report('Sin solución cartesiana')
                    return

            else:
                if self.target is None:
                    self.target = list(self.measured)

                targets = []

                for i in range(6):
                    desired = (
                        self.target[i]
                        + self.command[i] / 100.0
                        * self.arm_max_velocity[i] * dt
                    )

                    desired = max(
                        self.measured[i] - self.max_tracking_error,
                        min(
                            self.measured[i] + self.max_tracking_error,
                            desired,
                        ),
                    )

                    targets.append(
                        max(
                            self.arm_lower_limits[i],
                            min(self.arm_upper_limits[i], desired),
                        )
                    )

            self.target = targets
            self.send(targets)
            self.moving = True

        if gripper_active:
            if self.gripper_pub.get_subscription_count()==0:
                self.gripper_target=None
                self.gripper_moving=False
                self.report('Controlador gripper no conectado')
                return
            if self.gripper_target is None:
                self.gripper_target=self.gripper_measured
            desired=self.gripper_target+self.gripper_command/100.0*self.gripper_max_velocity*dt
            self.gripper_target=max(0.0,min(self.gripper_max_width,desired))
            self.send_gripper(self.gripper_target)
            self.gripper_moving=True

        if arm_active and gripper_active:
            self.report('Movimiento articular y gripper')
        elif arm_active:
            self.report(
                "Movimiento cartesiano"
                if self.modo_cartesiano
                else "Movimiento articular"
            )
        elif gripper_active:
            self.report('Movimiento gripper')
        else:
            self.report('Listo' if fresh else 'Esperando estado articular')

def main(args=None):
    rclpy.init(args=args)
    node=PiperControl()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.moving and node.measured is not None:
            node.send(node.measured)
        if node.gripper_moving and node.gripper_measured is not None:
            node.send_gripper(node.gripper_measured)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
