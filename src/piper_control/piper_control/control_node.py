"""Normalized joint commands -> bounded Gazebo trajectories, like the real bridge."""
import math
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32MultiArray, String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

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
        self.pub=self.create_publisher(JointTrajectory,self.trajectory_topic,1)
        self.gripper_pub=self.create_publisher(JointTrajectory,self.gripper_trajectory_topic,1)
        self.status=self.create_publisher(String,self.status_topic,1)
        self.create_subscription(Float32MultiArray,self.command_topic,self.on_command,1)
        self.create_subscription(JointState,self.state_topic,self.on_state,qos_profile_sensor_data)
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
        self.command=list(msg.data[:6])
        self.gripper_command=float(msg.data[6]) if len(msg.data)==7 else 0.0
        self.command_stamp=time.monotonic()

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

    def tick(self):
        now=time.monotonic()
        dt=min(now-self.last_tick,2.0/self.publish_rate)
        self.last_tick=now
        fresh=self.measured is not None and now-self.state_stamp<self.state_timeout
        gripper_fresh=self.gripper_measured is not None and now-self.gripper_state_stamp<self.state_timeout
        command_fresh=now-self.command_stamp<self.command_timeout
        arm_active=fresh and command_fresh and any(self.command)
        gripper_active=gripper_fresh and command_fresh and self.gripper_command!=0.0

        if not arm_active and self.moving:
            # Replace the previous short trajectory with a bounded hold target.
            self.send(self.measured if fresh else self.target)
            self.moving=False
            self.target=None
        if not gripper_active and self.gripper_moving:
            self.send_gripper(self.gripper_measured if gripper_fresh else self.gripper_target)
            self.gripper_moving=False
            self.gripper_target=None

        if arm_active:
            if self.pub.get_subscription_count()==0:
                self.target=None
                self.moving=False
                self.report('Controlador no conectado')
                return
            if self.target is None:
                self.target=list(self.measured)
            targets=[]
            for i in range(6):
                desired=self.target[i]+self.command[i]/100.0*self.arm_max_velocity[i]*dt
                desired=max(self.measured[i]-self.max_tracking_error,min(self.measured[i]+self.max_tracking_error,desired))
                targets.append(max(self.arm_lower_limits[i],min(self.arm_upper_limits[i],desired)))
            self.target=targets
            self.send(targets)
            self.moving=True

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
            self.report('Movimiento articular')
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
