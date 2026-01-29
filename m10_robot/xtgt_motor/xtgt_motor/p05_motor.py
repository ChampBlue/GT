#!/usr/bin/python3
"""

"""
import time
import serial
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, TransformStamped, PoseWithCovarianceStamped, PoseStamped, Vector3
from nav_msgs.msg import Odometry
from std_msgs.msg import Int32
from sensor_msgs.msg import Imu
from tf_transformations import quaternion_from_euler as e2q
from tf_transformations import euler_from_quaternion as q2e
from tf2_ros import TransformBroadcaster
from serial.serialutil import SerialException

# >>> [ADD] 통신용
import socket, threading

#p : 0.84 -> almost 360 degree

""" PARAMETERS """    
# pid gain   
Kp = 0.0
Ki = 0.0
Kd = 0.0 #0.01

# car parameters
Len_bw_wheel = 0.782 # [m]
Vel_max = 3.4 # [m/s]   

SERVER_IP   = '192.168.0.20'
SERVER_PORT = 9902
LINK_TIMEOUT_SEC = 1.5     
RECV_PERIOD = 0.5         

class MotorController(Node):
    def __init__(self):
        super().__init__('hb_motor_control')

        self.ser = serial.Serial(port="/dev/ttyMOTOR",
                        baudrate=115200,
                        bytesize=serial.EIGHTBITS,
                        parity=serial.PARITY_NONE,
                        stopbits=serial.STOPBITS_ONE,
                        timeout=0.05
        )

        self.rx_buf = b""
        self.ser.reset_input_buffer() 

        self.current_time = self.get_clock().now()
        
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.yaw = None
        self.prev_yaw = None

        self.vel = 0.0   #default 0.5
        self.angular_vel = 0.0
        self.encoder_vel = 0.0
        self.encoder_angular_vel = 0.0

        self.prev_error = 0
                
        self.data = [0xFF, 1, 1, 0, 1, 0, 0]      # [_, enable, left_dir, left_pwm, right_dir, right_pwm, mnq, ]

        self.declare_parameter('odom_frame_id', 'odom')
        self.declare_parameter('base_footprint_frame_id', 'base_footprint')
        self.declare_parameter('ip', '192.168.0.20')
        self.declare_parameter('port', 9902)
        self.odom_frame = str(self.get_parameter('odom_frame_id').value)
        self.base_footprint_frame = str(self.get_parameter('base_footprint_frame_id').value)
        self.ip = str(self.get_parameter('ip').value)
        self.port = int(self.get_parameter('port').value)
        
        self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)
        self.create_subscription(Int32, 'stop', self.stopper_callback, 10)
        self.create_subscription(Int32, 'mnq_manual', self.mnq_manual_callback, 10)
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)

        self.stop = False
        self.mnq = 0
        self.print_mnq = False

        # self.tf_br = TransformBroadcaster(self)

        self.odom_timer = self.create_timer(0.02, self.pub_odom)

        self._last_tcp_rx = None
        self._tcp_thread = threading.Thread(target=self._tcp_watch, daemon=True)
        self._tcp_thread.start()
        
    """ ----------------- PUB&SUB ---------------- """
    def stopper_callback(self, msg):
        if msg.data == 0:
            self.stop = True
        elif msg.data == 1:
            self.stop = False
            
    def cmd_vel_callback(self, msg):
        self.vel = msg.linear.x
        self.angular_vel = msg.angular.z

    def mnq_manual_callback(self, msg):
        self.mnq = msg.data

    def pub_odom(self):
        now = self.get_clock().now()
        dt = (now - self.current_time).nanoseconds * 1e-9
        self.current_time = now

        if dt < 0.0:
            dt = 0.0
        if dt > 0.1:
            dt = 0.1

        self.theta += self.encoder_angular_vel * dt

        dx = self.encoder_vel * dt * math.cos(self.theta)
        dy = self.encoder_vel * dt * math.sin(self.theta)
        self.x += dx
        self.y += dy

        qx, qy, qz, qw = e2q(0.0, 0.0, self.theta)

        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_footprint_frame
        odom.pose.pose = Pose(
            position=Point(x=self.x, y=self.y, z=0.0),
            orientation=Quaternion(x=qx, y=qy, z=qz, w=qw)
        )
        odom.twist.twist.linear = Vector3(x=self.encoder_vel, y=0.0, z=0.0)
        odom.twist.twist.angular = Vector3(x=0.0, y=0.0, z=self.encoder_angular_vel)

        odom.pose.covariance = [1e-3, 0., 0., 0., 0., 0.,
                                0., 1e-3, 0., 0., 0., 0.,
                                0., 0., 1e6, 0., 0., 0.,
                                0., 0., 0., 1e6, 0., 0.,
                                0., 0., 0., 0., 1e6, 0.,
                                0., 0., 0., 0., 0., 1e3]

        odom.twist.covariance = [1e-3, 0., 0., 0., 0., 0.,
                                 0., 1e-3, 0., 0., 0., 0.,
                                 0., 0., 1e6, 0., 0., 0.,
                                 0., 0., 0., 1e6, 0., 0.,
                                 0., 0., 0., 0., 1e6, 0.,
                                 0., 0., 0., 0., 0., 1e3]

        self.odom_pub.publish(odom)
        
    """ ----------------- ENCODER ---------------- """
    def read_encoder(self):
        if self.ser.in_waiting > 0:
            try:
                line = self.ser.readline().decode('utf-8').strip()
                vel, angular_vel, l_enc, r_enc, adc, mnq = line.split(',')
                self.encoder_vel = float(vel)
                self.encoder_angular_vel = float(angular_vel)
                if mnq == '1':
                    #print("shot")
                    self.get_logger().info('shot')
            except ValueError:
                print("value error")
        else:
            print("pico no data")
            time.sleep(0.5)                

    """ -------------- MOTOR CONTROL -------------- """            
    def vel2pwm(self, vel, angular_vel):
        vel_l = vel - (Len_bw_wheel/2)*angular_vel
        vel_r = vel + (Len_bw_wheel/2)*angular_vel
        
        m = max(abs(vel_l), abs(vel_r), 1e-9)
        if m > Vel_max:
            scale = Vel_max / m
            vel_l *= scale
            vel_r *= scale

        pwm_l = vel_l * 255/Vel_max
        pwm_r = vel_r * 255/Vel_max
        
        if pwm_l > 254:
            pwm_l = 254
        elif pwm_l < -254:
            pwm_l = -254

        if pwm_r > 254:
            pwm_r = 254
        elif pwm_r < -254:
            pwm_r = -254

        return pwm_l, pwm_r
    
    """ ----------- CONNECTION WITH PICO ----------- """   
    def tx_pico(self, left_pwm, right_pwm):
        data = self.data
        left_pwm = int(left_pwm)
        right_pwm = int(right_pwm)
        
        if self.stop == True:
            self.ser.write(bytes([0xFF, 0, 0, 0, 0, 0, self.mnq]))
        else:
            if left_pwm < 0:
                data[2] = 1
                data[3] = left_pwm * -1
            elif left_pwm >= 0:
                data[2] = 0
                data[3] = left_pwm
            
            if right_pwm < 0:
                data[4] = 0
                data[5] = right_pwm * -1
            elif right_pwm >= 0:
                data[4] = 1
                data[5] = right_pwm

            data[6] = self.mnq
            self.data = data
            self.ser.write(bytes(self.data))
            
    """ ------------------ MAIN ----------------- """        
    def run_loop(self):
        self.get_logger().info("Start publish encoder data with Odometry...")
        
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.0)  # 콜백 처리
            self.read_encoder()
            
            pwm_l, pwm_r = self.vel2pwm(self.vel, self.angular_vel)
            self.tx_pico(int(pwm_l), int(pwm_r))
            time.sleep(0.01)

    def _tcp_watch(self):
        while rclpy.ok():
            sock = None
            try:
                sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                sock.settimeout(2.0)
                sock.connect((self.ip, self.port))
                sock.settimeout(RECV_PERIOD)

                self._last_tcp_rx = time.time()

                self.get_logger().info(f"TCP 연결됨: {self.ip}:{self.port}")
                # 통신 복구시 재가동 할라면 밑에꺼 주석 해제 
                #self.stop = False

                while rclpy.ok():
                    try:
                        data = sock.recv(1024)
                        now = time.time()
                        if data:
                            self._last_tcp_rx = now
                        if now - self._last_tcp_rx > LINK_TIMEOUT_SEC:
                            if not self.stop:
                                self.get_logger().warn(f"TCP 수신 없음 {LINK_TIMEOUT_SEC}s → 정지")
                            self.stop = True
                        time.sleep(0.0)
                    except socket.timeout:
                        now = time.time()
                        if self._last_tcp_rx is None or (now - self._last_tcp_rx > LINK_TIMEOUT_SEC):
                            if not self.stop:
                                self.get_logger().warn(f"TCP 수신 없음 {LINK_TIMEOUT_SEC}s → 정지")
                            self.stop = True
                        continue
                    except Exception:
                        break  
            except Exception:
                self.stop = True
                time.sleep(0.5)
            finally:
                if sock:
                    try: sock.close()
                    except Exception: pass
                # 재연결 대기
                time.sleep(0.5)

def main():
    rclpy.init()
    mc = MotorController()
    try:
        mc.run_loop()
    except KeyboardInterrupt:
        try:
            mc.ser.write(bytes([0xFF, 0, 0, 0, 0, 0, 0]))
        except Exception:
            pass
    finally:
        mc.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
