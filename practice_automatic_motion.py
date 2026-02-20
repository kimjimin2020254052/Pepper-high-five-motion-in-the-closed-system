import rclpy
from rclpy.node import Node
from rclpy.time import Time
import cv2
import numpy as np
import mediapipe as mp
from sensor_msgs.msg import JointState, Image, CameraInfo
from tf2_ros import Buffer, TransformListener # for lookup_transform
import tf2_geometry_msgs # for do_transform_point
from geometry_msgs.msg import PointStamped, Twist
from cv_bridge import CvBridge
from naoqi_bridge_msgs.msg import JointAnglesWithSpeed
from pepper_motion_py.pepper_config import dic_joints, joint_order
from pepper_motion_py.PepperKinematics_main.inverse_kinematics import get_arm_all_angles, get_arm_partial_angles
import message_filters
from rclpy.duration import Duration
from collections import deque

class highFiveNode(Node):
    def __init__(self):
        super().__init__('highfive_node')
        # TF2
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.h1 = 0.0
        self.h2 = 0.0
        self.t1 = 0.0
        self.t2 = 0.0
        self.ik_counter = 0.0
        # Calibration
        self.fx = 0.0
        self.fy = 0.0
        self.cx = 0.0
        self.cy = 0.0
        self.Xc = None
        self.Yc = None
        self.Zc = None
        # locking
        self.Z_d_history = deque(maxlen=5)
        self.reject_count = 0
        # Tracking
        self.HS_degree = 0
        self.HUD_degree = 0
        self.k_p = 0.001
        self.latest_depth = None
        # Motion, high five
        self.action_lock_time = self.get_clock().now()
        # hands tracking to use mediapipe
        self.mp_hands = mp.solutions.hands
        self.hands = self.mp_hands.Hands(
            static_image_mode=False,
            max_num_hands=1,
            model_complexity=0,
            min_tracking_confidence=0.5,
            # confidence percentage
            min_detection_confidence=0.5
        )
        
        # To get pepper's wrist data
        self.joint_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )
        self.current_joint_angles = {}

        # Image & Depth sub
        self.bridge = CvBridge()
        self.info_sub = self.create_subscription(CameraInfo, '/camera/front/camera_info', self.info_callback, 10)
        self.image_sub = self.create_subscription(Image, '/camera/front/image_raw', self.image_callback, 10)
        self.depth_sub = self.create_subscription(Image, '/camera/depth/image_raw', self.depth_callback, 10)
        self.joint_pub = self.create_publisher(JointAnglesWithSpeed, '/joint_angles', 10)
        self.vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
    # get focal length and center positions
    def info_callback(self, msg):
        self.fx = msg.k[0]
        self.fy = msg.k[4]
        self.cx = msg.k[2]
        self.cy = msg.k[5]

    def depth_callback(self, msg):
        self.latest_depth = self.bridge.imgmsg_to_cv2(msg, msg.encoding) 

    def image_callback(self, msg):
        if self.latest_depth is None:
            return
        try : 
            color_frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            depth_raw = self.latest_depth
            
            depth_display = cv2.normalize(depth_raw, None, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8U)
            depth_display = cv2.cvtColor(depth_display, cv2.COLOR_GRAY2BGR)
            ch, cw = color_frame.shape[:2]
            dh, dw = depth_raw.shape[:2]
            # Tracking wrist in the Color image
            rgb_frame = cv2.cvtColor(color_frame, cv2.COLOR_BGR2RGB)
            results_c = self.hands.process(rgb_frame)

            if results_c.multi_hand_landmarks:
                u_c = int(results_c.multi_hand_landmarks[0].landmark[9].x*cw)
                v_c = int(results_c.multi_hand_landmarks[0].landmark[9].y*ch)

                offset_x = u_c - self.cx
                offset_y = v_c - self.cy
                self.update_head_target(offset_x, offset_y)
                
                # precent to uc, vc excessive
                u_c_center = max(0, min(u_c, cw-1))
                v_c_center = max(0, min(v_c, ch-1))
                
                u_start, u_end = max(0, u_c_center-3), min(cw, u_c_center+10)
                v_start, v_end = max(0, v_c_center-10), min(ch, v_c_center+3)

                roi = depth_raw[v_start:v_end, u_start:u_end]
                valid_values = roi[(roi>100)&(roi<1600)]

                if valid_values.size>0:
                    Z_d = np.median(valid_values)/1000
                    Z_d = self.filter_Z_d(Z_d)

                # Z direction offset
                self.Z_c = Z_d + 0.035
                self.X_c = float(self.Z_c*(u_c-self.cx)/self.fx)
                self.Y_c = float(self.Z_c*(v_c-self.cy)/self.fy)

                # We can use TF2
                point_in_cam = PointStamped()
                point_in_cam.header.frame_id = 'CameraTop_optical_frame' 
                point_in_cam.header.stamp = rclpy.time.Time(nanoseconds=0).to_msg()
                point_in_cam.point.x = self.X_c
                point_in_cam.point.y = self.Y_c
                point_in_cam.point.z = self.Z_c
                try:
                    # self.get_logger().info(f"Cam Coords: Xc={self.X_c:.2f}, Yc={self.Y_c:.2f}, Zc={self.Z_c:.2f}")
                    # lookup_transform
                    transform = self.tf_buffer.lookup_transform('torso', point_in_cam.header.frame_id,rclpy.time.Time(nanoseconds=0), timeout=rclpy.duration.Duration(seconds=0.1))  
                    
                    # do_transform_point
                    point_in_torso = tf2_geometry_msgs.do_transform_point(point_in_cam, transform)
                    # IK model only use mm unit
                    px = point_in_torso.point.x * 1000
                    py = point_in_torso.point.y * 1000
                    pz = point_in_torso.point.z * 1000

                    current_time = self.get_clock().now()
                    if current_time < self.action_lock_time:
                        self.get_logger().info('Action in progress... ignoring other steps.', throttle_duration_sec=1.0)
                    else:
                        self.get_logger().info(f'Z_d : {Z_d:.3f}, px : {px:.3f}, py : {py:.3f}, pz : {pz:.3f}')
                        if px >  800:                    
                            self.set_up_motion()            
                        elif 600 < px <= 800 :
                            # for stop
                            stop_msg = Twist()
                            stop_msg.linear.x = 0.0
                            stop_msg.angular.z = 0.0
                            self.vel_pub.publish(stop_msg)
                            # setting vector matrix
                            sx, sy, sz = -57.0, -150.0, 87.0
                            upper_arm_length = 181.20
                            S = np.array([sx, sy, sz])
                            P = np.array([px, py, pz])
                            V = P-S
                            # to make real movement of arm
                            RSUD_gain = 0.2
                            dist = np.linalg.norm(V)
                            if dist>0:
                                # make a unit vecter and can find position of goal elbow
                                E = S + V*(upper_arm_length/dist)
                                ex, ey, ez = E[0], E[1], E[2]
                                try:
                                    RS = get_arm_partial_angles(ex, ey, ez, 'right')
                                    if RS:
                                        RSUD, RSS = RS
                                        self.get_logger().info(f'high_five_wating . . .')
                                        self.ready_high_five(RSUD+RSUD_gain, RSS)
                                    else :
                                        self.get_logger().warn("IK No Solution")
                                except Exception as ik_e:
                                    self.get_logger().error(f'IK Error : {ik_e}')

                        elif px <= 600:
                            lower_arm_length = 150
                            sx, sy, sz = -57.0, -150.0, 87.0
                            upper_arm_length = 181.20
                            S = np.array([sx, sy, sz])
                            P = np.array([px, py, pz])
                            V = P-S
                            dist = np.linalg.norm(V)
                            if dist >0:
                                 # make a unit vecter and can find position of goal wrist
                                E = S + V*(upper_arm_length/dist)
                                ex, ey, ez = E[0], E[1], E[2]
                                W = S + V*((upper_arm_length+lower_arm_length)/dist)
                                wx, wy, wz = W[0], W[1], W[2]
                                ARM = get_arm_all_angles(ex, ey, ez, wx, wy, wz, 'right')
                                try:
                                    if ARM:
                                        RSUD, RSS, RES, REUD = ARM
                                        self.get_logger().info(f'!!!!!!!!High_Five!!!!!!!!!')
                                        self.action_high_five(RSUD, RSS, RES, REUD)
                                    else :
                                        self.get_logger().warn("IK2 No Solution")
                                except Exception as ik2_e:
                                    self.get_logger().error(f'IK2 Error : {ik2_e}')     

                except Exception as e:
                    self.get_logger().error(f'error : {e}')
                cv2.circle(color_frame, (u_c, v_c), 5, (0, 255, 0), cv2.FILLED)
                cv2.circle(depth_display, (u_c_center, v_c_center), 1, (0, 0, 255), 2)
                cv2.rectangle(depth_display, (u_start, v_start), (u_end, v_end), (0, 255, 0), 2)
                cv2.imshow("Depth ROI Check", depth_display)
            else:
                stop_msg = Twist()
                stop_msg.linear.x = 0.0
                stop_msg.angular.z = 0.0
                self.vel_pub.publish(stop_msg)
        
        except Exception as e:
            self.get_logger().error(f'Error : {e}')
        
        if color_frame is not None:
            cv2.imshow("color", color_frame)
            cv2.waitKey(1) 

    # Z_d filter
    def filter_Z_d(self, new_z):
        if len(self.Z_d_history) < 5:
            self.Z_d_history.append(new_z)
            return new_z
        mean = np.mean(self.Z_d_history)
        if new_z > 1.2*mean or new_z < 0.8*mean:
            self.reject_count += 1
            if self.reject_count > 10:
                self.get_logger().warn(f'Resettiing filter history!')
                self.Z_d_history.clear()
                self.reject_count = 0
                return new_z
            return mean
        self.reject_count = 0
        self.Z_d_history.append(new_z)
        return np.mean(self.Z_d_history)
                

    # get angle of the Shoulder joints
    def joint_state_callback(self, msg):
        for name, pos in zip(msg.name, msg.position):
            self.current_joint_angles[name] = pos
        for name, vel in zip(msg.name, msg.velocity):
            self.current_joint_velocity[name] = vel
        self.h1 = self.current_joint_angles.get(dic_joints["HUD"]["name"],0.0)
        self.h2 = self.current_joint_angles.get(dic_joints["HS"]["name"], 0.0)
        self.t1 = self.current_joint_angles.get(dic_joints["RSUD"]["name"], 0.0) 
        self.t2 = self.current_joint_angles.get(dic_joints["RSS"]["name"], 0.0)
    
    def update_head_target(self, offset_x, offset_y):
        if abs(offset_x)<20: offset_x = 0
        if abs(offset_y)<20: offset_y = 0
        # convert to the radian unit
        self.HS_degree = self.h2 - (offset_x * self.k_p)
        self.HUD_degree = self.h1 + (offset_y * self.k_p)
        self.HS_degree = max(min(self.HS_degree, dic_joints["HS"]["max"]*0.9), dic_joints["HS"]["min"]*0.9)
        self.HUD_degree = max(min(self.HUD_degree, dic_joints["HUD"]["max"]*0.9), dic_joints["HUD"]["min"]*0.9)

    def set_up_motion(self):
        # head
        msg = JointAnglesWithSpeed()
        msg.speed = 0.1
        msg.relative = 0
        msg.joint_names = [dic_joints[k]["name"] for k in joint_order]
        current_angles = list([float(dic_joints[k]["default"]) for k in joint_order])
        current_angles[0] = self.HS_degree
        current_angles[1] = self.HUD_degree
        msg.joint_angles = current_angles
        # base
        msg2 = Twist()        
        msg2.linear.x = float(0.03)
        angular_velocity = self.HS_degree * 0.5
        msg2.angular.z = float(angular_velocity)
        # publish
        self.joint_pub.publish(msg)
        self.vel_pub.publish(msg2)

    # move joint to do high_five
    def ready_high_five(self, RSUD, RSS):
        msg = JointAnglesWithSpeed()
        msg.speed = 0.1
        msg.relative = 0
        msg.joint_names = [dic_joints[k]["name"] for k in joint_order]
        current_angles = list([float(dic_joints[k]["default"]) for k in joint_order])
        current_angles[0] = self.HS_degree
        current_angles[1] = self.HUD_degree
        current_angles[8] = RSUD
        current_angles[9] = RSS
        current_angles[11] = 1.562
        current_angles[12] = -1.82
        current_angles[13] = 1.0
        msg.joint_angles = current_angles
        self.joint_pub.publish(msg)

    def action_high_five(self, RSUD, RSS, RES, REUD):
        lock_duration = rclpy.duration.Duration(seconds=1.5)
        self.action_lock_time = self.get_clock().now() + lock_duration
        msg = JointAnglesWithSpeed()
        msg.speed = 0.2
        msg.relative = 0
        msg.joint_names = [dic_joints[k]["name"] for k in joint_order]
        current_angles = list([float(dic_joints[k]["default"]) for k in joint_order])
        current_angles[0] = self.HS_degree
        current_angles[1] = self.HUD_degree
        current_angles[8] = RSUD
        current_angles[9] = RSS
        current_angles[11] = REUD
        current_angles[12] = -1.82
        current_angles[13] = 1.0
        msg.joint_angles = current_angles
        self.joint_pub.publish(msg)

    def default(self):
        # head
        msg = JointAnglesWithSpeed()
        msg.speed = 0.1
        msg.relative = 0
        msg.joint_names = [dic_joints[k]["name"] for k in joint_order]
        current_angles = list([float(dic_joints[k]["default"]) for k in joint_order])
        msg.joint_angles = current_angles
        self.joint_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = highFiveNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
