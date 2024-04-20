
from ROSCar_Lib import ROSCar
import numpy as np

#ros lib
import rclpy
from rclpy.node import Node
from std_msgs.msg import String,Float32,Int32,Bool
from geometry_msgs.msg import Twist,TransformStamped,Quaternion
from sensor_msgs.msg import Imu,MagneticField, JointState
from rclpy.clock import Clock
import math
from tf2_ros import StaticTransformBroadcaster,TransformBroadcaster

from nav_msgs.msg import Odometry




def quaternion_from_euler(ai, aj, ak):  # xyz rpy
    ai /= 2.0
    aj /= 2.0
    ak /= 2.0
    ci = math.cos(ai)
    si = math.sin(ai)
    cj = math.cos(aj)
    sj = math.sin(aj)
    ck = math.cos(ak)
    sk = math.sin(ak)
    cc = ci*ck
    cs = ci*sk
    sc = si*ck
    ss = si*sk

    q = np.empty((4, ))
    q[0] = cj*sc - sj*cs # x    
    q[1] = cj*ss + sj*cc # y
    q[2] = cj*cs - sj*sc # z
    q[3] = cj*cc + sj*ss # w

    return q


class roscar_driver_node(Node):
    def __init__(self):
        super().__init__("driver_node") # 节点名称

        self.car = ROSCar()

        # 创建订阅者
        self.sub_cmd_vel  = self.create_subscription(Twist,"cmd_vel",self.cmd_vel_callback,1)
        # 创建发布者
        self.volPublisher = self.create_publisher(Float32,"voltage",100)
        self.velPublisher = self.create_publisher(Twist,"vel_raw",50)
        self.imuPublisher = self.create_publisher(Imu,"imu/data",100)
        self.magPublisher = self.create_publisher(MagneticField,"/imu/mag",100)
        self.odompublisher= self.create_publisher(Odometry, '/odom_raw', 10)

        # 创建定时器，定时调用发布数据的函数
        timer_period = 0.1 
        self.timer_ = self.create_timer(timer_period,self.pub_data) 


        # odom
        self.tf_broadcaster = TransformBroadcaster(self)
        self.last_vel_time_ = self.get_clock().now()
        self.x_pos_ = 0.0
        self.y_pos_ = 0.0
        self.heading_ = 0.0
        
        # 开启接收数据线程
        self.car.create_receive_threading()

    def cmd_vel_callback(self,msg:Twist):

        vx = msg.linear.x
        vy = msg.linear.y
        angle = msg.angular.z
        self.car.set_car_motion(vx,vy,angle)


    
    def publish_odom(self, msg:Twist):
        pass
        current_time = self.get_clock().now()
        vel_dt_ = (current_time - self.last_vel_time_).nanoseconds / 1e9

        linear_velocity_x_ = msg.linear.x
        linear_velocity_y_ = msg.linear.y
        angular_velocity_z_ = msg.angular.z

        
        delta_heading = angular_velocity_z_ * vel_dt_
        delta_x = (linear_velocity_x_ * math.cos(self.heading_) - linear_velocity_y_ * math.sin(self.heading_)) * vel_dt_
        delta_y = (linear_velocity_x_ * math.sin(self.heading_) + linear_velocity_y_ * math.cos(self.heading_)) * vel_dt_

        self.x_pos_ += delta_x
        self.y_pos_ += delta_y
        self.heading_ += delta_heading

        odom_msg = Odometry()
        odom_msg.header.stamp = current_time.to_msg()
        odom_msg.header.frame_id = "odom"
        odom_msg.child_frame_id = "base_footprint"

        odom_msg.pose.pose.position.x = self.x_pos_
        odom_msg.pose.pose.position.y = self.y_pos_
        odom_msg.pose.pose.position.z = 0.0

        odom_quat = Quaternion()
        
        quat = quaternion_from_euler(0.0, 0.0, self.heading_)
        odom_quat.x, odom_quat.y, odom_quat.z, odom_quat.w = quat[0],quat[1],quat[2],quat[3]
        odom_msg.pose.pose.orientation = odom_quat

        odom_msg.pose.covariance[0] = 0.001
        odom_msg.pose.covariance[7] = 0.001
        odom_msg.pose.covariance[35] = 0.001

        odom_msg.twist.twist.linear.x = linear_velocity_x_
        odom_msg.twist.twist.linear.y = linear_velocity_y_
        odom_msg.twist.twist.linear.z = 0.0
        odom_msg.twist.twist.angular.x = 0.0
        odom_msg.twist.twist.angular.y = 0.0
        odom_msg.twist.twist.angular.z = angular_velocity_z_

        odom_msg.twist.covariance[0] = 0.0001
        odom_msg.twist.covariance[7] = 0.0001
        odom_msg.twist.covariance[35] = 0.0001

        self.odompublisher.publish(odom_msg)

        # Publish transform
        transform_stamped = TransformStamped()
        transform_stamped.header.stamp = current_time.to_msg()
        transform_stamped.header.frame_id = "odom"
        transform_stamped.child_frame_id = "base_footprint"
        transform_stamped.transform.translation.x = self.x_pos_
        transform_stamped.transform.translation.y = self.y_pos_
        transform_stamped.transform.translation.z = 0.0
        transform_stamped.transform.rotation = odom_quat
        self.tf_broadcaster.sendTransform(transform_stamped)

        self.last_vel_time_ = current_time

        

    def pub_data(self):
        time_stamp = self.get_clock().now()
        # 创建消息
        battery = Float32()
        twist = Twist()
        imu = Imu()
        mag = MagneticField()

        battery.data = self.car.Get_Bat()
        vx, vy, angular = self.car.Get_motion_data()
        ax, ay, az = self.car.get_accelerometer_data()
        gx, gy, gz = self.car.get_gyroscope_data()
        mx, my, mz = self.car.get_magnetometer_data()
        roll,pitch,yaw = self.car.Get_attitude_data(toAngle=False)
        quat = quaternion_from_euler(roll,pitch,yaw)
        
        # 获取速度数据
        twist.linear.x = vx * 1.0
        twist.linear.y = vy * 1.0
        twist.angular.z = angular * 1.0
        # 获取陀螺仪的数据
        
        
        imu.header.stamp = time_stamp.to_msg()
        imu.header.frame_id = "imu_link"
        imu.linear_acceleration.x = ax 
        imu.linear_acceleration.y = ay
        imu.linear_acceleration.z = az
        imu.angular_velocity.x = gx
        imu.angular_velocity.y = gy
        imu.angular_velocity.z = gz
        imu.orientation.x =quat[0]
        imu.orientation.y =quat[1]
        imu.orientation.z =quat[2]
        imu.orientation.w =quat[3]

        # 磁力计数据
        mag.header.stamp = time_stamp.to_msg()
        mag.header.frame_id = "imu_link"
        mag.magnetic_field.x = mx
        mag.magnetic_field.y = my
        mag.magnetic_field.z = mz

        # odom
        self.publish_odom(twist)

        # 发布消息
        self.volPublisher.publish(battery)
        self.velPublisher.publish(twist)
        self.imuPublisher.publish(imu)
        self.magPublisher.publish(mag)
        # self.get_logger().info(f"Publishing: {battery.data}")
        # self.get_logger().info(f"Publishing: {twist}")
        # self.get_logger().info(f"Publishing: {imu.angular_velocity}")
        # self.get_logger().info(f"Publishing: {mag.magnetic_field}")
    

def main(args = None):
    rclpy.init(args=args)	# 初始化rcl库
    node = roscar_driver_node() # 创建节点（实例化节点）
    rclpy.spin(node)		# 旋转，相当于while循环
    node.destroy_node()		# 销毁系欸节点
    rclpy.shutdown()		# 结束rcl

if __name__ == '__main__':
    main()