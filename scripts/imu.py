#!/usr/bin/env python
#
# Subsriber to ins and sensors message and convert to IMU message
#Prasenjit Mukherjee(2013 - MIT Licence (TODO: put license text here?)
#  

import rospy
import tf
from geometry_msgs.msg import Quaternion,Vector3,Vector3Stamped
from sensor_msgs.msg import Imu
from vectornav.msg import ins,sensors 

class ImuPublisher:
    def __init__(self):
        rospy.init_node('imu_publisher')
        self.roll = 0;
        self.pitch = 0;
        self.yaw = 0;
        self.uncertainty = 0;

        self.accel_x = 0;
        self.accel_y = 0;
        self.accel_z = 0;

        self.gyro_x = 0;
        self.gyro_y = 0;
        self.gyro_z = 0;

        self.mag_x = 0;
        self.mag_y = 0;
        self.mag_z = 0;

        self.pub_rate = rospy.get_param('pub_rate',10.0)
        self.frame_id = rospy.get_param('frame_id','imu_link')
        self.quat_covar = rospy.get_param('quat_covar',0.01)
        self.gyro_covar = rospy.get_param('gyro_covar',0.01)
        self.accel_covar = rospy.get_param('accel_covar', 0.01)

        rospy.Subscriber('vectornav/ins',ins,self.ins_cb)
        rospy.Subscriber('vectornav/imu',sensors,self.sensors_cb)
        self.imu_pub = rospy.Publisher('imu',Imu)
        self.mag_pub = rospy.Publisher('vectornav/mag',Vector3Stamped)

        rospy.Timer(rospy.Duration(1.0/self.pub_rate), self.imu_publisher)

        rospy.spin()

    def sensors_cb(self, msg_in):
        self.accel_x = msg_in.Accel.x
        self.accel_y = msg_in.Accel.y
        self.accel_z = msg_in.Accel.z

        self.gyro_x = msg_in.Gyro.x
        self.gyro_y = msg_in.Gyro.y
        self.gyro_z = msg_in.Gyro.z

        self.mag_x = msg_in.Mag.x
        self.mag_y = msg_in.Mag.y
        self.mag_z = msg_in.Mag.z


    def ins_cb(self, msg_in):
        self.roll = msg_in.RPY.x;
        self.pitch = msg_in.RPY.y;
        self.yaw = msg_in.RPY.z;
        self.uncertainty = AttUncertainty

    def imu_publisher(self, event):
        msg_out = Imu()
        msg_out.header.stamp = rospy.Time.now()
        msg_out.header.frame_id = self.frame_id
        quat = tf.transformations.quaternion_from_euler(self.roll, self.pitch, self.yaw)
        msg_out.orientation = Quaternion(quat[0],quat[1],quat[2],quat[3])
        msg_out.orientation_covariance[0] = self.quat_covar
        msg_out.orientation_covariance[4] = self.quat_covar
        msg_out.orientation_covariance[8] = self.quat_covar

        msg_out.angular_velocity = Vector3(self.gyro_x, self.gyro_y, self.gyro_z)
        msg_out.angular_velocity_covariance[0] = self.gyro_covar
        msg_out.angular_velocity_covariance[4] = self.gyro_covar
        msg_out.angular_velocity_covariance[8] = self.gyro_covar

        msg_out.linear_acceleration = Vector3(self.accel_x, self.accel_y, self.accel_z)

        msg_out.linear_acceleration_covariance[0] = self.accel_covar
        msg_out.linear_acceleration_covariance[4] = self.accel_covar
        msg_out.linear_acceleration_covariance[8] = self.accel_covar

        mag_out = Vector3Stamped()
        mag_out.header.stamp = rospy.Time.now()
        mag_out.header.frame_id = self.frame_id
        mag_out.vector = Vector3(self.mag_x, self.mag_y, self.mag_z)


        self.mag_pub.publish(mag_out)
        self.imu_pub.publish(msg_out)



if __name__ == "__main__": 
    ImuPublisher()
