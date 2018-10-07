#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from gnss_driver.msg import Gps
import math

# Because of transformations
import tf_conversions
import tf2_ros
import geometry_msgs.msg


def gnss_odom_provider():
    global pubOdom, hasFirstOdom, firstPosition, eulerFirst, lastOdom, pubTwist

    hasFirstOdom = False
    firstPosition = geometry_msgs.msg.Vector3()
    lastOdom = Odometry()
    eulerFirst = [0,0,0]

    rospy.init_node('gnss_odom_provider', anonymous=True)
    rate = rospy.Rate(100) # hz
    ID = str(abs(hash(rospy.get_caller_id())) % (10 ** 8))
    rospy.loginfo('gnss_odom_provider ' + str(ID) + ': initialized')

    rospy.Subscriber('/gnss_driver/odometry', Gps, callbackGnssOdom)
    rospy.Subscriber('/vehicle/twist', geometry_msgs.msg.TwistStamped, callbackTwist)
#     ---
# header: 
#   seq: 37415
#   stamp: 
#     secs: 1536207140
#     nsecs:  83595251
#   frame_id: "base_footprint"
# twist: 
#   linear: 
#     x: 2.06944441795
#     y: 0.0
#     z: 0.0
#   angular: 
#     x: 0.0
#     y: 0.0
#     z: -0.0124376651859
# ---

# rospy.Subscriber('/vehicle/wheel_speed_report')
# ---
# header: 
#   seq: 88255
#   stamp: 
#     secs: 1536207273
#     nsecs:  72549498
#   frame_id: ''
# front_left: 45.6800003052
# front_right: 45.5999984741
# rear_left: 45.5999984741
# rear_right: 45.5999984741
# ---

# rospy.Subscriber('/vehicle/steering_report')

# rospy.Subcriber('/vehicle/imu/data_raw')
# ---
# header: 
#   seq: 77869
#   stamp: 
#     secs: 1536207169
#     nsecs: 203228832
#   frame_id: "base_footprint"
# orientation: 
#   x: 0.0
#   y: 0.0
#   z: 0.0
#   w: 0.0
# orientation_covariance: [-1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
# angular_velocity: 
#   x: 0.0006
#   y: 0.0
#   z: 0.0002
# angular_velocity_covariance: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
# linear_acceleration: 
#   x: 0.24
#   y: 0.0
#   z: 9.92
# linear_acceleration_covariance: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
# ---


    pubOdom = rospy.Publisher('/odometry/gnss', Odometry, queue_size = 10)
    pubTwist = rospy.Publisher('/odometry/twist', geometry_msgs.msg.TwistWithCovarianceStamped, queue_size = 10)

    while not rospy.is_shutdown():
        rate.sleep()

def callbackTwist(twist):
    global pubTwist
    twistCov = geometry_msgs.msg.TwistWithCovarianceStamped()
    twistCov.header = twist.header
    twistCov.twist.twist = twist.twist
    pubTwist.publish(twistCov)


def callbackGnssOdom(gpsOdom):
    global pubOdom, hasFirstOdom, firstPosition, eulerFirst, lastOdom

    odom = Odometry()
    #Position
    odom.pose.pose.position.x = gpsOdom.localization.position.x
    odom.pose.pose.position.y = gpsOdom.localization.position.y
    odom.pose.pose.position.z = gpsOdom.localization.position.z
    #Orientation
    quaternion = (
        gpsOdom.localization.orientation.qx,
        gpsOdom.localization.orientation.qy,
        gpsOdom.localization.orientation.qz,
        gpsOdom.localization.orientation.qw)
    euler = tf_conversions.transformations.euler_from_quaternion(quaternion)
    q = tf_conversions.transformations.quaternion_from_euler(euler[0], euler[1], euler[2] + math.pi/2)

    odom.pose.pose.orientation.x = q[0]
    odom.pose.pose.orientation.y = q[1]
    odom.pose.pose.orientation.z = q[2]
    odom.pose.pose.orientation.w = q[3]
    #Twist | Linear Velocity
    odom.twist.twist.linear.x = gpsOdom.localization.linear_velocity.x
    odom.twist.twist.linear.y = gpsOdom.localization.linear_velocity.y
    odom.twist.twist.linear.z = gpsOdom.localization.linear_velocity.z
    #Twist | Angular Velocity
    odom.twist.twist.angular.x = gpsOdom.localization.angular_velocity.x
    odom.twist.twist.angular.y = gpsOdom.localization.angular_velocity.y
    odom.twist.twist.angular.z = gpsOdom.localization.angular_velocity.z
    #Header
    odom.header = gpsOdom.header
    odom.header.frame_id = "map"
    odom.child_frame_id = "base_footprint"

    pubOdom.publish(odom)
    # if hasFirstOdom:
    #     brMap = tf2_ros.TransformBroadcaster()
    #     tMap = geometry_msgs.msg.TransformStamped()

    #     tMap.header.stamp = odom.header.stamp
    #     tMap.header.frame_id = "map"
    #     tMap.child_frame_id = "base_footprint"

    #     quaternion = (
    #         odom.pose.pose.orientation.x,
    #         odom.pose.pose.orientation.y,
    #         odom.pose.pose.orientation.z,
    #         odom.pose.pose.orientation.w)
    #     eulerCurrent = tf_conversions.transformations.euler_from_quaternion(quaternion)

    #     quadMap = tf_conversions.transformations.quaternion_from_euler(eulerCurrent[0] - eulerFirst[0], eulerCurrent[1] - eulerFirst[1], eulerCurrent[2] - eulerFirst[2])

    #     tMap.transform.translation.x = odom.pose.pose.position.x - firstPosition.x
    #     tMap.transform.translation.y = odom.pose.pose.position.y - firstPosition.y
    #     tMap.transform.translation.z = odom.pose.pose.position.z - firstPosition.z
    #     tMap.transform.rotation.x = quadMap[0]
    #     tMap.transform.rotation.y = quadMap[1]
    #     tMap.transform.rotation.z = quadMap[2]
    #     tMap.transform.rotation.w = quadMap[3]

    #     brMap.sendTransform(tMap)

    #     brOdom = tf2_ros.TransformBroadcaster()
    #     tOdom = geometry_msgs.msg.TransformStamped()
    #     tOdom.header.stamp = odom.header.stamp
    #     tOdom.header.frame_id = "odom"
    #     tOdom.child_frame_id = "base_footprint"

    #     quaternion = (
    #         lastOdom.pose.pose.orientation.x,
    #         lastOdom.pose.pose.orientation.y,
    #         lastOdom.pose.pose.orientation.z,
    #         lastOdom.pose.pose.orientation.w)
    #     eulerLast = tf_conversions.transformations.euler_from_quaternion(quaternion)
    #     quadOdom = tf_conversions.transformations.quaternion_from_euler(eulerCurrent[0] - eulerLast[0], eulerCurrent[1] - eulerLast[1], eulerCurrent[2] - eulerLast[2])

    #     tOdom.transform.translation.x = odom.pose.pose.position.x - lastOdom.pose.pose.position.x
    #     tOdom.transform.translation.y = odom.pose.pose.position.y - lastOdom.pose.pose.position.y
    #     tOdom.transform.translation.z = odom.pose.pose.position.z - lastOdom.pose.pose.position.z
    #     tOdom.transform.rotation.x = quadOdom[0]
    #     tOdom.transform.rotation.y = quadOdom[1]
    #     tOdom.transform.rotation.z = quadOdom[2]
    #     tOdom.transform.rotation.w = quadOdom[3]

    #     brOdom.sendTransform(tOdom)

    # else:
    #     firstPosition = odom.pose.pose.position
    #     quaternion = (
    #         odom.pose.pose.orientation.x,
    #         odom.pose.pose.orientation.y,
    #         odom.pose.pose.orientation.z,
    #         odom.pose.pose.orientation.w)
    #     eulerFirst = tf_conversions.transformations.euler_from_quaternion(quaternion)
    #     hasFirstOdom = True
   
    # lastOdom = odom

    # g = Gps()
    # g.localization.linear_velocity


if __name__ == '__main__':
    try:
        gnss_odom_provider()
    except rospy.ROSInterruptException:
        pass

