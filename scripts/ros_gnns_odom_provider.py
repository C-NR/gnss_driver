#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from gnss_driver.msg import Gps
import math

# Because of transformations
import tf_conversions
import tf2_ros
import geometry_msgs.msg
from geographic_msgs.msg import GeoPoint
from geodesy.utm import UTMPoint
from sensor_msgs.msg import NavSatFix


def gnss_odom_provider():
    global pubOdom, hasFirstOdom, firstPosition, eulerFirst, lastOdom, pubTwist, pubOrigin, pubOdomV, pubPos, pubOdomRel, offset, lastDiff

    hasFirstOdom = False
    firstPosition = geometry_msgs.msg.Vector3()
    lastOdom = Odometry()
    eulerFirst = [0,0,0]
    offset = [0, 0 ,0]
    lastDiff = 0

    rospy.init_node('gnss_odom_provider', anonymous=True)
    rate = rospy.Rate(100) # hz
    ID = str(abs(hash(rospy.get_caller_id())) % (10 ** 8))
    rospy.loginfo('gnss_odom_provider ' + str(ID) + ': initialized')

    rospy.Subscriber('/gnss_driver/odometry', Gps, callbackGnssOdom)
    rospy.Subscriber('/vehicle/twist', geometry_msgs.msg.TwistStamped, callbackTwist)

    rospy.Subscriber('/planning/closest_point_on_lane', geometry_msgs.msg.Vector3, callbackOffset)
   
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
    pubOdomRel = rospy.Publisher('/odometry/gnss_relative', Odometry, queue_size = 10)
    # pubOdomV = rospy.Publisher('/odometry/vehicle', Odometry, queue_size = 10)
    pubTwist = rospy.Publisher('/odometry/twist', geometry_msgs.msg.TwistWithCovarianceStamped, queue_size = 10)
    pubOrigin = rospy.Publisher('/localization/map_origin', GeoPoint, queue_size = 10)
    pubPos = rospy.Publisher('/localization/gnss_fix', NavSatFix, queue_size = 10)

    rospy.Timer(rospy.Duration(.1), publishOrigin)

    while not rospy.is_shutdown():
        rate.sleep()

def publishOrigin(timerStats):
    global pubOrigin, pubPos
    if hasFirstOdom:
        pUtm = UTMPoint(easting=firstPosition.x, northing=firstPosition.y, altitude=firstPosition.z, zone=11, band='S')
        pubOrigin.publish( pUtm.toMsg())
    pUtm = UTMPoint(easting= lastOdom.pose.pose.position.x + firstPosition.x, northing= lastOdom.pose.pose.position.y + firstPosition.y, altitude= lastOdom.pose.pose.position.z + firstPosition.z, zone=11, band='S')
    gp = pUtm.toMsg()
    fix = NavSatFix()
    fix.latitude = gp.latitude
    fix.longitude = gp.longitude
    fix.altitude = gp.altitude
    pubPos.publish(fix)

    # conv = getMeridianConvergence(gp.latitude, gp.longitude)
    # print("meridian convergence: " + str(conv))

def callbackOffset(closestLanePoint):
    global offset, lastDiff, diff
    f = 0.1
    diff = lastOdom.pose.pose.position.y - closestLanePoint.y
    if diff > 0.1:
        f = 0.00001
        #diff = 0
    offset[1] = offset[1] - f * diff
    # offset[1] = offset[1] - (f * diff + (1.0 - f) * lastDiff)
    lastDiff = diff

def callbackTwist(twist):
    global pubTwist
    twistCov = geometry_msgs.msg.TwistWithCovarianceStamped()
    twistCov.header = twist.header
    twistCov.twist.twist = twist.twist
    pubTwist.publish(twistCov)


def callbackGnssOdom(gpsOdom):
    global pubOdom, hasFirstOdom, firstPosition, eulerFirst, lastOdom, pubOrigin, offset

    odom = Odometry()
    pUtm = UTMPoint(easting=gpsOdom.localization.position.x, northing=gpsOdom.localization.position.y, altitude=gpsOdom.localization.position.z, zone=11, band='S')
    gp = pUtm.toMsg()
    fix = NavSatFix()
    fix.latitude = gp.latitude
    fix.longitude = gp.longitude
    fix.altitude = gp.altitude
    conv = getMeridianConvergence(gp.latitude, gp.longitude)
    # print("meridian convergence: " + str(conv))

    # offset[0] = offset[0] + 0.005
    # offset[1] = offset[1] + 0.005
    gpsOdom.localization.position.x = gpsOdom.localization.position.x + offset[0]
    gpsOdom.localization.position.y = gpsOdom.localization.position.y + offset[1]
    gpsOdom.localization.position.z = gpsOdom.localization.position.z + offset[2]

    if hasFirstOdom:
        #Position
        odom.pose.pose.position.x = gpsOdom.localization.position.x - firstPosition.x
        odom.pose.pose.position.y = gpsOdom.localization.position.y - firstPosition.y
        odom.pose.pose.position.z = gpsOdom.localization.position.z - firstPosition.z
        #Orientation
        quaternion = (
            gpsOdom.localization.orientation.qx,
            gpsOdom.localization.orientation.qy,
            gpsOdom.localization.orientation.qz,
            gpsOdom.localization.orientation.qw)
        euler = tf_conversions.transformations.euler_from_quaternion(quaternion)
        q = tf_conversions.transformations.quaternion_from_euler(euler[0], euler[1], euler[2] + math.pi/2 - conv)       
        odom.pose.pose.orientation.x = q[0]
        odom.pose.pose.orientation.y = q[1]
        odom.pose.pose.orientation.z = q[2]
        odom.pose.pose.orientation.w = q[3]
    else:
        firstPosition.x = gpsOdom.localization.position.x
        firstPosition.y = gpsOdom.localization.position.y
        firstPosition.z = gpsOdom.localization.position.z
        odom.pose.pose.position.x = 0
        odom.pose.pose.position.y = 0
        odom.pose.pose.position.z = 0
        q = tf_conversions.transformations.quaternion_from_euler(0, 0, math.pi/2 - conv)       
        odom.pose.pose.orientation.x = q[0]
        odom.pose.pose.orientation.y = q[1]
        odom.pose.pose.orientation.z = q[2]
        odom.pose.pose.orientation.w = q[3]
        hasFirstOdom = True


    relOdom = Odometry() #odom
    relOdom.pose.pose.position.x = 0#relOdom.pose.pose.position.x - lastOdom.pose.pose.position.x
    relOdom.pose.pose.position.y = 0#relOdom.pose.pose.position.y - lastOdom.pose.pose.position.y
    relOdom.pose.pose.position.z = 0#relOdom.pose.pose.position.z - lastOdom.pose.pose.position.z
    q = tf_conversions.transformations.quaternion_from_euler(0, 0, 0)       
    relOdom.pose.pose.orientation.x = q[0]
    relOdom.pose.pose.orientation.y = q[1]
    relOdom.pose.pose.orientation.z = q[2]
    relOdom.pose.pose.orientation.w = q[3]
    relOdom.header = gpsOdom.header
    relOdom.header.frame_id = "odom"
    relOdom.child_frame_id = "base_footprint"

    pubOdomRel.publish(relOdom)

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

    lastOdom = odom


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

def getMeridianConvergence(longitude, latitude):
    lambda_ = longitude
    phi = latitude

    latId  = int(math.floor(phi))
    longId = int(math.floor(lambda_))

    lambda_0 = float(longId / 6) * 6.0

    #west of zero meridian
    if longitude < 0.0:
        lambda_0 -= 3.0
    #east of zero meridian
    else:
        lambda_0 += 3.0

    #exception for Norway
    if latId >= 56 and latId < 64:
        #sea area west of Norway
        if longId >= 0 and longId < 3:
            lambda_0 = 1.5
        #Norway
        elif longId >= 3 and longId < 12:
            lambda_0 = 7.5

    #exception for Svalbard
    elif latId >= 72:  
        #sea area west of Svalbard
        if longId >= 0 and longId < 9:     
            lambda_0 = 4.5      
        #western Svalbard
        elif longId >= 9 and longId < 21:     
            lambda_0 = 15.0      
        #eastern Svalbard
        elif longId >= 21 and longId < 33:     
            lambda_0 = 27.0      
        #sea area east of Svalbard
        elif longId >= 33 and longId < 42:      
            lambda_0 = 37.5
      
    phi = phi * math.pi / 180.0
    lambda_ = lambda_ * math.pi / 180.0
    lambda_0 = lambda_0 * math.pi / 180.0

    return math.atan(math.tan(lambda_ - lambda_0) * math.sin(phi))
  

if __name__ == '__main__':
    try:
        gnss_odom_provider()
    except rospy.ROSInterruptException:
        pass

