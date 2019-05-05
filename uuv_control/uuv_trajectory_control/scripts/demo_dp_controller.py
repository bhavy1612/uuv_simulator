import rospy
from uuv_control_msgs.srv import *
import numpy as np
import uuv_waypoints
from uuv_control_msgs.msg import Waypoint as WaypointMsg 
import time
from std_msgs.msg import Time, String

if __name__ == '__main__':

    rospy.init_node('waypoint_server')

    wp1 = uuv_waypoints.Waypoint(0, 0, -2.4, 0.3, inertial_frame_id='world', use_fixed_heading=True)
    wp2 = uuv_waypoints.Waypoint(0, 5, -2.4, 0.3, inertial_frame_id='world', use_fixed_heading=True)
    wp3 = uuv_waypoints.Waypoint(3, 0, -2, 0.5, inertial_frame_id='world', use_fixed_heading=True)
    wp4 = uuv_waypoints.Waypoint(3, 3, -2, 0.5, inertial_frame_id='world', use_fixed_heading=True)

    wp1_msg = WaypointMsg()
    wp1_msg = wp1.to_message()
    wp2_msg = WaypointMsg()
    wp2_msg = wp2.to_message()
    wp3_msg = WaypointMsg()
    wp3_msg = wp3.to_message()
    wp4_msg = WaypointMsg()
    wp4_msg = wp4.to_message()

    print 'waiting for waypoint server'
    rospy.wait_for_service('anahita/start_waypoint_list')

    wp_set = [wp1_msg, wp2_msg]

    try:
        init_waypoint_set = rospy.ServiceProxy('anahita/start_waypoint_list', InitWaypointSet)
        
        start_time = Time()
        start_time.data.secs = rospy.get_rostime().to_sec()
        start_time.data.nsecs = rospy.get_rostime().to_nsec()
        
        interpolator = String()
        interpolator.data = 'cubic_interpolator'
        print 'adding waypoints....'
        resp = init_waypoint_set(start_time = start_time, 
                                start_now = True,
                                max_forward_speed = 0.5, 
                                heading_offset = 0.5, 
                                interpolator = interpolator,
                                waypoints = wp_set)

        if resp.success:
            print "waypoints successfully added"
        else:
            print "waypoints addition failed"
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e