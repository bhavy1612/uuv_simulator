#!/usr/bin/env python
import rospy
import numpy as np
from uuv_control_interfaces import DPPIDControllerBase, DPControllerBase
from uuv_control_msgs.srv import *
from PID import PIDRegulator
import math
import tf.transformations as trans

class ROV_CascadedController(DPControllerBase):
    _LABEL = 'Cascaded PID dynamic position controller'

    def __init__(self):
        DPControllerBase.__init__(self, True)
        self._logger.info('Initializing: ' + self._LABEL)

        self._force_torque = np.zeros(6)

        self._is_init = True
        self._logger.info(self._LABEL + ' ready')

        self._mass = rospy.get_param("pid/mass")
        self._inertial = rospy.get_param("pid/inertial")
        self._use_cascaded_pid = rospy.get_param("use_cascaded_pid")

        self._last_vel = np.zeros(6)
        self._last_t = None

        # update mass, moments of inertia
        self._inertial_tensor = np.array(
          [[self._inertial['ixx'], self._inertial['ixy'], self._inertial['ixz']],
           [self._inertial['ixy'], self._inertial['iyy'], self._inertial['iyz']],
           [self._inertial['ixz'], self._inertial['iyz'], self._inertial['izz']]])
        self._mass_inertial_matrix = np.vstack((
          np.hstack((self._mass*np.identity(3), np.zeros((3, 3)))),
          np.hstack((np.zeros((3, 3)), self._inertial_tensor))))
        
        self._velocity_pid = rospy.get_param('velocity_control')
        self._position_pid = rospy.get_param('position_control')
        
        self._lin_vel_pid_reg = PIDRegulator(self._velocity_pid['linear_p'], self._velocity_pid['linear_i'], self._velocity_pid['linear_d'], self._velocity_pid['linear_sat'])
        self._ang_vel_pid_reg = PIDRegulator(self._velocity_pid['angular_p'], self._velocity_pid['angular_i'], self._velocity_pid['angular_d'], self._velocity_pid['angular_sat'])

        self._lin_pos_pid_reg = PIDRegulator(self._position_pid['pos_p'], self._position_pid['pos_i'], self._position_pid['pos_d'], self._position_pid['pos_sat'])
        self._ang_pos_pid_reg = PIDRegulator(self._position_pid['rot_p'], self._position_pid['rot_i'], self._position_pid['rot_d'], self._position_pid['rot_sat'])

        self._logger.info('Cascaded PID controller ready!')


    def _reset_controller(self):
        super(ROV_CascadedController, self).reset_controller()
        self._force_torque = np.zeros(6)

    def update_controller(self):
        if not self._is_init:
            return False

        t = rospy.get_time()
        if self._last_t is None:
            self._last_t = t
            self._last_vel = self._vehicle_model._vel
            return False

        dt = t - self._last_t
        if dt <= 0:
            self._last_t = t
            self._last_vel = self._vehicle_model._vel 
            return False

        vel = np.zeros(6)
        vel = self._vehicle_model._vel

        acc = np.zeros(6)
        acc = (vel - self._last_vel) / dt

        p = self._vehicle_model._pose['pos']
        q = self._vehicle_model._pose['rot']

        # Compute control output:
        t = rospy.get_rostime().to_sec()

        # Position error
        e_pos_world = self._reference['pos'] - p
        e_pos_body = trans.quaternion_matrix(q).transpose()[0:3,0:3].dot(e_pos_world)

        # Error quaternion wrt body frame
        e_rot_quat = trans.quaternion_multiply(trans.quaternion_conjugate(q), self._reference['rot'])

        if np.linalg.norm(e_pos_world[0:2]) > 5.0:
            # special case if we are far away from goal:
            # ignore desired heading, look towards goal position
            heading = math.atan2(e_pos_world[1],e_pos_world[0])
            quat_des = np.array([0, 0, math.sin(0.5*heading), math.cos(0.5*heading)])
            e_rot_quat = trans.quaternion_multiply(trans.quaternion_conjugate(q), quat_des)

        # Error angles
        e_rot = np.array(trans.euler_from_quaternion(e_rot_quat))

        v_linear = self._lin_pos_pid_reg.regulate(e_pos_body, t)
        v_angular = self._ang_pos_pid_reg.regulate(e_rot, t)

        e_linear_vel = v_linear - np.array([vel[0], vel[1], vel[2]])
        e_angular_vel = v_angular - np.array([vel[3], vel[4], vel[5]])

        a_linear = self._lin_vel_pid_reg.regulate(e_linear_vel, t)
        a_angular = self._ang_vel_pid_reg.regulate(e_angular_vel, t)

        accel = np.hstack((a_linear, a_angular)).transpose()
        if self._use_cascaded_pid:
            self._force_torque = self._mass_inertial_matrix.dot(accel)
        else:
            self._force_torque = self._vehicle_model.compute_force(acc, vel)
                    
        # Publish control forces and torques
        self.publish_control_wrench(self._force_torque)
        self._last_t = t
        self._last_vel = self._vehicle_model._vel

        return True


if __name__ == '__main__':
    print('Starting Cascaded PID Controller')
    rospy.init_node('rov_cascaded_pid_controller')

    try:
        node = ROV_CascadedController()
        rospy.spin()
    except rospy.ROSInterruptException:
        print('caught exception')
    print('exiting')
