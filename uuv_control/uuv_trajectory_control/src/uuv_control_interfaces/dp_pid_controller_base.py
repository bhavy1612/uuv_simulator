# Copyright (c) 2016 The UUV Simulator Authors.
# All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import numpy as np
import rospy
from uuv_control_msgs.srv import *
from uuv_control_interfaces.dp_controller_base import DPControllerBase
import tf.transformations as trans
import math
from PID import PIDRegulator

class DPPIDControllerBase(DPControllerBase):
    """
    This is an abstract class for PID-based controllers. The base class method
    update_controller must be overridden in other for a controller to work.
    """

    def __init__(self, *args):
        # Start the super class
        DPControllerBase.__init__(self, *args)
        self._logger.info('Initializing: PID controller')
        # Proportional gains
        self._Kp = np.zeros(shape=(6, 6))
        # Derivative gains
        self._Kd = np.zeros(shape=(6, 6))
        # Integral gains
        self._Ki = np.zeros(shape=(6, 6))
        # Integrator component
        self._int = np.zeros(6)
        # Error for the vehicle pose
        self._error_pose = np.zeros(6)

        if rospy.has_param('~Kp'):
            Kp_diag = rospy.get_param('~Kp')
            if len(Kp_diag) == 6:
                self._Kp = np.diag(Kp_diag)
            else:
                raise rospy.ROSException('Kp matrix error: 6 coefficients '
                                         'needed')

        self._logger.info('Kp=' + str([self._Kp[i, i] for i in range(6)]))

        if rospy.has_param('~Kd'):
            Kd_diag = rospy.get_param('~Kd')
            if len(Kd_diag) == 6:
                self._Kd = np.diag(Kd_diag)
            else:
                raise rospy.ROSException('Kd matrix error: 6 coefficients '
                                         'needed')

        self._logger.info('Kd=' + str([self._Kd[i, i] for i in range(6)]))

        if rospy.has_param('~Ki'):
            Ki_diag = rospy.get_param('~Ki')
            if len(Ki_diag) == 6:
                self._Ki = np.diag(Ki_diag)
            else:
                raise rospy.ROSException('Ki matrix error: 6 coefficients '
                                         'needed')

        self._logger.info('Ki=' + str([self._Ki[i, i] for i in range(6)]))

        self._services['set_pid_params'] = rospy.Service(
            'set_pid_params',
            SetPIDParams,
            self.set_pid_params_callback)
        self._services['get_pid_params'] = rospy.Service(
            'get_pid_params',
            GetPIDParams,
            self.get_pid_params_callback)

        self._logger.info('PID controller ready!')

        self._mass = rospy.get_param("pid/mass")
        self._inertial = rospy.get_param("pid/inertial")

        # update mass, moments of inertia
        self._inertial_tensor = np.array(
          [[self._inertial['ixx'], self._inertial['ixy'], self._inertial['ixz']],
           [self._inertial['ixy'], self._inertial['iyy'], self._inertial['iyz']],
           [self._inertial['ixz'], self._inertial['iyz'], self._inertial['izz']]])
        self._mass_inertial_matrix = np.vstack((
          np.hstack((self._mass*np.identity(3), np.zeros((3, 3)))),
          np.hstack((np.zeros((3, 3)), self._inertial_tensor))))
        
        print self._mass_inertial_matrix

        self._velocity_pid = rospy.get_param('velocity_control')
        print self._velocity_pid

        self._position_pid = rospy.get_param('position_control')
        print self._position_pid

        self._lin_vel_pid_reg = PIDRegulator(self._velocity_pid['linear_p'], self._velocity_pid['linear_i'], self._velocity_pid['linear_d'], 2)
        self._ang_vel_pid_reg = PIDRegulator(0, 0, 0, 1)
        self._lin_pos_pid_reg = PIDRegulator(self._position_pid['pos_p'], self._position_pid['pos_i'], self._position_pid['pos_d'], self._position_pid['pos_sat'])
        self._ang_pos_pid_reg = PIDRegulator(self._position_pid['rot_p'], self._position_pid['rot_i'], self._position_pid['rot_d'], self._position_pid['rot_sat'])


    def _reset_controller(self):
        super(DPPIDControllerBase, self)._reset_controller()
        self._error_pose = np.zeros(6)
        self._int = np.zeros(6)

    def set_pid_params_callback(self, request):
        kp = request.Kp
        kd = request.Kd
        ki = request.Ki
        if len(kp) != 6 or len(kd) != 6 or len(ki) != 6:
            return SetPIDParamsResponse(False)
        self._Kp = np.diag(kp)
        self._Ki = np.diag(ki)
        self._Kd = np.diag(kd)
        return SetPIDParamsResponse(True)

    def get_pid_params_callback(self, request):
        return GetPIDParamsResponse(
            [self._Kp[i, i] for i in range(6)],
            [self._Kd[i, i] for i in range(6)],
            [self._Ki[i, i] for i in range(6)])

    def update_pid(self):
        if not self.odom_is_init:
            return

        return self.position_control()

        # Update integrator
        # self._int += 0.5 * (self.error_pose_euler + self._error_pose) * self._dt
        # Store current pose error
        # self._error_pose = self.error_pose_euler
        # return np.dot(self._Kp, self.error_pose_euler) \
        #     + np.dot(self._Kd, self._errors['vel']) \
        #     + np.dot(self._Ki, self._int)

    def position_control(self):

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

        e_linear_vel = v_linear - np.array([self._vehicle_model._vel[0], self._vehicle_model._vel[1], self._vehicle_model._vel[2]])
        e_angular_vel = v_angular - np.array([self._vehicle_model._vel[3], self._vehicle_model._vel[4], self._vehicle_model._vel[5]])

        a_linear = self._lin_vel_pid_reg.regulate(e_linear_vel, t)
        
        a_angular = self._ang_vel_pid_reg.regulate(e_angular_vel, t)

        accel = np.hstack((a_linear, a_angular)).transpose()
        force_torque = self._mass_inertial_matrix.dot(accel)

        return force_torque
        
