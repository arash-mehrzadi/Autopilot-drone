# -*- coding: utf-8 -*-
import math
import sys
import time
from math import sqrt, pi

import numpy as np

import sim
import simConst

PI = math.pi


def angle_between_vectors(a, b):  # a -> b

    a = a.unit_vector()
    b = b.unit_vector()
    angle = math.acos(b.dot(a))
    if (a.multiply(b)).z > 0.0:
        return -angle
    return angle

def distance_between_points(pos_1, pos_2):
    return math.sqrt((pos_1.x - pos_2.x) ** 2 + (pos_1.y - pos_2.y) ** 2)

class States:
    MOVING = 1
    ROTATING = 2
    ROUNDING = 3


class EulerAngles:
    def __init__(self, roll=0.0, pitch=0.0, yaw=0.0):
        self.roll = roll
        self.pitch = pitch
        self.yaw = yaw

        self.rad_to_deg = 180.0 / pi
        self.deg_to_rad = 1.0 / self.rad_to_deg


class Vector3:
    def __init__(self, x=0.0, y=0.0, z=0.0):
        self.x = x
        self.y = y
        self.z = z

    def minus(self, vect):
        res = Vector3()
        res.x = self.x - vect.x
        res.y = self.y - vect.y
        res.z = self.z - vect.z

        return res

    def multiply(self, vect):
        res = Vector3()
        res.x = self.y * vect.z - self.z * vect.y
        res.y = self.z * vect.x - self.x * vect.z
        res.z = self.x * vect.y - self.y * vect.x

        return res

    def dot(self, vect):
        return self.x * vect.x + self.y * vect.y + self.z * vect.z

    def length(self):
        return sqrt(self.x ** 2 + self.y ** 2 + self.z ** 2)

    def unit_vector(self):
        n = 1.0 / self.length()
        return Vector3(x=self.x * n, y=self.y * n, z=self.z * n)



class Matrix3:
    def __init__(self, m=None):
        if m is None:
            self.m = [[0., 0., 0.], [0., 0., 0.], [0., 0., 0.]]
        else:
            self.m = m

    def determinant(self):
        return self.m[0][0] * self.m[1][1] * self.m[2][2] - self.m[0][0] * self.m[1][2] * self.m[2][1] - self.m[0][1] * \
               self.m[1][0] * self.m[2][2] + self.m[0][1] * self.m[1][2] * self.m[2][0] + self.m[0][2] * self.m[1][0] * \
               self.m[2][1] - self.m[0][2] * self.m[1][1] * self.m[2][0]

    def inverse(self):
        inv = Matrix3()
        det = self.m[0][0] * self.m[1][1] * self.m[2][2] - self.m[0][0] * self.m[1][2] * self.m[2][1] - self.m[0][1] * \
              self.m[1][0] * self.m[2][2] + self.m[0][1] * self.m[1][2] * self.m[2][0] + self.m[0][2] * self.m[1][0] * \
              self.m[2][1] - self.m[0][2] * self.m[1][1] * self.m[2][0]
        invDet = 1.0 / det

        inv.m[0][0] = (self.m[1][1] * self.m[2][2] - self.m[1][2] * self.m[2][1]) * invDet
        inv.m[0][1] = (self.m[0][2] * self.m[2][1] - self.m[0][1] * self.m[2][2]) * invDet
        inv.m[0][2] = (self.m[0][1] * self.m[1][2] - self.m[0][2] * self.m[1][1]) * invDet

        inv.m[1][0] = (self.m[2][0] * self.m[1][2] - self.m[1][0] * self.m[2][2]) * invDet
        inv.m[1][1] = (self.m[0][0] * self.m[2][2] - self.m[0][2] * self.m[2][0]) * invDet
        inv.m[1][2] = (self.m[0][2] * self.m[1][0] - self.m[0][0] * self.m[1][2]) * invDet

        inv.m[2][0] = (self.m[1][0] * self.m[2][1] - self.m[1][1] * self.m[2][0]) * invDet
        inv.m[2][1] = (self.m[0][1] * self.m[2][0] - self.m[0][0] * self.m[2][1]) * invDet
        inv.m[2][2] = (self.m[0][0] * self.m[1][1] - self.m[0][1] * self.m[1][0]) * invDet

        return inv

    def multiply(self, v):
        res = Vector3()
        res.x = self.m[0][0] * v[0] + self.m[0][1] * v[1] + self.m[0][2] * v[2]
        res.y = self.m[1][0] * v[0] + self.m[1][1] * v[1] + self.m[1][2] * v[2]
        res.z = self.m[2][0] * v[0] + self.m[2][1] * v[1] + self.m[2][2] * v[2]

        return res

    def toQuaternion(self):

        q = Quaternion()

        t = self.m[0][0] + self.m[1][1] + self.m[2][2] + 1

        if t > 0:
            s = 0.5 / math.sqrt(t)
            q.w = 0.25 / s
            q.x = (self.m[2][1] - self.m[1][2]) * s
            q.y = (self.m[0][2] - self.m[2][0]) * s
            q.z = (self.m[1][0] - self.m[0][1]) * s

            return q

        max = self.m[0][0]

        iColMax = 0

        for iRow in range(0, 3):
            for iCol in range(0, 3):
                if self.m[iRow][iCol] > max:
                    max = self.m[iRow][iCol]
                    iColMax = iCol

        if iColMax == 0:
            s = 1.0 / (math.sqrt(1.0 + self.m[0][0] - self.m[1][1] - self.m[2][2]) * 2.0)
            q.w = (self.m[1][2] + self.m[2][1]) * s
            q.x = 0.5 * s
            q.y = (self.m[0][1] + self.m[1][0]) * s
            q.z = (self.m[0][2] + self.m[2][0]) * s
        elif iColMax == 1:
            s = 1.0 / (math.sqrt(1.0 + self.m[1][1] - self.m[0][0] - self.m[2][2]) * 2.0)
            q.w = (self.m[0][2] + self.m[2][0]) * s
            q.x = (self.m[0][1] + self.m[1][0]) * s
            q.y = 0.5 * s
            q.z = (self.m[1][2] + self.m[2][1]) * s
        else:
            s = 1.0 / (math.sqrt(1.0 + self.m[2][2] - self.m[0][0] - self.m[1][1]) * 2.0)
            q.w = (self.m[0][1] + self.m[1][0]) * s
            q.x = (self.m[0][2] + self.m[2][0]) * s
            q.y = (self.m[1][2] + self.m[2][1]) * s
            q.z = 0.5 * s

        return q


class Quaternion:
    def __init__(self, w=1.0, x=0.0, y=0.0, z=0.0):
        self.w = w
        self.x = x
        self.y = y
        self.z = z

    def set_from_vector(self, angle, dir):
        half_angle = angle / 2.0
        sin_half_angle = math.sin(half_angle)
        self.w = math.cos(half_angle)
        self.x = sin_half_angle * dir.x
        self.y = sin_half_angle * dir.y
        self.z = sin_half_angle * dir.z

    def multiply(self, q):
        res = Quaternion()
        res.w = self.w * q.w - self.x * q.x - self.y * q.y - self.z * q.z
        res.x = self.w * q.x + self.x * q.w + self.y * q.z - self.z * q.y
        res.y = self.w * q.y - self.x * q.z + self.y * q.w + self.z * q.x
        res.z = self.w * q.z + self.x * q.y - self.y * q.x + self.z * q.w
        return res

    def multiply_to_number(self, num):
        res = Quaternion()
        res.w = self.w * num
        res.x = self.x * num
        res.y = self.y * num
        res.z = self.z * num
        return res

    def plus(self, q):
        res = Quaternion()
        res.w = self.w + q.w
        res.x = self.x + q.x
        res.y = self.y + q.y
        res.z = self.z + q.z

        return res

    def get_by_index(self, index):
        if index == 0:
            return self.w
        elif index == 1:
            return self.x
        elif index == 2:
            return self.y
        elif index == 3:
            return self.z

    def dot(self, q):
        return self.w * q.w + self.x * q.x + self.y * q.y + self.z * q.z

    def norm(self):
        return self.x ** 2 + self.y ** 2 + self.z ** 2 + self.w ** 2

    def magnitude(self):
        return math.sqrt(self.norm())

    def conjugate(self):
        res = Quaternion()
        res.w = self.w
        res.x = - self.x
        res.y = - self.y
        res.z = - self.z

        return res

    def inverse(self):
        res = Quaternion()
        n = self.norm()
        res.w = self.w / n
        res.x = - self.x / n
        res.y = - self.y / n
        res.z = -self.z / n
        return res

    def rotate(self, v):
        vect_quad = Quaternion(w=0.0, x=v.x, y=v.y, z=v.z)
        q1 = self.multiply(vect_quad)
        res = q1.multiply(self.inverse())
        return Vector3(x=res.x, y=res.y, z=res.z)

    def toMatrix3(self):
        matr = Matrix3()

        xx = self.x ** 2
        xy = self.x * self.y
        xz = self.x * self.z
        xw = self.x * self.w

        yy = self.y ** 2
        yz = self.y * self.z
        yw = self.y * self.w

        zz = self.z ** 2
        zw = self.z * self.w

        matr.m[0][0] = 1 - 2 * (yy + zz)
        matr.m[0][1] = 2 * (xy - zw)
        matr.m[0][2] = 2 * (xz + yw)

        matr.m[1][0] = 2 * (xy + zw)
        matr.m[1][1] = 1 - 2 * (xx + zz)
        matr.m[1][2] = 2 * (yz - xw)

        matr.m[2][0] = 2 * (xz - yw)
        matr.m[2][1] = 2 * (yz + xw)
        matr.m[2][2] = 1 - 2 * (xx + yy)

        return matr

    def to_euler_angles(self):
        angles = EulerAngles()

        norm = self.norm()
        test = self.x * self.y + self.z * self.w
        abs_test = math.fabs(test)
        if abs_test < 0.005 * norm:
            if test > 0:
                # singularity at north pole
                angles.yaw = -2.0 * math.atan2(self.x, self.w)
                angles.roll = 0
                angles.pitch = math.pi / 2.0

            else:
                # singularity at south pole
                angles.yaw = 2.0 * math.atan2(self.x, self.w)
                angles.roll = 0
                angles.pitch = - math.pi / 2.0

            return angles

        m = 2.0 * (self.x * self.z - self.w * self.y)

        angles.roll = math.atan2(2.0 * (self.y * self.z + self.w * self.x), 2.0 * (self.w ** 2 + self.z ** 2) - 1.0)
        angles.pitch = - math.atan( m / math.sqrt(1.0 - m ** 2))
        angles.yaw = math.atan2(2.0 * (self.x * self.y + self.w * self.z), 2.0 * (self.w ** 2 + self.x ** 2) - 1.0)

        return angles

class PIDController:
    def __init__(self, frequency):
        self.frequency = frequency
        self.k_p = 0.0
        self.k_i = 0.0
        self.k_d = 0.0
        self.e_prev = 0.0
        self.sum_err = 0.0
        self.max_sum_err = 3.4028234664e+38
        self.p = 0.0
        self.i = 0.0
        self.d = 0.0

    def set_coefficients(self, k_p, k_i, k_d ):
        self.k_p = k_p
        self.k_i = k_i / self.frequency
        self.k_d = k_d * self.frequency

    def output(self, current_err):
        self.sum_err += current_err
        if self.sum_err > self.max_sum_err:
            self.sum_err = self.max_sum_err
        self.p = self.k_p * current_err
        self.i = self.k_i * self.sum_err
        self.d = self.k_d * (current_err - self.e_prev)
        u = self.p + self.i + self.d
        self.e_prev = current_err
        return u

class Robot:
    def __init__(self, host='127.0.0.1', port=19999):

        self._hostname = host
        self._port = port

        self._sleep_time = 0.05

        self._client_id = self._init_client_id()
        self._left_MT_handle, self._right_MT_handle = self._init_MTs_handles()
        self._robot_handle = self._init_robot_handle()
        self._sensors_handle, self._sensor_values = self._init_sensors_handle()
        self._target_handle = self._init_target_handle()
        self._target_position = self._init_target_position()
        self._robot_orientation = self._init_robot_orientation()
        self._robot_position = self._init_robot_position()

        # orientation of all the sensors:
        self._sensor_loc = np.array([-PI/2, -50/180.0 * PI, -30/180.0 * PI, -10/180.0 * PI, 10/180.0 * PI,
                                     30/180.0 * PI, 50/180.0 * PI, PI/2, PI/2, 130/180.0 * PI, 150/180.0 * PI,
                                     170/180.0 * PI, -170/180.0 * PI, -150/180.0 * PI, -130/180.0 * PI, -PI/2])

    def _init_client_id(self):

        client_id = sim.simxStart(self._hostname, self._port, True, True, 5000, 5)

        if client_id == -1:
            # print 'Connection not successful'
            sys.exit('Could not connect')
        else:
            # print 'Connected to remote API server'

        return client_id

    def update(self):
        while(True):
            v_l, v_r = self.step()

            self.move(v_l, v_r)

            # other stuff

    def move_quad(self, pos):
        # Initialize IMU
        err, self.quadTargetHandle = sim.simxGetObjectHandle(
            self.clientID, 'Quadricopter_target', sim.simx_opmode_blocking)
        # Sets quadcopter position
        sim.simxSetObjectPosition(
            self.clientID, self.quadTargetHandle, -1, pos, sim.simx_opmode_oneshot)

    def step(self):
        #
        # Check if we detect something
        #
        #
        return 1, 1

    def _init_target_position(self):
        self.sleep()
        error_code, target_position = sim.simxGetObjectPosition(self._client_id, self._target_handle, -1, sim.simx_opmode_streaming)
        return target_position

    def read_target_position(self):
        self.sleep()
        error_code, target_position = sim.simxGetObjectPosition(self._client_id, self._target_handle, -1, sim.simx_opmode_buffer)
        return target_position

    def _init_robot_position(self):
        self.sleep()
        error_code, robot_position = sim.simxGetObjectPosition(self._client_id, self._robot_handle, -1, sim.simx_opmode_streaming)
        return robot_position

    def _init_robot_orientation(self):
        self.sleep()
        error_code, euler_angles = sim.simxGetObjectOrientation(self._client_id, self._robot_handle, -1, sim.simx_opmode_streaming)
        return euler_angles

    def read_robot_orientation(self):
        self.sleep()
        error_code, euler_angles = sim.simxGetObjectOrientation(self._client_id, self._robot_handle, -1, sim.simx_opmode_buffer)
        return euler_angles

    def read_robot_position(self):
        self.sleep()
        error_code, robot_position = sim.simxGetObjectPosition(self._client_id, self._robot_handle, -1, sim.simx_opmode_buffer)
        return robot_position

    def _rotate_to_zero(self):

        print("Rotate to Zero degree...")

        self.sleep()

        rot1, rot2 = 0, 0

        min_speed = 0.1
        delta = 0.005

        ornt = self.read_robot_orientation()
        norm = np.linalg.norm(ornt)

        if ornt[0] > 0:
            rot1, rot2 = min_speed, -min_speed  # поворачивать против часовой
        else:
            rot1, rot2 = -min_speed, min_speed

        while( delta < norm or norm > delta):

            angles = self.read_robot_orientation()

            norm = np.linalg.norm(angles)

            self.move(rot1, rot2)

        self.stop_move()

    def _rotate_at_90_degrees(self):

        print("Rotate at 90 degrees...")

        foo = self.read_robot_orientation()
        ornt = np.linalg.norm(foo)
        norm = ornt
        rad_90 = self.deg2rad(90)
        rad_180 = self.deg2rad(180)

        if foo[0] > 0:
            if ornt > rad_90:
                tmp = rad_180 - ornt
                ornt = rad_90 - tmp
            else:
                ornt = rad_90 + ornt
        else:
            if ornt < rad_90:
                ornt = rad_90 - ornt
            else:
                tmp = rad_180 - ornt
                ornt = rad_180 - (rad_90 - tmp)

        desp = 0.005
        speed = 0.02
        min = ornt - desp
        max = ornt + desp
        while not (norm > min and norm < max):
            self.move(-speed, speed)
            norm = np.linalg.norm(self.read_robot_orientation())
#             print(norm, ornt)

    def rotate_robot_to_target(self):

        print("Rotate to target...")

        self._rotate_to_zero()

        alpha = self._get_angles_between_robot_and_target()

        print("ALPHA: " + str(alpha))

        sign = True  # right side

        rad_alpha = 0

        if alpha < 180:
            sign = True
            rad_alpha = self.deg2rad(alpha)
        elif alpha >= 180:
            sign = False
            rad_alpha = self.deg2rad(alpha - 180)

        norm = 0
        desp = 0.005
        speed = 0.02
        min = rad_alpha - desp
        max = rad_alpha + desp

        while not (norm > min and norm < max):
            if sign:
                self.move(speed, -speed)
            else:
                self.move(-speed, speed)

            norm = np.linalg.norm(self.read_robot_orientation())
#             print(norm, rad_alpha)

    def _get_angles_between_robot_and_target(self):

        self.sleep()

        x0, y0, _ = self.read_robot_position()
        x1, y1, _ = self.read_target_position()

        delta_y, delta_x = 0, 0

        if y0 > y1 and x0 > x1:
            delta_y = y1 - y0
            delta_x = x1 - x0
        else:
            delta_y = y0 - y1
            delta_x = x0 - x1

        alpha = math.atan2(delta_y, delta_x) / PI * 180

        if alpha < 0:
            alpha += 360

        return alpha

    def rad2deg(self, rad):
        return rad * 180 / PI

    def deg2rad(self, deg):
        return deg * PI / 180

    def read_data_from_face_sensors(self):
        #
        # Read data from first 8 sensors
        #
        sensor_val = np.array([])

        for x in xrange(1, 8 + 1):
            error_code, detection_state, detected_point, detected_object_handle, detected_surface_normal_vector = sim.simxReadProximitySensor(self._client_id, self._sensors_handle[x], sim.simx_opmode_buffer)
            sensor_val = np.append(sensor_val, np.linalg.norm(detected_point))
        return sensor_val

    def _init_robot_handle(self):
        error_code, robot_handle = sim.simxGetObjectHandle(self._client_id, 'Pioneer_p3dx', sim.simx_opmode_oneshot_wait)
        return robot_handle

    def _init_MTs_handles(self):
        error_code, left_motor_handle  = sim.simxGetObjectHandle(self._client_id, 'Pioneer_p3dx_leftMotor', sim.simx_opmode_oneshot_wait)
        error_code, right_motor_handle = sim.simxGetObjectHandle(self._client_id, 'Pioneer_p3dx_rightMotor', sim.simx_opmode_oneshot_wait)
        # @TODO: add checks for error codes
        return left_motor_handle, right_motor_handle

    def get_left_MT_handle(self):
        return self._left_MT_handle

    def get_right_MT_handle(self):
        return self._right_MT_handle

    def get_client_id(self):
        return self._client_id

    def init_sensors(self):
        # Initialize IMU
        err, self.quadHandle = sim.simxGetObjectHandle(
            self.clientID, 'Quadricopter', sim.simx_opmode_blocking)

        # Initialize Rotors
        sim_rotors.init_rotors(self.clientID)

        # Reset quadcopter position
        err, self.pos = sim.simxGetObjectPosition(
            self.clientID, self.quadHandle, -1, sim.simx_opmode_buffer)
        self.pos[2] = 1.0
        sim.simxSetObjectPosition(
            self.clientID, self.quadHandle, -1, self.pos, sim.simx_opmode_oneshot)
        err, self.orig_location = sim.simxGetObjectPosition(
            self.clientID, self.quadHandle, -1, sim.simx_opmode_buffer)

    def stop_move(self):
        error_code = sim.simxSetJointTargetVelocity(self._client_id, self._left_MT_handle,  0, sim.simx_opmode_streaming)
        error_code = sim.simxSetJointTargetVelocity(self._client_id, self._right_MT_handle, 0, sim.simx_opmode_streaming)

    def sleep(self, sec=None):
        if sec is None:
            sec = self._sleep_time
        time.sleep(sec)

    def _init_target_handle(self):
        error_code, target_h = sim.simxGetObjectHandle(self._client_id, 'target', sim.simx_opmode_oneshot_wait)
        return target_h

class quad_helper(object):
    def __init__(self, clientID):
        self.clientID = clientID
        self.quadHandle = None
        self.pos = [0, 0, 0]
        self.rotor_data = [0.0, 0.0, 0.0, 0.0]
        self.orig_location = [0, 0, 0]
        self.curr_location = [0, 0, 0]
        self.target_z = 0.0

        '''
        Initializing angular rate PID controller
        '''
        # Tuned by plotting output graphs in vrep
        self.P_roll_rate = 0.1
        self.P_pitch_rate = 0.1
        self.P_yaw_rate = 0.002
        self.D_roll_rate = 0.001
        self.D_pitch_rate = 0.001
        self.D_yaw_rate = 0

        self.roll_rate_pid = PID.PID(self.P_roll_rate, 0, self.D_roll_rate)
        self.pitch_rate_pid = PID.PID(self.P_pitch_rate, 0, self.D_pitch_rate)
        self.yaw_rate_pid = PID.PID(self.P_yaw_rate, 0, self.D_yaw_rate)

        self.roll_rate_pid.SetPoint = 0
        self.pitch_rate_pid.SetPoint = 0
        self.yaw_rate_pid.SetPoint = 0

        '''
        Initializing attitude PID controller
        '''
        # Tuned by plotting output graphs in vrep
        self.P_roll = 0.6
        self.P_pitch = 0.6
        self.P_yaw = 0.25
        self.P_height = 0.2
        self.D_roll = 0.1
        self.D_pitch = 0.1
        self.D_yaw = 0
        self.D_height = 0.1

        self.roll_pid = PID.PID(self.P_roll, 0, self.D_roll)
        self.pitch_pid = PID.PID(self.P_pitch, 0, self.D_pitch)
        self.yaw_pid = PID.PID(self.P_yaw, 0, self.D_yaw)
        self.height_pid = PID.PID(self.P_height, 0, self.D_height)

        self.roll_pid.SetPoint = 0
        self.pitch_pid.SetPoint = 0
        self.yaw_pid.SetPoint = 0
        self.height_pid.SetPoint = 2

        '''
        Initializing position PID controller
        '''
        # Tuned by plotting output graphs in vrep
        self.P_x = 0.02
        self.P_y = 0.02
        self.I_x = 0.003
        self.I_y = 0.003
        self.D_x = 0.03
        self.D_y = 0.03

        self.x_pid = PID.PID(self.P_x, self.I_x, self.D_x)
        self.y_pid = PID.PID(self.P_y, self.I_y, self.D_y)

        self.x_pid.SetPoint = 0
        self.y_pid.SetPoint = 0

    def _init_client_id(self):
        sim.simxFinish(-1)

        self.client_id = sim.simxStart('127.0.0.1', 19999, True, True, 5000, 5)

        if self.client_id != -1:
            print('Connected to remote API server')

        else:
             print('Connection not successful')
            sys.exit('Could not connect')

    def _init_handles(self):

        self._init_MTs_handle()

        self._init_target_handle()

        self._init_robot_handle()

    def _init_robot_handle(self):
        # handle of robot
        error_code, self.bot_handle = sim.simxGetObjectHandle(self.client_id, self.BOT_NAME, sim.simx_opmode_oneshot_wait)

    def _init_target_handle(self):
        # get handle of target robot
        error_code, self.target_handle = sim.simxGetObjectHandle(self.client_id, self.TARGET_NAME, sim.simx_opmode_oneshot_wait)

    def _init_MTs_handle(self):
        # get handles of robot MTs
        error_code, self.left_motor_handle = sim.simxGetObjectHandle(self.client_id, 'Pioneer_p3dx_leftMotor', sim.simx_opmode_oneshot_wait)
        error_code, self.right_motor_handle = sim.simxGetObjectHandle(self.client_id, 'Pioneer_p3dx_rightMotor', sim.simx_opmode_oneshot_wait)

    def _init_sensor_handles(self):

        self.sensor_handles = []  # empty list for handles

        for x in range(1, 16 + 1):
            error_code, sensor_handle = sim.simxGetObjectHandle(self.client_id, 'Pioneer_p3dx_ultrasonicSensor' + str(x), sim.simx_opmode_oneshot_wait)

            self.sensor_handles.append(sensor_handle)

            sim.simxReadProximitySensor(self.client_id, sensor_handle, sim.simx_opmode_streaming)

    def _init_values(self):

        error_code, _ = sim.simxGetObjectPosition(self.client_id, self.target_handle, -1, sim.simx_opmode_oneshot)

        error_code, _ = sim.simxGetObjectPosition(self.client_id, self.bot_handle, -1, sim.simx_opmode_oneshot)

        error_code, _ = sim.simxGetObjectOrientation(self.client_id, self.bot_handle, -1, sim.simx_opmode_streaming)

    def read_values(self):

        error_code, target_pos = sim.simxGetObjectPosition(self.client_id, self.target_handle, -1, sim.simx_opmode_streaming)
        self.target_pos = Vector3(x=target_pos[0], y=target_pos[1], z=target_pos[2])

        error_code, bot_pos = sim.simxGetObjectPosition(self.client_id, self.bot_handle, -1, sim.simx_opmode_streaming)
        self.bot_pos = Vector3(x=bot_pos[0], y=bot_pos[1], z=bot_pos[2])

        error_code, bot_euler_angles = sim.simxGetObjectOrientation(self.client_id, self.bot_handle, -1, sim.simx_opmode_streaming)
        self.bot_euler_angles = Vector3(x=bot_euler_angles[0], y=bot_euler_angles[1], z=bot_euler_angles[2])

    def stop_move(self):
        error_code = sim.simxSetJointTargetVelocity(self.client_id, self.left_motor_handle,  0, sim.simx_opmode_streaming)
        error_code = sim.simxSetJointTargetVelocity(self.client_id, self.right_motor_handle, 0, sim.simx_opmode_streaming)

    def read_from_sensors(self):

        for i in range(0, 16):

            error_code, detection_state, detected_point, detected_object_handle, detected_surface_normal_vector = sim.simxReadProximitySensor(self.client_id, self.sensor_handles[i], sim.simx_opmode_streaming)

            dist = math.sqrt(detected_point[0] ** 2 + detected_point[1] ** 2 + detected_point[2] ** 2)

            if dist < self.MIN_DETECTION_DIST:
                self.detect[i] = self.MIN_DETECTION_DIST
            elif dist > self.MAX_DETECTION_DIST or detection_state is False:
                self.detect[i] = self.MAX_DETECTION_DIST
            else:
                self.detect[i] = self.MAX_DETECTION_DIST - ((dist - self.MAX_DETECTION_DIST) / (self.MIN_DETECTION_DIST - self.MAX_DETECTION_DIST))

     def print_about_info(self):
        print("Algorithm: {0}\nTarget name: {1}\nBot name: {2}\nSpeed of MT: {3}".format(self.about, self.TARGET_NAME, self.BOT_NAME, self.MT_SPEED))

    def calc_lenght_of_robot_track(self):

        if self.prev_pos is None:
            self.prev_pos = self.bot_pos

        self.track_length += distance_between_points(self.prev_pos, self.bot_pos)
        self.prev_pos = self.bot_pos

        print self.track_length

    def tick(self):
        time.sleep(self.SLEEP_TIME)

    def loop(self):
        pass

    def action_moving(self):
        pass

    def action_rotating(self):
        pass

    def action_rounding(self):
        pass


class DistBug(BugBase):
    def __init__(self, target_name='target', bot_name='Bot', MT_speed=1.0):
        BugBase.__init__(self, target_name, bot_name, MT_speed)
        self.min_dist_to_target = None
        self.about = "Algorithm Dist-Bug"

        self.print_about_info()

    def loop(self):

        self._init_values()

        while True:

            self.tick()

            self.stop_move()
            self.read_values()

            # self.calc_lenght_of_robot_track()

            self.read_from_sensors()

            self.target_pos.z = self.bot_pos.z = 0.0

            q_rot = Quaternion()
            q_rot.set_from_vector(self.bot_euler_angles.z, Vector3(0.0, 0.0, 1.0))
            self.bot_dir = q_rot.rotate(Vector3(1.0, 0.0, 0.0))

            if self.state == States.MOVING:
                self.action_moving()
            elif self.state == States.ROTATING:
                self.action_rotating()
            elif self.state == States.ROUNDING:
                self.action_rounding()

    def action_moving(self):

        if self.detect[4] < 0.6:

            self.state = States.ROTATING
            tmp = Quaternion()
            tmp.set_from_vector(self.PI / 2.0, Vector3(0.0, 0.0, 1.0))
            self.target_dir = tmp.rotate(self.bot_dir)

            return

        angle = angle_between_vectors(self.bot_dir, self.target_pos.minus(self.bot_pos))

        if math.fabs(angle) > 1.0 * self.PI / 180.0:
            sim.simxSetJointTargetVelocity(self.client_id, self.left_motor_handle,  self.MT_SPEED + angle, sim.simx_opmode_streaming)
            sim.simxSetJointTargetVelocity(self.client_id, self.right_motor_handle, self.MT_SPEED - angle, sim.simx_opmode_streaming)
        else:
            sim.simxSetJointTargetVelocity(self.client_id, self.left_motor_handle,  self.MT_SPEED, sim.simx_opmode_streaming)
            sim.simxSetJointTargetVelocity(self.client_id, self.right_motor_handle, self.MT_SPEED, sim.simx_opmode_streaming)

    def action_rotating(self):

        angle = angle_between_vectors(self.bot_dir, self.target_dir)

        if math.fabs(angle) > 5.0 * self.PI / 180.0:
            sim.simxSetJointTargetVelocity(self.client_id, self.left_motor_handle,   angle, sim.simx_opmode_streaming)
            sim.simxSetJointTargetVelocity(self.client_id, self.right_motor_handle, -angle, sim.simx_opmode_streaming)
        else:
            self.state = States.ROUNDING

    def action_rounding(self):

        tmp_dir = Quaternion()
        tmp_dir.set_from_vector(self.PI / 2.0, Vector3(0.0, 0.0, 1.0))
        perp_bot_dir = tmp_dir.rotate(self.bot_dir)

        angle = angle_between_vectors(perp_bot_dir, self.target_pos.minus(self.bot_pos))

        if self.min_dist_to_target is None or self.min_dist_to_target <= distance_between_points(self.bot_pos, self.target_pos):
            self.min_dist_to_target = distance_between_points(self.bot_pos, self.target_pos)
        elif math.fabs(angle) < 5.0 / 180.0 * self.PI:
            self.state = States.MOVING
            return

        delta = self.detect[7] - self.detect[8]

        if delta < 0.0:
            obstacle_dist = self.detect[7] - self.INDENT_DIST
        else:
            obstacle_dist = self.detect[8] - self.INDENT_DIST

        u_obstacle_dist_stab = self.obstacle_dist_stab_PID.output(obstacle_dist)
        u_obstacle_follower = self.obstacle_follower_PID.output(delta)

        sim.simxSetJointTargetVelocity(self.client_id, self.left_motor_handle,  self.MT_SPEED + u_obstacle_follower + u_obstacle_dist_stab - (1 - self.detect[4]), sim.simx_opmode_streaming)
        sim.simxSetJointTargetVelocity(self.client_id, self.right_motor_handle, self.MT_SPEED - u_obstacle_follower - u_obstacle_dist_stab + (1 - self.detect[4]), sim.simx_opmode_streaming)

class RobotRoundingState:
    START = 1
    PROCESS = 2
    END = 3

class Bug1(BugBase):
    def __init__(self, target_name='target', bot_name='Bot', MT_speed=1.0):
        BugBase.__init__(self, target_name, bot_name, MT_speed)
        self.about = "Algorithm Bug1"
        self.print_about_info()

        self.bot_rounding_state = None
        self.min_dist_to_target = 10000
        self.rounding_start_pos = None

    def loop(self):

        self._init_values()

        while True:

            self.tick()

            self.stop_move()
            self.read_values()

            # self.calc_lenght_of_robot_track()

            self.read_from_sensors()

            self.target_pos.z = self.bot_pos.z = 0.0

            q_rot = Quaternion()
            q_rot.set_from_vector(self.bot_euler_angles.z, Vector3(0.0, 0.0, 1.0))
            self.bot_dir = q_rot.rotate(Vector3(1.0, 0.0, 0.0))

            if self.state == States.MOVING:
                self.action_moving()
            elif self.state == States.ROTATING:
                self.action_rotating()
            elif self.state == States.ROUNDING:
                self.action_rounding()

    def action_moving(self):

        if self.detect[4] < 0.6:

            self.state = States.ROTATING
            tmp = Quaternion()
            tmp.set_from_vector(self.PI / 2.0, Vector3(0.0, 0.0, 1.0))
            self.target_dir = tmp.rotate(self.bot_dir)

            return

        angle = angle_between_vectors(self.bot_dir, self.target_pos.minus(self.bot_pos))

        if math.fabs(angle) > 1.0 * self.PI / 180.0:
            sim.simxSetJointTargetVelocity(self.client_id, self.left_motor_handle,  self.MT_SPEED + angle, sim.simx_opmode_streaming)
            sim.simxSetJointTargetVelocity(self.client_id, self.right_motor_handle, self.MT_SPEED - angle, sim.simx_opmode_streaming)
        else:
            sim.simxSetJointTargetVelocity(self.client_id, self.left_motor_handle,  self.MT_SPEED, sim.simx_opmode_streaming)
            sim.simxSetJointTargetVelocity(self.client_id, self.right_motor_handle, self.MT_SPEED, sim.simx_opmode_streaming)

    def action_rotating(self):

        angle = angle_between_vectors(self.bot_dir, self.target_dir)

        if math.fabs(angle) > 5.0 * self.PI / 180.0:
            sim.simxSetJointTargetVelocity(self.client_id, self.left_motor_handle,   angle, sim.simx_opmode_streaming)
            sim.simxSetJointTargetVelocity(self.client_id, self.right_motor_handle, -angle, sim.simx_opmode_streaming)
        else:
            self.state = States.ROUNDING

            if self.bot_rounding_state is None:
                self.bot_rounding_state = RobotRoundingState.START
                self.rounding_start_pos = self.bot_pos

    def action_rounding(self):

        if self.bot_rounding_state is RobotRoundingState.START:
            if self.is_cur_pos_near_start_rounding_pos(0.5) is False:
                self.bot_rounding_state = RobotRoundingState.PROCESS

        elif self.bot_rounding_state is RobotRoundingState.PROCESS:
            if self.min_dist_to_target > distance_between_points(self.bot_pos, self.target_pos):
                self.min_dist_to_target = distance_between_points(self.bot_pos, self.target_pos)

            if self.is_cur_pos_near_start_rounding_pos(0.2) is True:
                self.bot_rounding_state = RobotRoundingState.END

        elif self.bot_rounding_state is RobotRoundingState.END:

            dist_to_target = distance_between_points(self.bot_pos, self.target_pos)

            if dist_to_target * 0.95 < self.min_dist_to_target < dist_to_target * 1.05:
                self.state = States.MOVING
                self.bot_rounding_state = None
                self.rounding_start_pos = None
                return

        delta = self.detect[7] - self.detect[8]

        if delta < 0.0:
            obstacle_dist = self.detect[7] - self.INDENT_DIST
        else:
            obstacle_dist = self.detect[8] - self.INDENT_DIST

        u_obstacle_dist_stab = self.obstacle_dist_stab_PID.output(obstacle_dist)
        u_obstacle_follower = self.obstacle_follower_PID.output(delta)

        sim.simxSetJointTargetVelocity(self.client_id, self.left_motor_handle,  self.MT_SPEED + u_obstacle_follower + u_obstacle_dist_stab - (1 - self.detect[4]), sim.simx_opmode_streaming)
        sim.simxSetJointTargetVelocity(self.client_id, self.right_motor_handle, self.MT_SPEED - u_obstacle_follower - u_obstacle_dist_stab + (1 - self.detect[4]), sim.simx_opmode_streaming)

    def is_cur_pos_near_start_rounding_pos(self, radius):
        # (x-x0)^2 + (y-y0)^2 <= R^2
        return (self.bot_pos.x - self.rounding_start_pos.x) ** 2 + (self.bot_pos.y - self.rounding_start_pos.y) <= radius ** 2

class Bug2(BugBase):
    def __init__(self, target_name='target', bot_name='Bot', MT_speed=1.0):
        BugBase.__init__(self, target_name, bot_name, MT_speed)
        self.start_target_pos = None
        self.start_bot_pos = None
        self.about = "Algorithm Bug2"

        self.print_about_info()

    def loop(self):

        self._init_values()

        while True:

            self.tick()

            self.stop_move()
            self.read_values()

            # self.calc_lenght_of_robot_track()

            if self.start_bot_pos is None:
                self.start_bot_pos = self.bot_pos
            if self.start_target_pos is None:
                self.start_target_pos = self.target_pos

            self.read_from_sensors()

            self.target_pos.z = self.bot_pos.z = 0.0

            q_rot = Quaternion()
            q_rot.set_from_vector(self.bot_euler_angles.z, Vector3(0.0, 0.0, 1.0))
            self.bot_dir = q_rot.rotate(Vector3(1.0, 0.0, 0.0))

            if self.state == States.MOVING:
                self.action_moving()
            elif self.state == States.ROTATING:
                self.action_rotating()
            elif self.state == States.ROUNDING:
                self.action_rounding()

    def action_moving(self):

        if self.detect[4] < 0.6:

            self.state = States.ROTATING
            tmp = Quaternion()
            tmp.set_from_vector(self.PI / 2.0, Vector3(0.0, 0.0, 1.0))
            self.target_dir = tmp.rotate(self.bot_dir)

            return

        angle = angle_between_vectors(self.bot_dir, self.target_pos.minus(self.bot_pos))

        if math.fabs(angle) > 1.0 * self.PI / 180.0:
            sim.simxSetJointTargetVelocity(self.client_id, self.left_motor_handle,  self.MT_SPEED + angle, sim.simx_opmode_streaming)
            sim.simxSetJointTargetVelocity(self.client_id, self.right_motor_handle, self.MT_SPEED - angle, sim.simx_opmode_streaming)
        else:
            sim.simxSetJointTargetVelocity(self.client_id, self.left_motor_handle,  self.MT_SPEED, sim.simx_opmode_streaming)
            sim.simxSetJointTargetVelocity(self.client_id, self.right_motor_handle, self.MT_SPEED, sim.simx_opmode_streaming)

    def action_rotating(self):

        angle = angle_between_vectors(self.bot_dir, self.target_dir)

        if math.fabs(angle) > 5.0 * self.PI / 180.0:
            sim.simxSetJointTargetVelocity(self.client_id, self.left_motor_handle,   angle, sim.simx_opmode_streaming)
            sim.simxSetJointTargetVelocity(self.client_id, self.right_motor_handle, -angle, sim.simx_opmode_streaming)
        else:
            self.state = States.ROUNDING

    def action_rounding(self):

        if self.is_bot_on_the_constant_direction():
            self.state = States.MOVING
            return

        delta = self.detect[7] - self.detect[8]

        if delta < 0.0:
            obstacle_dist = self.detect[7] - self.INDENT_DIST
        else:
            obstacle_dist = self.detect[8] - self.INDENT_DIST

        u_obstacle_dist_stab = self.obstacle_dist_stab_PID.output(obstacle_dist)
        u_obstacle_follower = self.obstacle_follower_PID.output(delta)

        sim.simxSetJointTargetVelocity(self.client_id, self.left_motor_handle,  self.MT_SPEED + u_obstacle_follower + u_obstacle_dist_stab - (1 - self.detect[4]), sim.simx_opmode_streaming)
        sim.simxSetJointTargetVelocity(self.client_id, self.right_motor_handle, self.MT_SPEED - u_obstacle_follower - u_obstacle_dist_stab + (1 - self.detect[4]), sim.simx_opmode_streaming)

    def is_bot_on_the_constant_direction(self):
        # (x-x1)/(x2-x1) = (y-y1)/(y2-y1).
        diff_x = (self.bot_pos.x - self.start_bot_pos.x) / (self.start_target_pos.x - self.start_bot_pos.x)
        diff_y = (self.bot_pos.y - self.start_bot_pos.y) / (self.start_target_pos.y - self.start_bot_pos.y)
        delta = 0.01
        if diff_x - delta < diff_y < diff_x + delta:
            return True
        return False



















##########################



import argparse
import sys

def create_parser():
    parser = argparse.ArgumentParser(prog="main.py",
                                     description='''Autopilot-drone''',
                                     epilog=''' 2022 ''')

    parser.add_argument('-a', '--algorithm', choices=['BUG1', 'BUG2', 'DISTBUG'], help="Choose bug algorithm.", default='DISTBUG')
    parser.add_argument('-s', '--speed', type=int, help="Speed of roobot MTs", default=1.0)
    parser.add_argument('-t', '--targetname', type=str, help="Name of target on scene", default='target')
    parser.add_argument('-b', '--botname', type=str, help="Name of bot on scene", default='Bot')

    return parser


if __name__ == '__main__':

    parser = create_parser()
    namespace = parser.parse_args(sys.argv[1:])

    bug = None

    if namespace.algorithm == "BUG1":
        bug = Bug1(target_name=namespace.targetname, bot_name=namespace.botname, MT_speed=namespace.speed)
    elif namespace.algorithm == "BUG2":
        bug = Bug2(target_name=namespace.targetname, bot_name=namespace.botname, MT_speed=namespace.speed)
    elif namespace.algorithm == "DISTBUG":
        bug = DistBug(target_name=namespace.targetname, bot_name=namespace.botname, MT_speed=namespace.speed)
    else:

        exit(-2)

    bug.loop()

