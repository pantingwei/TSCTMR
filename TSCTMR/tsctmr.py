import numpy as np
import math

class TSCTMR:
    def __init__(self):
        """
           cx, cy is the coordinates of image center in pixel coordinate system.
           f is the focal length of the camera, measured in pixels.
        """
        self.cx = 965
        self.cy = 545
        self.f = 965

    def associate(self, uav1_pose, uav2_pose, uav1_objects, uav2_objects):
        """
        Input: uav1_pose, uav2_pose, uav1_objects, uav2_objects
        Output: association result of TSC
        """
        self.uav1_pose, self.uav2_pose = uav1_pose, uav2_pose

        distance_cost = np.zeros((len(uav1_objects), len(uav2_objects)))
        altitude = np.zeros((len(uav1_objects), len(uav2_objects)))

        for i in range(len(uav1_objects)):
            for j in range(len(uav2_objects)):
                distance_cost[i, j], altitude[i, j] = self.get_cost(uav1_objects[i], uav2_objects[j])
        h_hat = self.get_meanH(distance_cost, altitude)

        dis_h_cost = np.exp(distance_cost) * np.sqrt(np.square(altitude - h_hat))
        row, col = self.assignment(dis_h_cost)

        return row, col



    def get_cost(self, uav1_obj, uav2_obj):

        """
         This function is used to obtain the distance cost and intersection altitude determined by two targets detected
         by different drones.
        """

        R_mw = self.get_R_matrix(self.uav1_pose[3:6])
        R_nw = self.get_R_matrix(self.uav2_pose[3:6])
        T_nm = np.array([[self.uav2_pose[0]], [self.uav2_pose[1]], [self.uav2_pose[2]]]) - np.array(
            [[self.uav1_pose[0]], [self.uav1_pose[1]], [self.uav1_pose[2]]])

        obj1 = [np.array([[self.f], [uav1_obj[1] - self.cx], [uav1_obj[2] - self.cy]]), self.uav1_pose[0:3], self.uav1_pose[3:7]]
        obj2 = [np.array([[self.f], [uav2_obj[1] - self.cx], [uav2_obj[2] - self.cy]]), self.uav2_pose[0:3], self.uav2_pose[3:7]]

        distance, altitude = self.cal_cost(obj1, obj2, R_mw, R_nw,T_nm)
        return distance, altitude

    def cal_cost(self, obj1, obj2, R_mw, R_nw,T_nm):

        a = np.dot(R_mw, obj1[0])  # 左视图中的像点转换到世界坐标系
        b = np.dot(R_nw, obj2[0])  # 右视图中的像点转换到世界坐标系
        c = np.cross(a.T, b.T).T

        B = np.append(np.append(a, -b, axis=1), c, axis=1)
        result = np.dot(np.linalg.inv(B), T_nm)

        vector_L = ((result[0, 0] * a).T)[0]
        vector_R = ((result[1, 0] * b).T)[0]

        loc_11 = [obj1[1][0] + vector_L[0], obj1[1][1] + vector_L[1], obj1[1][2] + vector_L[2]]
        loc_22 = [obj2[1][0] + vector_R[0], obj2[1][1] + vector_R[1], obj2[1][2] + vector_R[2]]

        distance = self.get_dis(loc_11[0], loc_22[0], loc_11[1], loc_22[1], loc_11[2], loc_22[2])
        altitude = (loc_11[2] + loc_22[2]) / 2

        return distance, altitude

    def get_R_matrix(self, attitude):
        """
         This function calculates the camera's rotation matrix relative to the world coordinate system using the input
         attitude angles.
        """

        yaw = attitude[0]
        pitch = attitude[1]
        roll = attitude[2]

        R_x = np.array([[1, 0, 0],
                        [0, math.cos(roll), -math.sin(roll)],
                        [0, math.sin(roll), math.cos(roll)]])

        R_y = np.array([[math.cos(pitch), 0, math.sin(pitch)],
                        [0, 1, 0],
                        [-math.sin(pitch), 0, math.cos(pitch)]])

        R_z = np.array([[math.cos(yaw), -math.sin(yaw), 0],
                        [math.sin(yaw), math.cos(yaw), 0],
                        [0, 0, 1]])

        R = np.dot(R_z, np.dot(R_y, R_x))

        return R

    def get_dis(self, x1, x2, y1, y2, z1, z2):
        return math.sqrt(math.pow(x1 - x2, 2) + math.pow(y1 - y2, 2) + math.pow(z1 - z2, 2))

    def get_meanH(self,distance_cost, altitude_cost):
        """
         This function is used to obtain an estimate of the altitude of ground targets.
        """

        row, col = distance_cost.shape

        new_col = min([5, col])
        dis_cost = np.zeros((row, new_col))
        h_cost = np.zeros((row, new_col))

        for i in range(row):
            row_data = list(distance_cost[i])
            row_data.sort()
            inx = distance_cost[i] <= row_data[new_col - 1]
            dis_cost[i] = distance_cost[i][inx][:new_col]
            h_cost[i] = altitude_cost[i][inx][:new_col]

        h_list = []
        num_list = []
        for i in range(row):
            for j in range(new_col):
                h = h_cost[i, j]
                dis_h_cost = np.exp(distance_cost) * np.sqrt(np.square(altitude_cost - h))
                r, c = self.assignment(dis_h_cost)
                number = len(r)
                sum_cost = 0
                for inx_r in r:
                    for inx_c in c:
                        sum_cost += dis_h_cost[inx_r, inx_c]
                mean_cost = sum_cost / number
                score_h = ((2 * np.exp(-3 * mean_cost)) / (1 + np.exp(-3 * mean_cost))) * ((1 - np.exp(-3 * number)) /
                                                                                           (1 + np.exp(-3 * number)))
                h_list.append(h)
                num_list.append(score_h)

        return h_list[num_list.index(max(num_list))]


    def assignment(self, cost_mat):

        """
           Association strategy
        """

        row, col = cost_mat.shape

        r = []
        c = []
        for i in range(row):
            min_col = cost_mat[i,:].argmin()
            min_row = cost_mat[:,min_col].argmin()
            if min_row == i:
                r.append(i)
                c.append(min_col)

        return r, c
