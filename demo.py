from TSCTMR.tsctmr import *
from TSCTMR.datasets import *

if __name__ == '__main__':
    uav1_pose, uav2_pose, uav1_objects, uav2_objects = load_data()

    associater = TSCTMR()
    row, col = associater.associate(uav1_pose, uav2_pose, uav1_objects, uav2_objects)

    print("Association Results:")
    for i in range(len(row)):
        print(uav1_objects[row[i]][0], uav2_objects[col[i]][0])