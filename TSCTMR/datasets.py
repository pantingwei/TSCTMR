def load_data():
    """
        Load UAV pose and object data from text files.
    """
    uav1_pose = [] # [latitude, longitude, altitude, yaw, pitch, roll]
    uav2_pose = []
    uav1_objects = [] #[[ID, center_x, center_y]....[]]
    uav2_objects = []

    file_path = ['uav1.txt', 'uav2.txt'] # Paths to the UAV data files
    uav_pose = [uav1_pose, uav2_pose] # Lists to store UAV poses
    uav_objects = [uav1_objects, uav2_objects] # Lists to store the detected objects

    for i, txt in enumerate(file_path):
        f = open(txt)
        for line_number, line in enumerate(f.readlines()):
            line = line.strip('\n').split(' ')
            if line_number == 0:
                for data in line:
                    uav_pose[i].append(float(data))
            else:
                objects = []
                for k, data in enumerate(line):
                    if k == 0:
                        objects.append(data)
                    else:
                        objects.append(float(data))
                uav_objects[i].append(objects)
    return uav1_pose, uav2_pose, uav1_objects, uav2_objects