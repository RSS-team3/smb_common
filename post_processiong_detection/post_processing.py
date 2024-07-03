import rospy
import rospkg
from tf.transformations import quaternion_matrix

import numpy as np

# messages we have: 
# - x, y and z location of the object
# - name of the object detected

# messages we get: locations with respect to the camera frame at that time
# -> get world location of the robot from the SLAM
# -> do a coordinate transformation to get the locations of

def object_detections_to_world_frame():

    pass



def filter_real_detections(detected_objects, confidence_scores, confidence_threshold, num_detections_threshold):
    same_detections = 0
    real_detection_indexes = []
    i = 0
    
    # filter out low confidence detections
    np.where(confidence_scores < confidence_threshold)

    for j in range(len(detected_objects)):
        if detected_objects[i] == detected_objects[j]:
            same_detections += 1

        elif detected_objects[i] != detected_objects[j]:
            if same_detections >= num_detections_threshold:
                real_detection_indexes.extend(range(i, j))
            i = j
            same_detections = 0

    return real_detection_indexes
    

def 



            