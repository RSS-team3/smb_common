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


def filter_real_detections(detected_objects:list, confidence_threshold:float, num_detections_threshold:int, object_radius:float):
    """ Function that filters out real object detections and outputs thea averge location of the detected objec
    
    Args:
        detected_objects (list): List of tuples containing the detected object name, object location (x, y, z), and object confidence score.
        confidence_threshold (float): Threshold for filtering out low confidence detections.
        num_detections_threshold (int): Minimum number of detections to consider an object as real.
        object_radius (float): Radius to consider for merging nearby detections.
    
    Returns:
        list: List of tuples containing the real detected object name and its average location (x, y, z).
    """

    # Filter out low confidence detections
    confident_detected_objects = [obj for obj in detected_objects if obj[2] > confidence_threshold]
    if not confident_detected_objects:
        return []
    
    same_detections = 0
    real_detection_indices = []
    i = 0
    
    # set base object location (first detection)
    base_object_location = confident_detected_objects[i][1]

    # Identify the real detections and add their indices to a list
    # Calculate the sequence length (# of sequential detections of the same object around the same location)
    for j in range(len(confident_detected_objects)):

        # Calculate the distance between the base object (detected first) and the current object
        base_object_distance = np.linalg.norm(np.array(base_object_location) - np.array(confident_detected_objects[j][1]))

        if confident_detected_objects[i][2] == confident_detected_objects[j][2] and base_object_distance < object_radius:
            same_detections += 1

        else:
            if same_detections >= num_detections_threshold:
                real_detection_indices.extend(range(i, j))

            # update variables
            i = j
            same_detections = 0
            
            # update base object location
            base_object_location = confident_detected_objects[i][1]

    return confident_detected_objects[real_detection_indices]

            
    



            