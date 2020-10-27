#!/usr/bin/env python
"""!
This module contains the main code needed to collect synthetic data. It takes a user's provided bag
file and uses it as a script to manipulate the target robots. These robots are moved into position
at the right points and the data is collected. There are several formats that are created. They are:

    - YOLO
    - Something else
"""
from gazebo_msgs.srv import SetModelState
from Robot import Robot
import rosbag
import rospy
import tf2_ros


def initializeBagFile():
    """!
    This function looks up the name of the bag file from the parameter server and loads that bag file. It also determines
    the rate at which the code should step through the bag file.
    @return A tuple with - (An object containing the loaded bagfile, the rate at which it should be stepped through)
    @throw rospy.ROSInitException Thrown if there is no bag file parameter on the server.
    @throw rosbag.ROSBagException Thrown if the bag file can't be loaded for some reason.
    """
    # Assume the parameter specifying the bag file name is a private parameter.
    parameter_name = '~bag_file'
    # Make sure it exists before trying to read it.
    if not rospy.has_param(parameter_name):
        # Throw an error if it doesn't.
        error_string = 'Please specify the bag file to replay on the parameter "bag_file"'
        rospy.logfatal(error_string)
        raise rospy.ROSInitException(error_string)
    # If this point is reached, then the parameter at least exists.
    bag_file = rospy.get_param(parameter_name)
    # Open the bag file. This might throw an error, so catch it and send a warning
    # to the user.
    try:
        bag = rosbag.Bag(f=bag_file)
    except Exception as ex:
        error_string = 'Unable to open {bag}: {error}'.format(
            bag=bag_file, error=ex)
        rospy.logfatal(error_string)
        raise rospy.ROSInitException(error_string)
    # If this point is reached, the bag file is loaded and ready.
    # Now look up the simulated rate with the same method as the bag parameter.
    parameter_name = '~simulated_rate'
    default_rate = 30.0
    if not rospy.has_param(parameter_name):
        rospy.logwarn(
            'No simulated rate specificed on the parameter "simulated_rate" Using a default of %f instead.', default_rate)
    simulated_rate = rospy.get_param(parameter_name, default_rate)
    return (bag, simulated_rate)


def lookupCameraFrame():
    """!
    This function looks up the name of the frame_id used for the camera in TF for use in determining distances
    and projections. It uses a default if not found.
    @return The string representing the camera's frame_id in the TF tree.
    """
    parameter_name = '~camera_frame_id'
    default_camera_frame = 'camera/base_link'
    if not rospy.has_param(parameter_name):
        rospy.logwarn('Frame parameter not found on {param}, using {default} instead'.format(
            param=parameter_name, default=default_camera_frame))
    camera_frame = rospy.get_param(parameter_name, default_camera_frame)
    return camera_frame


def initializeRobots():
    """!
    This function determines the names of each robot that should be controlled. It then creates
    a @ref Robot object for each robot. Those objects perform the correct initialization of needed
    parameters.
    @return A list of all created Robot objects
    @throw rospy.ROSInitException Thrown if no robot_list parameter is found or if no robots are
    specified.
    """
    # Determine the list of robots and error if not found.
    parameter_name = '/robot_list'
    if not rospy.has_param(parameter_name):
        error_string = 'No robot names specified on {param}'.format(
            param=parameter_name)
        rospy.logfatal(error_string)
        raise rospy.ROSInitException(error_string)
    robot_names = rospy.get_param(parameter_name)
    # Throw an error if there are no robots specified.
    if len(robot_names) <= 1:
        error_string = 'Must specify at least one robot in {param}'.format(
            param=parameter_name)
        rospy.logfatal(error_string)
        raise rospy.ROSInitException(error_string)
    # There is a chance the user specifies only a single robot without brackets, which would make
    # this parameter as a string. Check for that and convert it to a list for use later.
    if type(robot_names) is not list:
        single_robot_name = robot_names
        robot_names = [single_robot_name]
    # Create each robot object and add it to a list.
    robot_list = []
    for name in robot_names:
        robot_list.append(Robot(name))
    return robot_list

    # def initalizeBackgroundSubtractor(robot_list, camera_image_topic, gazebo_pub):
    #     """
    #     Create the subtractor, then move every robot into the air, capture an image, and do some sort of
    #     initialization depending on the subtractor
    #     @return the subtractor
    #     """


if __name__ == "__main__":
    rospy.init_node(name='collect_data')
    # Load bag file
    # (bag, simulated_rate) = initializeBagFile()
    # Setup camera
    camera_frame_id = lookupCameraFrame()
    # Load each robot
    robot_list = initializeRobots()
    # Create the client to tell each robot to move in Gazebo.
    gazebo_client_name = '/gazebo/set_model_state'
    rospy.loginfo('Waiting for Gazebo to start...')
    rospy.wait_for_service(gazebo_client_name)
    gazebo_client = rospy.ServiceProxy(
        name=gazebo_client_name, service_class=SetModelState)
    # Create the TF lookup
    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)
    # initalizeBackgroundSubtractor()

    # # For each view: for topic, msg, t in bag.read_messages(topics=['chatter', 'numbers']):
    # 	# For each robot:
    # 		# Determine its last pose
    # 		pose = extractPose()
    # 		robot.storePose(pose)
    # 		# Put it there
    # 		msg = robot.createRobotMessage()
    # 		gazebo_pub.pub(msg)
    # 		# Capture new image
    # 		image = wait_for_message()
    # 		# Background subtract
    # 		subtracted_image = backgroundsubtract.subtract()
    # 		# Determine distance from camera
    # 		# Move out of way
    # 		msg = robot.createRobotMessage()
    # 		gazebo_pub.pub(msg)
    # 	# Based on distances, layer robots appropriately
    # 	# Write outputs to file
    # # Shut down and close bag file
