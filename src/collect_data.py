#!/usr/bin/env python
"""!
This module contains the main code needed to collect synthetic data. It takes a user's provided bag
file and uses it as a script to manipulate the target robots. These robots are moved into position
at the right points and the data is collected. There are several formats that are created. They are:

    - YOLO
    - Something else
"""
import cv2
from cv_bridge import CvBridge
from gazebo_msgs.srv import GetModelState
from gazebo_msgs.srv import SetModelState
from Robot import Robot
import rosbag
import rospy
from sensor_msgs.msg import Image
import tf2_ros


def initializeBackgroundSubtractor(robot_list, gazebo_set_pose_client):
    """!
    Create the background subtractor. Then, manipulate the Gazebo environment to allow
    the capture of a background image. This uses the MOG2 algorithm.
    @param robot_list A list of the robots that shouldn't be part of the background.
    @param gazebo_set_pose_client A server client used to move every robot within Gazebo.
    @return The background subtractor used in the script.
    """
    # Set a large history to maximize the detection of differences from background.
    background_history = 100
    # ! @todo Explore how shadow detection impacts the results.
    background_subtractor = cv2.createBackgroundSubtractorMOG2(
        history=background_history, detectShadows=False)
    # Create a client to determine where each robot is located within the Gazebo environment
    gazebo_get_pose_name = '/gazebo/get_model_state'
    rospy.loginfo('Waiting to find robot poses...')
    rospy.wait_for_service(gazebo_get_pose_name)
    gazebo_get_pose_client = rospy.ServiceProxy(
        name=gazebo_get_pose_name, service_class=GetModelState)
    # Move all the robots way up in the sky, presumably outside of the view of the camera.
    rospy.loginfo('Moving robots and capturing background...')
    for robot in robot_list:
        # Create each message via the robot object.
        robot_current_state = gazebo_get_pose_client(robot.getName(), '')
        robot.recordPose(robot_current_state.pose)
        robot_new_pose_request = robot.createSetModelStateRequest()
        robot_new_pose_request.pose.position.z = 1e6
        moveRobot(gazebo_set_pose_client, robot_new_pose_request)
    # Sleep briefly to allow the robots to finish moving.
    rospy.sleep(duration=1.0)
    # Capture N images and apply to subtractor
    # We need to convert sensor_msgs/Image to cv2::Mat
    bridge = CvBridge()
    # cv_image = bridge.imgmsg_to_cv2(image_message, desired_encoding='passthrough')
    for _ in range(background_history):
        image = rospy.wait_for_message(
            topic='camera/image_raw', topic_type=Image)
        image_mat = bridge.imgmsg_to_cv2(img_msg=image)
        background_subtractor.apply(image_mat)
    return background_subtractor


def initializeBagFile():
    """!
    This function looks up the name of the bag file from the parameter server and loads that bag file. It also determines
    the rate at which the code should step through the bag file.
    @return A tuple with - (An object containing the loaded bagfile, a float representing the period at which
    it should be stepped through)
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
            'No simulated rate specificed on the parameter "simulated_rate" Using a default of %f hz instead.', default_rate)
    simulated_rate = rospy.get_param(parameter_name, default_rate)
    simulated_period = 1.0 / simulated_rate
    return (bag, simulated_period)


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
    if len(robot_names) < 1:
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


def moveRobot(client, pose_request):
    """!
    @brief Move a robot within Gazebo using the provided server client and request.
    @param client The ROS server client used to tell Gazebo to move a model
    @param pose_request The request for which model to move and where to move it.
    @return True if the move was successful, false otherwise.
    """
    # Move the robot to the new point in the sky
    result = client(pose_request)
    # Do some error checking
    if not result.success:
        rospy.logerr('Error moving robot: {message}'.format(
            result.status_message))
    return result.success


if __name__ == "__main__":
    rospy.init_node(name='collect_data')
    # Load bag file
    (bag, simulated_period) = initializeBagFile()
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
    # Set up the subtractor and initialize
    background_subtractor = initializeBackgroundSubtractor(
        robot_list, gazebo_client)
    # Start the time at the begining and iterate in chunks of the period provided.
    time_window_start = bag.get_start_time()
    time_window_end = time_window_start + simulated_period
    # We will need a bridge to convert sensor_msgs/Image and cv::Mat
    bridge = CvBridge()
    # Loop through the whole bag file until the end is reached. The two times above
    # provide a moving window. The last iteration we want is when the window crosses
    # the bag end time. The next iteration would have both start and end of the
    # window past the bag file then. So use that as the break.
    while time_window_start < bag.get_end_time():
        # Start by moving every robot out of the way. They don't need to be in new
        # spots. They will also be collision free, since they either have a pose from
        # the bag file or the initial poses specified at launch.
        for robot in robot_list:
            robot_new_pose_request = robot.createSetModelStateRequest()
            robot_new_pose_request.pose.position.z = 0
            result = moveRobot(gazebo_client, robot_new_pose_request)
            if not result:
                rospy.logfatal('Unable to move robot, killing processing.')
                exit()
        # Now, go through each robot and move it into the scene.
        for robot in robot_list:
            (topics, msgs, times) = bag.read_messages(topics=robot.pose_topic,
                                                      start_time=time_window_start, end_time=time_window_end)
            # We only care about the first message, since there is one pose per time chunk.
            pose = msgs.message
            # Move the robot to that pose
            # Sleep briefly to allow the image to update.
            rospy.sleep(0.25)
            # Capture an image
            image = rospy.wait_for_message(
                topic='camera/image_raw', topic_type=Image)
            # Convert to cv::Mat
            image_mat = bridge.imgmsg_to_cv2(img_msg=image)
            # Use the background subtractor to find the robot. Be sure to set the
            # learning rate to 0 to prevent updating the background image.
            # Convert the mask into the robot's ID.
            # Store this image mask
            # Look up the robot's 3D pose
            # Determine the robot's distance from the camera
            # Project the key points into the camera
        # Move every robot back to the ground
        for robot in robot_list:
            pass
        # Wait a bit for the image to update.
        rospy.sleep(0.25)
        # Capture an image
        # @todo Make this its own function
        image = rospy.wait_for_message(
            topic='camera/image_raw', topic_type=Image)
        # Convert to cv::Mat
        image_mat = bridge.imgmsg_to_cv2(img_msg=image)
        # Write the background image
        # Layer the masks according to the distances
        # For the new format, split out the pixels by robot again, since this
        # now accounts for occlusions
        # Create bounding boxes for each robot for YOLO
        # Write everything to file
        # Update the times
        time_window_start = time_window_end
        time_window_end += simulated_period
    # When this is finally done, record any meta information needed by networks.
    # Close the bag file
    bag.close()
