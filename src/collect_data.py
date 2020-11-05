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
from geometry_msgs.msg import Transform
import ParameterLookup
from Robot import Robot
import rosbag
import rospy
from sensor_msgs.msg import Image
import tf2_py
import tf2_ros


def captureImage():
    """!
    @brief Captures the latest image from Gazebo.

    it gets updated in all sensors and visuals. This is related
    to the processing speed and (to the best of my knowledge),
    can't be detected programatticaly. The best bet is to just
    wait a little bit before capturing any images to allow the
    simulations to update.
    @return A cv2::Mat of the latest scene in Gazebo.
    """
    GAZEBO_IMAGE_WAIT = 1.0
    bridge = CvBridge()
    rospy.sleep(duration=GAZEBO_IMAGE_WAIT)
    image = rospy.wait_for_message(
        topic='camera/image_raw', topic_type=Image)
    image_mat = bridge.imgmsg_to_cv2(img_msg=image)
    return image_mat


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
        history=background_history, varThreshold=500, detectShadows=False)
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
        robot_transform_msg = Transform()
        robot_transform_msg.translation.x = robot_current_state.pose.position.x
        robot_transform_msg.translation.y = robot_current_state.pose.position.y
        robot_transform_msg.translation.z = robot_current_state.pose.position.z
        robot_transform_msg.rotation = robot_current_state.pose.orientation
        robot.recordTransform(robot_transform_msg)
        robot_new_pose_request = robot.createSetModelStateRequest()
        robot_new_pose_request.pose.position.z = 1e6
        moveRobot(gazebo_set_pose_client, robot_new_pose_request)
    # Capture N images and apply to subtractor
    for _ in range(background_history):
        image = captureImage()
        background_subtractor.apply(image)
    return background_subtractor


def initializeReplay():
    """!
    @brief Load the TF tree from the provided bag file.

    This initialization takes a bag file, as provided by the user on the parameter
    server and reads the entire TF tree into a buffer for use in replaying.
    @return A tuple containing:
        1. The tf buffer
        2. A rospy.Time of the start time of the bag file
        3. A rospy.Time of the end time of the bag file
        4. A rospy.Duration of the period used during processing
    @throw rospy.ROSInitException Thrown if the bag file can't be opened or found.
    """
    rospy.loginfo('Loading bag file...')
    # Determine where the bag file is located via parameter server.
    bag_file = ParameterLookup.lookup(parameter='~bag_file')
    # Open the bag file. This might throw an error, so catch it and send a warning
    # to the user.
    try:
        bag = rosbag.Bag(f=bag_file)
    except Exception as ex:
        error_string = 'Unable to open {bag}: {error}'.format(
            bag=bag_file, error=ex)
        rospy.logfatal(msg=error_string)
        raise rospy.ROSInitException(error_string)
    # If this point is reached, the bag file is loaded and ready.
    # Now look up the simulated rate with the same method as the bag parameter.
    rate = ParameterLookup.lookupWithDefault(
        parameter='~simulated_rate', default=30.0)
    period = rospy.Duration(secs=(1.0 / rate))
    # Now, parse the bag file into a tf buffer
    start_time = rospy.Time(bag.get_start_time())
    end_time = rospy.Time(bag.get_end_time())
    tf_buffer = tf2_py.BufferCore((end_time - start_time))
    for topic, msg, _ in bag.read_messages(topics=['/tf', '/tf_static']):
        for msg_tf in msg.transforms:
            if topic == '/tf_static':
                tf_buffer.set_transform_static(msg_tf, 'default_authority')
            else:
                tf_buffer.set_transform(msg_tf, 'default_authority')
    # Now that everything is loaded, we don't need the bag file.
    bag.close()
    return (tf_buffer, start_time, end_time, period)


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
    robot_names = ParameterLookup.lookup('/robot_list')
    # Throw an error if there are no robots specified.
    if len(robot_names) < 1:
        error_string = 'Must specify at least one robot in /robot_list'
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
    # Load the buffer containing the motion record from the bag file.
    (tf_buffer, start_time, end_time, period) = initializeReplay()
    current_time = start_time
    # Setup camera
    camera_frame_id = ParameterLookup.lookupWithDefault(
        parameter='~camera_frame_id', default='camera/base_link')
    # Look up what frame to use as the reference point for all placement.
    global_frame_id = ParameterLookup.lookupWithDefault(
        parameter='~global_frame', default='map')
    # Load each robot
    robot_list = initializeRobots()
    # Create the client to tell each robot to move in Gazebo.
    gazebo_client_name = '/gazebo/set_model_state'
    rospy.loginfo('Waiting for Gazebo to start...')
    rospy.wait_for_service(gazebo_client_name)
    gazebo_client = rospy.ServiceProxy(
        name=gazebo_client_name, service_class=SetModelState)
    # Create the TF lookup
    gazebo_tf_buffer = tf2_ros.Buffer()
    gazebo_tf_listener = tf2_ros.TransformListener(gazebo_tf_buffer)
    # Set up the subtractor and initialize
    background_subtractor = initializeBackgroundSubtractor(
        robot_list, gazebo_client)
    # Create a subscriber to the camera. However, this isn't actually used. The system
    # uses wait_for_message to get the latest image. However, Gazebo won't update sensor
    # simulations unless there is a subscriber. So this tricks Gazebo into doing so.
    name = rospy.Subscriber(name='camera/image_raw',
                            data_class=Image, callback=None, queue_size=1)
    # Loop through the whole bag file until the end is reached. The two times above
    # provide a moving window. The last iteration we want is when the window crosses
    # the bag end time. The next iteration would have both start and end of the
    # window past the bag file then. So use that as the break.
    rospy.loginfo('Beginning data collection...')
    while current_time < end_time:
        # Start by moving every robot out of the way. They don't need to be in new
        # spots. They will also be collision free, since they either have a pose from
        # the bag file or the initial poses specified at launch.
        for robot in robot_list:
            robot_new_pose_request = robot.createSetModelStateRequest()
            robot_new_pose_request.pose.position.z = 1e6
            result = moveRobot(gazebo_client, robot_new_pose_request)
            if not result:
                rospy.logfatal('Unable to move robot, killing processing.')
                exit()
        # Now, go through each robot and move it into the scene.
        for robot in robot_list:
            # Look up each robot's pose from the bag file TF tree.
            try:
                robot_transform = tf_buffer.lookup_transform_core(
                    global_frame_id, robot.getFullFrame(), current_time)
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException), argument:
                rospy.logwarn('%s' % argument)
                # @TODO Figure out what to do when a single robot can't be found on the TF tree yet.
            robot.recordTransform(robot_transform.transform)
            # Move the robot and allow time for Gazebo to update
            moveRobot(gazebo_client, robot.createSetModelStateRequest())
            image = captureImage()
            # Use the background subtractor to find the robot. Be sure to set the
            # learning rate to 0 to prevent updating the background image.
            foreground_mask = background_subtractor.apply(
                image, learningRate=0.0)
            robot.recordPixelMask(foreground_mask)
            robot_new_pose_request = robot.createSetModelStateRequest()
            robot_new_pose_request.pose.position.z = 1e6
            result = moveRobot(gazebo_client, robot_new_pose_request)
            if not result:
                rospy.logfatal('Unable to move robot, killing processing.')
                exit()
        # Move every robot back to the ground
        for robot in robot_list:
            robot_new_pose_request = robot.createSetModelStateRequest()
            result = moveRobot(gazebo_client, robot_new_pose_request)
            if not result:
                rospy.logfatal('Unable to move robot, killing processing.')
                exit()
        # Capture an image
        image = captureImage()
        # Have each output format write their information.
        for writer in data_writers:
            writer.outputScene(robot_list, image, None)
        # Look up the robot's 3D pose
        # Determine the robot's distance from the camera
        # Project the key points into the camera
        # Write the background image
        # Layer the masks according to the distances
        # For the new format, split out the pixels by robot again, since this
        # now accounts for occlusions
        # Create bounding boxes for each robot for YOLO
        # Write everything to file
        # Update the user on progress.
        percent_complete = (current_time - start_time).to_sec() / \
            (end_time - start_time).to_sec() * 100.0
        rospy.loginfo_throttle(
            period=5.0, msg='%.1f%% completed' % percent_complete)
        # Update the times
        current_time += period
    # When this is finally done, record any meta information needed by networks.
    rospy.loginfo('100%% completed')
