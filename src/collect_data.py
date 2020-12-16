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
import label_writers
import numpy
import ParameterLookup
from Robot import Robot
import rosbag
import rospy
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Image
from tf2_geometry_msgs import PointStamped
import tf2_py
import tf2_ros


def captureImage(sleep_duration=2.0):
    """!
    @brief Captures the latest image from Gazebo.

    @note There is a lag between when Gazebo moves a model and
    when it gets updated in all sensors and visuals. This is related
    to the processing speed and (to the best of my knowledge),
    can't be detected programatticaly. The best bet is to just
    wait a little bit before capturing any images to allow the
    simulations to update.
    @param sleep_duration The amount to sleep before capturing the image
    in order to let Gazebo finish updates.
    @return A cv2::Mat of the latest scene in Gazebo and a sensor_msgs/CameraInfo message.
    """
    bridge = CvBridge()
    rospy.sleep(duration=sleep_duration)
    image = rospy.wait_for_message(
        topic='camera/image_raw', topic_type=Image)
    image_mat = bridge.imgmsg_to_cv2(img_msg=image)
    camera_info = rospy.wait_for_message(
        topic='camera/camera_info', topic_type=CameraInfo)
    return (image_mat, camera_info)


def initializeBackgroundSubtractor(robot_list, gazebo_set_pose_client):
    """!
    Create the background subtractor. Then, manipulate the Gazebo environment to allow
    the capture of a background image. This uses the MOG2 algorithm.
    @param robot_list A list of the robots that shouldn't be part of the background.
    @param gazebo_set_pose_client A server client used to move every robot within Gazebo.
    @return The background subtractor used in the script.
    """
    # Set a large history to maximize the detection of differences from background.
    background_history = ParameterLookup.lookupWithDefault(
        parameter='~background_subtractor/history_length', default=100)
    # Explore how shadow detection impacts the results.
    var_threshold = ParameterLookup.lookupWithDefault(
        parameter='~background_subtractor/var_threshold', default=500)
    detect_shadows = ParameterLookup.lookupWithDefault(
        parameter='~background_subtractor/detect_shadows', default=False)
    background_subtractor = cv2.createBackgroundSubtractorMOG2(
        history=background_history, varThreshold=var_threshold, detectShadows=detect_shadows)
    # Create a client to determine where each robot is located within the Gazebo environment.
    # This is only done at initialization to move every robot out of the way. It is assumed the
    # robots start in a stable location. So the system is safe to just move them up in the air,
    # but leave their X/Y the same. Given Gazebo's propensity to send colliding objects flying,
    # this seems like the best course of action. The user is unlikely to start the node if
    # the simulation freaks out right away. Using the bag file first location is possible, but
    # would require passing the tf_buffer into this function. After initialization, the bag file
    # positions are used for the rest of the node.
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
    rospy.sleep(2.0)
    for _ in range(background_history):
        # Sleep long enough for any noise to update
        (image, _) = captureImage(sleep_duration=0.25)
        background_subtractor.apply(image)
    return background_subtractor


def initializeReplay(global_frame_id, robot_list):
    """!
    @brief Load the TF tree from the provided bag file.

    This initialization takes a bag file, as provided by the user on the parameter
    server and reads the entire TF tree into a buffer for use in replaying.
    @param global_frame_id The global refernce frame used by Gazebo, often 'map'
    @param robot_list The list of robots in the simulation. Used to determine the start
    and end times that have valid TF transforms for all robots.
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
    if 'example_replay.bag' in bag_file:
        rospy.logwarn(
            msg='You might be using the example bag file. Did you mean to provide your own using the bag_file parameter?')
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
    # Find the first start_time that has a valid transform for every single robot.
    good_start_time = False
    # This only finishes if every robot has a viable transform at this time step
    while not good_start_time and start_time < end_time:
        good_start_time = True
        for robot in robot_list:
            (is_good, _) = tf_buffer.can_transform_core(
                global_frame_id, robot.getFullFrame(), start_time)
            good_start_time = good_start_time and is_good
        if not good_start_time:
            start_time += period
    # If the start time is now greater than the end time, then there were no
    # time frames that had valid transforms for every robot. This is a problem.
    if not start_time < end_time:
        error_string = 'Unable to find valid transforms for all robots in bag file.'
        rospy.logfatal(error_string)
        raise rospy.exceptions.ROSInitException(error_string)
    # Now do the same thing for the end time, but decrement instead. The end time
    # is only a gate on when the whole simulation should end, so it does not have to
    # be a whole number of increments from start time. Therefore, just start at the
    # end of the bag file.
    good_end_time = False
    # You technically need to watch for the end > start since there may be a specific
    # setup that only has frame interpolation in a very narrow time range and if period
    # is larger than that range, end_time is liable to never exist within that range.
    while not good_end_time and end_time > start_time:
        good_end_time = True
        for robot in robot_list:
            (is_good, _) = tf_buffer.can_transform_core(
                global_frame_id, robot.getFullFrame(), end_time)
            good_end_time = good_end_time and is_good
        if not good_end_time:
            end_time -= period
    if not end_time > start_time:
        error_string = 'Unable to find valid transforms for all robots in bag file.'
        rospy.logfatal(error_string)
        raise rospy.exceptions.ROSInitException(error_string)
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


def removeOutsideRobot(mask, robot, tf_buffer, camera_frame_id, camera_info):
    """!
    @brief Removes everything in the provided mask that is not within the bounds
    of the robot.

    Each mask is only on a single robot. If you project the vertices of the bounding
    shape onto the image, there cannot possibly be any part of the mask outside of the
    containing rectangle on the image. This provides a simple, but powerful filter for
    the image.
    @param mask The mask to filter.
    @param robot The robot currently being segmented in the image.
    @param tf_buffer A TF buffer to use to lookup point transforms. This is required
    to project the robot's bounding shape points into the camera.
    @param camera_frame_id The name of the frame in TF used by the camera.
    @param camera_info The camera_info ROS message with camera calibration values.
    @return An image with everything outside of the robot's area removed.
    """
    # First, get the bounding shape of the robot, which is a list of point mesages.
    bounding_shape = robot.getBoundingShape()
    # Project each point onto the image.
    points_image = numpy.zeros(shape=(len(bounding_shape), 2))
    count = 0
    for point in bounding_shape:
        # TF needs a stamp to transform appropriately.
        point_source_stamped = PointStamped()
        point_source_stamped.header.stamp = rospy.Time.now()
        point_source_stamped.header.frame_id = robot.getFullFrame()
        point_source_stamped.point = point
        # Transform into the camera
        try:
            point_target_stamped = tf_buffer.transform(
                object_stamped=point_source_stamped, target_frame=camera_frame_id, timeout=rospy.Duration(2.0))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as error:
            rospy.logerr(msg='TF transform error: {error}'.format(error=error))
            raise
        # Project onto the image.
        point_target_numpy = numpy.zeros(shape=(1, 3))
        point_target_numpy[0][0] = point_target_stamped.point.x
        point_target_numpy[0][1] = point_target_stamped.point.y
        point_target_numpy[0][2] = point_target_stamped.point.z
        intrinsic = numpy.array(object=camera_info.K).reshape(3, 3)
        zeros = numpy.zeros(shape=(3, 1))
        (point_image, _) = cv2.projectPoints(objectPoints=point_target_numpy,
                                             rvec=zeros, tvec=zeros, cameraMatrix=intrinsic, distCoeffs=None)
        point_image = numpy.squeeze(point_image)
        # Extract the point and save for comparison with the rest.
        points_image[count, :] = point_image
        count += 1
    # Determine the outer bounds on the images to get the bounding rectangle.
    # These are used as pixel indices, so need to be ints.
    mins = points_image.min(axis=0)
    mins = numpy.floor(mins)
    mins = mins.astype(dtype=numpy.int16)
    maxs = points_image.max(axis=0)
    maxs = numpy.ceil(maxs)
    maxs = maxs.astype(dtype=numpy.int16)
    # Use that rectangle to create an image mask
    new_image = numpy.zeros_like(mask)
    new_image[mins[1]:maxs[1], mins[0]:maxs[0]
              ] = mask[mins[1]:maxs[1], mins[0]:maxs[0]]
    return new_image


if __name__ == "__main__":
    rospy.init_node(name='collect_data')
    # Set up the different data writers.
    data_writers = [
        label_writers.BoundingShape('bounding_shape'),
        label_writers.YOLO('yolo'),
        label_writers.STVNet('stvnet')
    ]
    # Flag if an instance mask is needed for each robot.
    mask_required = False
    for writer in data_writers:
        mask_required = mask_required or writer.requireInstanceMask()
    # Setup camera
    camera_frame_id = ParameterLookup.lookupWithDefault(
        parameter='~camera_frame_id', default='camera/optical_link')
    # Look up what frame to use as the reference point for all placement.
    global_frame_id = ParameterLookup.lookupWithDefault(
        parameter='~global_frame', default='map')
    # Load each robot
    robot_list = initializeRobots()
    # Load the buffer containing the motion record from the bag file.
    (tf_buffer, start_time, end_time, period) = initializeReplay(
        global_frame_id, robot_list)
    current_time = start_time
    # Create the client to tell each robot to move in Gazebo.
    gazebo_client_name = '/gazebo/set_model_state'
    rospy.loginfo('Waiting for Gazebo to start...')
    rospy.wait_for_service(gazebo_client_name)
    gazebo_client = rospy.ServiceProxy(
        name=gazebo_client_name, service_class=SetModelState)
    # Create the TF lookup
    gazebo_tf_buffer = tf2_ros.Buffer()
    gazebo_tf_listener = tf2_ros.TransformListener(gazebo_tf_buffer)
    # Create a subscriber to the camera. However, this isn't actually used. The system
    # uses wait_for_message to get the latest image. However, Gazebo won't update sensor
    # simulations unless there is a subscriber. So this tricks Gazebo into doing so.
    name = rospy.Subscriber(name='camera/image_raw',
                            data_class=Image, callback=None, queue_size=1)
    # Set up the subtractor and initialize. Only required for instance
    if mask_required:
        background_subtractor = initializeBackgroundSubtractor(
            robot_list, gazebo_client)
    # Loop through the whole bag file until the end is reached. The two times above
    # provide a moving window. The last iteration we want is when the window crosses
    # the bag end time. The next iteration would have both start and end of the
    # window past the bag file then. So use that as the break.
    rospy.loginfo('Beginning data collection...')
    while current_time < end_time:
        # Update the user on progress.
        percent_complete = (current_time - start_time).to_sec() / \
            (end_time - start_time).to_sec() * 100.0
        rospy.loginfo_throttle(
            period=5.0, msg='%.1f%% completed' % percent_complete)
        # Look up each robot's pose from the bag file TF tree.
        for robot in robot_list:
            try:
                robot_transform = tf_buffer.lookup_transform_core(
                    global_frame_id, robot.getFullFrame(), current_time)
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException), argument:
                rospy.logwarn('%s' % argument)
            robot.recordTransform(robot_transform.transform)
        # Start by moving every robot out of the way. They don't need to be in new
        # spots. They will also be collision free, since they either have a pose from
        # the bag file or the initial poses specified at launch.
        # This only needs done for instance segmentation, which is not always required.
        if mask_required:
            for robot in robot_list:
                robot_new_pose_request = robot.createSetModelStateRequest()
                robot_new_pose_request.pose.position.z = 1e6
                result = moveRobot(gazebo_client, robot_new_pose_request)
                if not result:
                    rospy.logfatal('Unable to move robot, killing processing.')
                    exit()
            # Now, go through each robot and move it into the scene.
            for robot in robot_list:
                # Move the robot and allow time for Gazebo to update
                moveRobot(gazebo_client, robot.createSetModelStateRequest())
                (image, camera_info) = captureImage()
                # Use the background subtractor to find the robot. Be sure to set the
                # learning rate to 0 to prevent updating the background image.
                foreground_mask = background_subtractor.apply(
                    image, learningRate=0.0)
                # Remove anything not within the bounding box of the robot, since those
                # can't possibly be included.
                foreground_mask = removeOutsideRobot(
                    foreground_mask, robot, gazebo_tf_buffer, camera_frame_id, camera_info)
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
        (image, camera_info) = captureImage()
        # Have each output format write their information.
        for writer in data_writers:
            writer.outputScene(robot_list, image, camera_info)
        # Update the times
        current_time += period
    # When this is finally done, record any meta information needed by networks.
    for writer in data_writers:
        writer.finalizeOutput(robot_list)
    rospy.loginfo('100%% completed')
