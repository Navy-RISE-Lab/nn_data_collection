#!/usr/bin/env python
import rosbag
import rospy


def loadBagFile():
    """
    This function looks up the name of the bag file from the parameter server and loads that bag file. It also determines
    the rate at which the code should step through the bag file.
    @return A tuple with - (An object containing the loaded bagfile, the rate at which it should be stepped through)

    Tests - Test if: bag file exists, if it doesn't exist, if nothing is on the parameter server, and if the bag file doesn't load.
    """
    # Assume the parameter specifying the bag file name is a private parameter.
    parameter_name = '~bag_file'
    # Make sure it exists before trying to read it.
    if not rospy.has_param(parameter_name):
        # Throw an error if it doesn't.
        error_string = 'Please specify the bag file to replay on the parameter "bag_file"'
        rospy.logfatal(error_string)
        raise (rospy.ROSInitException(error_string))
    # If this point is reached, then the parameter at least exists.
    bag_file = rospy.get_param(parameter_name)
    # Open the bag file. This might throw an error if there is an issue with the file. Go ahead and
    # let that exception pass up, since it will contain useful information to the user.
    bag = rosbag.Bag(f=bag_file)
    # If this point is reached, the bag file is loaded and ready.
    return bag

    # def loadCamera():
    #     """
    #     Look up the name of the topic to subscribe to for the camera image and camera information. Get the frame_id of the camera for use
    #     by TF transforms later.
    #     @return Those three parameters as a tuple - image topic, camera information topic, frame_id.

    #     Tests - Test if: verify if topics/frame_id exist. Default values/exit if not found.
    #     """

    # def loadRobots():
    #     """
    #     Look up the names of all the robots that should be controlled in this code. Then, create a robot object for each one.
    #     @return A list of all Robot objects

    #     Tests - must provide at least 1 robot
    #     """

    # class Robot:
    #     """
    #     This class contains all of the data needed to interface with one particular robot.
    #     """

    #     def __init__(self, name):
    #         """
    #         Given the name of the robot, look up all the relevant parameters from the server and store them for reference.

    #         Test - watch for defaults when a value isn't found or warn/crash when an important value isn't found
    #         """
    #         # self.name
    #         # self.class_id
    #         # etc.
    #     def createRobotMessage():
    #         """
    #         This creates a Gazebo XXXX method that will move this robot to the specified point in Gazebo.
    #         @return the message, ready to send.
    #         """

    # def initalizeBackgroundSubtractor(robot_list, camera_image_topic, gazebo_pub):
    #     """
    #     Create the subtractor, then move every robot into the air, capture an image, and do some sort of
    #     initialization depending on the subtractor
    #     @return the subtractor
    #     """


if __name__ == "__main__":
    rospy.init_node(name='collect_data')
    # Load bag file
    bag = loadBagFile()
    # Setup camera
    # loadCamera()
    # loadRobots()
    # gazebo_pub = subscribe to Gazebo movement topic
    # tf_lookup = subscribe to TF
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
