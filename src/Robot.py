"""!
This module contains the Robot class to support the robot specific calculations
and hold relevant data.
"""
from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import Point
from geometry_msgs.msg import Transform
import rospy
import ParameterLookup


class Robot(object):
    """!
    The Robot class holds the relevant information for each robot, including class
    and ID information, frame_ids, and keypoints.
    """

    def __init__(self, name):
        """!
        The init function instantiates the class and loads all of the relevant
        information from the parameter server.
        @param nameThe name of this robot. Also includes how to find information
        about it on the parameter server.
        """
        # Set the name
        self._name = name
        # Look up the class number and associated ID
        self._class_id = ParameterLookup.lookup(
            '~' + self._resolveString('class'))
        self._id = ParameterLookup.lookup('~' + self._resolveString("id"))
        # Also look up the keypoints used for this robot
        self._keypoints = self._initializeKeypoints(
            '~' + self._resolveString("keypoints"))
        # Determine what name to use to find the robot in the TF tree.
        self._frame_id = ParameterLookup.lookupWithDefault(
            '~' + self._resolveString("frame_id"), "base_link")
        # Set the pose to the origin.
        transform = Transform()
        transform.rotation.w = 1.0
        self.recordTransform(transform)

    def _initializeKeypoints(self, param_name):
        """!
        This is an internal function to look up and process the list of keypoints
        that is provided for the robot. It converts them to a list of
        geometry_msgs/Point. All parameters are assumed within the robot's namespace
        (which is also within the node's namespace)
        @param param_name The parameter to lookup, without the robot's name.
        @return The keypoints as a list of geometry_msgs/Point.
        @throw rospy.InitException Thrown if the keypoints parameter is not
        found.
        """
        keypoint_param = ParameterLookup.lookup(param_name)
        # The specifications are imported as a list of lists, so iterate
        # through the outer one and convert the inner into the message
        # format.
        keypoints = []
        for keypoint_list in keypoint_param:
            # Verify that each point has three entitites for the 3 dimensions.
            if len(keypoint_list) != 3:
                error_string = 'Keypoints must have all three dimensions: [x, y, z]'
                rospy.logfatal(error_string)
                raise rospy.ROSInitException(error_string)
            keypoint_message = Point()
            keypoint_message.x = keypoint_list[0]
            keypoint_message.y = keypoint_list[1]
            keypoint_message.z = keypoint_list[2]
            keypoints.append(keypoint_message)
        return keypoints

    def _resolveString(self, value):
        """!
        @brief Prepend "<name>/" to the provided term.
        @param value The value to prepend.
        @return A string - "<name>/value"
        """
        result = self.getName() + '/' + value
        return result

    def getFullFrame(self):
        """!
        Returns the full frame ID used to find this robot on the TF tree. This is assumed
        to take the form <robot_name>/<frame> where <frame> was specified via parameter
        server.
        @return A string representing the full frame ID.
        """
        full_id = self.getName() + '/' + self._frame_id
        return full_id

    def createSetModelStateRequest(self, new_transform=None):
        """!
        Create a gazebo_msgs/ModelState message based on the recorded pose.
        @param robot_transform The geometry_msgs/Transform that represents the transform
        from the global frame to the robot's frame. Used when creating the message.
        @return A gazebo_msgs/ModelState containing the robot's transform.
        """
        msg = ModelState()
        msg.model_name = self.getName()
        if new_transform is None:
            transform = self.getTransform()
        else:
            transform = new_transform
        msg.pose.position.x = transform.translation.x
        msg.pose.position.y = transform.translation.y
        msg.pose.position.z = transform.translation.z
        msg.pose.orientation = transform.rotation
        return msg

    def getName(self):
        """!
        Get the robot's name.
        @return The robot's name as a string.
        """
        return self._name

    def getPixelMask(self):
        """!
        Get the latest pixel mask for this robot.
        @return The pixel mask where 1 represents the robot in the image and
        0 represents everything else.
        """
        return self._mask

    def getTransform(self):
        """!
        Get the robot's current transform from the global frame to the robot's
        frame. This includes the actual z value, not the one out of the camera frame.
        @return The robot's current transform.
        """
        return self._transform

    def recordPixelMask(self, mask):
        """!
        Update the stored mask of an image where all 1 values represent pixels featuring
        this robot and 0 is everything else.
        @param mask The image mask to record.
        """
        self._mask = mask

    def recordTransform(self, transform):
        """!
        Update the record of the robot's new transform. This should be the transform from
        the bag file, not the tranform to remove the robot from the camera's field of view.
        @param transform The new transform for the robot. It represents the transform from the
        global frame to the frame of the robot.
        """
        self._transform = transform
