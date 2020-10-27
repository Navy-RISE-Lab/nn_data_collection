"""!
This module contains the Robot class to support the robot specific calculations
and hold relevant data.
"""
from geometry_msgs.msg import Point
import rospy


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
        self._class_id = self._lookupParam("class")
        self._id = self._lookupParam("id")
        # Also look up the keypoints used for this robot
        self._keypoints = self._lookupKeypoints("keypoints")
        # Determine what name to use to find the robot in the TF tree.
        self._frame_id = self._lookupParam("frame_id", "base_link")

    def _lookupKeypoints(self, param_name):
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
        keypoint_param = self._lookupParam(param_name)
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

    def _lookupParam(self, param_name, default_value=None):
        """!
        This is an internal function that assists with looking up values on the
        parameter server. If not found, one of two things happens. If no default
        is provided, an Exception is raised. Otherwise, the default is used
        instead. All parameters are assumed within the robot's namespace (which
        is also within the node's namespace)
        @param param_name The parameter to lookup, without the robot's name.
        @param default_value If provided, the value to use. If None, will throw
        an error if the parameter isn't found.
        @return The value specified by the parameter, or default if not found and
        a default is provided.
        @throw rospy.InitException Thrown if the parameter is not found and no
        default is provided.
        """
        # Resolve the name relative to the robot's name.
        resolved_param_name = "~" + self._name + "/" + param_name
        if not rospy.has_param(resolved_param_name):
            if default_value is None:
                error_string = 'Parameter not found for {name}'.format(
                    name=resolved_param_name)
                rospy.logfatal(error_string)
                raise rospy.ROSInitException(error_string)
            else:
                error_string = 'Parameter not found for {name}. Using default instead: {default}'.format(
                    name=resolved_param_name, default=default_value)
                rospy.logwarn(error_string)
        # Otherwise, look up the value.
        value = rospy.get_param(resolved_param_name, default_value)
        return value
