"""!
This module contains the Robot class to support the robot specific calculations
and hold relevant data.
"""
import rospy


class Robot(object):
    """!
    The Robot class holds the relevant information for each robot, including class
    and ID information, frame_ids, and keypoints.
    """

    def _lookupParamWithDefault(self, param_name, default_value):
        """!
        This is an internal function that assists with looking up values on the
        parameter server. Specifically, it it will warn the user and use a default
        if the parameter is not found. All parameters are assumed within the
        robot's namespace (which is also within the node's namespace)
        @param param_name The parameter to lookup, without the robot's name.
        @param default_value The default to use if not found.
        @return The value specificed by the parameter (or default if not found).
        """
        # Resolve the name relative to the robot's name.
        resolved_param_name = "~" + self.name + "/" + param_name
        if not rospy.has_param(resolved_param_name):
            error_string = 'Parameter not found for {name}. Using default instead: {default}'.format(
                name=resolved_param_name, default=default_value)
            rospy.logwarn(error_string)
        value = rospy.get_param(resolved_param_name, default_value)
        return value

    def __init__(self, name):
        """!
        The init function instantiates the class and loads all of the relevant
        information from the parameter server.
        @param nameThe name of this robot. Also includes how to find information
        about it on the parameter server.
        """
        # Set the name
        self.name = name
        # Look up the class number and associated ID
        self.class_id = self._lookupParamWithDefault("class", 0)
        self.id = self._lookupParamWithDefault("id", 1)
