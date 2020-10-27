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
        default is found.
        """
        # Resolve the name relative to the robot's name.
        resolved_param_name = "~" + self.name + "/" + param_name
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
        self.class_id = self._lookupParam("class")
        self.id = self._lookupParam("id")
        # Also look up the keypoints used for this robot
