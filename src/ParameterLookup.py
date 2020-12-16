"""!
This module provides helper functions to look up parameters on
the parameter server. If the value is not found it provides some
sort of feedback to the user.
"""
import rospy


def lookup(parameter):
    """
    @brief Look up an essential parameter.

    If this parameter is not found, the user is warned and an
    Exception is raised.
    @param parameter The parameter to look up.
    @return The value of the parameter from the server.
    @throw rospy.InitException Thrown if the parameter is not found.
    """
    if not rospy.has_param(param_name=parameter):
        error_string = 'Parameter not found for {param}'.format(
            param=parameter)
        rospy.logfatal(msg=error_string)
        raise rospy.ROSInitException(error_string)
    # This is only reached if the parameter exists, so look it up.
    result = rospy.get_param(param_name=parameter)
    return result


def lookupWithDefault(parameter, default):
    """
    @brief Look up a parameter with a default if not found.

    If this parameter does not exist, warn the user and use
    a default instead.
    @param parameter The parameter to look for.
    @param default The default value to use if not found.
    """
    if not rospy.has_param(param_name=parameter):
        warn_string = 'Parameter not found for {param}. Using default instead: {default}'.format(
            param=parameter, default=default)
        rospy.logwarn(warn_string)
    result = rospy.get_param(param_name=parameter, default=default)
    return result
