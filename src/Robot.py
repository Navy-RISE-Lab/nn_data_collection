"""!
This module contains the Robot class to support the robot specific calculations
and hold relevant data.
"""


class Robot(object):
    """
    The Robot class holds the relevant information for each robot, including class
    and ID information, frame_ids, and keypoints.
    """

    def __init__(self, name):
        """
        The init function instantiates the class and loads all of the relevant
        information from the parameter server.
        @param nameThe name of this robot. Also includes how to find information
        about it on the parameter server.
        """
        self.name = name
