import cv2
import os
import ParameterLookup
import Robot
import rospy


class STVNet(object):
    """
    This module writes formatted data to file following the format used
    for STVNet described in https://github.com/sgawalsh/stvNet/issues/2.
    """

    def __init__(self, name):
        """
        Initialize the class by determining where the data should be written.
        @param name The name of this particular data writer. Used to look
        up values on the parameter server.
        """
        self._name = name
        # Look up where the data should go.
        output_param = '~' + self._name + '/output_folder'
        self._output_location = ParameterLookup.lookup(parameter=output_param)
        # This is used as a base path, so make sure it ends in a '/'
        if self._output_location[-1] != '/':
            self._output_location += '/'
        self._createDirIfNotFound(self._output_location)
        # Count how many images have been written for use in the names.
        self._count = 0

    def outputScene(self, robot_list, raw_image, camera_info):
        """!
        @brief Write the data in the STVNet format for a single scene.
        @param robot_list The list of all robot information.
        @param raw_image The cv2::Mat of the scene.
        @param camera_info A sensor_msgs/CameraInfo with the latest
        information on the camera used to capture @ref raw_image.
        """
        # Each robot goes in its own folder, so place the raw and segmented
        # images in each.
        for robot in robot_list:
            image_path = self._output_location + robot.getName() + '/JPEGImages/'
            self._createDirIfNotFound(image_path)
            segment_path = self._output_location + robot.getName() + '/mask/'
            self._createDirIfNotFound(segment_path)
            image_name = '{:08d}'.format(self._count) + '.png'
            # Write the corresponding images
            cv2.imwrite(filename=image_path + image_name, img=raw_image)
            cv2.imwrite(filename=segment_path + image_name,
                        img=robot.getPixelMask())
            # Do the keypoint projections here
        self._count += 1
        pass

    def _createDirIfNotFound(self, path):
        """!
        @brief Create a given directory if it does not already exist.

        This will also create any nestings of directories so that the
        full path will exist.
        @param path The path to check for.
        """
        if not os.path.exists(path):
            warn_string = 'Output directory {path} did not originally exist. Creating it'
            warn_string = warn_string.format(path=path)
            rospy.logwarn(msg=warn_string)
            os.makedirs(path)
