import cv2
import os
import ParameterLookup
import rospy


class YOLO(object):
    """
    This class provides a means of writing the Darknet/YOLO format
    to disk. It consists of bounding boxes and class IDs per image.
    Each raw image has an associated label text file. Each line of
    the text file represents another object. The line of each line
    is as follows:
    <class id> <bounding box center x> <bounding box center y>
    <bounding box width> <bounding box height>
    Noteably, the dimensions are proportional to the overall width
    and height of the image, so range from 0 to 1 inclusive.
    Bounding boxes can overlap.
    """

    def __init__(self, name):
        """!
        Initialize the writer, including determining where to put
        all of the data files.
        @param name The name of this particular data writer. Used to
        look up values on the parameter server.
        """
        self._name = name
        # Look up where the data should go.
        output_param = "~" + self._name + "/output_folder"
        self._output_location = ParameterLookup.lookup(parameter=output_param)
        # This is used to prepend image names, so make sure it ends in a '/'
        if self._output_location[-1] != '/':
            self._output_location += '/'
        if not os.path.exists(self._output_location):
            rospy.logwarn(msg='Output directory {path} did not originally exist. Creating it'.format(
                path=self._output_location))
            os.makedirs(self._output_location)
        # Count how many images have been written for use in the names.
        self._count = 0

    def outputScene(self, robot_list, raw_image, camera_info):
        """!
        @brief Write the data in Darkent format described above.
        @param robot_list The list of all robot information.
        @param raw_image The cv2::Mat of the scene.
        @param camera_info A sensor_msgs/CameraInfo with the latest
        information on the camera used to capture @ref raw_image.
        """
        # Write the raw image first.
        raw_image_file = self._output_location + \
            '{:08d}'.format(self._count) + '.png'
        cv2.imwrite(filename=raw_image_file, img=raw_image)
        self._count += 1
