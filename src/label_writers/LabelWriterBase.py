from abc import ABCMeta, abstractmethod
import cv2
import numpy
import os
import ParameterLookup
import rospy
from tf2_geometry_msgs import PointStamped
import tf2_ros


class LabelWriterBase(object):
    """
    @brief This is the base class for every subclass that writes in different formats.

    New writers should inherit this and implement the necessary abstract methods. It
    also provides some common functionality for each subclass. If any of the methods
    are going to be used, you must call super().__init__(name), otherwise they will
    fail.
    """
    __metaclass__ = ABCMeta

    def __init__(self, name):
        """
        Initialize the super class with common parameters available to all classes.
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
        # Create a TF listener to transform key points.
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(buffer=self._tf_buffer)
        timeout_param = '~' + self._name + '/tf_timeout'
        self._tf_timeout = ParameterLookup.lookupWithDefault(
            parameter=timeout_param, default=2.0)
        # Look up the camera frame ID to project points.
        self._camera_frame_id = ParameterLookup.lookupWithDefault(
            parameter='~camera_frame_id', default='camera/base_link')

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

    def _projectPointIntoFrame(self, point, source_frame_id, target_frame_id):
        """!
        @brief Transform a point from one frame of reference to another.
        @param point The geometry_msgs/Point to transform.
        @param source_frame_id The fully resolved frame_id the point is in reference to.
        @param target_frame_id The fully resolved frame_id to which the point should be
        transformed.
        @return The geometry_msgs/Point in the new frame of reference.
        """
        # First, create a PointStamped, since that is what is needed to transform.
        point_source_stamped = PointStamped()
        point_source_stamped.header.stamp = rospy.Time.now()
        point_source_stamped.header.frame_id = source_frame_id
        point_source_stamped.point = point
        # Do the transform. Give a timeout to let it wait for the latest TF if it would
        # otherwise need to extrapolate.
        timeout = rospy.Duration(self._tf_timeout)
        try:
            point_target_stamped = self._tf_buffer.transform(
                object_stamped=point_source_stamped, target_frame=target_frame_id, timeout=timeout)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as error:
            rospy.logerr(msg='TF transform error: {error}'.format(error=error))
            raise
        return point_target_stamped.point

    def _projectPointsIntoImage(self, points, camera_info):
        """!
        @brief Take points in the camera's frame of reference and project it onto the
        image.
        @param points A list of geometry_msgs/Point to project. The points should be in the
        camera's frame of reference, typically its optical frame if following convention. This
        is a frame with the positive Z axis extending out perpendicular to the image towards the
        subject.
        @param camera_info The sensor_msgs/CameraInfo message containing the camera
        information.
        @return A list of tuples of the pixel coordinates - (x, y).
        """
        number_of_points = len(points)
        # Create an array of all points to convert them all at once.
        points_camera = numpy.zeros(shape=(number_of_points, 3))
        for i in range(number_of_points):
            points_camera[i][0] = points[i].x
            points_camera[i][1] = points[i].y
            points_camera[i][2] = points[i].z
        # The camera_info message has the camera matrix as a 9 element list and
        # CV needs a 3x3 numpy array.
        intrinsic = numpy.array(object=camera_info.K).reshape(3, 3)
        # There is no translation or rotation needed
        zeros = numpy.zeros(shape=(3, 1))
        # Do the actual conversion. The second argument is the Jacobian, which isn't needed.
        (points_image, _) = cv2.projectPoints(objectPoints=points_camera,
                                              rvec=zeros, tvec=zeros, cameraMatrix=intrinsic, distCoeffs=None)
        points_image = numpy.squeeze(points_image)
        # Pull out each point into its own tuple.
        result = []
        for i in range(number_of_points):
            point_tuple = (points_image[i][0], points_image[i][1])
            result.append(point_tuple)
        return result

    @abstractmethod
    def finalizeOutput(self, robot_list):
        """!
        @brief Perform any last minute formatting or writing after all frames have
        been processed.
        @param robot_list The list of all robot information.
        """
        pass

    @abstractmethod
    def outputScene(self, robot_list, raw_image, camera_info):
        """!
        @brief Write all the needed data for a single frame.
        @param robot_list The list of all robot information.
        @param raw_image The cv2::Mat of the scene.
        @param camera_info A sensor_msgs/CameraInfo with the latest
        information on the camera used to capture @ref raw_image.
        """
        pass

    @abstractmethod
    def requireInstanceMask(self):
        """!
        @brief Tells interested parties if this writer will need an instance pixel
        mask to write the format.

        Image processing is computationally expensive. To cut down on this, the node
        only generates an instance mask if a format requires one. If all requested
        formats don't it skips that step and robot.getPixelMask returns None.
        @return True if this format requires a pixel instance mask for labeling
        correctly and False otherwise.

        @note This should probably be a property, but this is the only way I know
        how to force its inclusion in the subclasses.
        """
