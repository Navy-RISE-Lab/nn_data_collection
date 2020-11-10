import cv2
import numpy
import os
import ParameterLookup
import Robot
import rospy
from tf2_geometry_msgs import PointStamped
import tf2_ros


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
        # Create a TF listener to transform key points.
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(buffer=self._tf_buffer)
        timeout_param = '~' + self._name + '/tf_timeout'
        self._tf_timeout = ParameterLookup.lookupWithDefault(
            parameter=timeout_param, default=2.0)
        # Look up the camera frame ID to project points.
        self._camera_frame_id = ParameterLookup.lookupWithDefault(
            parameter='~camera_frame_id', default='camera/base_link')

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
            base_file_name = '{:08d}'.format(self._count)
            # Write the raw image
            image_path = self._output_location + robot.getName() + '/JPEGImages/'
            self._createDirIfNotFound(image_path)
            image_name = base_file_name + '.png'
            cv2.imwrite(filename=image_path + image_name, img=raw_image)
            # Write the pixel mask
            segment_path = self._output_location + robot.getName() + '/mask/'
            self._createDirIfNotFound(segment_path)
            cv2.imwrite(filename=segment_path + image_name,
                        img=robot.getPixelMask())
            # Do the keypoint projections and write to a file.
            label_path = self._output_location + robot.getName() + '/labels/'
            self._createDirIfNotFound(label_path)
            label_file = open(label_path + base_file_name + '.txt', mode='w')
            label_file.write(str(robot.getClassID()) + ' ')
            keypoints = robot.getKeypoints()
            for keypoint in keypoints:
                (x, y) = self._projectPointIntoImage(point=keypoint,
                                                     source_frame_id=robot.getFullFrame(), camera_info=camera_info)
                # For some reason, cv stores this as a ndarray instead of Mat. So use the shape
                # for dimensions. It is (rows x columns).
                scaled_x = x / raw_image.shape[1]
                scaled_y = y / raw_image.shape[0]
                cv2.circle(img=raw_image, center=(int(x), int(y)), radius=3,
                           color=(0, 255, 0), thickness=-1)
                label_file.write(str(scaled_x) + ' ' + str(scaled_y) + ' ')
            label_file.write('\n')
            label_file.close()
        self._count += 1
        cv2.imshow('image', raw_image)
        cv2.waitKey(0)
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

    def _projectPointIntoImage(self, point, source_frame_id, camera_info):
        """!
        @brief Takes a geometry_msgs/Point in the robot's frame and projects it
        into the image.

        This goes through two steps. It projects into the camera's frame of
        reference, then it uses the camera parameters to project into the image
        itself.
        @param point The geometry_msgs/Point to project.
        @param source_frame_id The fully resolved frame_id the point is in reference to.
        @param camera_info The sensor_msgs/CameraInfo contining the camera
        information.
        @return The a tuple of the pixel coordinates - (x, y).
        @throw Any underlying Exceptions raised.
        """
        # First, create a stamped message, since that is what the transform needs.
        point_robot_stamped = PointStamped()
        point_robot_stamped.header.stamp = rospy.Time.now()
        point_robot_stamped.header.frame_id = source_frame_id
        point_robot_stamped.point = point
        # Do the transform. Give a timeout to let it wait for the latest TF if it would otherwise
        # need to extrapolate.
        timeout = rospy.Duration(self._tf_timeout)
        point_camera_stamped = self._tf_buffer.transform(
            object_stamped=point_robot_stamped, target_frame=self._camera_frame_id, timeout=timeout)
        # Use CV's projectPoints, but it needs input as numpy so convert everything.
        point_camera = numpy.zeros(shape=(1, 3))
        point_camera[0][0] = point_camera_stamped.point.x
        point_camera[0][1] = point_camera_stamped.point.y
        point_camera[0][2] = point_camera_stamped.point.z
        distortion = numpy.array(camera_info.D)
        intrinsic = numpy.array(camera_info.K).reshape(3, 3)
        # There is no translation or rotation needed
        zeros = numpy.zeros(shape=(3, 1))
        # Do the actual conversion. The second argument is the Jacobian, which isn't needed.
        (point_image, _) = cv2.projectPoints(objectPoints=point_camera,
                                             rvec=zeros, tvec=zeros, cameraMatrix=intrinsic, distCoeffs=distortion)
        # Extract the points. It outputs a 3D matrix, so go ahead and reduce the dimensions,
        # which are mostly empty.
        point_image = point_image.flatten()
        result = (point_image[0], point_image[1])
        return result
