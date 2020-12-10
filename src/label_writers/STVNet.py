import cv2
from label_writers import LabelWriterBase


class STVNet(LabelWriterBase):
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
        super(STVNet, self).__init__(name)

    def finalizeOutput(self, robot_list):
        """!
        @brief Does nothing as there is no master list or other global data files
        for this format.
        @param robot_list The list of all robot information.
        """
        pass

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
            label_file.write(str(robot.getID()) + ' ')
            keypoints = robot.getKeypoints()
            # Transform each keypoint into the camera's frame of reference.
            keypoints_transformed = []
            for keypoint in keypoints:
                keypoint_transformed = self._projectPointIntoFrame(
                    keypoint, robot.getFullFrame(), self._camera_frame_id)
                keypoints_transformed.append(keypoint_transformed)
            # Now, project them onto the image.
            image_points = self._projectPointsIntoImage(
                points=keypoints_transformed, camera_info=camera_info)
            # Write each to file in the scaled format.
            for point in image_points:
                scaled_x = point[0] / raw_image.shape[1]
                scaled_y = point[1] / raw_image.shape[0]
                label_file.write(str(scaled_x) + ' ' + str(scaled_y) + ' ')
            label_file.write('\n')
            label_file.close()
        self._count += 1
        pass
