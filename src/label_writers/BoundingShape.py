import cv2
from label_writers import LabelWriterBase
import ParameterLookup


class BoundingShape(LabelWriterBase):
    """!
    @brief For each frame, record the vertices of the bounding shape in an
    arbitrary frame.

    Each raw image has an assoicated text file. Each line on the text file has
    the unique ID of each object and the points in a provided frame. The format
    is as follows:

    id x1 y1 z1 x2 y2 z2 ...
    """

    def __init__(self, name):
        super(BoundingShape, self).__init__(name)
        self._output_frame = ParameterLookup.lookup(
            parameter='~' + self._name + '/output_frame')

    def finalizeOutput(self, robot_list):
        """!
        @brief Unusued but required.
        @param robot_list The list of all robot information.
        """
        pass

    def outputScene(self, robot_list, raw_image, camera_info):
        """!
        @brief Write the data in the format described above.
        @param robot_list The list of all robot information.
        @param raw_image The cv2::Mat of the scene.
        @param camera_info A sensor_msgs/CameraInfo with the latest
        information on the camera used to capture @ref raw_image.
        """
        id_string = '{:08d}'.format(self._count)
        # Write the raw image first.
        raw_image_file = self._output_location + id_string + '.png'
        cv2.imwrite(filename=raw_image_file, img=raw_image)
        # Add the label file
        label_file = self._output_location + id_string + '.txt'
        with open(label_file, 'w') as file:
            # Each robot will be a new line
            for robot in robot_list:
                # Write its unique ID first
                file.write(str(robot.getID()) + ' ')
                # Then write each vertex to the same line
                vertices = robot.getBoundingShape()
                for vertex in vertices:
                    vertex_transformed = self._projectPointIntoFrame(
                        vertex, robot.getFullFrame(), self._output_frame)
                    file.write(str(vertex_transformed.x) + ' ' +
                               str(vertex_transformed.y) + ' ' + str(vertex_transformed.z) + ' ')
                file.write('\n')
