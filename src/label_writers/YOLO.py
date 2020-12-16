import cv2
from label_writers import LabelWriterBase
import ParameterLookup


class YOLO(LabelWriterBase):
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
        super(YOLO, self).__init__(name)
        self._main_list = ParameterLookup.lookup(
            parameter='~' + self._name + '/main_list')

    def finalizeOutput(self, robot_list):
        """!
        @brief Write the list of all images included in this dataset.

        This file name and location is determined by the parameter
        "main_list"
        @param robot_list The list of all robot information.
        """
        with open(self._main_list, 'w') as file:
            # The count increments after each scene, so is one higher
            # than the actual number of images.
            for i in range(self._count):
                line = self._output_location + '{:08d}'.format(i) + '.png\n'
                file.write(line)

    def outputScene(self, robot_list, raw_image, camera_info):
        """!
        @brief Write the data in Darkent format described above.
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
            # Create a bounding box for each robot.
            for robot in robot_list:
                # Transform every bounding point into the image to find the
                # image bounding box.
                vertices = robot.getBoundingShape()
                vertices_transformed = []
                for vertex in vertices:
                    vertex_transformed = self._projectPointIntoFrame(
                        vertex, robot.getFullFrame(), self._camera_frame_id)
                    vertices_transformed.append(vertex_transformed)
                # Project them onto the image
                image_points = self._projectPointsIntoImage(
                    vertices_transformed, camera_info)
                # Find the min and max of x and y for the bounding box
                min_x = float('inf')
                min_y = float('inf')
                max_x = float('-inf')
                max_y = float('-inf')
                for point in image_points:
                    min_x = min(min_x, point[0])
                    min_y = min(min_y, point[1])
                    max_x = max(max_x, point[0])
                    max_y = max(max_y, point[1])
                # Determine the center and width of this rectangle, scaled by the image
                center_x = ((max_x + min_x) / 2.0) / raw_image.shape[1]
                center_y = ((max_y + min_y) / 2.0) / raw_image.shape[0]
                width = float(max_x - min_x) / raw_image.shape[1]
                height = float(max_y - min_y) / raw_image.shape[0]
                # Write everything to file
                output_line = '{cid:s} {cx:0.5f} {cy:0.5f} {w:0.5f} {h:0.5f}\n'.format(
                    cid=str(robot.getClassID()), cx=center_x, cy=center_y, w=width, h=height)
                file.write(output_line)
        self._count += 1

    def requireInstanceMask(self):
        return False
