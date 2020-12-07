# nn_data_collection #
This package provides a means to automatically generate labeled training data
using Gazebo to generate synthetic data.

## Setup ##
To start, there are a few things that need done.

### Motion Plan ###
The node recreates a provided series of motions for an arbitrary number of
robots. This is defined through use of a bag file. Specifically, it reads
the ```/tf``` and ```/tf_static``` topics. This can be created through any
method, including simulating autonomous movement, joystick control, or even
using real-world robots. During motion planning, record the two topics to a bag
file. The robot frames should all trace back to a common reference frame such
as "map" or "world".

An example bag file is in the test subfolder. It contains a single
robot driving forward for 10 seconds. The parameters specified in the
yaml file are all set to work with this example.

### Simulation Environment ###
The system needs access to a Gazebo simulation that has the same robots used
to generate the data. It should also have a camera. This environment doesn't
have to match the one used to record, but it should generally allow the
recreation of movement without any collisions. The following topics and
services should be available or remapped:

#### Subscriptions ####
This will resolve into the node's namespace by default.
- **camera/image_raw** *(sensor_msgs/Image)* - The image from the simulated
camera in Gazebo.

#### Service Clients ####
These will look at the global level by default.
- **/gazebo/set_model_state** *(gazebo_msgs/SetModelState)* - Used to directly
move the robots in Gazebo.
- **/gazebo/get_model_state** *(gazebo_msgs/GetModelState)* - Used to initially
lookup each robot's pose in Gazebo.

### Configuration ###
See the [configuration file](config/params.yaml) for examples of the parameters
that should be set. They can be specified on the command line, in the launch
file, or via YAML file. The last method is prefered because of the number of
parameters. They are as follows:

- **bag_file** *(required, string)* - The location of the bag file previously
recorded.
- **simulated_rate** *(optional, float)* - The number of frames per second that
will be simulated when creating the data.
- **camera_frame_id** *(default, string)* - The name of the frame within the TF
tree that contains the camera's optical frame (the frame the image resides in,
with the Z axis perpendicular to the lens)
- **global_frame** *(default, string)* - The name of the frame within the TF
tree that is the common parent frame for everything else. Usually "map" or
"world"
- **/robot_list** *(required, list or string)* - All of the robot names to
control in the data generation. These should exist in Gazebo and the bag file.
Regardless of the namespace, this should exist at the top level of the
parameter server due to lab convention.

There are three parameters available for tuning the background subtractor used
for creating the instance pixel masks. They each are within the
```background_subtractor``` namespace under the node. See OpenCV's documentation
[here](https://docs.opencv.org/master/de/de1/group__video__motion.html#ga2beb2dee7a073809ccec60f145b6b29c)
for more information.
- **history_length** *(default, int)* - How many images to store in the filter.
Note that updates do not occur after initialization. This primarily helps with
noise.
- **var_threshold** *(default, double)* - Threshold on the squared Mahalanobis
distance between the pixel and the model to assign it to the model or not.
- **detect_shadows** *(default,  bool)* - Determines if the algorithm should detect
shadows. If so, marks them with a lighter color.

There are also a number of parameters that should be specified for each robot:

- **class** *(required, string or numeric)* - A unique ID to represent the
class/type of the robot.
- **id** *(required, string or numeric)* - A unique ID for this particular robot
that is different from all other robots.
- **frame_id** *(optional, string)* - The base name of the frame in the TF tree
to use for looking up the robot's position. This is usually "base_footprint" or
"base_link" and will be resolved into the robot's namespace.
- **keypoints** *(required, list, can be empty)* - A list of points on or
around the robot that is used by some detection algorithms. This is a list of 3
element lists representing the x, y, and z components of each point. They are
relative to ```frame_id```.
- **bounding_shape** *(required, list)* - A list of points representing the
vertices of the bounding shape (usually a rectangular box) around the robot.
This is a list of 3 element lists representing the x, y, and z components of
each point. They are relative to ```frame_id```. Beyond the requirement to
circumscribe the robot, there are no constraints on this shape.

Lastly, each output format has it's own parameters. See below for information
on those. You can customize which formats are written, but this requires editing
the code. Edit the lines that appear like this:
```python
data_writers = [
	label_writers.BoundingShape('bounding_shape'),
	label_writers.YOLO('yolo'),
	label_writers.STVNet('stvnet')
]
```
You can add and remove as many writers as you want. You can even
duplicate writers as long as the names and output folders are unique. A future
version will add better customization.
### Output Writers ###
This node provides a flexible framework to write data in multiple formats. This
avoids the need to run the node multiple times or write conversion scripts.
Adding new writers is described below. Currently, there are three output
formats. Each has its own parameters that should be customized. All appear
within the writer's namespace (i.e. */node_ns/writer_name/param*)

#### Bounding Shape ####
This is a simple format that writes the bounding shape points after they have
been projected into a given frame of reference. Each scene is a seperate image
with an associated text file. Each line of the text file provides a single
robots' unique ID and all vertices in x1, y1, z1, x2, y2, z2, ... format.

- **output_folder** *(required, string)* - Where this data should be written.
- **tf_timeout** *(optional, float)* - How long to wait for a transform within
the Gazebo simulation before erroring. Useful for tuning slow systems.
- **output_frame** *(required, string)* - The frame on the TF tree to project
the bounding shape vertices into before writing.

#### STVNet ####
This writes to the format described in
[this issue discussion](https://github.com/sgawalsh/stvNet/issues/2) for the
network of the same name.

- **output_folder** *(required, string)* - Where this data should be written.
- **tf_timeout** *(optional, float)* - How long to wait for a transform within
the Gazebo simulation before erroring. Useful for tuning slow systems.

#### YOLO ####
This writes in the output format used for YOLO/Darknet, as best described  in
[this guide](https://github.com/AlexeyABdarknet#how-to-train-to-detect-your-custom-objects)

- **output_folder** *(required, string)* - Where this data should be written.
- **tf_timeout** *(optional, float)* - How long to wait for a transform within
the Gazebo simulation before erroring. Useful for tuning slow systems.
- **main_list** *(required, string)* - The location of the main text file that
YOLO stores all of the image file paths and names in. See step 6
[here](https://github.com/AlexeyABdarknet#how-to-train-to-detect-your-custom-objects)
for more information.

## Generating Data ##
Once everything is configured, you can run the node with
```rosrun nn_data_collection collect_data.py```
or put it in a launch file (recommended). The node will alert you if there are
any errors or connection problems. Once running, it will periodically print
a new line showing progress as a percent complete. Depending on how many robotts
and how many formats you have, this could take some time to complete. However,
if you successfully see any percent complete message, the system has finished
one full iteration and can be safely left to run.

After finishing, the data will all reside in the folders specified. You should
be able to go ahead and use the data straight away!

## Adding New Writers ##
The data writer classes are set up to be relatively modular, in order to allow
new formats to be added. To create a new format, follow these steps:

1. Create the file in the ```src/label_writers``` folder.
2. Add the following skeleton code to your file:
```python
from label_writers import LabelWriterBase

class NAME(LabelWriterBase):
	"""
	Describe your format here.
	"""
	
	def __init__(self, name):
		"""
		Describe any unique initialization that happens. output_folder and
		tf_timeout are automatically looked up on the parameter server by the
		parent class.
		"""
		# You must call this!
		super(NAME, self).__init__(name)
		# Do any other initialization you need.

	def finalizeOutput(self, robot_list):
		"""
		Create any master lists or post processing that occurs after every
		image has been already processed.
		"""
		pass

	def outputScene(self, robot_list, raw_image, camera_info):
		"""
		Perform any operations that occur at each image here.
		:param robot_list A list of Robot classes. Each stores useful
		information about that robot, such as names and keypoints.
		:param raw_image The unlabeled image to use.
		:param camera_info The sensor_msgs/CameraInfo message.
		"""
		pass
```
3. Fill out all of the methods to output your format. Note that there are
methods available in the parent class that can assist with transforming data
and projecting points onto an image. There is also a wealth of information
stored in each Robot class in robot_list.
4. Add your new class to the ```__init__.py``` file.
5. Add your class to the list of data_writers defined in the main function
right near the beginning of the function. They are defined like so:
```python
data_writers = [
	label_writers.BoundingShape('bounding_shape'),
    label_writers.YOLO('yolo'),
    label_writers.STVNet('stvnet')
]
```

After that, you should have everything you need! The script will automatically
call your class at each image/scene and again at the end of the script.