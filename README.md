This is the project repo for the final project of the Udacity Self-Driving Car Nanodegree: Programming a Real Self-Driving Car. For more information about the project, see the project introduction [here](https://classroom.udacity.com/nanodegrees/nd013/parts/6047fe34-d93c-4f50-8336-b70ef10cb4b2/modules/e1a23b06-329a-4684-a717-ad476f0d8dff/lessons/462c933d-9f24-42d3-8bdc-a08a5fc866e4/concepts/5ab4b122-83e6-436d-850f-9f4d26627fd9).

---

# TEAM MEMBERS
* Steve Frazee - stevefrazee123@gmail.com
* Sumit Chopra - sumitch@umich.edu
* Agneev Guin - agneevguin@gmail.com
* Márton Helli - rdhelli@gmail.com


## Waypoint Updater (partial)
The goal of the first part of the waypoint updater is to publish a list of waypoints directly in front of the car for the car to follow. We are first given a list of waypoints that wraps around the entire track. From here we need to find the waypoint that is closest to the car and in front of the car. Once we have that, we can publish a list starting from that waypoint and include N waypoints after that. This task was broken down into 2 steps: first find the closest waypoint and second determine whether or not that waypoint is in front of or behind the car.

### Find the closest waypoint
Given the [x,y] coordinates of our car's location, we need to find the closest [x,y] waypoint. One way to do this would be a brute force search of the list of waypoints we are given. The advantage of this is that the list of waypoints is already given to us so we won't need to construct a new data structure. The disadvantage is that the list contains 10902 waypoints. Since a brute force search scales in O(N) time, we will have to perform 10000+ distance comparisons near the end of the list. Another option would be to create a KDTree from the list of waypoints. The advantage of this is that a nearest neighbor lookup scales in O(logN) time. The disadvantage is we have to construct the KDTree which takes O(NlogN) time. Since we only have to construct the KDTree once and we have to perform lookups many times per second, it makes much more sense to use a KDTree and take the penalty of construction while reaping the benefits of a much faster nearest neighbor search time.

Using the scipy.spatial library the KDTree is easily constructed once in the waypoints_cb function:
```
    def waypoints_cb(self, lane):
        ## called once on startup
        # save base waypoints as lane object
        self.base_waypoints = lane
        # create list of [x,y] waypoint positions to initialize kd_tree
        if not self.kd_tree:
            kd_tree_pts = [[waypoint.pose.pose.position.x, waypoint.pose.pose.position.y] for waypoint in lane.waypoints]
            self.kd_tree = KDTree(kd_tree_pts)
```

Now that we have a KDTree constructed we can find the closest waypoint to the car's current location by querying the tree:
```
    closest_waypoint_idx = self.kd_tree.query(self.current_pose)[1]

```

### Checking if the closest waypoint is in front or behind the car
These waypoints are eventually going to be sent to the drive by wire node to create control commands so the car can follow the waypoints. We don't want the car to try and follow a waypoint that is behind the car, so we now need to make sure that the closest waypoint we found is actually in front of the car. The method I chose to accomplish this is to compare the distance between the nearest neighbor and the previous waypoint and the distance between the car and the previous waypoint. If the car is closer to the previous waypoint than the nearest neighbor, then the nearest neighbor must be in front of the car. If the nearest neighbor is closer, then it is behind the car. In this case we can just take the next waypoint in the list to get the waypoint that is ahead of the car.

Using the euclidean function from the scipy.spatial.distance library, the distances between the car and previous waypoint and the nearest neighbor and previous waypoint are compared:
```
    def get_closest_waypoint_idx(self):
        # get index of waypoint closest to the car
        closest_waypoint_idx = self.kd_tree.query(self.current_pose)[1]
        # check if point is behind or ahead of vehicle by comparing distance to previous point
        previous_waypoint = [self.base_waypoints.waypoints[closest_waypoint_idx - 1].pose.pose.position.x, self.base_waypoints.waypoints[closest_waypoint_idx - 1].pose.pose.position.y]
        current_waypoint = [self.base_waypoints.waypoints[closest_waypoint_idx].pose.pose.position.x, self.base_waypoints.waypoints[closest_waypoint_idx].pose.pose.position.y]
        car_dist = euclidean(self.current_pose, previous_waypoint)
        waypoint_dist = euclidean(current_waypoint, previous_waypoint)
        # if the car is further away from the previous waypoint than the closest waypoint, then the closest waypoint is behind the car and we should take the next waypoint in the list
        if car_dist > waypoint_dist:
            closest_waypoint_idx = (closest_waypoint_idx + 1) % len(self.base_waypoints.waypoints)
        return closest_waypoint_idx
```

## Control Subsystem

### Drive By Wire (DBW) Node
The DBW node is the final step in the self driving vehicle’s system. At this point we have a target linear and angular velocity and must adjust the vehicle’s controls accordingly. In this project we control 3 things: throttle, steering, brakes. As such, we have 3 distinct controllers to interface with the vehicle.

#### Throttle Controller
The throttle controller is a simple PID controller that compares the current velocity with the target velocity and adjusts the throttle accordingly. The throttle gains were tuned using trial and error for allowing reasonable acceleration without oscillation around the set-point.

#### Steering Controller
This controller translates the proposed linear and angular velocities into a steering angle based on the vehicle’s steering ratio and wheelbase length. To ensure our vehicle drives smoothly, we cap the maximum linear and angular acceleration rates. The steering angle computed by the controller is also passed through a low pass filter to reduce possible jitter from noise in velocity data.

#### Braking Controller
This is the simplest controller of the three. It takes takes into consideration the vehicle mass, the wheel radius, as well as the brake_deadband to determine the deceleration force.  Despite the simplicity, we found that it works well to ensure reasonable stopping distances.


It uses the Controller class from twist_controller.py to calculate the required throttle, brake and steering inputs.
The commands are only published when the driver enables the dbw module.

```

        while not rospy.is_shutdown():

            # If velocity values are published use the control function to calculate the throttle, brake and steering commands.
            if not None in (self.current_vel, self.linear_vel, self.angular_vel):
            	self.throttle, self.brake, self.steering = self.controller.control(self.current_vel,
            														self.dbw_enabled,
            														self.linear_vel,
            														self.angular_vel)

            # if <dbw is enabled>:
            #   self.publish(throttle, brake, steer)
            if self.dbw_enabled:
            	self.publish(self.throttle, self.brake, self.steering)

```

### twist_controller.py

The file defines the Controller class that performs the calculations to get the throttle, steering and brake inputs.

The control function takes the current velocity, linear velocity, angular velocity, dbw enabled flag; and returns the throttle, brake and steering commands

It uses functions from the YawController class in yaw_controller.py to calculate the steering and functions from PID class in pid.py to adjust the throttle depending on the velocity error. It also sets the throttle to zero when the target velocity is less than the current velocity and when the vehicle is stopped.

The brake torque is calculated based on the velocity error and throttle values. If the vehicle is stopped i.e. the reference linear velocity = 0 and the current velocity is below 0.1, the brake torque is a constant 700 N-m, else if the reference velocity is lower than the current velocity & the throttle input is below 0.1, the brake torque is calculated based on the deceleration required, the mass of the vehicle and the wheel radius

It returns the calculated values only if the dbw module is enabled, else it returns a 0 value for all commands.

```

    def control(self, current_vel, dbw_enabled, linear_vel, angular_vel):
        # The function uses the YawController class and PID class to calculate the throttle, steering inputs and applies the brake based on throttle, velocity.
        # Returns throttle, brake, steer

        if not dbw_enabled:
        	self.throttle_controller.reset()
        	return 0., 0., 0.

    	current_vel = self.vel_lpf.filt(current_vel)

    	steering = self.yaw_controller.get_steering(linear_vel, angular_vel, current_vel)

    	vel_error = linear_vel - current_vel
    	self.last_vel = current_vel

    	current_time = rospy.get_time()
    	sample_time = current_time - self.last_time
    	self.last_time = current_time

    	throttle = self.throttle_controller.step(vel_error, sample_time)
    	brake = 0

    	if linear_vel == 0. and current_vel < 0.1:
    		throttle = 0
    		brake = 700 #N-m - to hold the car in place if we stopped at a light. 

        elif throttle < .1 and vel_error < 0:
            throttle = 0
            decel = max(vel_error, self.decel_limit)
            brake = abs(decel)*self.vehicle_mass*self.wheel_radius

        return throttle, brake, steering

```

### yaw_controller.py

This file defines the YawController class that converts the target linear and angular velocities to steering commands.

```

    def get_angle(self, radius):
        angle = atan(self.wheel_base / radius) * self.steer_ratio
        return max(self.min_angle, min(self.max_angle, angle))

    def get_steering(self, linear_velocity, angular_velocity, current_velocity):
        angular_velocity = current_velocity * angular_velocity / linear_velocity if abs(linear_velocity) > 0. else 0.

        if abs(current_velocity) > 0.1:
            max_yaw_rate = abs(self.max_lat_accel / current_velocity);
            angular_velocity = max(-max_yaw_rate, min(max_yaw_rate, angular_velocity))

        return self.get_angle(max(current_velocity, self.min_speed) / angular_velocity) if abs(angular_velocity) > 0. else 0.0;

```

### pid.py

This file defines the PID class that calculates the throttle input based on velocity error if the velocity error is positive i.e. the target velocity is more than the current velocity.

```

    def step(self, error, sample_time):

        integral = self.int_val + error * sample_time;
        derivative = (error - self.last_error) / sample_time;

        val = self.kp * error + self.ki * integral + self.kd * derivative;

        if val > self.max:
            val = self.max
        elif val < self.min:
            val = self.min
        else:
            self.int_val = integral
        self.last_error = error

        return val
```

## Traffic light classifier
This repository is part of the Udacity Capstone Project for the detection and classification of traffic lights. 
The model evaluates every 4th frame from the camera and determines the presence of traffic lights and also classifies them as Red, Yellow or Green.
Based on the state of the traffic light, the waypoints are updated to determine the motion of the car.


### DATASET PREPARATION

To obtain a good training dataset, we recorded a video sequence from the simulator and used it to annotate the images. We used VGG Image Annotation ([VIA Annotation Tool]( http://www.robots.ox.ac.uk/~vgg/software/via/via.html))  for this purpose.
The tool developed by Oxford University allows to drag rectangles over the image and can be used to generate csv or json files.
More than 150 images were annotated for the simulation environment and 50 images from the real environment. 

This was further compiled with the datasets from the previous students to improve performance. Thanks to [ooleksyuk](https://github.com/ooleksyuk/CarND-Traffic-Light-Detector-Classifier) and [Az4z3l](https://github.com/Az4z3l/CarND-Traffic-Light-Detection) for their efforts and providing us with bigger datasets. This really helped us increase the dataset to over 200 images per each class.

We highly appreciate the blog by [WuStangDan](https://medium.com/@WuStangDan/step-by-step-tensorflow-object-detection-api-tutorial-part-1-selecting-a-model-a02b6aabe39e) for the generation of the TF records and the training process. However, due the latest modifications in the Tensorflow object detection API, certain python functions had to be picked from legacy versions, recreated and merged to be able to use.

Due to the different format of the annotated images, a converter was created to generate XML files from CSV and YAML formats containing the image as the key, the image size, bounding box regions and the classes. The XML files were then used to create the TFrecord files for tensorflow training and testing.

	python create_tf_record.py --data_dir=data \
        --labels_dir=data/labels \
        --labels_map_path=config/labels_map.pbtxt \
        --output_path=data/train.record

### TRAINING PROCESS

The tensorflow package contains the file object_detection/model_main.py which is used for the training. 
After analyzing the results of different models, we chose to use rfn_resnet101_coco as our base model.
This is obtained from documentation [here](https://github.com/tensorflow/models/blob/master/research/object_detection/g3doc/detection_model_zoo.md).

	python object_detection/model_main.py \
        --pipeline_config_path=config/rfcn_resnet101_coco.config \
        --model_dir=models/rfcn_resnet101_coco 

The training was run on 4x 1080 GTX GPY system for over 12 hours and the inference models were picked from the 20000 checkpoint.
The below parameters were set in as json inputs while running the training:

	"anchors":              [3,9, 4,13, 5,17, 6,20, 7,22, 9,30, 11,34, 15,46, 20,27],
        "labels":               ["red", "yellow", "green"],
        "train_times":          2,
        "batch_size":           8,
        "learning_rate":        5e-5,
        "nb_epochs":            20000,
        "warmup_epochs":        5,
        "ignore_thresh":        0.5,
        "gpus":                 "0,1,2,3"
Further on, the model was exported from the checkpoint file using the following script.

	python object_detection/export_inference_graph.py \
	    --input_type=image_tensor \
	    --pipeline_config_path=config/rfcn_resnet101_coco.config \
	    --trained_checkpoint_prefix=models/rfcn_resnet101_coco/model.ckpt-20000 \
	    --output_directory=models/output/rfcn_resnet101_coco


### OPTIMIZE NETWORK

Due to the large computational time and the large size of ~ 200 MB for the pb model, the model could not be used for evaluation for the use in this project. Thus, along with the improvisation of the inference time, we had to ensure that the network is less than 100 MB. Due to the availability of limited data, there was a possibility of augmenting the data further. But this would not have reduce the size. To optimize the network, we used the transform_graph script by tensorflow and used it to optimize the weights. This used the existing frozen model, removed the unused nodes, converting them from floats to ints and quantizing the weights. The resultant optimized model was reduced to less than 50MB. On evaluating with the workspace, the inference time turned out to be around 0.2 secs.

	bazel-bin/tensorflow/tools/graph_transforms/transform_graph \
		--in_graph=/models/frozen_inference_graph_sim.pb \
		--out_graph=/models/optimized_graph_sim.pb 
		--inputs='image_tensor' 
		--outputs='detection_boxes,detection_scores,detection_classes,num_detections' 
		--transforms='strip_unused_nodes(type=uint8, shape="1,299,299,3") fold_constants(ignore_errors=true) fold_batch_norms fold_old_batch_norms quantize_weights'


### IMPROVING INFERENCE TIME

A number of attempts were made testing the inference on the workspace, virtual machines and native environments. We attempted to use the map data to run the inference only when the car is about 300 waypoints away from the traffic light. The average inference time was noted around 0.22 seconds with the rfcn network on the workspace. To avoid the delay due to the large network, we trained our network with ssd_mobilenet. This new model being extremely light-weight, we observed a huge drop in the inference time resulting to 0.02 seconds. However, due to the environment testing condition, the classified netowrk output seemed to have a delay of about 4 seconds to the human eye. This could be caused due to the hardware constraints and its a challenge to view the best possible results.

### TROUBLESHOOTING

1. Missing dbw_mkz_msgs ros packages in workspace.
Everytime the workspace is restarted the ros packages need to be reinstalled. The following package errors are displayed.

		CMake Error at /opt/ros/kinetic/share/catkin/cmake/catkinConfig.cmake:83 (find_package):
		  Could not find a package configuration file provided by "dbw_mkz_msgs" with
		  any of the following names:

		    dbw_mkz_msgsConfig.cmake
		    dbw_mkz_msgs-config.cmake

		  Add the installation prefix of "dbw_mkz_msgs" to CMAKE_PREFIX_PATH or set
		  "dbw_mkz_msgs_DIR" to a directory containing one of the above files.  If
		  "dbw_mkz_msgs" provides a separate development package or SDK, be sure it
		  has been installed.

The packages can be reinstalled by an apt-update and installing the missing packages.

	sudo apt-get update
	sudo apt-get install -y ros-kinetic-dbw-mkz-msgs
	cd /home/workspace/CarND-Capstone/ros
	rosdep install --from-paths src --ignore-src --rosdistro=kinetic -y

2. Tensorflow models installation
The COCO models used for the training required the Tensorflow Object Detection API. The framework misses some of the protobuf libraries and these need to be separately installed. When using tensorflow virtual environments, the python paths overwrite each other and the API does not get installed properly.

The manual installation steps need to be carried out and are provided [here](https://github.com/tensorflow/models/blob/master/research/object_detection/g3doc/installation.md).

However, certain protobuf files continue to cause problems. 
Running the setup scripts fixes some of these but also brings in more missing files. Some of them are listed below.

	1. ImportError: cannot import name 'string_int_label_map_pb2'
	2. ImportError: cannot import name 'anchor_generator_pb2'
	3. ImportError: cannot import name 'input_reader_pb2'
While one of the errors disappear on running the install script below, the other two appear after the installation.

	cd /tensorflow/models/research/
	python setup.py build
	python setup.py install

3. Bazel installation for network optimization
Follow the steps from [here](https://docs.bazel.build/versions/master/install-ubuntu.html).

4. Version compatibility with Tensorflow
The tensorflow models available under the Object Detection API only provide models compatible with tensorflow 1.12 and above. This affected us for the batch_non_max_suppression during the post processing stage.
However the project required the use of obsolete tensorflow 1.3. To ensure compatibility older models like ssd_mobilenet_v1_coco_11_06_2017 were tested over ssd_mobilenet_v1_coco_2018_01_28 but the access to these older models are not easy. They are not listed and the links are mostly broken. An update on the Carla environment would effectively provide possibilities to newer efficient models.

The solution suggested by [Az4z3l](https://github.com/Az4z3l/CarND-Traffic-Light-Detection) to use tensorflow 1.4 seemed to work for us. This also required the specific version of protobuf. The challenges were overcome and we thank all the previous contributors to have worked upon such solutions.

## Waypoint Updater (full)

The goal of this part of the project was to use the index of the closest traffic light which shows 'RED', in order to modify the waypoint velocities and achieve a smooth deceleration to standstill before the stopline. When the 'RED' light changes to 'GREEN', the vehicle can proceed and use the original waypoints, also taking into account the acceleration limits.

In order to achieve this, the `WaypointUpdater` has to subscribe to the `/traffic_waypoint` topic, which publishes the waypoint corresponding with the stop line of the next traffix light.	The `LOOKAHEAD_WPS` was used to not stop for red lights that are too far. Before the tl_detector was finished, ground truth data could be used to test the functionality.

### generate_lane()
The code related to the waypoint updating was confined to a separate function for clarity. The need for deceleration was decided based upon whether we received a red light waypoint index, and whether it is close enough already. If not, the waypoints calculation described in Waypoint Updater (partial) was left intact. If yes, the deceleration function was used to update the waypoint velocities.

```
        if self.stopline_wp_idx == -1 or self.stopline_wp_idx >= farthest_idx:
            lane.waypoints = base_wps
        else:
            lane.waypoints = self.decelerate(base_wps, closest_idx)
```

### decelerate()
Once a deceleration was initiated, the stop waypoint index was determined with the aid of a buffer from the stopline. One waypoint was enough in practice.

```
        # the car should stop a distance before the stopline
        stop_idx = max(self.stopline_wp_idx - closest_idx - STOPLINE_BUFFER_WPS, 0)
```
Then, a square root function was applied to achieve a gradually increasing deceleration. Very low velocities should be exchanged with zero speed instead.
```
        dist = self.distance(waypoints, i, stop_idx)
        vel = math.sqrt(2 * MAX_DECEL * dist)
        if vel < 1.:
            vel = 0.
        vel = min(vel, self.get_waypoint_velocity(p))
        self.set_waypoint_velocity(p, vel)
```

---
## Original instructions

Please use **one** of the two installation options, either native **or** docker installation.

### Native Installation

* Be sure that your workstation is running Ubuntu 16.04 Xenial Xerus or Ubuntu 14.04 Trusty Tahir. [Ubuntu downloads can be found here](https://www.ubuntu.com/download/desktop).
* If using a Virtual Machine to install Ubuntu, use the following configuration as minimum:
  * 2 CPU
  * 2 GB system memory
  * 25 GB of free hard drive space

  The Udacity provided virtual machine has ROS and Dataspeed DBW already installed, so you can skip the next two steps if you are using this.

* Follow these instructions to install ROS
  * [ROS Kinetic](http://wiki.ros.org/kinetic/Installation/Ubuntu) if you have Ubuntu 16.04.
  * [ROS Indigo](http://wiki.ros.org/indigo/Installation/Ubuntu) if you have Ubuntu 14.04.
* [Dataspeed DBW](https://bitbucket.org/DataspeedInc/dbw_mkz_ros)
  * Use this option to install the SDK on a workstation that already has ROS installed: [One Line SDK Install (binary)](https://bitbucket.org/DataspeedInc/dbw_mkz_ros/src/81e63fcc335d7b64139d7482017d6a97b405e250/ROS_SETUP.md?fileviewer=file-view-default)
* Download the [Udacity Simulator](https://github.com/udacity/CarND-Capstone/releases).

### Docker Installation
[Install Docker](https://docs.docker.com/engine/installation/)

Build the docker container
```bash
docker build . -t capstone
```

Run the docker file
```bash
docker run -p 4567:4567 -v $PWD:/capstone -v /tmp/log:/root/.ros/ --rm -it capstone
```

### Port Forwarding
To set up port forwarding, please refer to the [instructions from term 2](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/16cf4a78-4fc7-49e1-8621-3450ca938b77)

### Usage

1. Clone the project repository
```bash
git clone https://github.com/udacity/CarND-Capstone.git
```

2. Install python dependencies
```bash
cd CarND-Capstone
pip install -r requirements.txt
```
3. Make and run styx
```bash
cd ros
catkin_make
source devel/setup.sh
roslaunch launch/styx.launch
```
4. Run the simulator

### Real world testing
1. Download [training bag](https://s3-us-west-1.amazonaws.com/udacity-selfdrivingcar/traffic_light_bag_file.zip) that was recorded on the Udacity self-driving car.
2. Unzip the file
```bash
unzip traffic_light_bag_file.zip
```
3. Play the bag file
```bash
rosbag play -l traffic_light_bag_file/traffic_light_training.bag
```
4. Launch your project in site mode
```bash
cd CarND-Capstone/ros
roslaunch launch/site.launch
```
5. Confirm that traffic light detection works on real life images

### Other library/driver information
Outside of `requirements.txt`, here is information on other driver/library versions used in the simulator and Carla:

Specific to these libraries, the simulator grader and Carla use the following:

|        | Simulator | Carla  |
| :-----------: |:-------------:| :-----:|
| Nvidia driver | 384.130 | 384.130 |
| CUDA | 8.0.61 | 8.0.61 |
| cuDNN | 6.0.21 | 6.0.21 |
| TensorRT | N/A | N/A |
| OpenCV | 3.2.0-dev | 2.4.8 |
| OpenMP | N/A | N/A |

We are working on a fix to line up the OpenCV versions between the two.
