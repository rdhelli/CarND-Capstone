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
