# pose_prediction

1. trimbag.py trims a bag file to the specified time range
2. video_box.py takes in a bag file and generates bounding prism/lables on the cars detected, outputting a video to check qualitative results
3. box_csv.py produces a csv file that contains the coordinates of four corners of the bounding labels.
4. gen_darknet_label.py populates a directory in darknet format (images and labels)

Recommended Usage:
1. Trim your large bag into a small portion (1-2 minutes) using trimbag.py
2. Run video_box.py and visually check the accuracy/consistency of labeling
3. Create the necessary directories to store images/labels in darknet format
3. Run gen_darknet_label.py to populate the image/label directories

Label generation pipeline:
1. Transform from the tracked mocap marker to the base_link of each car.
2. Transform from base_link to centroid of observed cars, and camera of observing car.
3. Transform from centroid of observed cars to find corners of bounding prism.
4. Project 3D coordinates of corners onto the video frame.
5. We take the min and max of x and y of the eight corners and produce the labels/bounding prisms. 
6. [gen_darknet_label only] We save the labels in darknet format (centerX, centerY, width, height)

Transforms:
1. Transforms are stored as a 4x4 numpy array, and represent the rotation and translation of a point with respect to some reference frames.
2. A_T_B represents object A w.r.t. object B.
3. To change reference frames, we can multiply by other transforms.
4. C_T_B matmul A_T_B = A_T_C
5. To take the inverse, we use the inverse_transform method in video_box.py and gen_darknet_label.py. Taking the inverse of the numpy array will not perform the expected operation.

Using car24 and car26 as an example:
mark24_T_cam26 = base26_T_cam26 * mark26_T_base26 * world_T_mark26 * mark24_T_world
center24_T_cam26 = mark24_T_cam26 * base24_T_mark24 * center24_T_base24
 
mark26_T_world and mark24_T_world is given by the mocap data, world_T_mark26 is the inverse of mark26_T_world

