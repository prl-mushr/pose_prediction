# pose_prediction

1. video_box.py takes in a bag file and generate bounding prism/lables on the cars detected.
2. box_csv.py produces a csv file that contains the coordinates of four corners of the bounding labels.



Method pipeline：
1. Car pose data correpsond to the marker. So we first need to find the center of baselink given the offset from marker to baselink.
2. Then we can find the center of the bouding prism given the offset from the center of baselink to the prism.
3. We compute the 3D coordinates of eight corners of the prism, then project them to 2D.
4. We take the min and max of x and y of the eight corners and produce the labels/bounding prisms. 

Take car 24 and 26 as an example:
mark24_T_cam26 = base26_T_cam26 * mark26_T_base26 * world_T_mark26 * mark24_T_world
center24_T_cam26 = mark24_T_cam26 * base24_T_mark24 * center24_T_base24
 
mark26_T_world and mark24_T_world is given by the mocap data, world_T_mark26 is the inverse of mark26_T_world

