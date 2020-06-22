/*
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <std_msgs/Int32.h>


int main(int argc, char** argv){
    rosbag::Bag bag;
    bag.open("/home/tudorf/mushr/catkin_ws/src/learning_image_geometry/src/output.bag");  // BagMode is Read by default

    //int car_size = 1; // assume car image is a 2D sqaure, and the side is 1 foot long.
    //int upper_left, upper_right, bottom_left, bottom_right = 0;
    //int x,y;

    ROS_INFO("Entering for loop");
    bool never_null = true;

    for(rosbag::MessageInstance const m: rosbag::View(bag))
    {
      std_msgs::Int32::ConstPtr i = m.instantiate<std_msgs::Int32>();
      if (i != nullptr)
      {
        never_null = false;
        ROS_INFO("%d", i->data);
      }
        // std::cout << i->data << std::endl;
        //x_ul = x - 1/2;
        //y_ul = y + 1/2;


    }
    ROS_INFO("%d", never_null);
    bag.close();
}
*/
#include <ros/ros.h>
#include <ros/console.h>

#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <geometry_msgs/PoseStamped.h>
#include <tf/tf.h>

#include <boost/foreach.hpp>
#define foreach BOOST_FOREACH

int main(int argc, char** argv) {
    rosbag::Bag bag;
    // TODO: Remove absolute paths
    bag.open("/home/seanyc/catkin_ws/src/learning_image_geometry/src/output.bag", rosbag::bagmode::Read);

    std::vector<std::string> topics;
    double dx = 0.305;
    double dy = 0.1525;
    double dz = 0.1525;

    topics.push_back(std::string("/car24/PoseStamped"));

    rosbag::View view(bag, rosbag::TopicQuery(topics));

    foreach(rosbag::MessageInstance const m, view) {
        geometry_msgs::PoseStamped::ConstPtr stampedPose = m.instantiate<geometry_msgs::PoseStamped>();
        if (stampedPose != NULL) {

            std::cout << stampedPose->header << std::endl;

            double x = stampedPose->pose.position.x;
            double y = stampedPose->pose.position.y;
            double z = stampedPose->pose.position.z;

            tf::Quaternion q(
                stampedPose->pose.orientation.x,
                stampedPose->pose.orientation.y,
                stampedPose->pose.orientation.z,
                stampedPose->pose.orientation.w
            );
            tf::Matrix3x3 m(q);
            double roll, pitch, yaw;
            m.getRPY(roll, pitch, yaw);

            // Roll: y,z
            // Pitch: x,z
            // Yaw: x,y
            double cx_fb = dx * cos(yaw)  * cos(pitch);
            double cy_fb = dx * sin(yaw)  * cos(roll);
            double cz_fb = dx * cos(roll) * sin(pitch);
            
            double cx_lr = dy * sin(yaw)  * cos(pitch);
            double cy_lr = dy * cos(yaw)  * cos(roll);
            double cz_lr = dy * cos(roll) * sin(pitch);

            double cx_tb = dz * cos(yaw) * sin(pitch);
            double cy_tb = dz * sin(yaw) * sin(roll);
            double cz_tb = dz * cos(pitch) * cos(roll);

            // Eight Corners:
            geometry_msgs::Point front_left_top;
            front_left_top.x = x + cx_fb - cx_lr + cx_tb;
            front_left_top.y = y + cy_fb - cy_lr + cy_tb;
            front_left_top.z = z + cz_fb - cz_lr + cz_tb;

            geometry_msgs::Point front_left_bottom;
            front_left_bottom.x = x + cx_fb - cx_lr - cx_tb;
            front_left_bottom.y = y + cy_fb - cy_lr - cy_tb;
            front_left_bottom.z = z + cz_fb - cz_lr - cz_tb;
                                                        
            geometry_msgs::Point front_right_top;
            front_right_top.x = x + cx_fb + cx_lr + cx_tb;
            front_right_top.y = y + cy_fb + cy_lr + cy_tb;
            front_right_top.z = z + cz_fb + cz_lr + cz_tb;
                                                        
            geometry_msgs::Point front_right_bottom;
            front_right_bottom.x = x + cx_fb + cx_lr - cx_tb;
            front_right_bottom.y = y + cy_fb + cy_lr - cy_tb;
            front_right_bottom.z = z + cz_fb + cz_lr - cz_tb;
                                                        
            geometry_msgs::Point back_left_top;
            back_left_top.x = x - cx_fb - cx_lr + cx_tb;
            back_left_top.y = y - cy_fb - cy_lr + cy_tb;
            back_left_top.z = z - cz_fb - cz_lr + cz_tb;

            geometry_msgs::Point back_left_bottom;
            back_left_bottom.x = x - cx_fb - cx_lr - cx_tb;
            back_left_bottom.y = y - cy_fb - cy_lr - cy_tb;
            back_left_bottom.z = z - cz_fb - cz_lr - cz_tb;
                                                        
            geometry_msgs::Point back_right_top;
            back_right_top.x = x - cx_fb + cx_lr + cx_tb;
            back_right_top.y = y - cy_fb + cy_lr + cy_tb;
            back_right_top.z = z - cz_fb + cz_lr + cz_tb;
                                                        
            geometry_msgs::Point back_right_bottom;
            back_right_bottom.x = x - cx_fb + cx_lr - cx_tb;
            back_right_bottom.y = y - cy_fb + cy_lr - cy_tb;
            back_right_bottom.z = z - cz_fb + cz_lr - cz_tb;
            
        }
    }
    bag.close();
}