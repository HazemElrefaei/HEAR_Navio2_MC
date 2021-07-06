#include <ros/ros.h>
#include <std_srvs/SetBool.h>
#include <std_msgs/Float32.h>

#include <fstream>
#include <sstream>
#include <string>
#include <vector>

const std::string file_path_x = "/home/ahmed/waypoints_x.csv";
const std::string file_path_y = "/home/ahmed/waypoints_y.csv";

bool start_traj = false;

bool read_file(std::string fileName, std::vector<float>& vec){
    std::ifstream ifs(fileName);
    if(ifs.is_open()){
        std::string line;
        while(std::getline(ifs, line)){
            vec.push_back(std::stof(line));
        }
        return true;
    }
    return false;
}

bool srvCallback(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res){
    start_traj = req.data;
    ROS_INFO("start trajectory called");
    res.success = true;
    return true;
}

int main(int argc, char **argv){
    ros::init(argc, argv, "trajectory_node");
    ros::NodeHandle nh;
    ros::Rate rt = 100;

    ros::ServiceServer srv = nh.advertiseService("/start_trajectory", &srvCallback);

    ros::Publisher pub_waypoint_x = nh.advertise<std_msgs::Float32>("/waypoint_reference/x", 10);
    ros::Publisher pub_waypoint_y = nh.advertise<std_msgs::Float32>("/waypoint_reference/y", 10);

    std::vector<float> wp_x, wp_y;
    
    if(!(read_file(file_path_x, wp_x) && read_file(file_path_y, wp_y))){
        ROS_ERROR("Could not read file\n");
        return 0;
    }

    int i = 0;
    int max_sz = wp_x.size() < wp_y.size() ? wp_x.size() : wp_y.size();
    std_msgs::Float32 wp_x_msg, wp_y_msg;
    while(ros::ok()){
        if(start_traj){
            if(i < max_sz){
                wp_x_msg.data = wp_x[i];
                wp_y_msg.data = wp_y[i];
                
                pub_waypoint_x.publish(wp_x_msg);
                pub_waypoint_y.publish(wp_y_msg);
                i++;
            }
            else {
                i = 0;
                start_traj = false;
            }
        }
        ros::spinOnce();
        rt.sleep();
    }


}