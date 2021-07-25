#include <ros/ros.h>
#include <std_srvs/SetBool.h>
#include <hear_msgs/set_float.h>
#include <hear_msgs/set_bool.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Point.h>
#include <std_srvs/Empty.h>

#include <fstream>
#include <sstream>
#include <string>
#include <vector>

const float TAKE_OFF_VELOCITY = 0.5; //in m/s
const float LAND_VELOCITY = 0.75; // in m/s
const std::string file_path_x = "/home/ahmed/Waypoints/waypoints_x.csv";
const std::string file_path_y = "/home/ahmed/Waypoints/waypoints_y.csv";
const std::string file_path_z = "/home/ahmed/Waypoints/waypoints_z.csv";
const std::string file_path_vel_x = "/home/ahmed/Waypoints/waypoints_vel_x.csv";
const std::string file_path_vel_y = "/home/ahmed/Waypoints/waypoints_vel_y.csv";

bool start_traj = false;
bool take_off_flag = false;
bool land_flag = false;

float take_off_height = 1.0;
float land_height = -0.4;
geometry_msgs::Point current_pos;
std_msgs::Float32 current_yaw;
ros::ServiceClient height_offset_client;

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

void pos_Cb(const geometry_msgs::Point::ConstPtr& msg){
    current_pos = *msg;
}
void yaw_Cb(const geometry_msgs::Point::ConstPtr& msg){
    current_yaw.data = msg->x;
}

bool height_Cb(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res){
    hear_msgs::set_float t_srv;
    t_srv.request.data = current_pos.z;
    height_offset_client.call(t_srv);
    ROS_INFO("height offset called");
    return true;
}

bool take_off_Cb(hear_msgs::set_float::Request& req, hear_msgs::set_float::Response& res){
    take_off_height = req.data;
    ROS_INFO("take off called");
    take_off_flag = true;
    return true;
}

bool land_Cb(hear_msgs::set_float::Request& req, hear_msgs::set_float::Response& res){
    land_height = req.data;
    ROS_INFO("land called");
    land_flag = true;
    return true;
}

bool srvCallback(hear_msgs::set_bool::Request& req, hear_msgs::set_bool::Response& res){
    start_traj = req.data;
    ROS_INFO("start trajectory called");
    return true;
}

int main(int argc, char **argv){
    ros::init(argc, argv, "trajectory_node");
    ros::NodeHandle nh;
    ros::Rate rt = 100;

    ros::ServiceServer srv = nh.advertiseService("start_trajectory", &srvCallback);
    ros::ServiceServer takeOff_srv = nh.advertiseService("take_off", &take_off_Cb);
    ros::ServiceServer land_srv = nh.advertiseService("land", &land_Cb);
    ros::ServiceServer height_offset_srv = nh.advertiseService("init_height", &height_Cb);

    height_offset_client = nh.serviceClient<hear_msgs::set_float>("set_height_offset");

    ros::Subscriber pos_sub = nh.subscribe<geometry_msgs::Point>("/pos_horizon", 10, &pos_Cb);
    ros::Subscriber yaw_sub = nh.subscribe<geometry_msgs::Point>("/providers/yaw", 10, &yaw_Cb);

    ros::Publisher pub_waypoint_x = nh.advertise<std_msgs::Float32>("/waypoint_reference/x", 10);
    ros::Publisher pub_waypoint_y = nh.advertise<std_msgs::Float32>("/waypoint_reference/y", 10);
    ros::Publisher pub_waypoint_z = nh.advertise<std_msgs::Float32>("/waypoint_reference/z", 10);
    ros::Publisher pub_waypoint_yaw = nh.advertise<std_msgs::Float32>("/waypoint_reference/yaw", 10);

    ros::Publisher pub_waypoint_vel_x = nh.advertise<std_msgs::Float32>("/waypoint_reference/vel/x", 10);
    ros::Publisher pub_waypoint_vel_y = nh.advertise<std_msgs::Float32>("/waypoint_reference/vel/y", 10);

    std::vector<float> wp_x, wp_y, wp_z, wp_vel_x, wp_vel_y;
    bool en_wp_x, en_wp_y, en_wp_z, en_wp_vel_x, en_wp_vel_y;
 
    if(!(read_file(file_path_x, wp_x))){
        ROS_WARN("Could not read file for x waypoints.\n ...Disabling trajectory for x channel");
        en_wp_x = false;
    }else{ en_wp_x = true; }
    if(!(read_file(file_path_y, wp_y))){
        ROS_WARN("Could not read file for y waypoints.\n ...Disabling trajectory for y channel");
        en_wp_y = false;
    }else{ en_wp_y = true; }
    if(!(read_file(file_path_z, wp_z))){
        ROS_WARN("Could not read file for z waypoints.\n ...Disabling trajectory for z channel");
        en_wp_z = false;
    }else{ en_wp_z = true; }

    if(!(read_file(file_path_vel_x, wp_vel_x))){
        ROS_WARN("Could not read file for x waypoints.\n ...Disabling velocity reference for x channel");
        en_wp_vel_x = false;
    }else{ en_wp_vel_x = true; }
    if(!(read_file(file_path_vel_y, wp_vel_y))){
        ROS_WARN("Could not read file for x waypoints.\n ...Disabling velocity reference for y channel");
        en_wp_vel_y = false;
    }else{ en_wp_vel_y = true; }


    int i = 0;
    int sz_x = wp_x.size(), sz_y = wp_y.size(),  sz_z = wp_z.size();
    if(en_wp_vel_x){ 
        if(wp_vel_x.size() != sz_x){
            ROS_ERROR("Size of velocity reference vector is not equal to position reference");
            return 1;
        }
    }
    if(en_wp_vel_y){ 
        if(wp_vel_y.size() != sz_y){
            ROS_ERROR("Size of velocity reference vector is not equal to position reference");
            return 1;
        }
    }

    std_msgs::Float32 wp_x_msg, wp_y_msg, wp_z_msg, wp_vel_x_msg, wp_vel_y_msg;
    float z_ref = 0.0;
    bool take_off_started = false;
    bool land_started = false;
    bool trajectory_finished = false, trajectory_started = false;
    while(ros::ok()){
        if(land_flag){
            if(!land_started){
                ROS_INFO("land started");                
                land_started = true;
                wp_x_msg.data = current_pos.x;
                wp_y_msg.data = current_pos.y;
                z_ref = current_pos.z - 0.1;

                pub_waypoint_x.publish(wp_x_msg);
                pub_waypoint_y.publish(wp_y_msg);
            }
            z_ref -= (rt.expectedCycleTime()).toSec()*LAND_VELOCITY; 
            if(z_ref <= land_height){
                land_flag = false;
                land_started = false;
                ROS_INFO("land finished");
            }else{
                wp_z_msg.data = z_ref;
                pub_waypoint_z.publish(wp_z_msg);
            }
        }
        else if(take_off_flag){
            if(!take_off_started){
                ROS_INFO("take off started");
                take_off_started = true;
                wp_x_msg.data = current_pos.x;
                wp_y_msg.data = current_pos.y;
                z_ref = current_pos.z + 0.1;

                pub_waypoint_x.publish(wp_x_msg);
                pub_waypoint_y.publish(wp_y_msg);
            }
            z_ref += (rt.expectedCycleTime()).toSec()*TAKE_OFF_VELOCITY; 
            if(z_ref >= take_off_height){
                take_off_flag = false;
                take_off_started = false;
                ROS_INFO("take off finished");
            }
            else{
                wp_z_msg.data = z_ref;
                pub_waypoint_z.publish(wp_z_msg);
            }
        }
        else if(start_traj){
            trajectory_finished = true;
            if(!trajectory_started){
                ROS_INFO("Trajectory Started");
                trajectory_started = true;
            }
            if(i < sz_x && en_wp_x){
                wp_x_msg.data = wp_x[i];
                pub_waypoint_x.publish(wp_x_msg);
                if(en_wp_vel_x){
                    wp_vel_x_msg.data = wp_vel_x[i];
                    pub_waypoint_vel_x.publish(wp_vel_x_msg);
                }
                trajectory_finished = false;
            }
            if(i < sz_y && en_wp_y){
                wp_y_msg.data = wp_y[i];
                pub_waypoint_y.publish(wp_y_msg);
                if(en_wp_vel_y){
                    wp_vel_y_msg.data = wp_vel_y[i];
                    pub_waypoint_vel_y.publish(wp_vel_y_msg);
                }
                trajectory_finished = false;
            }
            if(i < sz_z && en_wp_z){
                wp_z_msg.data = wp_z[i];
                pub_waypoint_z.publish(wp_z_msg);
                trajectory_finished = false;
            }
            if(trajectory_finished){
                ROS_INFO("Trajectory finished");
                start_traj = false;
                trajectory_started = false;
                i = 0;
            }
            i++;
        }
        ros::spinOnce();
        rt.sleep();
    }


}