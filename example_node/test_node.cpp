//TODO: Check again the whole naming and archeticting of SetRelativeWaypoint and SetAbsoluteWaypoint
#include <iostream>

#include "HEAR_core/std_logger.hpp"
#include "HEAR_mission/Wait.hpp"
#include "HEAR_mission/WaitForCondition.hpp"
#include "HEAR_mission/Arm.hpp"
#include "HEAR_mission/Disarm.hpp"
#include "HEAR_mission/MissionScenario.hpp"
#include "HEAR_mission/UserCommand.hpp"
#include "HEAR_mission/SetRestNormSettings.hpp"
#include "HEAR_mission/SetHeightOffset.hpp"
#include "HEAR_mission/ResetController.hpp"
#include "HEAR_mission/SwitchTrigger.hpp"
#include "HEAR_mission/SetRelativeWaypoint.hpp"
#include "HEAR_mission/SetAbsoluteWaypoint.hpp"
#include "HEAR_mission/UpdateController.hpp"
#include "HEAR_ROS_BRIDGE/ROSUnit_UpdateControllerClnt.hpp"
//
#include "HEAR_ROS_BRIDGE/ROSUnit_UpdateControllerSrv.hpp"
#include "HEAR_control/PIDController.hpp"
//
#include "HEAR_ROS_BRIDGE/ROSUnit_InfoSubscriber.hpp"
#include "HEAR_ROS_BRIDGE/ROSUnit_Factory.hpp"
#include "HEAR_ROS_BRIDGE/ROSUnit_RestNormSettingsClnt.hpp"
#include "HEAR_ROS_BRIDGE/ROSUnit_ControlOutputSubscriber.hpp"


const float TAKE_OFF_HEIGHT = 1.0;
const float LAND_HEIGHT = -0.3;

//#define AUTO_TEST
#define TESTING
//#define BIG_HEXA
#define SMALL_HEXA

int main(int argc, char** argv) {
    Logger::assignLogger(new StdLogger());

    //****************ROS Units********************
    ros::init(argc, argv, "flight_scenario");
    ros::NodeHandle nh;

    // float kpx, kpy, kpz, kiz, kdx, kdy, kdz, kpr, kpp, kpyaw, kpyr, kdr, kdp;
    // nh.param<float>("controller_params/pid/x/kp", kpx, 1.0803);
    // nh.param<float>("controller_params/pid/x/kd", kdx, 0.4851);
    // nh.param<float>("controller_params/pid/y/kp", kpy, 1.0687);
    // nh.param<float>("controller_params/pid/y/kd", kdy, 0.4799);
    // nh.param<float>("controller_params/pid/z/kp", kpz, 1.2851);
    // nh.param<float>("controller_params/pid/z/ki", kiz, 0.3433);
    // nh.param<float>("controller_params/pid/z/kd", kdz, 0.22);
    // nh.param<float>("controller_params/pid/roll/kp", kpr, 0.3265);
    // nh.param<float>("controller_params/pid/roll/kd", kdr, 0.0565);
    // nh.param<float>("controller_params/pid/pitch/kp", kpp, 0.3569);
    // nh.param<float>("controller_params/pid/pitch/kd", kdp, 0.0617);
    // nh.param<float>("controller_params/pid/yaw/kp", kpyaw, 3.2);
    // nh.param<float>("controller_params/pid/yaw_rate/kp", kpyr, 0.32);



    ROSUnit* ros_updt_ctr = new ROSUnit_UpdateControllerClnt(nh);
    // ROSUnit* ros_info_sub = new ROSUnit_InfoSubscriber(nh);
    // ROSUnit* ros_restnorm_settings = new ROSUnit_RestNormSettingsClnt(nh);
    
    ROSUnit_Factory ROSUnit_Factory_main{nh};
    ROSUnit* ros_arm_srv = ROSUnit_Factory_main.CreateROSUnit(ROSUnit_tx_rx_type::Client,
                                                            ROSUnit_msg_type::ROSUnit_Bool, 
                                                            "arm");
    ROSUnit* set_vo_offset_srv = ROSUnit_Factory_main.CreateROSUnit(ROSUnit_tx_rx_type::Client,
                                                            ROSUnit_msg_type::ROSUnit_Bool, 
                                                            "set_map_frame_offset");
    ROSUnit* ros_en_infilt_srv = ROSUnit_Factory_main.CreateROSUnit(ROSUnit_tx_rx_type::Client,
                                                            ROSUnit_msg_type::ROSUnit_Bool, 
                                                            "enable_inner_filter");
    ROSUnit* ros_en_outfilt_srv = ROSUnit_Factory_main.CreateROSUnit(ROSUnit_tx_rx_type::Client,
                                                            ROSUnit_msg_type::ROSUnit_Bool, 
                                                            "enable_outer_filter");
    ROSUnit* ros_take_off_client = ROSUnit_Factory_main.CreateROSUnit(ROSUnit_tx_rx_type::Client,
                                                                      ROSUnit_msg_type::ROSUnit_Float,
                                                                      "take_off");
    ROSUnit* ros_land_client = ROSUnit_Factory_main.CreateROSUnit(ROSUnit_tx_rx_type::Client,
                                                                      ROSUnit_msg_type::ROSUnit_Float,
                                                                      "land");
    ROSUnit* ros_start_traj_clnt = ROSUnit_Factory_main.CreateROSUnit(ROSUnit_tx_rx_type::Client,
                                                            ROSUnit_msg_type::ROSUnit_Bool, 
                                                            "start_trajectory");

    ROSUnit* ros_trig_pid_z = ROSUnit_Factory_main.CreateROSUnit(ROSUnit_tx_rx_type::Client,
                                                            ROSUnit_msg_type::ROSUnit_Bool, 
                                                            "pid_z_trig");
    // ROSUnit* ros_rst_ctr = ROSUnit_Factory_main.CreateROSUnit(ROSUnit_tx_rx_type::Client,
    //                                                         ROSUnit_msg_type::ROSUnit_Empty,
    //                                                         "reset_z");
    ROSUnit* ros_flight_command = ROSUnit_Factory_main.CreateROSUnit(ROSUnit_tx_rx_type::Server,
                                                                    ROSUnit_msg_type::ROSUnit_Empty,
                                                                    "flight_command");//TODO: Change to user_command
	ROSUnit* ros_set_height_offset = ROSUnit_Factory_main.CreateROSUnit(ROSUnit_tx_rx_type::Client,
                                                                    ROSUnit_msg_type::ROSUnit_Empty,
                                                                    "init_height"); 
//     //*****************Flight Elements*************

    MissionElement* update_controller_pid_x = new UpdateController();
    MissionElement* update_controller_pid_y = new UpdateController();
    MissionElement* update_controller_pid_z = new UpdateController();
    MissionElement* update_controller_pid_roll = new UpdateController();
    MissionElement* update_controller_pid_pitch = new UpdateController();
    MissionElement* update_controller_pid_yaw = new UpdateController();
    MissionElement* update_controller_pid_yaw_rate = new UpdateController();

    // MissionElement* reset_z = new ResetController();

    MissionElement* arm_motors = new Arm();
    MissionElement* disarm_motors = new Disarm();
    MissionElement* enable_pid_z = new Arm();
    MissionElement* disable_pid_z = new Disarm();
    MissionElement* disable_in_filt = new Disarm();
    MissionElement* disable_out_filt = new Disarm();
    MissionElement* start_trajectory = new Arm();
    MissionElement* set_vo_offset = new Arm();

    MissionElement* take_off = new SwitchTrigger(TAKE_OFF_HEIGHT);
    MissionElement* land = new SwitchTrigger(LAND_HEIGHT);

    MissionElement* user_command = new UserCommand();

    MissionElement* set_height_offset = new Arm(); 
    
    //******************Connections***************
    update_controller_pid_x->getPorts()[(int)UpdateController::ports_id::OP_0]->connect((ros_updt_ctr)->getPorts()[(int)ROSUnit_UpdateControllerClnt::ports_id::IP_0_PID]);
    update_controller_pid_y->getPorts()[(int)UpdateController::ports_id::OP_0]->connect((ros_updt_ctr)->getPorts()[(int)ROSUnit_UpdateControllerClnt::ports_id::IP_0_PID]);
    update_controller_pid_z->getPorts()[(int)UpdateController::ports_id::OP_0]->connect((ros_updt_ctr)->getPorts()[(int)ROSUnit_UpdateControllerClnt::ports_id::IP_0_PID]);
    update_controller_pid_roll->getPorts()[(int)UpdateController::ports_id::OP_0]->connect((ros_updt_ctr)->getPorts()[(int)ROSUnit_UpdateControllerClnt::ports_id::IP_0_PID]);
    update_controller_pid_pitch->getPorts()[(int)UpdateController::ports_id::OP_0]->connect((ros_updt_ctr)->getPorts()[(int)ROSUnit_UpdateControllerClnt::ports_id::IP_0_PID]);
    update_controller_pid_yaw->getPorts()[(int)UpdateController::ports_id::OP_0]->connect((ros_updt_ctr)->getPorts()[(int)ROSUnit_UpdateControllerClnt::ports_id::IP_0_PID]);
    update_controller_pid_yaw_rate->getPorts()[(int)UpdateController::ports_id::OP_0]->connect((ros_updt_ctr)->getPorts()[(int)ROSUnit_UpdateControllerClnt::ports_id::IP_0_PID]);

    // reset_z->getPorts()[(int)ResetController::ports_id::OP_0]->connect(ros_rst_ctr->getPorts()[(int)ROSUnit_EmptyClnt::ports_id::IP_0]);

    arm_motors->getPorts()[(int)Arm::ports_id::OP_0]->connect(ros_arm_srv->getPorts()[(int)ROSUnit_SetBoolClnt::ports_id::IP_0]);
    disarm_motors->getPorts()[(int)Disarm::ports_id::OP_0]->connect(ros_arm_srv->getPorts()[(int)ROSUnit_SetBoolClnt::ports_id::IP_0]);
    enable_pid_z->getPorts()[(int)Arm::ports_id::OP_0]->connect(ros_trig_pid_z->getPorts()[(int)ROSUnit_SetBoolClnt::ports_id::IP_0]);
    disable_pid_z->getPorts()[(int)Disarm::ports_id::OP_0]->connect(ros_trig_pid_z->getPorts()[(int)ROSUnit_SetBoolClnt::ports_id::IP_0]);
    disable_in_filt->getPorts()[(int)Disarm::ports_id::OP_0]->connect(ros_en_infilt_srv->getPorts()[(int)ROSUnit_SetBoolClnt::ports_id::IP_0]);
    disable_out_filt->getPorts()[(int)Disarm::ports_id::OP_0]->connect(ros_en_outfilt_srv->getPorts()[(int)ROSUnit_SetBoolClnt::ports_id::IP_0]);
    start_trajectory->getPorts()[(int)Arm::ports_id::OP_0]->connect(ros_start_traj_clnt->getPorts()[(int)ROSUnit_SetBoolClnt::ports_id::IP_0]);
    set_vo_offset->getPorts()[(int)Arm::ports_id::OP_0]->connect(set_vo_offset_srv->getPorts()[(int)ROSUnit_SetBoolClnt::ports_id::IP_0]);

    ros_flight_command->getPorts()[(int)ROSUnit_EmptySrv::ports_id::OP_0]->connect(user_command->getPorts()[(int)UserCommand::ports_id::IP_0]);
    
    set_height_offset->getPorts()[(int)Arm::ports_id::OP_0]->connect(ros_set_height_offset->getPorts()[(int)ROSUnit_EmptyClnt::ports_id::IP_0]);
    take_off->getPorts()[(int)SwitchTrigger::ports_id::OP_0]->connect(ros_take_off_client->getPorts()[(int)ROSUnit_SetFloatClnt::ports_id::IP_0]);
    land->getPorts()[(int)SwitchTrigger::ports_id::OP_0]->connect(ros_land_client->getPorts()[(int)ROSUnit_SetFloatClnt::ports_id::IP_0]);

    //absolute_zero_Z_relative_waypoint->connect(ros_set_path_clnt);

    //*************Setting Flight Elements*************
    #ifdef SMALL_HEXA
    ((UpdateController*)update_controller_pid_x)->pid_data.kp = 0.8786; //0.51639 * 0.8;
    ((UpdateController*)update_controller_pid_x)->pid_data.ki = 0.0;
    ((UpdateController*)update_controller_pid_x)->pid_data.kd = -0.3441; //0.21192 * 0.8;
    ((UpdateController*)update_controller_pid_x)->pid_data.kdd = 0.0;
    ((UpdateController*)update_controller_pid_x)->pid_data.anti_windup = 0;
    ((UpdateController*)update_controller_pid_x)->pid_data.en_pv_derivation = 1;
    ((UpdateController*)update_controller_pid_x)->pid_data.dt = (float)1.0/120.0;
    ((UpdateController*)update_controller_pid_x)->pid_data.id = block_id::PID_X;

    ((UpdateController*)update_controller_pid_y)->pid_data.kp = 0.6714;// 0.51639 * 0.8;
    ((UpdateController*)update_controller_pid_y)->pid_data.ki = 0.0;
    ((UpdateController*)update_controller_pid_y)->pid_data.kd =  -0.2440;// * 0.8;
    ((UpdateController*)update_controller_pid_y)->pid_data.kdd = 0.0;
    ((UpdateController*)update_controller_pid_y)->pid_data.anti_windup = 0;
    ((UpdateController*)update_controller_pid_y)->pid_data.en_pv_derivation = 1;
    ((UpdateController*)update_controller_pid_y)->pid_data.dt = (float)1.0/120.0;
    ((UpdateController*)update_controller_pid_y)->pid_data.id = block_id::PID_Y;

    ((UpdateController*)update_controller_pid_z)->pid_data.kp = 1.2414; 
    ((UpdateController*)update_controller_pid_z)->pid_data.ki = 0.0; 
    ((UpdateController*)update_controller_pid_z)->pid_data.kd = -0.3316; 
    ((UpdateController*)update_controller_pid_z)->pid_data.kdd = 0.0;
    ((UpdateController*)update_controller_pid_z)->pid_data.anti_windup = 0;
    ((UpdateController*)update_controller_pid_z)->pid_data.en_pv_derivation = 1;
    ((UpdateController*)update_controller_pid_z)->pid_data.dt = (float)1.0/120.0;
    ((UpdateController*)update_controller_pid_z)->pid_data.id = block_id::PID_Z;

    ((UpdateController*)update_controller_pid_roll)->pid_data.kp =  0.4991; //0.225*0.8; //0.172195; //0.3302; //0.286708; //0.225 * 0.8; 
    ((UpdateController*)update_controller_pid_roll)->pid_data.ki = 0.0;
    ((UpdateController*)update_controller_pid_roll)->pid_data.kd = -0.0750; //0.04*0.8; //0.042464; //0.0931; //0.056559; //0.04 * 0.8;
    ((UpdateController*)update_controller_pid_roll)->pid_data.kdd = 0.0;
    ((UpdateController*)update_controller_pid_roll)->pid_data.anti_windup = 0;
    ((UpdateController*)update_controller_pid_roll)->pid_data.en_pv_derivation = 1;
    ((UpdateController*)update_controller_pid_roll)->pid_data.dt = 1.f/200.f;
    ((UpdateController*)update_controller_pid_roll)->pid_data.id = block_id::PID_ROLL;

    ((UpdateController*)update_controller_pid_pitch)->pid_data.kp = 0.5271; //0.225*0.8;// 0.3360; //0.2811;//0.275252; //0.225 * 0.8; 
    ((UpdateController*)update_controller_pid_pitch)->pid_data.ki = 0.0;
    ((UpdateController*)update_controller_pid_pitch)->pid_data.kd = -0.0792; //0.04*0.8;//0.0684; //0.053100; //0.0868;// 0.051266; //0.04 * 0.8; 
    ((UpdateController*)update_controller_pid_pitch)->pid_data.kdd = 0.0;
    ((UpdateController*)update_controller_pid_pitch)->pid_data.anti_windup = 0;
    ((UpdateController*)update_controller_pid_pitch)->pid_data.en_pv_derivation = 1;
    ((UpdateController*)update_controller_pid_pitch)->pid_data.dt = 1.f/200.f;
    ((UpdateController*)update_controller_pid_pitch)->pid_data.id = block_id::PID_PITCH;

    ((UpdateController*)update_controller_pid_yaw)->pid_data.kp = 1.6 * 2;
    ((UpdateController*)update_controller_pid_yaw)->pid_data.ki = 0.0;
    ((UpdateController*)update_controller_pid_yaw)->pid_data.kd = 0.0;
    ((UpdateController*)update_controller_pid_yaw)->pid_data.kdd = 0.0;
    ((UpdateController*)update_controller_pid_yaw)->pid_data.anti_windup = 0;
    ((UpdateController*)update_controller_pid_yaw)->pid_data.en_pv_derivation = 1;
    ((UpdateController*)update_controller_pid_yaw)->pid_data.dt = 1.f/120.f;
    ((UpdateController*)update_controller_pid_yaw)->pid_data.id = block_id::PID_YAW;

    ((UpdateController*)update_controller_pid_yaw_rate)->pid_data.kp = 0.16 * 2;
    ((UpdateController*)update_controller_pid_yaw_rate)->pid_data.ki = 0.0;
    ((UpdateController*)update_controller_pid_yaw_rate)->pid_data.kd = 0.0;
    ((UpdateController*)update_controller_pid_yaw_rate)->pid_data.kdd = 0.0;
    ((UpdateController*)update_controller_pid_yaw_rate)->pid_data.anti_windup = 0;
    ((UpdateController*)update_controller_pid_yaw_rate)->pid_data.en_pv_derivation = 1;
    ((UpdateController*)update_controller_pid_yaw_rate)->pid_data.dt = 1.f/200.f;
    ((UpdateController*)update_controller_pid_yaw_rate)->pid_data.id = block_id::PID_YAW_RATE;
    #endif

    #ifdef BIG_HEXA
    // ((UpdateController*)update_controller_pid_x)->pid_data.kp = kpx;
    // ((UpdateController*)update_controller_pid_x)->pid_data.ki = 0.0;
    // ((UpdateController*)update_controller_pid_x)->pid_data.kd = kdx;
    // ((UpdateController*)update_controller_pid_x)->pid_data.kdd = 0.0;
    // ((UpdateController*)update_controller_pid_x)->pid_data.anti_windup = 0;
    // ((UpdateController*)update_controller_pid_x)->pid_data.en_pv_derivation = 1;
    // ((UpdateController*)update_controller_pid_x)->pid_data.dt = (float)1.0/120.0;
    // ((UpdateController*)update_controller_pid_x)->pid_data.id = block_id::PID_X;

    // ((UpdateController*)update_controller_pid_y)->pid_data.kp = kpy;
    // ((UpdateController*)update_controller_pid_y)->pid_data.ki = 0.0;
    // ((UpdateController*)update_controller_pid_y)->pid_data.kd =  kdy;
    // ((UpdateController*)update_controller_pid_y)->pid_data.kdd = 0.0;
    // ((UpdateController*)update_controller_pid_y)->pid_data.anti_windup = 0;
    // ((UpdateController*)update_controller_pid_y)->pid_data.en_pv_derivation = 1;
    // ((UpdateController*)update_controller_pid_y)->pid_data.dt = (float)1.0/120.0;
    // ((UpdateController*)update_controller_pid_y)->pid_data.id = block_id::PID_Y;

    // ((UpdateController*)update_controller_pid_z)->pid_data.kp = kpz; 
    // ((UpdateController*)update_controller_pid_z)->pid_data.ki = kiz; 
    // ((UpdateController*)update_controller_pid_z)->pid_data.kd = kdz; 
    // ((UpdateController*)update_controller_pid_z)->pid_data.kdd = 0.0;
    // ((UpdateController*)update_controller_pid_z)->pid_data.anti_windup = 0;
    // ((UpdateController*)update_controller_pid_z)->pid_data.en_pv_derivation = 1;
    // ((UpdateController*)update_controller_pid_z)->pid_data.dt = (float)1.0/120.0;
    // ((UpdateController*)update_controller_pid_z)->pid_data.id = block_id::PID_Z;

    // ((UpdateController*)update_controller_pid_roll)->pid_data.kp = kpr*0.8;
    // ((UpdateController*)update_controller_pid_roll)->pid_data.ki = 0.0;
    // ((UpdateController*)update_controller_pid_roll)->pid_data.kd = kdr*0.8;
    // ((UpdateController*)update_controller_pid_roll)->pid_data.kdd = 0.0;
    // ((UpdateController*)update_controller_pid_roll)->pid_data.anti_windup = 0;
    // ((UpdateController*)update_controller_pid_roll)->pid_data.en_pv_derivation = 1;
    // ((UpdateController*)update_controller_pid_roll)->pid_data.dt = 1.f/200.f;
    // ((UpdateController*)update_controller_pid_roll)->pid_data.id = block_id::PID_ROLL;

    // ((UpdateController*)update_controller_pid_pitch)->pid_data.kp = kpp*0.8;
    // ((UpdateController*)update_controller_pid_pitch)->pid_data.ki = 0.0;
    // ((UpdateController*)update_controller_pid_pitch)->pid_data.kd =  kdp*0.8;
    // ((UpdateController*)update_controller_pid_pitch)->pid_data.kdd = 0.0;
    // ((UpdateController*)update_controller_pid_pitch)->pid_data.anti_windup = 0;
    // ((UpdateController*)update_controller_pid_pitch)->pid_data.en_pv_derivation = 1;
    // ((UpdateController*)update_controller_pid_pitch)->pid_data.dt = 1.f/200.f;
    // ((UpdateController*)update_controller_pid_pitch)->pid_data.id = block_id::PID_PITCH;

    // ((UpdateController*)update_controller_pid_yaw)->pid_data.kp = kpyaw;
    // ((UpdateController*)update_controller_pid_yaw)->pid_data.ki = 0.0;
    // ((UpdateController*)update_controller_pid_yaw)->pid_data.kd = 0.0;
    // ((UpdateController*)update_controller_pid_yaw)->pid_data.kdd = 0.0;
    // ((UpdateController*)update_controller_pid_yaw)->pid_data.anti_windup = 0;
    // ((UpdateController*)update_controller_pid_yaw)->pid_data.en_pv_derivation = 1;
    // ((UpdateController*)update_controller_pid_yaw)->pid_data.dt = 1.f/120.f;
    // ((UpdateController*)update_controller_pid_yaw)->pid_data.id = block_id::PID_YAW;

    // ((UpdateController*)update_controller_pid_yaw_rate)->pid_data.kp = kpyr;
    // ((UpdateController*)update_controller_pid_yaw_rate)->pid_data.ki = 0.0;
    // ((UpdateController*)update_controller_pid_yaw_rate)->pid_data.kd = 0.0;
    // ((UpdateController*)update_controller_pid_yaw_rate)->pid_data.kdd = 0.0;
    // ((UpdateController*)update_controller_pid_yaw_rate)->pid_data.anti_windup = 0;
    // ((UpdateController*)update_controller_pid_yaw_rate)->pid_data.en_pv_derivation = 1;
    // ((UpdateController*)update_controller_pid_yaw_rate)->pid_data.dt = 1.f/200.f;
    // ((UpdateController*)update_controller_pid_yaw_rate)->pid_data.id = block_id::PID_YAW_RATE;
        ((UpdateController*)update_controller_pid_x)->pid_data.kp = 0.6534;
    ((UpdateController*)update_controller_pid_x)->pid_data.ki = 0.0;
    ((UpdateController*)update_controller_pid_x)->pid_data.kd = 0.3831;
    ((UpdateController*)update_controller_pid_x)->pid_data.kdd = 0.0;
    ((UpdateController*)update_controller_pid_x)->pid_data.anti_windup = 0;
    ((UpdateController*)update_controller_pid_x)->pid_data.en_pv_derivation = 1;
    ((UpdateController*)update_controller_pid_x)->pid_data.dt = (float)1.0/120.0;
    ((UpdateController*)update_controller_pid_x)->pid_data.id = block_id::PID_X;

    ((UpdateController*)update_controller_pid_y)->pid_data.kp = 0.7176;
    ((UpdateController*)update_controller_pid_y)->pid_data.ki = 0.0;
    ((UpdateController*)update_controller_pid_y)->pid_data.kd =  0.4208;
    ((UpdateController*)update_controller_pid_y)->pid_data.kdd = 0.0;
    ((UpdateController*)update_controller_pid_y)->pid_data.anti_windup = 0;
    ((UpdateController*)update_controller_pid_y)->pid_data.en_pv_derivation = 1;
    ((UpdateController*)update_controller_pid_y)->pid_data.dt = (float)1.0/120.0;
    ((UpdateController*)update_controller_pid_y)->pid_data.id = block_id::PID_Y;

    ((UpdateController*)update_controller_pid_z)->pid_data.kp = 0.785493; 
    ((UpdateController*)update_controller_pid_z)->pid_data.ki = 0.098; 
    ((UpdateController*)update_controller_pid_z)->pid_data.kd = 0.239755; 
    ((UpdateController*)update_controller_pid_z)->pid_data.kdd = 0.0;
    ((UpdateController*)update_controller_pid_z)->pid_data.anti_windup = 0;
    ((UpdateController*)update_controller_pid_z)->pid_data.en_pv_derivation = 1;
    ((UpdateController*)update_controller_pid_z)->pid_data.dt = (float)1.0/120.0;
    ((UpdateController*)update_controller_pid_z)->pid_data.id = block_id::PID_Z;

    ((UpdateController*)update_controller_pid_roll)->pid_data.kp = 0.3227;
    ((UpdateController*)update_controller_pid_roll)->pid_data.ki = 0.0;
    ((UpdateController*)update_controller_pid_roll)->pid_data.kd = 0.0558;
    ((UpdateController*)update_controller_pid_roll)->pid_data.kdd = 0.0;
    ((UpdateController*)update_controller_pid_roll)->pid_data.anti_windup = 0;
    ((UpdateController*)update_controller_pid_roll)->pid_data.en_pv_derivation = 1;
    ((UpdateController*)update_controller_pid_roll)->pid_data.dt = 1.f/200.f;
    ((UpdateController*)update_controller_pid_roll)->pid_data.id = block_id::PID_ROLL;

    ((UpdateController*)update_controller_pid_pitch)->pid_data.kp = 0.2981;
    ((UpdateController*)update_controller_pid_pitch)->pid_data.ki = 0.0;
    ((UpdateController*)update_controller_pid_pitch)->pid_data.kd =  0.0515;
    ((UpdateController*)update_controller_pid_pitch)->pid_data.kdd = 0.0;
    ((UpdateController*)update_controller_pid_pitch)->pid_data.anti_windup = 0;
    ((UpdateController*)update_controller_pid_pitch)->pid_data.en_pv_derivation = 1;
    ((UpdateController*)update_controller_pid_pitch)->pid_data.dt = 1.f/200.f;
    ((UpdateController*)update_controller_pid_pitch)->pid_data.id = block_id::PID_PITCH;

    ((UpdateController*)update_controller_pid_yaw)->pid_data.kp = 1.6;
    ((UpdateController*)update_controller_pid_yaw)->pid_data.ki = 0.0;
    ((UpdateController*)update_controller_pid_yaw)->pid_data.kd = 0.0;
    ((UpdateController*)update_controller_pid_yaw)->pid_data.kdd = 0.0;
    ((UpdateController*)update_controller_pid_yaw)->pid_data.anti_windup = 0;
    ((UpdateController*)update_controller_pid_yaw)->pid_data.en_pv_derivation = 1;
    ((UpdateController*)update_controller_pid_yaw)->pid_data.dt = 1.f/120.f;
    ((UpdateController*)update_controller_pid_yaw)->pid_data.id = block_id::PID_YAW;

    ((UpdateController*)update_controller_pid_yaw_rate)->pid_data.kp = 0.16;
    ((UpdateController*)update_controller_pid_yaw_rate)->pid_data.ki = 0.0;
    ((UpdateController*)update_controller_pid_yaw_rate)->pid_data.kd = 0.0;
    ((UpdateController*)update_controller_pid_yaw_rate)->pid_data.kdd = 0.0;
    ((UpdateController*)update_controller_pid_yaw_rate)->pid_data.anti_windup = 0;
    ((UpdateController*)update_controller_pid_yaw_rate)->pid_data.en_pv_derivation = 1;
    ((UpdateController*)update_controller_pid_yaw_rate)->pid_data.dt = 1.f/200.f;
    ((UpdateController*)update_controller_pid_yaw_rate)->pid_data.id = block_id::PID_YAW_RATE;
    #endif

    // ((ResetController*)reset_z)->target_block = block_id::PID_Z;

    Wait wait_1s;
    wait_1s.wait_time_ms=1000;

    Wait wait_100ms;
    wait_100ms.wait_time_ms=100;

    Wait wait_340ms;
    wait_340ms.wait_time_ms=340;

    Wait wait_5s;
    wait_5s.wait_time_ms=5000;
  
    #ifdef TESTING
    MissionPipeline testing_pipeline;

    testing_pipeline.addElement((MissionElement*)&wait_1s);
    
    testing_pipeline.addElement((MissionElement*)update_controller_pid_x);
    testing_pipeline.addElement((MissionElement*)&wait_100ms);
    testing_pipeline.addElement((MissionElement*)update_controller_pid_y);
    testing_pipeline.addElement((MissionElement*)&wait_100ms);
    testing_pipeline.addElement((MissionElement*)update_controller_pid_z);
    testing_pipeline.addElement((MissionElement*)&wait_100ms);
    testing_pipeline.addElement((MissionElement*)update_controller_pid_roll);
    testing_pipeline.addElement((MissionElement*)&wait_100ms);
    testing_pipeline.addElement((MissionElement*)update_controller_pid_pitch);
    testing_pipeline.addElement((MissionElement*)&wait_100ms);
    testing_pipeline.addElement((MissionElement*)update_controller_pid_yaw);
    testing_pipeline.addElement((MissionElement*)&wait_100ms);
    testing_pipeline.addElement((MissionElement*)update_controller_pid_yaw_rate);
    testing_pipeline.addElement((MissionElement*)&wait_100ms);

    testing_pipeline.addElement((MissionElement*)set_height_offset); //TODO: (CHECK Desc) Set a constant height command/reference based on the current pos
    testing_pipeline.addElement((MissionElement*)&wait_1s);
    testing_pipeline.addElement((MissionElement*)user_command);
    // testing_pipeline.addElement((MissionElement*)reset_z); //Reset I-term to zero
    testing_pipeline.addElement((MissionElement*)disable_in_filt);
    testing_pipeline.addElement((MissionElement*)disable_out_filt);
    testing_pipeline.addElement((MissionElement*)&wait_100ms);
    testing_pipeline.addElement((MissionElement*)arm_motors);
    // testing_pipeline.addElement((MissionElement*)disable_pid_z);

    #ifdef AUTO_TEST
    testing_pipeline.addElement((MissionElement*)&wait_1s);
    #else
    testing_pipeline.addElement((MissionElement*)user_command);
    #endif
    
    // testing_pipeline.addElement((MissionElement*)reset_z); //Reset I-term to zero
    testing_pipeline.addElement((MissionElement*)arm_motors);
    testing_pipeline.addElement((MissionElement*)take_off);

    // testing_pipeline.addElement((MissionElement*)&wait_1s);
    // testing_pipeline.addElement((MissionElement*)&wait_340ms);
    // testing_pipeline.addElement((MissionElement*)enable_pid_z);

    #ifdef AUTO_TEST
    testing_pipeline.addElement((MissionElement*)&wait_5s);
    #else
    testing_pipeline.addElement((MissionElement*)user_command);
    #endif

    // testing_pipeline.addElement((MissionElement*)start_trajectory);
    // testing_pipeline.addElement((MissionElement*)user_command);

    // testing_pipeline.addElement((MissionElement*)forward_waypoint);
    // testing_pipeline.addElement((MissionElement*)user_command);
    // testing_pipeline.addElement((MissionElement*)right_waypoint);
    // testing_pipeline.addElement((MissionElement*)user_command);

    // testing_pipeline.addElement((MissionElement*)waypoint_set_rest_norm_settings);   
    // testing_pipeline.addElement((MissionElement*)&wait_100ms);
    // testing_pipeline.addElement((MissionElement*)absolute_origin_1m_height);
    // testing_pipeline.addElement((MissionElement*)absolute_waypoint_square_1);
    // testing_pipeline.addElement((MissionElement*)absolute_waypoint_square_2);
    // testing_pipeline.addElement((MissionElement*)absolute_waypoint_square_3);
    // testing_pipeline.addElement((MissionElement*)absolute_waypoint_square_4);
    // testing_pipeline.addElement((MissionElement*)absolute_waypoint_square_5);
    // testing_pipeline.addElement((MissionElement*)absolute_waypoint_square_6);
    // testing_pipeline.addElement((MissionElement*)absolute_waypoint_square_7);
    // testing_pipeline.addElement((MissionElement*)user_command);
    // testing_pipeline.addElement((MissionElement*)land_set_rest_norm_settings);   
    // testing_pipeline.addElement((MissionElement*)&wait_100ms);
    // testing_pipeline.addElement((MissionElement*)land_relative_waypoint);

    testing_pipeline.addElement((MissionElement*)land);
    #endif

    Logger::getAssignedLogger()->log("FlightScenario main_scenario",LoggerLevel::Info);
    MissionScenario main_scenario;

    #ifdef TESTING
    main_scenario.AddMissionPipeline(&testing_pipeline);
    #endif

    main_scenario.StartScenario();
    Logger::getAssignedLogger()->log("Main Done",LoggerLevel::Info);
    std::cout << "OK \n";
    while(ros::ok){
        ros::spinOnce();
    }
    return 0;
}