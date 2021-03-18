//TODO: Check again the whole naming and archeticting of SetRelativeWaypoint and SetAbsoluteWaypoint
// UAV controller tuning with SLAM
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
#include "HEAR_mission/SendBoolSignal.hpp"
#include "HEAR_mission/ResetController.hpp"
#include "HEAR_mission/SetRelativeWaypoint.hpp"
#include "HEAR_mission/SwitchTrigger.hpp"
#include "HEAR_mission/SetAbsoluteWaypoint.hpp"
#include "HEAR_mission/UpdateController.hpp"
#include "HEAR_ROS_BRIDGE/ROSUnit_UpdateControllerClnt.hpp"
#include "HEAR_math/ConstantFloat.hpp"
#include "HEAR_ROS_BRIDGE/ROSUnit_UpdateControllerSrv.hpp"
#include "HEAR_control/PIDController.hpp"
#include "HEAR_ROS_BRIDGE/ROSUnit_InfoSubscriber.hpp"
#include "HEAR_ROS_BRIDGE/ROSUnit_Factory.hpp"
#include "HEAR_ROS_BRIDGE/ROSUnit_RestNormSettingsClnt.hpp"
#include "HEAR_ROS_BRIDGE/ROSUnit_ControlOutputSubscriber.hpp"

//#define MRFT_X_SLAM
//#define MRFT_Y_SLAM
//#define MRFT_Z_SLAM

#define PID_X_SLAM
#define PID_Y_SLAM
#define PID_Z_SLAM

#define STEP_X

const float SLAM_FREQ = 30.0;

int main(int argc, char** argv) {
    Logger::assignLogger(new StdLogger());

    //****************ROS Units********************
    ros::init(argc, argv, "flight_scenario");
    ros::NodeHandle nh;

    ROSUnit* ros_updt_ctr = new ROSUnit_UpdateControllerClnt(nh);
    ROSUnit* ros_info_sub = new ROSUnit_InfoSubscriber(nh);
    ROSUnit* ros_restnorm_settings = new ROSUnit_RestNormSettingsClnt(nh);
    
    ROSUnit_Factory ROSUnit_Factory_main{nh};
    ROSUnit* ros_arm_srv = ROSUnit_Factory_main.CreateROSUnit(ROSUnit_tx_rx_type::Client,
                                                            ROSUnit_msg_type::ROSUnit_Bool, 
                                                            "arm");
#ifdef  MRFT_X_SLAM
    ROSUnit* ros_mrft_trigger_x = ROSUnit_Factory_main.CreateROSUnit(ROSUnit_tx_rx_type::Client,
                                                                      ROSUnit_msg_type::ROSUnit_Float,
                                                                      "mrft_switch_x");
#endif

#ifdef PID_X_SLAM
    ROSUnit* ros_slam_pid_trigger_x = ROSUnit_Factory_main.CreateROSUnit(ROSUnit_tx_rx_type::Client,
                                                                      ROSUnit_msg_type::ROSUnit_Float,
                                                                      "slam_pid_switch_x");
#endif
    
#ifdef  MRFT_Y_SLAM
    ROSUnit* ros_mrft_trigger_y = ROSUnit_Factory_main.CreateROSUnit(ROSUnit_tx_rx_type::Client,
                                                                      ROSUnit_msg_type::ROSUnit_Float,
                                                                      "mrft_switch_y");
#endif

#ifdef PID_Y_SLAM
    ROSUnit* ros_slam_pid_trigger_y = ROSUnit_Factory_main.CreateROSUnit(ROSUnit_tx_rx_type::Client,
                                                                      ROSUnit_msg_type::ROSUnit_Float,
                                                                      "slam_pid_switch_y");
#endif

#ifdef  MRFT_Z_SLAM                                                          
    ROSUnit* ros_mrft_trigger_z = ROSUnit_Factory_main.CreateROSUnit(ROSUnit_tx_rx_type::Client,
                                                                      ROSUnit_msg_type::ROSUnit_Float,
                                                                      "mrft_switch_z");
#endif

#ifdef PID_Z_SLAM
    ROSUnit* ros_slam_pid_trigger_z = ROSUnit_Factory_main.CreateROSUnit(ROSUnit_tx_rx_type::Client,
                                                                      ROSUnit_msg_type::ROSUnit_Float,
                                                                      "slam_pid_switch_z");
#endif

    ROSUnit* ros_pos_sub = ROSUnit_Factory_main.CreateROSUnit(ROSUnit_tx_rx_type::Subscriber,
                                                            ROSUnit_msg_type::ROSUnit_Point,
                                                            "global2inertial/position");
    ROSUnit* ros_rst_ctr = ROSUnit_Factory_main.CreateROSUnit(ROSUnit_tx_rx_type::Client,
                                                            ROSUnit_msg_type::ROSUnit_Int8,
                                                            "reset_controller");
    ROSUnit* ros_flight_command = ROSUnit_Factory_main.CreateROSUnit(ROSUnit_tx_rx_type::Server,
                                                                    ROSUnit_msg_type::ROSUnit_Empty,
                                                                    "flight_command");//TODO: Change to user_command
	ROSUnit* ros_set_path_clnt = ROSUnit_Factory_main.CreateROSUnit(ROSUnit_tx_rx_type::Client,
                                                                    ROSUnit_msg_type::ROSUnit_Poses,
                                                                    "uav_control/set_path");
    ROSUnit* ros_set_height_offset = ROSUnit_Factory_main.CreateROSUnit(ROSUnit_tx_rx_type::Client,
                                                                    ROSUnit_msg_type::ROSUnit_Float,
                                                                    "set_height_offset"); 
    ROSUnit* rosunit_set_map_frame_offset = ROSUnit_Factory_main.CreateROSUnit(ROSUnit_tx_rx_type::Client,
                                                                            ROSUnit_msg_type::ROSUnit_Bool,
                                                                            "set_map_frame_offset");
                                                                   
    ROSUnit* rosunit_yaw_provider = ROSUnit_Factory_main.CreateROSUnit(ROSUnit_tx_rx_type::Subscriber, 
                                                                    ROSUnit_msg_type::ROSUnit_Point,
                                                                    "/providers/yaw");
//     //*****************Flight Elements*************

    MissionElement* update_controller_pid_x = new UpdateController();
    MissionElement* update_controller_pid_y = new UpdateController();
    MissionElement* update_controller_pid_z = new UpdateController();
    MissionElement* update_controller_pid_roll = new UpdateController();
    MissionElement* update_controller_pid_pitch = new UpdateController();
    MissionElement* update_controller_pid_yaw = new UpdateController();
    MissionElement* update_controller_pid_yaw_rate = new UpdateController();

    #ifdef  MRFT_X_SLAM
    MissionElement* update_controller_mrft_x = new UpdateController();
    MissionElement* mrft_switch_on_x=new SwitchTrigger(3);
    MissionElement* mrft_switch_off_x=new SwitchTrigger(1);
    #endif

    #ifdef  MRFT_Y_SLAM
    MissionElement* update_controller_mrft_y = new UpdateController();
    MissionElement* mrft_switch_on_y=new SwitchTrigger(3);
    MissionElement* mrft_switch_off_y=new SwitchTrigger(1);
    #endif

    #ifdef  MRFT_Z_SLAM
    MissionElement* update_controller_mrft_z = new UpdateController();
    MissionElement* mrft_switch_on_z=new SwitchTrigger(3);
    MissionElement* mrft_switch_off_z=new SwitchTrigger(1);
    #endif

    #ifdef PID_X_SLAM
    MissionElement* update_controller_pid_salm_x = new UpdateController;
    MissionElement* pid_slam_switch_on_x = new SwitchTrigger(3);
    MissionElement* pid_slam_switch_off_x = new SwitchTrigger(1);
    #endif

    #ifdef PID_Y_SLAM
    MissionElement* update_controller_pid_salm_y = new UpdateController;
    MissionElement* pid_slam_switch_on_y = new SwitchTrigger(3);
    MissionElement* pid_slam_switch_off_y = new SwitchTrigger(1);
    #endif

    #ifdef PID_Z_SLAM
    MissionElement* update_controller_pid_salm_z = new UpdateController;
    MissionElement* pid_slam_switch_on_z = new SwitchTrigger(3);
    MissionElement* pid_slam_switch_off_z = new SwitchTrigger(1);
    #endif

    MissionElement* reset_z = new ResetController();

    MissionElement* arm_motors = new Arm();
    MissionElement* disarm_motors = new Disarm();

    MissionElement* user_command = new UserCommand();

    // MissionElement* state_monitor = new StateMonitor();

    MissionElement* set_restricted_norm_settings = new SetRestNormSettings(true, false, 0.8); 

    MissionElement* land_set_rest_norm_settings = new SetRestNormSettings(true, false, 0.15);

    MissionElement* set_height_offset = new SetHeightOffset();
    MissionElement* send_set_map_offset_signal = new SendBoolSignal(true); 
    MissionElement* initial_pose_waypoint = new SetRelativeWaypoint(0., 0., 0., 0.); //TODO: SetRelativeWaypoint needs substantial refactoring

    MissionElement* takeoff_relative_waypoint = new SetRelativeWaypoint(0., 0., 1.5, 0.);
    MissionElement* step_relative_waypoint_x = new SetRelativeWaypoint(0.8, 0., 0.0, 0.);
    MissionElement* land_relative_waypoint = new SetRelativeWaypoint(0., 0., -1.5, 0.);

    //******************Connections***************
    update_controller_pid_x->getPorts()[(int)UpdateController::ports_id::OP_0]->connect(ros_updt_ctr->getPorts()[(int)ROSUnit_UpdateControllerClnt::ports_id::IP_0_PID]);
    update_controller_pid_y->getPorts()[(int)UpdateController::ports_id::OP_0]->connect(ros_updt_ctr->getPorts()[(int)ROSUnit_UpdateControllerClnt::ports_id::IP_0_PID]);
    update_controller_pid_z->getPorts()[(int)UpdateController::ports_id::OP_0]->connect(ros_updt_ctr->getPorts()[(int)ROSUnit_UpdateControllerClnt::ports_id::IP_0_PID]);
    update_controller_pid_roll->getPorts()[(int)UpdateController::ports_id::OP_0]->connect(ros_updt_ctr->getPorts()[(int)ROSUnit_UpdateControllerClnt::ports_id::IP_0_PID]);
    update_controller_pid_pitch->getPorts()[(int)UpdateController::ports_id::OP_0]->connect(ros_updt_ctr->getPorts()[(int)ROSUnit_UpdateControllerClnt::ports_id::IP_0_PID]);
    update_controller_pid_yaw->getPorts()[(int)UpdateController::ports_id::OP_0]->connect(ros_updt_ctr->getPorts()[(int)ROSUnit_UpdateControllerClnt::ports_id::IP_0_PID]);
    update_controller_pid_yaw_rate->getPorts()[(int)UpdateController::ports_id::OP_0]->connect(ros_updt_ctr->getPorts()[(int)ROSUnit_UpdateControllerClnt::ports_id::IP_0_PID]);

    #ifdef  MRFT_X_SLAM
    update_controller_mrft_x->getPorts()[(int)UpdateController::ports_id::OP_0]->connect(ros_updt_ctr->getPorts()[(int)ROSUnit_UpdateControllerClnt::ports_id::IP_1_MRFT]);
    mrft_switch_on_x->getPorts()[(int)SwitchTrigger::ports_id::OP_0]->connect(ros_mrft_trigger_x->getPorts()[(int)ROSUnit_SetFloatClnt::ports_id::IP_0]);
    mrft_switch_off_x->getPorts()[(int)SwitchTrigger::ports_id::OP_0]->connect(ros_mrft_trigger_x->getPorts()[(int)ROSUnit_SetFloatClnt::ports_id::IP_0]);   
    #endif

    #ifdef  MRFT_Y_SLAM
    update_controller_mrft_y->getPorts()[(int)UpdateController::ports_id::OP_0]->connect(ros_updt_ctr->getPorts()[(int)ROSUnit_UpdateControllerClnt::ports_id::IP_1_MRFT]);
    mrft_switch_on_y->getPorts()[(int)SwitchTrigger::ports_id::OP_0]->connect(ros_mrft_trigger_y->getPorts()[(int)ROSUnit_SetFloatClnt::ports_id::IP_0]);
    mrft_switch_off_y->getPorts()[(int)SwitchTrigger::ports_id::OP_0]->connect(ros_mrft_trigger_y->getPorts()[(int)ROSUnit_SetFloatClnt::ports_id::IP_0]);   
    #endif

    #ifdef  MRFT_Z_SLAM
    update_controller_mrft_z->getPorts()[(int)UpdateController::ports_id::OP_0]->connect(ros_updt_ctr->getPorts()[(int)ROSUnit_UpdateControllerClnt::ports_id::IP_1_MRFT]);
    mrft_switch_on_z->getPorts()[(int)SwitchTrigger::ports_id::OP_0]->connect(ros_mrft_trigger_z->getPorts()[(int)ROSUnit_SetFloatClnt::ports_id::IP_0]);
    mrft_switch_off_z->getPorts()[(int)SwitchTrigger::ports_id::OP_0]->connect(ros_mrft_trigger_z->getPorts()[(int)ROSUnit_SetFloatClnt::ports_id::IP_0]);   
    #endif

    #ifdef PID_X_SLAM
    update_controller_pid_salm_x->getPorts()[(int)UpdateController::ports_id::OP_0]->connect(ros_updt_ctr->getPorts()[(int)ROSUnit_UpdateControllerClnt::ports_id::IP_0_PID]);
    pid_slam_switch_on_x->getPorts()[(int)SwitchTrigger::ports_id::OP_0]->connect(ros_slam_pid_trigger_x->getPorts()[(int)ROSUnit_SetFloatClnt::ports_id::IP_0]);
    pid_slam_switch_off_x->getPorts()[(int)SwitchTrigger::ports_id::OP_0]->connect(ros_slam_pid_trigger_x->getPorts()[(int)ROSUnit_SetFloatClnt::ports_id::IP_0]);
    #endif

    #ifdef PID_Y_SLAM
    update_controller_pid_salm_y->getPorts()[(int)UpdateController::ports_id::OP_0]->connect(ros_updt_ctr->getPorts()[(int)ROSUnit_UpdateControllerClnt::ports_id::IP_0_PID]);
    pid_slam_switch_on_y->getPorts()[(int)SwitchTrigger::ports_id::OP_0]->connect(ros_slam_pid_trigger_y->getPorts()[(int)ROSUnit_SetFloatClnt::ports_id::IP_0]);
    pid_slam_switch_off_y->getPorts()[(int)SwitchTrigger::ports_id::OP_0]->connect(ros_slam_pid_trigger_y->getPorts()[(int)ROSUnit_SetFloatClnt::ports_id::IP_0]);
    #endif

    #ifdef PID_Z_SLAM
    update_controller_pid_salm_z->getPorts()[(int)UpdateController::ports_id::OP_0]->connect(ros_updt_ctr->getPorts()[(int)ROSUnit_UpdateControllerClnt::ports_id::IP_0_PID]);
    pid_slam_switch_on_z->getPorts()[(int)SwitchTrigger::ports_id::OP_0]->connect(ros_slam_pid_trigger_z->getPorts()[(int)ROSUnit_SetFloatClnt::ports_id::IP_0]);
    pid_slam_switch_off_z->getPorts()[(int)SwitchTrigger::ports_id::OP_0]->connect(ros_slam_pid_trigger_z->getPorts()[(int)ROSUnit_SetFloatClnt::ports_id::IP_0]);
    #endif

    ros_pos_sub->getPorts()[(int)ROSUnit_PointSub::ports_id::OP_0]->connect(initial_pose_waypoint->getPorts()[(int)SetRelativeWaypoint::ports_id::IP_0]);
    rosunit_yaw_provider->getPorts()[(int)ROSUnit_PointSub::ports_id::OP_1]->connect(initial_pose_waypoint->getPorts()[(int)SetRelativeWaypoint::ports_id::IP_1]);
    
    ros_pos_sub->getPorts()[(int)ROSUnit_PointSub::ports_id::OP_0]->connect(takeoff_relative_waypoint->getPorts()[(int)SetRelativeWaypoint::ports_id::IP_0]);
    rosunit_yaw_provider->getPorts()[(int)ROSUnit_PointSub::ports_id::OP_1]->connect(takeoff_relative_waypoint->getPorts()[(int)SetRelativeWaypoint::ports_id::IP_1]);

    ros_pos_sub->getPorts()[(int)ROSUnit_PointSub::ports_id::OP_0]->connect(land_relative_waypoint->getPorts()[(int)SetRelativeWaypoint::ports_id::IP_0]);
    rosunit_yaw_provider->getPorts()[(int)ROSUnit_PointSub::ports_id::OP_1]->connect(land_relative_waypoint->getPorts()[(int)SetRelativeWaypoint::ports_id::IP_1]);

    ros_pos_sub->getPorts()[(int)ROSUnit_PointSub::ports_id::OP_0]->connect(step_relative_waypoint_x->getPorts()[(int)SetRelativeWaypoint::ports_id::IP_0]);
    rosunit_yaw_provider->getPorts()[(int)ROSUnit_PointSub::ports_id::OP_1]->connect(step_relative_waypoint_x->getPorts()[(int)SetRelativeWaypoint::ports_id::IP_1]);
    
    ros_pos_sub->getPorts()[(int)ROSUnit_PointSub::ports_id::OP_0]->connect(set_height_offset->getPorts()[(int)SetHeightOffset::ports_id::IP_0]);

    reset_z->getPorts()[(int)ResetController::ports_id::OP_0]->connect(ros_rst_ctr->getPorts()[(int)ROSUnit_SetInt8Clnt::ports_id::IP_0]);

    arm_motors->getPorts()[(int)Arm::ports_id::OP_0]->connect(ros_arm_srv->getPorts()[(int)ROSUnit_SetBoolClnt::ports_id::IP_0]);
    disarm_motors->getPorts()[(int)Disarm::ports_id::OP_0]->connect(ros_arm_srv->getPorts()[(int)ROSUnit_SetBoolClnt::ports_id::IP_0]);

    ros_flight_command->getPorts()[(int)ROSUnit_EmptySrv::ports_id::OP_0]->connect(user_command->getPorts()[(int)UserCommand::ports_id::IP_0]);

    set_restricted_norm_settings->getPorts()[(int)SetRestNormSettings::ports_id::OP_0]->connect(ros_restnorm_settings->getPorts()[(int)ROSUnit_RestNormSettingsClnt::ports_id::IP_0]);
    land_set_rest_norm_settings->getPorts()[(int)SetRestNormSettings::ports_id::OP_0]->connect(ros_restnorm_settings->getPorts()[(int)ROSUnit_RestNormSettingsClnt::ports_id::IP_0]);
      
    set_height_offset->getPorts()[(int)SetHeightOffset::ports_id::OP_0]->connect(ros_set_height_offset->getPorts()[(int)ROSUnit_SetFloatClnt::ports_id::IP_0]);
    send_set_map_offset_signal->getPorts()[(int)SendBoolSignal::ports_id::OP_0]->connect(rosunit_set_map_frame_offset->getPorts()[(int)ROSUnit_SetBoolClnt::ports_id::IP_0]);
    initial_pose_waypoint->getPorts()[(int)SetRelativeWaypoint::ports_id::OP_0]->connect(ros_set_path_clnt->getPorts()[(int)ROSUnit_SetPosesClnt::ports_id::IP_0]);
    takeoff_relative_waypoint->getPorts()[(int)SetRelativeWaypoint::ports_id::OP_0]->connect(ros_set_path_clnt->getPorts()[(int)ROSUnit_SetPosesClnt::ports_id::IP_0]);
    step_relative_waypoint_x->getPorts()[(int)SetRelativeWaypoint::ports_id::OP_0]->connect(ros_set_path_clnt->getPorts()[(int)ROSUnit_SetPosesClnt::ports_id::IP_0]);

    //absolute_zero_Z_relative_waypoint->connect(ros_set_path_clnt);
    land_relative_waypoint->getPorts()[(int)SetRelativeWaypoint::ports_id::OP_0]->connect(ros_set_path_clnt->getPorts()[(int)ROSUnit_SetPosesClnt::ports_id::IP_0]);

    //*************Setting Flight Elements*************

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
    ((UpdateController*)update_controller_pid_z)->pid_data.ki = 0.24; 
    ((UpdateController*)update_controller_pid_z)->pid_data.kd = 0.239755; 
    ((UpdateController*)update_controller_pid_z)->pid_data.kdd = 0.0;
    ((UpdateController*)update_controller_pid_z)->pid_data.anti_windup = 0;
    ((UpdateController*)update_controller_pid_z)->pid_data.en_pv_derivation = 1;
    ((UpdateController*)update_controller_pid_z)->pid_data.dt = (float)1.0/120.0;
    ((UpdateController*)update_controller_pid_z)->pid_data.id = block_id::PID_Z;

    ((UpdateController*)update_controller_pid_roll)->pid_data.kp = 0.3265;
    ((UpdateController*)update_controller_pid_roll)->pid_data.ki = 0.0;
    ((UpdateController*)update_controller_pid_roll)->pid_data.kd = 0.0565;
    ((UpdateController*)update_controller_pid_roll)->pid_data.kdd = 0.0;
    ((UpdateController*)update_controller_pid_roll)->pid_data.anti_windup = 0;
    ((UpdateController*)update_controller_pid_roll)->pid_data.en_pv_derivation = 1;
    ((UpdateController*)update_controller_pid_roll)->pid_data.dt = 1.f/200.f;
    ((UpdateController*)update_controller_pid_roll)->pid_data.id = block_id::PID_ROLL;

    ((UpdateController*)update_controller_pid_pitch)->pid_data.kp = 0.3569;
    ((UpdateController*)update_controller_pid_pitch)->pid_data.ki = 0.0;
    ((UpdateController*)update_controller_pid_pitch)->pid_data.kd =  0.0617;
    ((UpdateController*)update_controller_pid_pitch)->pid_data.kdd = 0.0;
    ((UpdateController*)update_controller_pid_pitch)->pid_data.anti_windup = 0;
    ((UpdateController*)update_controller_pid_pitch)->pid_data.en_pv_derivation = 1;
    ((UpdateController*)update_controller_pid_pitch)->pid_data.dt = 1.f/200.f;
    ((UpdateController*)update_controller_pid_pitch)->pid_data.id = block_id::PID_PITCH;

    ((UpdateController*)update_controller_pid_yaw)->pid_data.kp = 3.2;
    ((UpdateController*)update_controller_pid_yaw)->pid_data.ki = 0.0;
    ((UpdateController*)update_controller_pid_yaw)->pid_data.kd = 0.0;
    ((UpdateController*)update_controller_pid_yaw)->pid_data.kdd = 0.0;
    ((UpdateController*)update_controller_pid_yaw)->pid_data.anti_windup = 0;
    ((UpdateController*)update_controller_pid_yaw)->pid_data.en_pv_derivation = 1;
    ((UpdateController*)update_controller_pid_yaw)->pid_data.dt = 1.f/120.f;
    ((UpdateController*)update_controller_pid_yaw)->pid_data.id = block_id::PID_YAW;

    ((UpdateController*)update_controller_pid_yaw_rate)->pid_data.kp = 0.32;
    ((UpdateController*)update_controller_pid_yaw_rate)->pid_data.ki = 0.0;
    ((UpdateController*)update_controller_pid_yaw_rate)->pid_data.kd = 0.0;
    ((UpdateController*)update_controller_pid_yaw_rate)->pid_data.kdd = 0.0;
    ((UpdateController*)update_controller_pid_yaw_rate)->pid_data.anti_windup = 0;
    ((UpdateController*)update_controller_pid_yaw_rate)->pid_data.en_pv_derivation = 1;
    ((UpdateController*)update_controller_pid_yaw_rate)->pid_data.dt = 1.f/200.f;
    ((UpdateController*)update_controller_pid_yaw_rate)->pid_data.id = block_id::PID_YAW_RATE;

#ifdef MRFT_X_SLAM
    ((UpdateController*)update_controller_mrft_x)->mrft_data.beta = -0.77;
    ((UpdateController*)update_controller_mrft_x)->mrft_data.relay_amp = 0.15;
    ((UpdateController*)update_controller_mrft_x)->mrft_data.bias = 0.0;
    ((UpdateController*)update_controller_mrft_x)->mrft_data.id = block_id::MRFT_X;
#endif

#ifdef MRFT_Y_SLAM
    ((UpdateController*)update_controller_mrft_y)->mrft_data.beta = -0.77;
    ((UpdateController*)update_controller_mrft_y)->mrft_data.relay_amp = 0.15;
    ((UpdateController*)update_controller_mrft_y)->mrft_data.bias = 0.0;
    ((UpdateController*)update_controller_mrft_y)->mrft_data.id = block_id::MRFT_Y;
#endif

#ifdef MRFT_Z_SLAM
    ((UpdateController*)update_controller_mrft_z)->mrft_data.beta = -0.73;
    ((UpdateController*)update_controller_mrft_z)->mrft_data.relay_amp = 0.12; //0.1;
    ((UpdateController*)update_controller_mrft_z)->mrft_data.bias = 0.0;
    ((UpdateController*)update_controller_mrft_z)->mrft_data.id = block_id::MRFT_Z;
#endif

#ifdef PID_X_SLAM
    ((UpdateController*)update_controller_pid_salm_x)->pid_data.kp = 0.3612;
    ((UpdateController*)update_controller_pid_salm_x)->pid_data.ki = 0.0;
    ((UpdateController*)update_controller_pid_salm_x)->pid_data.kd = 0.2660;
    ((UpdateController*)update_controller_pid_salm_x)->pid_data.kdd = 0.0;
    ((UpdateController*)update_controller_pid_salm_x)->pid_data.anti_windup = 0;
    ((UpdateController*)update_controller_pid_salm_x)->pid_data.en_pv_derivation = 1;
    ((UpdateController*)update_controller_pid_salm_x)->pid_data.dt = (float)1.0/SLAM_FREQ;
    ((UpdateController*)update_controller_pid_salm_x)->pid_data.id = block_id::PID_SLAM_X;
#endif

#ifdef PID_Y_SLAM
    ((UpdateController*)update_controller_pid_salm_y)->pid_data.kp = 0.2350;
    ((UpdateController*)update_controller_pid_salm_y)->pid_data.ki = 0.0;
    ((UpdateController*)update_controller_pid_salm_y)->pid_data.kd = 0.1731;
    ((UpdateController*)update_controller_pid_salm_y)->pid_data.kdd = 0.0;
    ((UpdateController*)update_controller_pid_salm_y)->pid_data.anti_windup = 0;
    ((UpdateController*)update_controller_pid_salm_y)->pid_data.en_pv_derivation = 1;
    ((UpdateController*)update_controller_pid_salm_y)->pid_data.dt = (float)1.0/SLAM_FREQ;
    ((UpdateController*)update_controller_pid_salm_y)->pid_data.id = block_id::PID_SLAM_Y;
#endif

#ifdef PID_Z_SLAM
    ((UpdateController*)update_controller_pid_salm_z)->pid_data.kp = 0.2644; 
    ((UpdateController*)update_controller_pid_salm_z)->pid_data.ki = 0.0; 
    ((UpdateController*)update_controller_pid_salm_z)->pid_data.kd = 0.1614; 
    ((UpdateController*)update_controller_pid_salm_z)->pid_data.kdd = 0.0;
    ((UpdateController*)update_controller_pid_salm_z)->pid_data.anti_windup = 0;
    ((UpdateController*)update_controller_pid_salm_z)->pid_data.en_pv_derivation = 1;
    ((UpdateController*)update_controller_pid_salm_z)->pid_data.dt = (float)1.0/SLAM_FREQ;
    ((UpdateController*)update_controller_pid_salm_z)->pid_data.id = block_id::PID_SLAM_Z;
#endif

    ((ResetController*)reset_z)->target_block = block_id::PID_Z;

    Wait wait_1s;
    wait_1s.wait_time_ms=1000;

    Wait wait_100ms;
    wait_100ms.wait_time_ms=100;

    Wait wait_5s;
    wait_5s.wait_time_ms=5000;

    MissionPipeline mrft_pipeline;

    mrft_pipeline.addElement((MissionElement*)&wait_1s);
    
    mrft_pipeline.addElement((MissionElement*)update_controller_pid_x);
    mrft_pipeline.addElement((MissionElement*)update_controller_pid_y);
    mrft_pipeline.addElement((MissionElement*)update_controller_pid_z);
    mrft_pipeline.addElement((MissionElement*)update_controller_pid_roll);
    mrft_pipeline.addElement((MissionElement*)update_controller_pid_pitch);
    mrft_pipeline.addElement((MissionElement*)update_controller_pid_yaw);
    mrft_pipeline.addElement((MissionElement*)update_controller_pid_yaw_rate);

    #ifdef MRFT_X_SLAM
    mrft_pipeline.addElement((MissionElement*)update_controller_mrft_x);
    #endif

    #ifdef MRFT_Y_SLAM
    mrft_pipeline.addElement((MissionElement*)update_controller_mrft_y);
    #endif

    #ifdef MRFT_Z_SLAM
    mrft_pipeline.addElement((MissionElement*)update_controller_mrft_z);
    #endif

    #ifdef PID_X_SLAM
    mrft_pipeline.addElement((MissionElement*)update_controller_pid_salm_x);
    #endif

    #ifdef PID_Y_SLAM
    mrft_pipeline.addElement((MissionElement*)update_controller_pid_salm_y);
    #endif

    #ifdef PID_Z_SLAM
    mrft_pipeline.addElement((MissionElement*)update_controller_pid_salm_z);
    #endif

    mrft_pipeline.addElement((MissionElement*)set_height_offset); //TODO: (CHECK Desc) Set a constant height command/reference based on the current pos
    mrft_pipeline.addElement((MissionElement*)&wait_1s);
    mrft_pipeline.addElement((MissionElement*)set_restricted_norm_settings);
    mrft_pipeline.addElement((MissionElement*)initial_pose_waypoint);
    mrft_pipeline.addElement((MissionElement*)user_command);
    mrft_pipeline.addElement((MissionElement*)send_set_map_offset_signal);
    mrft_pipeline.addElement((MissionElement*)reset_z); //Reset I-term to zero
    mrft_pipeline.addElement((MissionElement*)&wait_100ms);
    mrft_pipeline.addElement((MissionElement*)arm_motors);
    mrft_pipeline.addElement((MissionElement*)user_command);
//    mrft_pipeline.addElement((MissionElement*)&wait_1s);

    mrft_pipeline.addElement((MissionElement*)reset_z); //Reset I-term to zero
    mrft_pipeline.addElement((MissionElement*)takeoff_relative_waypoint);

    mrft_pipeline.addElement((MissionElement*)user_command);
//    mrft_pipeline.addElement((MissionElement*)&wait_5s);

    #ifdef MRFT_X_SLAM
    mrft_pipeline.addElement((MissionElement*)mrft_switch_on_x);
    #endif

    #ifdef MRFT_Y_SLAM
    mrft_pipeline.addElement((MissionElement*)mrft_switch_on_y);
    #endif

    #ifdef MRFT_Z_SLAM
    mrft_pipeline.addElement((MissionElement*)mrft_switch_on_z);
    #endif

    #ifdef PID_X_SLAM
    mrft_pipeline.addElement((MissionElement*)pid_slam_switch_on_x);
    #endif

    #ifdef PID_Y_SLAM
    mrft_pipeline.addElement((MissionElement*)pid_slam_switch_on_y);
    #endif

    #ifdef PID_Z_SLAM
    mrft_pipeline.addElement((MissionElement*)pid_slam_switch_on_z);
    #endif

    #ifdef STEP_X
    mrft_pipeline.addElement((MissionElement*)user_command); 
    mrft_pipeline.addElement((MissionElement*)step_relative_waypoint_x);
    #endif

    mrft_pipeline.addElement((MissionElement*)user_command);  
//    mrft_pipeline.addElement((MissionElement*)&wait_5s);

    mrft_pipeline.addElement((MissionElement*)initial_pose_waypoint);

    #ifdef MRFT_X_SLAM
    mrft_pipeline.addElement((MissionElement*)mrft_switch_off_x);
    #endif

    #ifdef MRFT_Y_SLAM
    mrft_pipeline.addElement((MissionElement*)mrft_switch_off_y);
    #endif

    #ifdef MRFT_Z_SLAM
    mrft_pipeline.addElement((MissionElement*)mrft_switch_off_z);
    #endif

    #ifdef PID_X_SLAM
    mrft_pipeline.addElement((MissionElement*)pid_slam_switch_off_x);
    #endif

    #ifdef PID_Y_SLAM
    mrft_pipeline.addElement((MissionElement*)pid_slam_switch_off_y);
    #endif

    #ifdef PID_Z_SLAM
    mrft_pipeline.addElement((MissionElement*)pid_slam_switch_off_z);
    #endif

    mrft_pipeline.addElement((MissionElement*)user_command);
//    mrft_pipeline.addElement((MissionElement*)&wait_1s);
    
    mrft_pipeline.addElement((MissionElement*)land_set_rest_norm_settings);   
    mrft_pipeline.addElement((MissionElement*)&wait_100ms);
    mrft_pipeline.addElement((MissionElement*)land_relative_waypoint);


    Logger::getAssignedLogger()->log("FlightScenario main_scenario",LoggerLevel::Info);
    MissionScenario main_scenario;

    main_scenario.AddMissionPipeline(&mrft_pipeline);

    main_scenario.StartScenario();
    Logger::getAssignedLogger()->log("Main Done",LoggerLevel::Info);
    std::cout << "OK \n";
    while(ros::ok){
        ros::spinOnce();
    }
    return 0;
}