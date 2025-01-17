// UAV controller tuning for roll pitch and z channels
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

//#define MRFT_ROLL_CHAN
#define MRFT_PITCH_CHAN
//#define MRFT_Z_CHAN

#define SMALL_HEXA

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
#ifdef  MRFT_ROLL_CHAN
    ROSUnit* ros_mrft_trigger_roll = ROSUnit_Factory_main.CreateROSUnit(ROSUnit_tx_rx_type::Client,
                                                                      ROSUnit_msg_type::ROSUnit_Float,
                                                                      "mrft_switch_roll");
#endif
    
#ifdef  MRFT_PITCH_CHAN
    ROSUnit* ros_mrft_trigger_pitch = ROSUnit_Factory_main.CreateROSUnit(ROSUnit_tx_rx_type::Client,
                                                                      ROSUnit_msg_type::ROSUnit_Float,
                                                                      "mrft_switch_pitch");
#endif

#ifdef  MRFT_Z_CHAN                                                          
    ROSUnit* ros_mrft_trigger_z = ROSUnit_Factory_main.CreateROSUnit(ROSUnit_tx_rx_type::Client,
                                                                      ROSUnit_msg_type::ROSUnit_Float,
                                                                      "mrft_switch_z");
#endif
    ROSUnit* ros_trig_pid_z = ROSUnit_Factory_main.CreateROSUnit(ROSUnit_tx_rx_type::Client,
                                                            ROSUnit_msg_type::ROSUnit_Bool, 
                                                            "pid_z_trig");
    ROSUnit* ros_pos_sub = ROSUnit_Factory_main.CreateROSUnit(ROSUnit_tx_rx_type::Subscriber,
                                                            ROSUnit_msg_type::ROSUnit_Point,
                                                            "/pos_horizon");
    ROSUnit* ros_rst_ctr = ROSUnit_Factory_main.CreateROSUnit(ROSUnit_tx_rx_type::Client,
                                                            ROSUnit_msg_type::ROSUnit_Empty,
                                                            "reset_z");
    ROSUnit* ros_flight_command = ROSUnit_Factory_main.CreateROSUnit(ROSUnit_tx_rx_type::Server,
                                                                    ROSUnit_msg_type::ROSUnit_Empty,
                                                                    "flight_command");//TODO: Change to user_command
	ROSUnit* ros_set_path_clnt = ROSUnit_Factory_main.CreateROSUnit(ROSUnit_tx_rx_type::Client,
                                                                    ROSUnit_msg_type::ROSUnit_Poses,
                                                                    "uav_control/set_path");
    ROSUnit* ros_set_height_offset = ROSUnit_Factory_main.CreateROSUnit(ROSUnit_tx_rx_type::Client,
                                                                    ROSUnit_msg_type::ROSUnit_Float,
                                                                    "set_height_offset");                                                                    
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

    #ifdef  MRFT_ROLL_CHAN
    MissionElement* update_controller_mrft_roll = new UpdateController();
    MissionElement* mrft_switch_on_roll = new SwitchTrigger(1);
    MissionElement* mrft_switch_off_roll = new SwitchTrigger(0);
    #endif

    #ifdef  MRFT_PITCH_CHAN
    MissionElement* update_controller_mrft_pitch = new UpdateController();
    MissionElement* mrft_switch_on_pitch = new SwitchTrigger(1);
    MissionElement* mrft_switch_off_pitch = new SwitchTrigger(0);
    #endif

    #ifdef  MRFT_Z_CHAN
    MissionElement* update_controller_mrft_z = new UpdateController();
    MissionElement* mrft_switch_on_z=new SwitchTrigger(1);
    MissionElement* mrft_switch_off_z=new SwitchTrigger(-1);
    #endif

    MissionElement* reset_z = new ResetController();

    MissionElement* arm_motors = new Arm();
    MissionElement* disarm_motors = new Disarm();
    MissionElement* enable_pid_z = new Arm();
    MissionElement* disable_pid_z = new Disarm();

    MissionElement* user_command = new UserCommand();

    // MissionElement* state_monitor = new StateMonitor();

    MissionElement* set_restricted_norm_settings = new SetRestNormSettings(true, false, 0.5); 

    MissionElement* land_set_rest_norm_settings = new SetRestNormSettings(true, false, 0.15);

    MissionElement* set_height_offset = new SetHeightOffset();
    MissionElement* initial_pose_waypoint = new SetRelativeWaypoint(0., 0., 0., 0.); //TODO: SetRelativeWaypoint needs substantial refactoring

   
    MissionElement* takeoff_relative_waypoint = new SetRelativeWaypoint(0., 0., 1.0, 0.);

    MissionElement* land_relative_waypoint = new SetRelativeWaypoint(0., 0., -2., 0.);

    //******************Connections***************
    update_controller_pid_x->getPorts()[(int)UpdateController::ports_id::OP_0]->connect(ros_updt_ctr->getPorts()[(int)ROSUnit_UpdateControllerClnt::ports_id::IP_0_PID]);
    update_controller_pid_y->getPorts()[(int)UpdateController::ports_id::OP_0]->connect(ros_updt_ctr->getPorts()[(int)ROSUnit_UpdateControllerClnt::ports_id::IP_0_PID]);
    update_controller_pid_z->getPorts()[(int)UpdateController::ports_id::OP_0]->connect(ros_updt_ctr->getPorts()[(int)ROSUnit_UpdateControllerClnt::ports_id::IP_0_PID]);
    update_controller_pid_roll->getPorts()[(int)UpdateController::ports_id::OP_0]->connect(ros_updt_ctr->getPorts()[(int)ROSUnit_UpdateControllerClnt::ports_id::IP_0_PID]);
    update_controller_pid_pitch->getPorts()[(int)UpdateController::ports_id::OP_0]->connect(ros_updt_ctr->getPorts()[(int)ROSUnit_UpdateControllerClnt::ports_id::IP_0_PID]);
    update_controller_pid_yaw->getPorts()[(int)UpdateController::ports_id::OP_0]->connect(ros_updt_ctr->getPorts()[(int)ROSUnit_UpdateControllerClnt::ports_id::IP_0_PID]);
    update_controller_pid_yaw_rate->getPorts()[(int)UpdateController::ports_id::OP_0]->connect(ros_updt_ctr->getPorts()[(int)ROSUnit_UpdateControllerClnt::ports_id::IP_0_PID]);

    #ifdef  MRFT_ROLL_CHAN
    update_controller_mrft_roll->getPorts()[(int)UpdateController::ports_id::OP_0]->connect(ros_updt_ctr->getPorts()[(int)ROSUnit_UpdateControllerClnt::ports_id::IP_1_MRFT]);
    mrft_switch_on_roll->getPorts()[(int)SwitchTrigger::ports_id::OP_0]->connect(ros_mrft_trigger_roll->getPorts()[(int)ROSUnit_SetFloatClnt::ports_id::IP_0]);
    mrft_switch_off_roll->getPorts()[(int)SwitchTrigger::ports_id::OP_0]->connect(ros_mrft_trigger_roll->getPorts()[(int)ROSUnit_SetFloatClnt::ports_id::IP_0]);   
    #endif

    #ifdef  MRFT_PITCH_CHAN
    update_controller_mrft_pitch->getPorts()[(int)UpdateController::ports_id::OP_0]->connect(ros_updt_ctr->getPorts()[(int)ROSUnit_UpdateControllerClnt::ports_id::IP_1_MRFT]);
    mrft_switch_on_pitch->getPorts()[(int)SwitchTrigger::ports_id::OP_0]->connect(ros_mrft_trigger_pitch->getPorts()[(int)ROSUnit_SetFloatClnt::ports_id::IP_0]);
    mrft_switch_off_pitch->getPorts()[(int)SwitchTrigger::ports_id::OP_0]->connect(ros_mrft_trigger_pitch->getPorts()[(int)ROSUnit_SetFloatClnt::ports_id::IP_0]);   
    #endif

    #ifdef  MRFT_Z_CHAN
    update_controller_mrft_z->getPorts()[(int)UpdateController::ports_id::OP_0]->connect(ros_updt_ctr->getPorts()[(int)ROSUnit_UpdateControllerClnt::ports_id::IP_1_MRFT]);
    mrft_switch_on_z->getPorts()[(int)SwitchTrigger::ports_id::OP_0]->connect(ros_mrft_trigger_z->getPorts()[(int)ROSUnit_SetFloatClnt::ports_id::IP_0]);
    mrft_switch_off_z->getPorts()[(int)SwitchTrigger::ports_id::OP_0]->connect(ros_mrft_trigger_z->getPorts()[(int)ROSUnit_SetFloatClnt::ports_id::IP_0]);   
    #endif

    ros_pos_sub->getPorts()[(int)ROSUnit_PointSub::ports_id::OP_0]->connect(initial_pose_waypoint->getPorts()[(int)SetRelativeWaypoint::ports_id::IP_0]);
    rosunit_yaw_provider->getPorts()[(int)ROSUnit_PointSub::ports_id::OP_1]->connect(initial_pose_waypoint->getPorts()[(int)SetRelativeWaypoint::ports_id::IP_1]);
    
    ros_pos_sub->getPorts()[(int)ROSUnit_PointSub::ports_id::OP_0]->connect(takeoff_relative_waypoint->getPorts()[(int)SetRelativeWaypoint::ports_id::IP_0]);
    rosunit_yaw_provider->getPorts()[(int)ROSUnit_PointSub::ports_id::OP_1]->connect(takeoff_relative_waypoint->getPorts()[(int)SetRelativeWaypoint::ports_id::IP_1]);

    ros_pos_sub->getPorts()[(int)ROSUnit_PointSub::ports_id::OP_0]->connect(land_relative_waypoint->getPorts()[(int)SetRelativeWaypoint::ports_id::IP_0]);
    rosunit_yaw_provider->getPorts()[(int)ROSUnit_PointSub::ports_id::OP_1]->connect(land_relative_waypoint->getPorts()[(int)SetRelativeWaypoint::ports_id::IP_1]);

    ros_pos_sub->getPorts()[(int)ROSUnit_PointSub::ports_id::OP_0]->connect(set_height_offset->getPorts()[(int)SetHeightOffset::ports_id::IP_0]);

    reset_z->getPorts()[(int)ResetController::ports_id::OP_0]->connect(ros_rst_ctr->getPorts()[(int)ROSUnit_EmptyClnt::ports_id::IP_0]);

    arm_motors->getPorts()[(int)Arm::ports_id::OP_0]->connect(ros_arm_srv->getPorts()[(int)ROSUnit_SetBoolClnt::ports_id::IP_0]);
    disarm_motors->getPorts()[(int)Disarm::ports_id::OP_0]->connect(ros_arm_srv->getPorts()[(int)ROSUnit_SetBoolClnt::ports_id::IP_0]);
    enable_pid_z->getPorts()[(int)Arm::ports_id::OP_0]->connect(ros_trig_pid_z->getPorts()[(int)ROSUnit_SetBoolClnt::ports_id::IP_0]);
    disable_pid_z->getPorts()[(int)Disarm::ports_id::OP_0]->connect(ros_trig_pid_z->getPorts()[(int)ROSUnit_SetBoolClnt::ports_id::IP_0]);

    ros_flight_command->getPorts()[(int)ROSUnit_EmptySrv::ports_id::OP_0]->connect(user_command->getPorts()[(int)UserCommand::ports_id::IP_0]);

    set_restricted_norm_settings->getPorts()[(int)SetRestNormSettings::ports_id::OP_0]->connect(ros_restnorm_settings->getPorts()[(int)ROSUnit_RestNormSettingsClnt::ports_id::IP_0]);
    land_set_rest_norm_settings->getPorts()[(int)SetRestNormSettings::ports_id::OP_0]->connect(ros_restnorm_settings->getPorts()[(int)ROSUnit_RestNormSettingsClnt::ports_id::IP_0]);
      
    set_height_offset->getPorts()[(int)SetHeightOffset::ports_id::OP_0]->connect(ros_set_height_offset->getPorts()[(int)ROSUnit_SetFloatClnt::ports_id::IP_0]);
    initial_pose_waypoint->getPorts()[(int)SetRelativeWaypoint::ports_id::OP_0]->connect(ros_set_path_clnt->getPorts()[(int)ROSUnit_SetPosesClnt::ports_id::IP_0]);
    takeoff_relative_waypoint->getPorts()[(int)SetRelativeWaypoint::ports_id::OP_0]->connect(ros_set_path_clnt->getPorts()[(int)ROSUnit_SetPosesClnt::ports_id::IP_0]);


    //absolute_zero_Z_relative_waypoint->connect(ros_set_path_clnt);
    land_relative_waypoint->getPorts()[(int)SetRelativeWaypoint::ports_id::OP_0]->connect(ros_set_path_clnt->getPorts()[(int)ROSUnit_SetPosesClnt::ports_id::IP_0]);

    //*************Setting Flight Elements*************
    #ifdef SMALL_HEXA
    ((UpdateController*)update_controller_pid_x)->pid_data.kp = 0.51639 * 0.35; // 0.696435;
    ((UpdateController*)update_controller_pid_x)->pid_data.ki = 0.0;
    ((UpdateController*)update_controller_pid_x)->pid_data.kd = -0.21192 * 0.35; // 0.375166;
    ((UpdateController*)update_controller_pid_x)->pid_data.kdd = 0.0;
    ((UpdateController*)update_controller_pid_x)->pid_data.anti_windup = 0;
    ((UpdateController*)update_controller_pid_x)->pid_data.en_pv_derivation = 1;
    ((UpdateController*)update_controller_pid_x)->pid_data.dt = (float)1.0/120.0;
    ((UpdateController*)update_controller_pid_x)->pid_data.id = block_id::PID_X;

    ((UpdateController*)update_controller_pid_y)->pid_data.kp = 0.51639 * 0.35; // 0.568331;
    ((UpdateController*)update_controller_pid_y)->pid_data.ki = 0.0;
    ((UpdateController*)update_controller_pid_y)->pid_data.kd = -0.21192 * 0.35; // 0.306157;
    ((UpdateController*)update_controller_pid_y)->pid_data.kdd = 0.0;
    ((UpdateController*)update_controller_pid_y)->pid_data.anti_windup = 0;
    ((UpdateController*)update_controller_pid_y)->pid_data.en_pv_derivation = 1;
    ((UpdateController*)update_controller_pid_y)->pid_data.dt = (float)1.0/120.0;
    ((UpdateController*)update_controller_pid_y)->pid_data.id = block_id::PID_Y;

    ((UpdateController*)update_controller_pid_z)->pid_data.kp = 0.730936*0.8; 
    ((UpdateController*)update_controller_pid_z)->pid_data.ki = 0.0; 
    ((UpdateController*)update_controller_pid_z)->pid_data.kd = -0.190225*0.8; 
    ((UpdateController*)update_controller_pid_z)->pid_data.kdd = 0.0;
    ((UpdateController*)update_controller_pid_z)->pid_data.anti_windup = 0;
    ((UpdateController*)update_controller_pid_z)->pid_data.en_pv_derivation = 1;
    ((UpdateController*)update_controller_pid_z)->pid_data.dt = (float)1.0/120.0;
    ((UpdateController*)update_controller_pid_z)->pid_data.id = block_id::PID_Z;

    ((UpdateController*)update_controller_pid_roll)->pid_data.kp = 0.225 * 0.8; // 0.2121; //0.172195; //0.3302; //0.286708;
    ((UpdateController*)update_controller_pid_roll)->pid_data.ki = 0.0;
    ((UpdateController*)update_controller_pid_roll)->pid_data.kd = -0.04 * 0.8; // 0.0489; //0.042464; //0.0931; //0.056559;
    ((UpdateController*)update_controller_pid_roll)->pid_data.kdd = 0.0;
    ((UpdateController*)update_controller_pid_roll)->pid_data.anti_windup = 0;
    ((UpdateController*)update_controller_pid_roll)->pid_data.en_pv_derivation = 1;
    ((UpdateController*)update_controller_pid_roll)->pid_data.dt = 1.f/200.f;
    ((UpdateController*)update_controller_pid_roll)->pid_data.id = block_id::PID_ROLL;

    ((UpdateController*)update_controller_pid_pitch)->pid_data.kp = 0.225 * 0.8;  // 0.2506;// 0.3360; //0.2811;//0.275252;
    ((UpdateController*)update_controller_pid_pitch)->pid_data.ki = 0.0;
    ((UpdateController*)update_controller_pid_pitch)->pid_data.kd =  -0.04 * 0.8;  //0.0578;//0.0684; //0.053100; //0.0868;// 0.051266;
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
#else
    ((UpdateController*)update_controller_pid_x)->pid_data.kp = 0.6534*0.35;
    ((UpdateController*)update_controller_pid_x)->pid_data.ki = 0.0;
    ((UpdateController*)update_controller_pid_x)->pid_data.kd = -0.3831*0.35;
    ((UpdateController*)update_controller_pid_x)->pid_data.kdd = 0.0;
    ((UpdateController*)update_controller_pid_x)->pid_data.anti_windup = 0;
    ((UpdateController*)update_controller_pid_x)->pid_data.en_pv_derivation = 1;
    ((UpdateController*)update_controller_pid_x)->pid_data.dt = (float)1.0/120.0;
    ((UpdateController*)update_controller_pid_x)->pid_data.id = block_id::PID_X;

    ((UpdateController*)update_controller_pid_y)->pid_data.kp = 0.7176*0.35;
    ((UpdateController*)update_controller_pid_y)->pid_data.ki = 0.0;
    ((UpdateController*)update_controller_pid_y)->pid_data.kd =  -0.4208*0.35;
    ((UpdateController*)update_controller_pid_y)->pid_data.kdd = 0.0;
    ((UpdateController*)update_controller_pid_y)->pid_data.anti_windup = 0;
    ((UpdateController*)update_controller_pid_y)->pid_data.en_pv_derivation = 1;
    ((UpdateController*)update_controller_pid_y)->pid_data.dt = (float)1.0/120.0;
    ((UpdateController*)update_controller_pid_y)->pid_data.id = block_id::PID_Y;

    ((UpdateController*)update_controller_pid_z)->pid_data.kp = 0.785493; 
    ((UpdateController*)update_controller_pid_z)->pid_data.ki = 0.098; 
    ((UpdateController*)update_controller_pid_z)->pid_data.kd = -0.239755; 
    ((UpdateController*)update_controller_pid_z)->pid_data.kdd = 0.0;
    ((UpdateController*)update_controller_pid_z)->pid_data.anti_windup = 0;
    ((UpdateController*)update_controller_pid_z)->pid_data.en_pv_derivation = 1;
    ((UpdateController*)update_controller_pid_z)->pid_data.dt = (float)1.0/120.0;
    ((UpdateController*)update_controller_pid_z)->pid_data.id = block_id::PID_Z;

    ((UpdateController*)update_controller_pid_roll)->pid_data.kp = 0.3227;
    ((UpdateController*)update_controller_pid_roll)->pid_data.ki = 0.0;
    ((UpdateController*)update_controller_pid_roll)->pid_data.kd = -0.0558;
    ((UpdateController*)update_controller_pid_roll)->pid_data.kdd = 0.0;
    ((UpdateController*)update_controller_pid_roll)->pid_data.anti_windup = 0;
    ((UpdateController*)update_controller_pid_roll)->pid_data.en_pv_derivation = 1;
    ((UpdateController*)update_controller_pid_roll)->pid_data.dt = 1.f/200.f;
    ((UpdateController*)update_controller_pid_roll)->pid_data.id = block_id::PID_ROLL;

    ((UpdateController*)update_controller_pid_pitch)->pid_data.kp = 0.2981;
    ((UpdateController*)update_controller_pid_pitch)->pid_data.ki = 0.0;
    ((UpdateController*)update_controller_pid_pitch)->pid_data.kd =  -0.0515;
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
#endif

#ifdef MRFT_ROLL_CHAN
    ((UpdateController*)update_controller_mrft_roll)->mrft_data.beta = -0.73;
    ((UpdateController*)update_controller_mrft_roll)->mrft_data.relay_amp = 0.04;
    ((UpdateController*)update_controller_mrft_roll)->mrft_data.bias = 0.0;
    ((UpdateController*)update_controller_mrft_roll)->mrft_data.id = block_id::MRFT_ROLL;
#endif

#ifdef MRFT_PITCH_CHAN
    ((UpdateController*)update_controller_mrft_pitch)->mrft_data.beta = -0.73;
    ((UpdateController*)update_controller_mrft_pitch)->mrft_data.relay_amp = 0.04;
    ((UpdateController*)update_controller_mrft_pitch)->mrft_data.bias = 0.0;
    ((UpdateController*)update_controller_mrft_pitch)->mrft_data.id = block_id::MRFT_PITCH;
#endif

#ifdef MRFT_Z_CHAN
    ((UpdateController*)update_controller_mrft_z)->mrft_data.beta = -0.73;
    ((UpdateController*)update_controller_mrft_z)->mrft_data.relay_amp = 0.1; //0.1;
    ((UpdateController*)update_controller_mrft_z)->mrft_data.bias = 0.0;
    ((UpdateController*)update_controller_mrft_z)->mrft_data.id = block_id::MRFT_Z;
#endif

    ((ResetController*)reset_z)->target_block = block_id::PID_Z;

    Wait wait_1s;
    wait_1s.wait_time_ms=1000;

    Wait wait_100ms;
    wait_100ms.wait_time_ms=100;

    Wait wait_340ms;
    wait_340ms.wait_time_ms=400;

    MissionPipeline mrft_pipeline;

    mrft_pipeline.addElement((MissionElement*)&wait_1s);
    
    mrft_pipeline.addElement((MissionElement*)update_controller_pid_x);
    mrft_pipeline.addElement((MissionElement*)&wait_100ms);
    mrft_pipeline.addElement((MissionElement*)update_controller_pid_y);
    mrft_pipeline.addElement((MissionElement*)&wait_100ms);
    mrft_pipeline.addElement((MissionElement*)update_controller_pid_z);
    mrft_pipeline.addElement((MissionElement*)&wait_100ms);
    mrft_pipeline.addElement((MissionElement*)update_controller_pid_roll);
    mrft_pipeline.addElement((MissionElement*)&wait_100ms);
    mrft_pipeline.addElement((MissionElement*)update_controller_pid_pitch);
    mrft_pipeline.addElement((MissionElement*)&wait_100ms);
    mrft_pipeline.addElement((MissionElement*)update_controller_pid_yaw);
    mrft_pipeline.addElement((MissionElement*)&wait_1s);
    mrft_pipeline.addElement((MissionElement*)update_controller_pid_yaw_rate);
    mrft_pipeline.addElement((MissionElement*)&wait_100ms);

    #ifdef MRFT_ROLL_CHAN
    mrft_pipeline.addElement((MissionElement*)update_controller_mrft_roll);
    #endif

    #ifdef MRFT_PITCH_CHAN
    mrft_pipeline.addElement((MissionElement*)update_controller_mrft_pitch);
    #endif

    #ifdef MRFT_Z_CHAN
    mrft_pipeline.addElement((MissionElement*)update_controller_mrft_z);
    #endif

    mrft_pipeline.addElement((MissionElement*)set_height_offset); //TODO: (CHECK Desc) Set a constant height command/reference based on the current pos
    mrft_pipeline.addElement((MissionElement*)&wait_1s);
    mrft_pipeline.addElement((MissionElement*)set_restricted_norm_settings);
    mrft_pipeline.addElement((MissionElement*)initial_pose_waypoint);
    mrft_pipeline.addElement((MissionElement*)user_command);
    mrft_pipeline.addElement((MissionElement*)reset_z); //Reset I-term to zero
    mrft_pipeline.addElement((MissionElement*)&wait_100ms);
    mrft_pipeline.addElement((MissionElement*)arm_motors);
    mrft_pipeline.addElement((MissionElement*)disable_pid_z);

    mrft_pipeline.addElement((MissionElement*)user_command);
    mrft_pipeline.addElement((MissionElement*)reset_z); //Reset I-term to zero
    mrft_pipeline.addElement((MissionElement*)arm_motors);
    mrft_pipeline.addElement((MissionElement*)takeoff_relative_waypoint);
    mrft_pipeline.addElement((MissionElement*)&wait_1s);
    mrft_pipeline.addElement((MissionElement*)&wait_340ms);
    mrft_pipeline.addElement((MissionElement*)enable_pid_z);

    mrft_pipeline.addElement((MissionElement*)user_command);

    #ifdef MRFT_ROLL_CHAN
    mrft_pipeline.addElement((MissionElement*)mrft_switch_on_roll);
    #endif

    #ifdef MRFT_PITCH_CHAN
    mrft_pipeline.addElement((MissionElement*)mrft_switch_on_pitch);
    #endif

    #ifdef MRFT_Z_CHAN
    mrft_pipeline.addElement((MissionElement*)mrft_switch_on_z);
    #endif

    mrft_pipeline.addElement((MissionElement*)user_command);  
    mrft_pipeline.addElement((MissionElement*)initial_pose_waypoint);

    #ifdef MRFT_ROLL_CHAN
    mrft_pipeline.addElement((MissionElement*)mrft_switch_off_roll);
    #endif

    #ifdef MRFT_PITCH_CHAN
    mrft_pipeline.addElement((MissionElement*)mrft_switch_off_pitch);
    #endif

    #ifdef MRFT_Z_CHAN
    mrft_pipeline.addElement((MissionElement*)mrft_switch_off_z);
    #endif

    mrft_pipeline.addElement((MissionElement*)user_command);
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