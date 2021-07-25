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

#define MRFT_X_SLAM
//#define MRFT_Y_SLAM
//#define MRFT_Z_SLAM

//#define PID_X_SLAM
//#define PID_Y_SLAM
//#define PID_Z_SLAM

//#define STEP_X

const float SLAM_FREQ = 30.0;
const float TAKE_OFF_HEIGHT = 1.0;
const float LAND_HEIGHT = -0.3;


int main(int argc, char** argv) {
    Logger::assignLogger(new StdLogger());

    //****************ROS Units********************
    ros::init(argc, argv, "flight_scenario");
    ros::NodeHandle nh;

    ROSUnit* ros_updt_ctr = new ROSUnit_UpdateControllerClnt(nh);
    
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

    ROSUnit* ros_trig_pid_z = ROSUnit_Factory_main.CreateROSUnit(ROSUnit_tx_rx_type::Client,
                                                            ROSUnit_msg_type::ROSUnit_Bool, 
                                                            "pid_z_trig");
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

    #ifdef  MRFT_X_SLAM
    MissionElement* update_controller_mrft_x = new UpdateController();
    MissionElement* mrft_switch_on_x=new SwitchTrigger(1);
    MissionElement* mrft_switch_off_x=new SwitchTrigger(0);
    #endif

    #ifdef  MRFT_Y_SLAM
    MissionElement* update_controller_mrft_y = new UpdateController();
    MissionElement* mrft_switch_on_y=new SwitchTrigger(1);
    MissionElement* mrft_switch_off_y=new SwitchTrigger(0);
    #endif

    #ifdef  MRFT_Z_SLAM
    MissionElement* update_controller_mrft_z = new UpdateController();
    MissionElement* mrft_switch_on_z=new SwitchTrigger(1);
    MissionElement* mrft_switch_off_z=new SwitchTrigger(0);
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

    MissionElement* arm_motors = new Arm();
    MissionElement* disarm_motors = new Disarm();
    MissionElement* enable_pid_z = new Arm();
    MissionElement* disable_pid_z = new Disarm();
    MissionElement* disable_in_filt = new Disarm();
    MissionElement* disable_out_filt = new Disarm();
    MissionElement* set_vo_offset = new Arm();

    MissionElement* take_off = new SwitchTrigger(TAKE_OFF_HEIGHT);
    MissionElement* land = new SwitchTrigger(LAND_HEIGHT);

    MissionElement* user_command = new UserCommand();

    MissionElement* set_height_offset = new Arm();

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

    arm_motors->getPorts()[(int)Arm::ports_id::OP_0]->connect(ros_arm_srv->getPorts()[(int)ROSUnit_SetBoolClnt::ports_id::IP_0]);
    disarm_motors->getPorts()[(int)Disarm::ports_id::OP_0]->connect(ros_arm_srv->getPorts()[(int)ROSUnit_SetBoolClnt::ports_id::IP_0]);
    enable_pid_z->getPorts()[(int)Arm::ports_id::OP_0]->connect(ros_trig_pid_z->getPorts()[(int)ROSUnit_SetBoolClnt::ports_id::IP_0]);
    disable_pid_z->getPorts()[(int)Disarm::ports_id::OP_0]->connect(ros_trig_pid_z->getPorts()[(int)ROSUnit_SetBoolClnt::ports_id::IP_0]);
    disable_in_filt->getPorts()[(int)Disarm::ports_id::OP_0]->connect(ros_en_infilt_srv->getPorts()[(int)ROSUnit_SetBoolClnt::ports_id::IP_0]);
    disable_out_filt->getPorts()[(int)Disarm::ports_id::OP_0]->connect(ros_en_outfilt_srv->getPorts()[(int)ROSUnit_SetBoolClnt::ports_id::IP_0]);
    set_vo_offset->getPorts()[(int)Arm::ports_id::OP_0]->connect(set_vo_offset_srv->getPorts()[(int)ROSUnit_SetBoolClnt::ports_id::IP_0]);

    ros_flight_command->getPorts()[(int)ROSUnit_EmptySrv::ports_id::OP_0]->connect(user_command->getPorts()[(int)UserCommand::ports_id::IP_0]);

      
    set_height_offset->getPorts()[(int)Arm::ports_id::OP_0]->connect(ros_set_height_offset->getPorts()[(int)ROSUnit_EmptyClnt::ports_id::IP_0]);
    take_off->getPorts()[(int)SwitchTrigger::ports_id::OP_0]->connect(ros_take_off_client->getPorts()[(int)ROSUnit_SetFloatClnt::ports_id::IP_0]);
    land->getPorts()[(int)SwitchTrigger::ports_id::OP_0]->connect(ros_land_client->getPorts()[(int)ROSUnit_SetFloatClnt::ports_id::IP_0]);

    //*************Setting Flight Elements*************

    ((UpdateController*)update_controller_pid_x)->pid_data.kp = 0.696435*0.35; //0.51639 * 0.8;
    ((UpdateController*)update_controller_pid_x)->pid_data.ki = 0.0;
    ((UpdateController*)update_controller_pid_x)->pid_data.kd = -0.375166*0.35; //0.21192 * 0.8;
    ((UpdateController*)update_controller_pid_x)->pid_data.kdd = 0.0;
    ((UpdateController*)update_controller_pid_x)->pid_data.anti_windup = 0;
    ((UpdateController*)update_controller_pid_x)->pid_data.en_pv_derivation = 1;
    ((UpdateController*)update_controller_pid_x)->pid_data.dt = (float)1.0/120.0;
    ((UpdateController*)update_controller_pid_x)->pid_data.id = block_id::PID_X;

    ((UpdateController*)update_controller_pid_y)->pid_data.kp = 0.568331*0.5;// 0.51639 * 0.8;
    ((UpdateController*)update_controller_pid_y)->pid_data.ki = 0.0;
    ((UpdateController*)update_controller_pid_y)->pid_data.kd =  -0.306157*0.5;// * 0.8;
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

#ifdef MRFT_X_SLAM
    ((UpdateController*)update_controller_mrft_x)->mrft_data.beta = -0.735;
    ((UpdateController*)update_controller_mrft_x)->mrft_data.relay_amp = 0.07;
    ((UpdateController*)update_controller_mrft_x)->mrft_data.bias = 0.0;
    ((UpdateController*)update_controller_mrft_x)->mrft_data.num_of_peak_conf_samples = 5;
    ((UpdateController*)update_controller_mrft_x)->mrft_data.no_switch_delay_in_ms = 100;
    ((UpdateController*)update_controller_mrft_x)->mrft_data.id = block_id::MRFT_X;
#endif

#ifdef MRFT_Y_SLAM
    ((UpdateController*)update_controller_mrft_y)->mrft_data.beta = -0.735;
    ((UpdateController*)update_controller_mrft_y)->mrft_data.relay_amp = 0.07;
    ((UpdateController*)update_controller_mrft_y)->mrft_data.bias = 0.0;
    ((UpdateController*)update_controller_mrft_y)->mrft_data.no_switch_delay_in_ms = 100;
    ((UpdateController*)update_controller_mrft_y)->mrft_data.num_of_peak_conf_samples = 2;
    ((UpdateController*)update_controller_mrft_y)->mrft_data.id = block_id::MRFT_Y;
#endif

#ifdef MRFT_Z_SLAM
    ((UpdateController*)update_controller_mrft_z)->mrft_data.beta = -0.73;
    ((UpdateController*)update_controller_mrft_z)->mrft_data.relay_amp = 0.15; //0.1;
    ((UpdateController*)update_controller_mrft_z)->mrft_data.bias = 0.0;
    ((UpdateController*)update_controller_mrft_z)->mrft_data.id = block_id::MRFT_Z;
#endif

#ifdef PID_X_SLAM
    ((UpdateController*)update_controller_pid_salm_x)->pid_data.kp = 0.28896;
    ((UpdateController*)update_controller_pid_salm_x)->pid_data.ki = 0.0;
    ((UpdateController*)update_controller_pid_salm_x)->pid_data.kd = 0.2128;
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

    Wait wait_1s;
    wait_1s.wait_time_ms=1000;

    Wait wait_100ms;
    wait_100ms.wait_time_ms=100;

    Wait wait_340ms;
    wait_340ms.wait_time_ms=400;

    Wait wait_5s;
    wait_5s.wait_time_ms=5000;

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
    mrft_pipeline.addElement((MissionElement*)&wait_100ms);
    mrft_pipeline.addElement((MissionElement*)update_controller_pid_yaw_rate);
    mrft_pipeline.addElement((MissionElement*)&wait_100ms);

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
    mrft_pipeline.addElement((MissionElement*)user_command);
    mrft_pipeline.addElement((MissionElement*)disable_in_filt);
    mrft_pipeline.addElement((MissionElement*)&wait_100ms);
    mrft_pipeline.addElement((MissionElement*)set_vo_offset);
    mrft_pipeline.addElement((MissionElement*)arm_motors);
 
    mrft_pipeline.addElement((MissionElement*)user_command);
    mrft_pipeline.addElement((MissionElement*)arm_motors);
    mrft_pipeline.addElement((MissionElement*)take_off);

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
    
    mrft_pipeline.addElement((MissionElement*)land);   


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