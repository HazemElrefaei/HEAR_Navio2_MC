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
#include "HEAR_mission/ConstantUpdate.hpp"
#include "HEAR_mission/CallHeartbeat.hpp"
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
#include "HEAR_ROS_BRIDGE/ROSUnit_EmptyPub.hpp"
#include "HEAR_ROS_BRIDGE/ROSUnit_RestNormSettingsClnt.hpp"
#include "HEAR_ROS_BRIDGE/ROSUnit_ControlOutputSubscriber.hpp"


const float TAKE_OFF_HEIGHT = 1.0;
const float LAND_HEIGHT = -0.01;

const float GRAV = 9.8;
const float HOV_THRUST = 0.4;
const float THRUSTtoGRAV = HOV_THRUST/GRAV;
const float SCALE_GRAV = 1.0/GRAV;

const float YAW_SAT_VALUE = 0.87;

const float SAT_XY_VALUE = 0.87;
const float SAT_Z_VALUE = 22;
const float BIAS_Z_VALUE = 9.8;

const double HEARTBEAT_RATE = 10;

//#define STORE_KF_BIAS
//#define TRAJ_TEST
//#define SLAM_TEST
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
    
    /// HAZEM/////////////////////////
    ROSUnit* ros_scalegravity_client = ROSUnit_Factory_main.CreateROSUnit(ROSUnit_tx_rx_type::Client,
                                                                      ROSUnit_msg_type::ROSUnit_Float,
                                                                      "scale_gravity");
    ROSUnit* ros_thrustTogravity_client = ROSUnit_Factory_main.CreateROSUnit(ROSUnit_tx_rx_type::Client,
                                                                      ROSUnit_msg_type::ROSUnit_Float,
                                                                      "thrust_to_gravity");
    ROSUnit* ros_yaw_sat_client = ROSUnit_Factory_main.CreateROSUnit(ROSUnit_tx_rx_type::Client,
                                                                      ROSUnit_msg_type::ROSUnit_Float,
                                                                      "yaw_saturation");
    ROSUnit* ros_xy_sat_client = ROSUnit_Factory_main.CreateROSUnit(ROSUnit_tx_rx_type::Client,
                                                                      ROSUnit_msg_type::ROSUnit_Float,
                                                                      "xy_saturation");
    ROSUnit* ros_z_sat_client = ROSUnit_Factory_main.CreateROSUnit(ROSUnit_tx_rx_type::Client,
                                                                      ROSUnit_msg_type::ROSUnit_Float,
                                                                      "z_saturation");
    ROSUnit* ros_z_bias_client = ROSUnit_Factory_main.CreateROSUnit(ROSUnit_tx_rx_type::Client,
                                                                      ROSUnit_msg_type::ROSUnit_Float,
                                                                      "z_bias");
    
    ROSUnit* ros_heartbeat = ROSUnit_Factory_main.CreateROSUnit(ROSUnit_tx_rx_type::Publisher,
                                                                      ROSUnit_msg_type::ROSUnit_Empty,
                                                                      "heartbeat");
    /// HAZEM/////////////////////////
    
    ROSUnit* ros_start_traj_clnt = ROSUnit_Factory_main.CreateROSUnit(ROSUnit_tx_rx_type::Client,
                                                            ROSUnit_msg_type::ROSUnit_Bool, 
                                                            "start_trajectory");
    #ifdef SLAM_TEST
    ROSUnit* ros_slam_sw_clnt = ROSUnit_Factory_main.CreateROSUnit(ROSUnit_tx_rx_type::Client,
                                                            ROSUnit_msg_type::ROSUnit_Bool, 
                                                            "slam_switch");
    #endif
    ROSUnit* ros_rec_hover_thrust = ROSUnit_Factory_main.CreateROSUnit(ROSUnit_tx_rx_type::Client,
                                                            ROSUnit_msg_type::ROSUnit_Bool, 
                                                            "record_hover_thrust");
    ROSUnit* ros_rst_kalman = ROSUnit_Factory_main.CreateROSUnit(ROSUnit_tx_rx_type::Client,
                                                            ROSUnit_msg_type::ROSUnit_Empty,
                                                            "reset_kalman");
    ROSUnit* ros_flight_command = ROSUnit_Factory_main.CreateROSUnit(ROSUnit_tx_rx_type::Server,
                                                                    ROSUnit_msg_type::ROSUnit_Empty,
                                                                    "flight_command");//TODO: Change to user_command
	ROSUnit* ros_set_height_offset = ROSUnit_Factory_main.CreateROSUnit(ROSUnit_tx_rx_type::Client,
                                                                    ROSUnit_msg_type::ROSUnit_Empty,
                                                                    "init_height");
    ROSUnit* ros_send_opti_curr_pos = ROSUnit_Factory_main.CreateROSUnit(ROSUnit_tx_rx_type::Client,
                                                                    ROSUnit_msg_type::ROSUnit_Empty,
                                                                    "send_curr_pos_opti");
	ROSUnit* ros_send_slam_curr_pos = ROSUnit_Factory_main.CreateROSUnit(ROSUnit_tx_rx_type::Client,
                                                                    ROSUnit_msg_type::ROSUnit_Empty,
                                                                    "send_curr_pos_slam");
    #ifdef  STORE_KF_BIAS                                                            
    ROSUnit* ros_kf_freeze_bias_clnt = ROSUnit_Factory_main.CreateROSUnit(ROSUnit_tx_rx_type::Client,
                                                            ROSUnit_msg_type::ROSUnit_Bool, 
                                                            "kf_freeze_bias");
    #endif

    ROSUnit* ros_adjust_act_gain = ROSUnit_Factory_main.CreateROSUnit(ROSUnit_tx_rx_type::Client,
                                                            ROSUnit_msg_type::ROSUnit_Bool, 
                                                            "use_adjusted_act_gain");

    ROSUnit* ros_correct_biases = ROSUnit_Factory_main.CreateROSUnit(ROSUnit_tx_rx_type::Client,
                                                            ROSUnit_msg_type::ROSUnit_Bool, 
                                                            "correct_biases");


//     //*****************Flight Elements*************

    MissionElement* update_controller_pid_x = new UpdateController();
    MissionElement* update_controller_pid_y = new UpdateController();
    MissionElement* update_controller_pid_z = new UpdateController();
    MissionElement* update_controller_pid_roll = new UpdateController();
    MissionElement* update_controller_pid_pitch = new UpdateController();
    MissionElement* update_controller_pid_yaw = new UpdateController();
    MissionElement* update_controller_pid_yaw_rate = new UpdateController();

    #ifdef SLAM_TEST
    MissionElement* update_pid_slam_x = new UpdateController();
    MissionElement* update_pid_slam_y = new UpdateController();
    MissionElement* update_pid_slam_z = new UpdateController();
    #endif

    MissionElement* reset_kalman = new ResetController();

    MissionElement* arm_motors = new Arm();
    MissionElement* disarm_motors = new Disarm();
    MissionElement* rec_hover_thrust = new Arm();
    MissionElement* disable_in_filt = new Disarm();
    MissionElement* disable_out_filt = new Disarm();
    MissionElement* start_trajectory = new Arm();
    MissionElement* set_vo_offset = new Arm();

    #ifdef SLAM_TEST
    MissionElement* slam_switch_off = new Disarm();
    MissionElement* slam_switch_on = new Arm();
    #endif

    MissionElement* take_off = new SwitchTrigger(TAKE_OFF_HEIGHT);
    MissionElement* land = new SwitchTrigger(LAND_HEIGHT);

    MissionElement* scale_gravity = new ConstantUpdate(SCALE_GRAV);
    MissionElement* thrustTogravity = new ConstantUpdate(THRUSTtoGRAV);

    MissionElement* yaw_saturation_value = new ConstantUpdate(YAW_SAT_VALUE);

    MissionElement* saturation_xy = new ConstantUpdate(SAT_XY_VALUE);
    MissionElement* saturation_z = new ConstantUpdate(SAT_Z_VALUE);
    MissionElement* bias_z = new ConstantUpdate(BIAS_Z_VALUE);

    MissionElement* heartbeat = new CallHeartbeat(HEARTBEAT_RATE);

    MissionElement* user_command = new UserCommand();

    MissionElement* set_height_offset = new Arm(); 
    MissionElement* send_current_pos_ref_opti = new Arm(); 
    MissionElement* send_current_pos_ref_slam = new Arm(); 

    MissionElement* adjust_act_gain = new Arm();
    MissionElement* original_act_gain = new Disarm();

    MissionElement* correct_biases = new Arm();

    #ifdef STORE_KF_BIAS
    MissionElement* freeze_kf_bias = new Arm(); 
    #endif    
    //******************Connections***************
    update_controller_pid_x->getPorts()[(int)UpdateController::ports_id::OP_0]->connect((ros_updt_ctr)->getPorts()[(int)ROSUnit_UpdateControllerClnt::ports_id::IP_0_PID]);
    update_controller_pid_y->getPorts()[(int)UpdateController::ports_id::OP_0]->connect((ros_updt_ctr)->getPorts()[(int)ROSUnit_UpdateControllerClnt::ports_id::IP_0_PID]);
    update_controller_pid_z->getPorts()[(int)UpdateController::ports_id::OP_0]->connect((ros_updt_ctr)->getPorts()[(int)ROSUnit_UpdateControllerClnt::ports_id::IP_0_PID]);
    update_controller_pid_roll->getPorts()[(int)UpdateController::ports_id::OP_0]->connect((ros_updt_ctr)->getPorts()[(int)ROSUnit_UpdateControllerClnt::ports_id::IP_0_PID]);
    update_controller_pid_pitch->getPorts()[(int)UpdateController::ports_id::OP_0]->connect((ros_updt_ctr)->getPorts()[(int)ROSUnit_UpdateControllerClnt::ports_id::IP_0_PID]);
    update_controller_pid_yaw->getPorts()[(int)UpdateController::ports_id::OP_0]->connect((ros_updt_ctr)->getPorts()[(int)ROSUnit_UpdateControllerClnt::ports_id::IP_0_PID]);
    update_controller_pid_yaw_rate->getPorts()[(int)UpdateController::ports_id::OP_0]->connect((ros_updt_ctr)->getPorts()[(int)ROSUnit_UpdateControllerClnt::ports_id::IP_0_PID]);

    #ifdef SLAM_TEST
    update_pid_slam_x->getPorts()[(int)UpdateController::ports_id::OP_0]->connect((ros_updt_ctr)->getPorts()[(int)ROSUnit_UpdateControllerClnt::ports_id::IP_0_PID]);
    update_pid_slam_y->getPorts()[(int)UpdateController::ports_id::OP_0]->connect((ros_updt_ctr)->getPorts()[(int)ROSUnit_UpdateControllerClnt::ports_id::IP_0_PID]);
    update_pid_slam_z->getPorts()[(int)UpdateController::ports_id::OP_0]->connect((ros_updt_ctr)->getPorts()[(int)ROSUnit_UpdateControllerClnt::ports_id::IP_0_PID]);
    #endif

    reset_kalman->getPorts()[(int)ResetController::ports_id::OP_0]->connect(ros_rst_kalman->getPorts()[(int)ROSUnit_EmptyClnt::ports_id::IP_0]);

    arm_motors->getPorts()[(int)Arm::ports_id::OP_0]->connect(ros_arm_srv->getPorts()[(int)ROSUnit_SetBoolClnt::ports_id::IP_0]);
    disarm_motors->getPorts()[(int)Disarm::ports_id::OP_0]->connect(ros_arm_srv->getPorts()[(int)ROSUnit_SetBoolClnt::ports_id::IP_0]);
    rec_hover_thrust->getPorts()[(int)Arm::ports_id::OP_0]->connect(ros_rec_hover_thrust->getPorts()[(int)ROSUnit_SetBoolClnt::ports_id::IP_0]);
    disable_in_filt->getPorts()[(int)Disarm::ports_id::OP_0]->connect(ros_en_infilt_srv->getPorts()[(int)ROSUnit_SetBoolClnt::ports_id::IP_0]);
    disable_out_filt->getPorts()[(int)Disarm::ports_id::OP_0]->connect(ros_en_outfilt_srv->getPorts()[(int)ROSUnit_SetBoolClnt::ports_id::IP_0]);
    start_trajectory->getPorts()[(int)Arm::ports_id::OP_0]->connect(ros_start_traj_clnt->getPorts()[(int)ROSUnit_SetBoolClnt::ports_id::IP_0]);
    set_vo_offset->getPorts()[(int)Arm::ports_id::OP_0]->connect(set_vo_offset_srv->getPorts()[(int)ROSUnit_SetBoolClnt::ports_id::IP_0]);

    #ifdef SLAM_TEST
    slam_switch_off->getPorts()[(int)Disarm::ports_id::OP_0]->connect(ros_slam_sw_clnt->getPorts()[(int)ROSUnit_SetBoolClnt::ports_id::IP_0]);
    slam_switch_on->getPorts()[(int)Arm::ports_id::OP_0]->connect(ros_slam_sw_clnt->getPorts()[(int)ROSUnit_SetBoolClnt::ports_id::IP_0]);
    #endif

    ros_flight_command->getPorts()[(int)ROSUnit_EmptySrv::ports_id::OP_0]->connect(user_command->getPorts()[(int)UserCommand::ports_id::IP_0]);
    
    set_height_offset->getPorts()[(int)Arm::ports_id::OP_0]->connect(ros_set_height_offset->getPorts()[(int)ROSUnit_EmptyClnt::ports_id::IP_0]);
    send_current_pos_ref_opti->getPorts()[(int)Arm::ports_id::OP_0]->connect(ros_send_opti_curr_pos->getPorts()[(int)ROSUnit_EmptyClnt::ports_id::IP_0]);
    send_current_pos_ref_slam->getPorts()[(int)Arm::ports_id::OP_0]->connect(ros_send_slam_curr_pos->getPorts()[(int)ROSUnit_EmptyClnt::ports_id::IP_0]);
    take_off->getPorts()[(int)SwitchTrigger::ports_id::OP_0]->connect(ros_take_off_client->getPorts()[(int)ROSUnit_SetFloatClnt::ports_id::IP_0]);
    land->getPorts()[(int)SwitchTrigger::ports_id::OP_0]->connect(ros_land_client->getPorts()[(int)ROSUnit_SetFloatClnt::ports_id::IP_0]);

    /// HAZEM/////////////////////////
    scale_gravity->getPorts()[(int)ConstantUpdate::ports_id::OP_0]->connect(ros_scalegravity_client->getPorts()[(int)ROSUnit_SetFloatClnt::ports_id::IP_0]);
    thrustTogravity->getPorts()[(int)ConstantUpdate::ports_id::OP_0]->connect(ros_thrustTogravity_client->getPorts()[(int)ROSUnit_SetFloatClnt::ports_id::IP_0]);
    yaw_saturation_value->getPorts()[(int)ConstantUpdate::ports_id::OP_0]->connect(ros_yaw_sat_client->getPorts()[(int)ROSUnit_SetFloatClnt::ports_id::IP_0]);
    saturation_xy->getPorts()[(int)ConstantUpdate::ports_id::OP_0]->connect(ros_xy_sat_client->getPorts()[(int)ROSUnit_SetFloatClnt::ports_id::IP_0]);
    saturation_z->getPorts()[(int)ConstantUpdate::ports_id::OP_0]->connect(ros_z_sat_client->getPorts()[(int)ROSUnit_SetFloatClnt::ports_id::IP_0]);
    bias_z->getPorts()[(int)ConstantUpdate::ports_id::OP_0]->connect(ros_z_bias_client->getPorts()[(int)ROSUnit_SetFloatClnt::ports_id::IP_0]);

    heartbeat->getPorts()[(int)CallHeartbeat::ports_id::OP_0]->connect(ros_heartbeat->getPorts()[(int)ROSUnit_EmptyPub::ports_id::IP_0]);
    /// HAZEM/////////////////////////

    #ifdef STORE_KF_BIAS
    freeze_kf_bias->getPorts()[(int)Arm::ports_id::OP_0]->connect(ros_kf_freeze_bias_clnt->getPorts()[(int)ROSUnit_EmptyClnt::ports_id::IP_0]);
    #endif
    //absolute_zero_Z_relative_waypoint->connect(ros_set_path_clnt);

    adjust_act_gain->getPorts()[(int)Arm::ports_id::OP_0]->connect(ros_adjust_act_gain->getPorts()[(int)ROSUnit_SetBoolClnt::ports_id::IP_0]);
    original_act_gain->getPorts()[(int)Disarm::ports_id::OP_0]->connect(ros_adjust_act_gain->getPorts()[(int)ROSUnit_SetBoolClnt::ports_id::IP_0]);

    correct_biases->getPorts()[(int)Arm::ports_id::OP_0]->connect(ros_correct_biases->getPorts()[(int)ROSUnit_SetBoolClnt::ports_id::IP_0]);

    //*************Setting Flight Elements*************
    #ifdef SMALL_HEXA
    ((UpdateController*)update_controller_pid_x)->pid_data.kp = 12.67*0.5; //14.53157937; //0.583430204; //0.8786*0.5; //0.51639 * 0.8;
    ((UpdateController*)update_controller_pid_x)->pid_data.ki = 0.0;
    ((UpdateController*)update_controller_pid_x)->pid_data.kd = 5.69*0.5; //4.783743612; //0.211996855; //0.3441*0.5; //0.21192 * 0.8;
    ((UpdateController*)update_controller_pid_x)->pid_data.kdd = 0.0;
    ((UpdateController*)update_controller_pid_x)->pid_data.anti_windup = 0;
    ((UpdateController*)update_controller_pid_x)->pid_data.en_pv_derivation = 1;
    ((UpdateController*)update_controller_pid_x)->pid_data.dt = (float)1.0/120.0;
    ((UpdateController*)update_controller_pid_x)->pid_data.id = block_id::PID_X;

    ((UpdateController*)update_controller_pid_y)->pid_data.kp = 12.3*0.5; // 14.96570691; //0.592289033;// 0.6673*0.75;// 0.6714;// 0.51639 * 0.8;
    ((UpdateController*)update_controller_pid_y)->pid_data.ki = 0.0;
    ((UpdateController*)update_controller_pid_y)->pid_data.kd = 5.52*0.5; //4.926656836; //0.215215824;// 0.2583*0.75; //-0.2440;// * 0.8;
    ((UpdateController*)update_controller_pid_y)->pid_data.kdd = 0.0;
    ((UpdateController*)update_controller_pid_y)->pid_data.anti_windup = 0;
    ((UpdateController*)update_controller_pid_y)->pid_data.en_pv_derivation = 1;
    ((UpdateController*)update_controller_pid_y)->pid_data.dt = (float)1.0/120.0;
    ((UpdateController*)update_controller_pid_y)->pid_data.id = block_id::PID_Y;

    ((UpdateController*)update_controller_pid_z)->pid_data.kp = 24.809892; //32.00005633*0.8; //0.613969957; // 1.2414*0.75; 
    ((UpdateController*)update_controller_pid_z)->pid_data.ki = 0.0; 
    ((UpdateController*)update_controller_pid_z)->pid_data.kd = 7.4476; //8.328107017*0.8; //0.214920534; // 0.3316*0.75; 
    ((UpdateController*)update_controller_pid_z)->pid_data.kdd = 0.0;
    ((UpdateController*)update_controller_pid_z)->pid_data.anti_windup = 0;
    ((UpdateController*)update_controller_pid_z)->pid_data.en_pv_derivation = 1;
    ((UpdateController*)update_controller_pid_z)->pid_data.dt = (float)1.0/120.0;
    ((UpdateController*)update_controller_pid_z)->pid_data.id = block_id::PID_Z;

    ((UpdateController*)update_controller_pid_roll)->pid_data.kp = 0.3336; // 0.423827421; //0.225*0.8; //0.172195; //0.3302; //0.286708; //0.225 * 0.8; 
    ((UpdateController*)update_controller_pid_roll)->pid_data.ki = 0.0;
    ((UpdateController*)update_controller_pid_roll)->pid_data.kd = 0.0538; //0.063652685; //0.04*0.8; //0.042464; //0.0931; //0.056559; //0.04 * 0.8;
    ((UpdateController*)update_controller_pid_roll)->pid_data.kdd = 0.0;
    ((UpdateController*)update_controller_pid_roll)->pid_data.anti_windup = 0;
    ((UpdateController*)update_controller_pid_roll)->pid_data.en_pv_derivation = 1;
    ((UpdateController*)update_controller_pid_roll)->pid_data.dt = 1.f/200.f;
    ((UpdateController*)update_controller_pid_roll)->pid_data.id = block_id::PID_ROLL;

    ((UpdateController*)update_controller_pid_pitch)->pid_data.kp = 0.3561; // 0.434061228; //0.225*0.8;// 0.3360; //0.2811;//0.275252; //0.225 * 0.8; 
    ((UpdateController*)update_controller_pid_pitch)->pid_data.ki = 0.0;
    ((UpdateController*)update_controller_pid_pitch)->pid_data.kd = 0.0574; // 0.065189653; //0.04*0.8;//0.0684; //0.053100; //0.0868;// 0.051266; //0.04 * 0.8; 
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
    
    #ifdef SLAM_TEST
    ((UpdateController*)update_pid_slam_x)->pid_data.kp = 7.35*1.4; //0.311; //0.51639 * 0.8;
    ((UpdateController*)update_pid_slam_x)->pid_data.ki = 0.0;
    ((UpdateController*)update_pid_slam_x)->pid_data.kd = 3.93*1.4; //0.151; //0.21192 * 0.8;
    ((UpdateController*)update_pid_slam_x)->pid_data.kdd = 0.0;
    ((UpdateController*)update_pid_slam_x)->pid_data.anti_windup = 0;
    ((UpdateController*)update_pid_slam_x)->pid_data.en_pv_derivation = 1;
    ((UpdateController*)update_pid_slam_x)->pid_data.dt = (float)1.0/120.0;
    ((UpdateController*)update_pid_slam_x)->pid_data.id = block_id::PID_X;

    ((UpdateController*)update_pid_slam_y)->pid_data.kp = 8.02; //0.332;// 0.6714;// 0.51639 * 0.8;
    ((UpdateController*)update_pid_slam_y)->pid_data.ki = 0.0;
    ((UpdateController*)update_pid_slam_y)->pid_data.kd =  4.29; //0.161; //-0.2440;// * 0.8;
    ((UpdateController*)update_pid_slam_y)->pid_data.kdd = 0.0;
    ((UpdateController*)update_pid_slam_y)->pid_data.anti_windup = 0;
    ((UpdateController*)update_pid_slam_y)->pid_data.en_pv_derivation = 1;
    ((UpdateController*)update_pid_slam_y)->pid_data.dt = (float)1.0/120.0;
    ((UpdateController*)update_pid_slam_y)->pid_data.id = block_id::PID_Y;

    ((UpdateController*)update_pid_slam_z)->pid_data.kp = 12.176; //0.454*0.9; 
    ((UpdateController*)update_pid_slam_z)->pid_data.ki = 0.0; 
    ((UpdateController*)update_pid_slam_z)->pid_data.kd = 5.554; //0.197*0.9; 
    ((UpdateController*)update_pid_slam_z)->pid_data.kdd = 0.0;
    ((UpdateController*)update_pid_slam_z)->pid_data.anti_windup = 0;
    ((UpdateController*)update_pid_slam_z)->pid_data.en_pv_derivation = 1;
    ((UpdateController*)update_pid_slam_z)->pid_data.dt = (float)1.0/120.0;
    ((UpdateController*)update_pid_slam_z)->pid_data.id = block_id::PID_Z;
    #endif
    #endif

    #ifdef BIG_HEXA
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

    Wait wait_1s;
    wait_1s.wait_time_ms=1000;

    Wait wait_100ms;
    wait_100ms.wait_time_ms=100;
    Wait wait_50ms;
    wait_100ms.wait_time_ms=50;

    Wait wait_340ms;
    wait_340ms.wait_time_ms=340;

    Wait wait_5s;
    wait_5s.wait_time_ms=5000;
  
    #ifdef TESTING
    MissionPipeline testing_pipeline;

    testing_pipeline.addElement((MissionElement*)&wait_1s);
    
    testing_pipeline.addElement((MissionElement*)update_controller_pid_x);
    testing_pipeline.addElement((MissionElement*)update_controller_pid_y);
    testing_pipeline.addElement((MissionElement*)update_controller_pid_z);
    testing_pipeline.addElement((MissionElement*)update_controller_pid_roll);
    testing_pipeline.addElement((MissionElement*)update_controller_pid_pitch);
    testing_pipeline.addElement((MissionElement*)update_controller_pid_yaw);
    testing_pipeline.addElement((MissionElement*)update_controller_pid_yaw_rate);

    testing_pipeline.addElement((MissionElement*)scale_gravity);
    testing_pipeline.addElement((MissionElement*)thrustTogravity);
    testing_pipeline.addElement((MissionElement*)yaw_saturation_value);
    testing_pipeline.addElement((MissionElement*)saturation_xy);
    testing_pipeline.addElement((MissionElement*)saturation_z);
    testing_pipeline.addElement((MissionElement*)bias_z);

    testing_pipeline.addElement((MissionElement*)heartbeat);

    testing_pipeline.addElement((MissionElement*)set_height_offset); //TODO: (CHECK Desc) Set a constant height command/reference based on the current pos
    testing_pipeline.addElement((MissionElement*)&wait_1s);
    testing_pipeline.addElement((MissionElement*)user_command);
    testing_pipeline.addElement((MissionElement*)disable_in_filt);
    testing_pipeline.addElement((MissionElement*)disable_out_filt);
    testing_pipeline.addElement((MissionElement*)&wait_100ms);
    testing_pipeline.addElement((MissionElement*)set_vo_offset);
    testing_pipeline.addElement((MissionElement*)&wait_100ms);
    testing_pipeline.addElement((MissionElement*)reset_kalman);
    testing_pipeline.addElement((MissionElement*)arm_motors);

    #ifdef AUTO_TEST
    testing_pipeline.addElement((MissionElement*)&wait_1s);
    #else
    testing_pipeline.addElement((MissionElement*)user_command);
    #endif
    
    // testing_pipeline.addElement((MissionElement*)reset_z); //Reset I-term to zero
    testing_pipeline.addElement((MissionElement*)arm_motors);
    testing_pipeline.addElement((MissionElement*)take_off);

    #ifdef STORE_KF_BIAS
    testing_pipeline.addElement((MissionElement*)user_command);
    testing_pipeline.addElement((MissionElement*)freeze_kf_bias);
    #endif

    #ifdef AUTO_TEST
    testing_pipeline.addElement((MissionElement*)&wait_5s);
    #else
    testing_pipeline.addElement((MissionElement*)user_command);
    #endif

    testing_pipeline.addElement((MissionElement*)rec_hover_thrust);
    testing_pipeline.addElement((MissionElement*)&wait_100ms);
    testing_pipeline.addElement((MissionElement*)adjust_act_gain);
    testing_pipeline.addElement((MissionElement*)&wait_5s);
    testing_pipeline.addElement((MissionElement*)correct_biases);
    testing_pipeline.addElement((MissionElement*)&wait_1s);
    testing_pipeline.addElement((MissionElement*)user_command);

    #ifdef SLAM_TEST
    testing_pipeline.addElement((MissionElement*)update_pid_slam_x);
    testing_pipeline.addElement((MissionElement*)update_pid_slam_y);
    testing_pipeline.addElement((MissionElement*)update_pid_slam_z);
    testing_pipeline.addElement((MissionElement*)send_current_pos_ref_slam);
    testing_pipeline.addElement((MissionElement*)slam_switch_on);
    testing_pipeline.addElement((MissionElement*)user_command);
    #endif

    #ifdef TRAJ_TEST

    testing_pipeline.addElement((MissionElement*)start_trajectory);
    testing_pipeline.addElement((MissionElement*)user_command);
    #endif

    #ifdef SLAM_TEST
    testing_pipeline.addElement((MissionElement*)slam_switch_off);
    testing_pipeline.addElement((MissionElement*)send_current_pos_ref_opti);
    testing_pipeline.addElement((MissionElement*)update_controller_pid_x);
    testing_pipeline.addElement((MissionElement*)update_controller_pid_y);
    testing_pipeline.addElement((MissionElement*)update_controller_pid_z);
    testing_pipeline.addElement((MissionElement*)user_command);
    #endif


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