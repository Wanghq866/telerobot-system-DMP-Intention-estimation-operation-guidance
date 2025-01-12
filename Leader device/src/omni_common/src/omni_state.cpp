#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Wrench.h>
#include <geometry_msgs/WrenchStamped.h>
#include <urdf/model.h>
#include <sensor_msgs/JointState.h>

#include <string.h>
#include <stdio.h>
#include <math.h>
#include <assert.h>
#include <sstream>

#include <HL/hl.h>
#include <HD/hd.h>
#include <HDU/hduError.h>
#include <HDU/hduVector.h>
#include <HDU/hduMatrix.h>
#include <HDU/hduQuaternion.h>
#include <bullet/LinearMath/btMatrix3x3.h>
#include <pthread.h> //Linux下的线程库

#include "omni_msgs/OmniButtonEvent.h"
#include "omni_msgs/OmniFeedback.h"
#include "omni_msgs/OmniState.h"
#include "omni_msgs/SingleValWithTypes.h"
#include "omni_msgs/TactileCoefficient.h"
#include "omni_msgs/Creditability.h"
#include "omni_msgs/ForceforTouch.h"

float prev_time;
int calibrationStyle;

// 定义了一个表示omni状态的结构体
struct OmniState {
  hduVector3Dd position;  //3x1 vector of position
  hduVector3Dd velocity;  //3x1 vector of velocity
  hduVector3Dd inp_vel1;  //3x1 history of velocity used for filtering velocity estimate，用于滤波速度估计的3x1速度历史
  hduVector3Dd inp_vel2;
  hduVector3Dd inp_vel3;
  hduVector3Dd out_vel1;
  hduVector3Dd out_vel2;
  hduVector3Dd out_vel3;
  hduVector3Dd pos_hist1; //3x1 history of position used for 2nd order backward difference estimate of velocity，用于速度的二阶后向差分估计的3x1位置历史
  hduVector3Dd pos_hist2;
  hduQuaternion rot;
  hduVector3Dd joints;    // 前三个关节的？
  hduVector3Dd force;     //3 element double vector force[0], force[1], force[2]
  hduVector3Dd lock_pos;
  hduVector3Dd alpha;
  hduVector3Dd new_pos;
  float thetas[7];
  int buttons[2];
  int buttons_prev[2];
  bool lock;              // 是否使omni具备力功能
  bool close_gripper;
  double units_ratio;
  float maxcredit;
  hduVector3Dd fa_save1;
  hduVector3Dd fa_save2;
  int M;
};

// /phantom/pose话题，geometry_msgs/PoseStamped消息类型，表述带时间戳的末端位姿。本节点发布
// /phantom/joint_states话题，sensor_msgs/JointState消息类型，表述带时间戳的关节位置。本节点发布
// /phantom/force_feedback话题，omni_msgs/OmniFeedback消息类型，表述无时间戳的末端力与位置！！。本节点订阅 ！！！
// /phantom/button话题，omni_msgs/OmniButtonEvent消息类型，表述无时间戳的灰色和白色按钮的0和1（注意有两个按钮）。本节点发布
// /phantom/state话题，omni_msgs/OmniState消息类型，表述带时间戳的locked,close_gripper,pose(位置与姿态，即geometry_msgs/PoseStamped消息类型)，current, velocity。本节点发布。再详究！

// 定义了一个PhantomROS的类，作用：1.发布上述的话题，2.接收力话题
class PhantomROS {

public:
  ros::NodeHandle n;
  ros::Publisher state_publisher;
  ros::Publisher pose_publisher;
  ros::Publisher button_publisher;
  ros::Publisher joint_publisher;
  ros::Publisher touchforce_publisher;
  ros::Subscriber haptic_sub;
  ros::Subscriber alpha_sub;
  ros::Subscriber credit_sub;
  ros::Subscriber newtraj_sub;
  ros::Subscriber forceenable_sub;
  std::string omni_name, ref_frame, units;

  OmniState *state; //定义一个结构体

  void init(OmniState *s) {
    ros::param::param(std::string("~omni_name"), omni_name, std::string("phantom"));
    ros::param::param(std::string("~reference_frame"), ref_frame, std::string("/map"));
    ros::param::param(std::string("~units"), units, std::string("mm"));

    // Publish button state on NAME/button
    // 发布一个omni_msgs::OmniButtonEvent类型的/phantom/button话题
    std::ostringstream stream1; //输入输出操作
    stream1 << omni_name << "/button"; // stream1 == /phantom/button
    std::string button_topic = std::string(stream1.str()); // button_topic == /phantom/button
    button_publisher = n.advertise<omni_msgs::OmniButtonEvent>(button_topic.c_str(), 100); 

    // Publish on NAME/state
    // 发布一个omni_msgs::OmniState类型的/phantom/state话题
    std::ostringstream stream2;
    stream2 << omni_name << "/state"; // stream2 == /phantom/state
    std::string state_topic_name = std::string(stream2.str()); // state_topic_name == /phantom/state
    state_publisher = n.advertise<omni_msgs::OmniState>(state_topic_name.c_str(), 1); 

    // Subscribe to NAME/force_feedback
    // 订阅/phantom/force_feedback话题，回调函数是hantomROS类下的force_callback
    std::ostringstream stream3;
    stream3 << omni_name << "/force_feedback";
    std::string force_feedback_topic = std::string(stream3.str());
    haptic_sub = n.subscribe(force_feedback_topic.c_str(), 1, &PhantomROS::force_callback, this);

    // Publish on NAME/pose
    // 发布一个geometry_msgs::PoseStamped类型的/phantom/pose话题
    std::ostringstream stream4;
    stream4 << omni_name << "/pose";
    std::string pose_topic_name = std::string(stream4.str());
    pose_publisher = n.advertise<geometry_msgs::PoseStamped>(pose_topic_name.c_str(), 1); 

    // Publish on NAME/joint_states
    // 发布一个sensor_msgs::JointState类型的/phantom/joint_states话题
    std::ostringstream stream5;
    //stream5 << omni_name << "/joint_states";
    stream5 << "/joint_states_omni"; //joint_states_omni
    std::string joint_topic_name = std::string(stream5.str());
    joint_publisher = n.advertise<sensor_msgs::JointState>(joint_topic_name.c_str(), 1);

    // 创建订阅，接收信任度
    credit_sub = n.subscribe<omni_msgs::Creditability>("/creditability", 1, &PhantomROS::credit_callback, this);

    // 创建发布，发布发送给Touch的三维力
    touchforce_publisher = n.advertise<omni_msgs::ForceforTouch>("/force_in_touch", 1);

    // 创建订阅，接收发布的new轨迹（概率次大的轨迹）
    newtraj_sub = n.subscribe<omni_msgs::OmniFeedback>("/willtraj", 1, &PhantomROS::newtraj_callback, this);

    // 创建订阅，接收力使能标志位，标志位发布文件：forceenable.py
    forceenable_sub = n.subscribe<omni_msgs::SingleValWithTypes>("/force_enable",10, &PhantomROS::forceenable_callback, this);

    // 初始化state
    state = s;
    state->buttons[0] = 0;
    state->buttons[1] = 0;
    state->buttons_prev[0] = 0;
    state->buttons_prev[1] = 0;
    hduVector3Dd zeros(0, 0, 0);
    state->velocity = zeros;
    state->inp_vel1 = zeros;  //3x1 history of velocity
    state->inp_vel2 = zeros;  //3x1 history of velocity
    state->inp_vel3 = zeros;  //3x1 history of velocity
    state->out_vel1 = zeros;  //3x1 history of velocity
    state->out_vel2 = zeros;  //3x1 history of velocity
    state->out_vel3 = zeros;  //3x1 history of velocity
    state->pos_hist1 = zeros; //3x1 history of position
    state->pos_hist2 = zeros; //3x1 history of position
    state->lock = true; // true, false // 是否使omni具备力功能
    state->close_gripper = false;
    // {-28.613,-3.495,43.337} 主从操作中Touch的初始位姿
    state->lock_pos = {-16.56,0,-12.51}; //zeros; zero是主手的笛卡尔空间的『0，0，0』位置
    state->new_pos = {-16.56,0,-12.51}; 
    state->alpha = {1.0,1.0,1.0};
    state->maxcredit = .0;
    state->M = 0;
    state->fa_save1 = {.0,.0,.0};
    state->fa_save2 = {.0,.0,.0};
    if (!units.compare("mm"))
      state->units_ratio = 1.0;
    else if (!units.compare("cm"))
      state->units_ratio = 10.0;
    else if (!units.compare("dm"))
      state->units_ratio = 100.0;
    else if (!units.compare("m"))
      state->units_ratio = 1000.0;
    else
    {
      state->units_ratio = 1.0;
      ROS_WARN("Unknown units [%s] unsing [mm]", units.c_str());
      units = "mm";
    }
    ROS_INFO("PHaNTOM position given in [%s], ratio [%.1f]", units.c_str(), state->units_ratio);
  }
  
  // 回调函数
  // omnifeed表示传输进来的/phantom/force_feedback话题
  void force_callback(const omni_msgs::OmniFeedback::ConstPtr& omnifeed) {
    //Some people might not like this extra damping, but it helps to stabilize the overall force 
    //feedback. It isn't like we are getting direct impedance matching from the omni anyway
    // 有些人可能不喜欢这种额外的阻尼，但它有助于稳定整体力反馈。无论如何，我们并不是从omni获得直接阻抗匹配
    state->force[0] = omnifeed->force.x - 0.001 * state->velocity[0];
    state->force[1] = omnifeed->force.y - 0.001 * state->velocity[1];
    state->force[2] = omnifeed->force.z - 0.001 * state->velocity[2];

    state->lock_pos[0] = omnifeed->position.x;
    state->lock_pos[1] = omnifeed->position.y;
    state->lock_pos[2] = omnifeed->position.z;
  }

  void credit_callback(const omni_msgs::Creditability::ConstPtr& msg) {
    state->maxcredit = msg->creditabilityMax;
    state->M = msg->M;
    //ROS_INFO("maxcredit: %.3f", state->maxcredit);
    //ROS_INFO("M: %d", state->M);
  }

  void newtraj_callback(const omni_msgs::OmniFeedback::ConstPtr& msg) {
    state->new_pos[0] = msg->position.x;
    state->new_pos[1] = msg->position.y;
    state->new_pos[2] = msg->position.z;
  }

  void forceenable_callback(const omni_msgs::SingleValWithTypes::ConstPtr& msg) {
    state->lock = msg->boolval;
  }

  // 在main函数中被调用，将真实主手的状态发布出去
  void publish_omni_state() {
    // 发布omni_msgs::OmniState类型的消息
    // Build the state msg
    omni_msgs::OmniState state_msg;
    state_msg.header.stamp = ros::Time::now();
    // Locked
    state_msg.locked = state->lock;
    state_msg.close_gripper = state->close_gripper;
    // Position
    state_msg.pose.position.x = state->position[0];
    state_msg.pose.position.y = state->position[1];
    state_msg.pose.position.z = state->position[2];
    // Orientation
    state_msg.pose.orientation.x = state->rot.v()[0];
    state_msg.pose.orientation.y = state->rot.v()[1];
    state_msg.pose.orientation.z = state->rot.v()[2];
    state_msg.pose.orientation.w = state->rot.s();
    // Velocity
    state_msg.velocity.x = state->velocity[0];
    state_msg.velocity.y = state->velocity[1];
    state_msg.velocity.z = state->velocity[2];
    // TODO: Append Current to the state msg
    state_publisher.publish(state_msg);

    // 发布sensor_msgs::JointState类型的消息
    // Publish the JointState msg
    sensor_msgs::JointState joint_state;
    joint_state.header.stamp = ros::Time::now();
    joint_state.name.resize(6);
    joint_state.position.resize(6);
    joint_state.name[0] = "waist";
    joint_state.position[0] = -state->thetas[1];
    joint_state.name[1] = "shoulder";
    joint_state.position[1] = state->thetas[2];
    joint_state.name[2] = "elbow";
    joint_state.position[2] = state->thetas[3];
    joint_state.name[3] = "yaw";
    joint_state.position[3] = -state->thetas[4] + M_PI;
    joint_state.name[4] = "pitch";
    joint_state.position[4] = -state->thetas[5] - 3*M_PI/4;
    joint_state.name[5] = "roll";
    joint_state.position[5] = state->thetas[6] - M_PI;
    joint_publisher.publish(joint_state);

    // 发布geometry_msgs::PoseStamped类型的消息
    // Build the pose msg
    geometry_msgs::PoseStamped pose_msg;
    pose_msg.header = state_msg.header;
    pose_msg.header.frame_id = ref_frame;
    pose_msg.pose = state_msg.pose;
    pose_msg.pose.position.x /= 1000.0; // 将state_msg话题中的position(unit:mm)转换为米(m)单位
    pose_msg.pose.position.y /= 1000.0;
    pose_msg.pose.position.z /= 1000.0;
    pose_publisher.publish(pose_msg);

    omni_msgs::OmniButtonEvent button_event;
    button_event.grey_button = state->buttons[0];
    button_event.white_button = state->buttons[1];
    button_publisher.publish(button_event);

    omni_msgs::ForceforTouch forceshow;
    forceshow.header.stamp = ros::Time::now();
    forceshow.force.x = state->force[0];
    forceshow.force.y = state->force[1];
    forceshow.force.z = state->force[2];
    touchforce_publisher.publish(forceshow);
  }
};

// 1.从主手获取状态,通过指针的方式将主手真实的状态写入到结构体OmniState定义的state; 2. 向设备发送笛卡尔力
HDCallbackCode HDCALLBACK omni_state_callback(void *pUserData)
{
  OmniState *omni_state = static_cast<OmniState *>(pUserData);
  if (hdCheckCalibration() == HD_CALIBRATION_NEEDS_UPDATE) {
    ROS_DEBUG("Updating calibration...");
      hdUpdateCalibration(calibrationStyle);
    }
  hdBeginFrame(hdGetCurrentDevice()); // 更新设备状态以获取当前/最新信息
  //--------------------------------------------------------------------------------------------------------------------------------------------------------------
  // Get transform and angles, 获取变换和角度
  hduMatrix transform; // hduMatrix 4*4矩阵
  hdGetDoublev(HD_CURRENT_TRANSFORM, transform); // HD_CURRENT_TRANSFORM, 获取设备末端执行器的列主变换，获取值给transform，运动学？
  //std::cout<<transform<<std::endl;
  hdGetDoublev(HD_CURRENT_JOINT_ANGLES, omni_state->joints); //HD_CURRENT_JOINT_ANGLES, 获取关节角，获取值给omni_state->joints
  hduVector3Dd gimbal_angles;
  hdGetDoublev(HD_CURRENT_GIMBAL_ANGLES, gimbal_angles); //HD_CURRENT_GIMBAL_ANGLES, 获取设备万向节的角度，获取值给gimbal_angles
  //ROS_INFO("%.3f,%.3f,%.3f,%.3f,%.3f,%.3f",omni_state->joints[0],omni_state->joints[1],omni_state->joints[2],gimbal_angles[0],gimbal_angles[1],gimbal_angles[2]);
  // Position
  omni_state->position = hduVector3Dd(transform[3][2], transform[3][0], transform[3][1]);
  omni_state->position /= omni_state->units_ratio;
  // Orientation (quaternion)
  hduMatrix rotation(transform);
  rotation.getRotationMatrix(rotation);
  hduMatrix rotation_offset( 0.0, -1.0, 0.0, 0.0,
                             1.0,  0.0, 0.0, 0.0,
                             0.0,  0.0, 1.0, 0.0,
                             0.0,  0.0, 0.0, 1.0);
  rotation_offset.getRotationMatrix(rotation_offset);
  omni_state->rot = hduQuaternion(rotation_offset * rotation);
  //----------------------------------------------------------------------------------------------------------------------------------------------------------
  // Velocity estimation
  hduVector3Dd vel_buff(0, 0, 0);
  vel_buff = (omni_state->position * 3 - 4 * omni_state->pos_hist1 + omni_state->pos_hist2) / 0.002;  //(units)/s, 2nd order backward dif
  omni_state->velocity = (.2196 * (vel_buff + omni_state->inp_vel3) + .6588 * (omni_state->inp_vel1 + omni_state->inp_vel2)) / 1000.0
      - (-2.7488 * omni_state->out_vel1 + 2.5282 * omni_state->out_vel2 - 0.7776 * omni_state->out_vel3);  //cutoff freq of 20 Hz
  
  omni_state->pos_hist2 = omni_state->pos_hist1;
  omni_state->pos_hist1 = omni_state->position;
  omni_state->inp_vel3 = omni_state->inp_vel2;
  omni_state->inp_vel2 = omni_state->inp_vel1;
  omni_state->inp_vel1 = vel_buff;
  omni_state->out_vel3 = omni_state->out_vel2;
  omni_state->out_vel2 = omni_state->out_vel1;
  omni_state->out_vel1 = omni_state->velocity;

  // Set forces if locked
  // (omni_state->lock_pos - omni_state->position), unit: mm
  if (omni_state->lock == true) { //  && omni_state->buttons[0] == 0
    hduVector3Dd param_spring(0.5,0.1,0.1); // 将x方向的弹簧刚度设置为0.5，将y,z方向的弹簧刚度设置为0.1, (0.5,0.1,0.1), (0.1,0.05,0.05)
    hduVector3Dd fa(.0,.0,.0);
    hduVector3Dd C(.1,.1,.1);
    
    int M = omni_state->M;
    float CREDIT = omni_state->maxcredit;
    float Lamba_f = 0.98;
    float Lamba_t = 0.83;
    hduVector3Dd fa_save1(.0,.0,.0);
    hduVector3Dd fa_save2(.0,.0,.0);
    
    if (M == 0){
      fa = omni_state->units_ratio * param_spring * (omni_state->lock_pos - omni_state->position) - 0.005 * omni_state->velocity;
      }
    else if (M == 1){
      fa = omni_state->units_ratio * param_spring * (omni_state->lock_pos - omni_state->position) - 0.005 * omni_state->velocity;
      omni_state->fa_save1 = fa;
      //std::cout<<"M=1"<<fa[0]<<std::endl;
      }
    else if(M == 2){
      //fa = (fa_save1-C)*exp(-25*(lamda-CREDIT))+C;
      fa[0] = omni_state->units_ratio * param_spring[0] * (omni_state->lock_pos[0] - omni_state->position[0]) - 0.005 * omni_state->velocity[0];
      fa[1] = omni_state->fa_save1[1]*exp(-7.3*(Lamba_t-CREDIT));
      fa[2] = omni_state->fa_save1[2]*exp(-7.3*(Lamba_t-CREDIT));
      //std::cout<<"M=2"<<omni_state->lock_pos<<std::endl;
      // fa[1] = (omni_state->fa_save1[1])*(CREDIT-Lamba_t)/(Lamba_t-0.5);//(fa_save1[1])*exp(-13*(Lamba_t-CREDIT));
      // fa[2] = (omni_state->fa_save1[2])*(CREDIT-Lamba_t)/(Lamba_t-0.5);//(fa_save1[2])*exp(-13*(Lamba_t-CREDIT));
      //omni_state->fa_save2[1] = fa[1]; omni_state->fa_save2[2] = fa[2];
      
    }
    else if(M == 3){
      fa[0] = omni_state->units_ratio * param_spring[0] * (omni_state->lock_pos[0] - omni_state->position[0]) - 0.005 * omni_state->velocity[0];
      // fa[1] = (CREDIT-0.5)/(Lamba_f-0.5)*(omni_state->units_ratio * param_spring[1] * (omni_state->lock_pos[1] - omni_state->position[1]) - 0.005 * omni_state->velocity[1]);
      // fa[2] = (CREDIT-0.5)/(Lamba_f-0.5)*(omni_state->units_ratio * param_spring[2] * (omni_state->lock_pos[2] - omni_state->position[2]) - 0.005 * omni_state->velocity[2]);
      // fa[1] = (omni_state->fa_save2[1])*(Lamba_t-CREDIT)/(Lamba_t-0.5);//(fa_save2[1])*exp(-13*(CREDIT-Lamba_t));
      // fa[2] = (omni_state->fa_save2[2])*(Lamba_t-CREDIT)/(Lamba_t-0.5);//(fa_save2[2])*exp(-13*(CREDIT-Lamba_t));
      fa[1] = omni_state->fa_save1[1]*exp(-7.3*(CREDIT-0.2));
      fa[2] = omni_state->fa_save1[2]*exp(-7.3*(Lamba_t-0.2));
      //std::cout<<"M=3"<<fa<<std::endl;
    }

    omni_state->force = fa;
    
    /*
    // 改参数，调节吸引力！！！
    hduVector3Dd param_spring(0.5,0.1,0.1); // 将x方向的弹簧刚度设置为0.2，将y,z方向的弹簧刚度设置为0.015
    omni_state->force = param_spring * omni_state->units_ratio * omni_state->alpha * (omni_state->lock_pos - omni_state->position) - 
                        0.005 * omni_state->velocity;
                        */
  }
  hduVector3Dd feedback;
  // Notice that we are changing Y <---> Z and inverting the Z-force_feedback
  feedback[0] = omni_state->force[1]; //feedback[0],omni的x方向,正值向右，负值向左
  feedback[1] = omni_state->force[2]; //feedback[1],omni的z方向,正值向上，负值向下
  feedback[2] = omni_state->force[0]; //feedback[2],omni的-y方向，正值向后，负值向前
  //ROS_INFO("force:[%.3f,%.3f.%.3f]",feedback[0],feedback[1],feedback[2]);
  // hdSetDoublev, 设置设备参数; HD_CURRENT_FORCE, 向设备发送力 ！！！
  hdSetDoublev(HD_CURRENT_FORCE, feedback);

  // hduVector3Dd Tor_gravity;
  // Tor_gravity[0] = 0.0;
  // Tor_gravity[1] = 0.164158*cos(omni_state->joints[1]*57.3)+0.117294*(omni_state->joints[1]*57.3-0.5*3.1415926);
  // Tor_gravity[2] = 0.09405*sin(omni_state->joints[2]*57.3);
  // std::cout<<Tor_gravity<<std::endl;
  // hdSetDoublev(HD_CURRENT_JOINT_TORQUE, Tor_gravity);

  //Get buttons
  int nButtons = 0;
  hdGetIntegerv(HD_CURRENT_BUTTONS, &nButtons); // HD_CURRENT_BUTTONS 获取按钮状态，获取的值给nButtons
  omni_state->buttons[0] = (nButtons & HD_DEVICE_BUTTON_1) ? 1 : 0; // HD_DEVICE_BUTTON_1 按钮1的状态
  omni_state->buttons[1] = (nButtons & HD_DEVICE_BUTTON_2) ? 1 : 0;

  hdEndFrame(hdGetCurrentDevice()); //结束触觉帧。使力和其他状态写入设备。hdBeginFrame（）和hdEndFrame（）应始终在同一调度程序内成对出现。

  HDErrorInfo error;
  if (HD_DEVICE_ERROR(error = hdGetError())) {
    hduPrintError(stderr, &error, "Error during main scheduler callback");
    if (hduIsSchedulerError(&error))
      return HD_CALLBACK_DONE;
  }

  float t[7] = { 0., omni_state->joints[0], omni_state->joints[1], omni_state->joints[2] - omni_state->joints[1], 
                     gimbal_angles[0], gimbal_angles[1], gimbal_angles[2] };
  for (int i = 0; i < 7; i++){
    omni_state->thetas[i] = t[i];
  }
  return HD_CALLBACK_CONTINUE; // 返回值：HD_CALLBACK_DONE or HD_CALLBACK_CONTINUE，根据返回值决定回调函数是否继续运行
}

// Automatic Calibration of Phantom Device - No character inputs，phantom设备的自动校准-无字符输入
void HHD_Auto_Calibration() {
  int supportedCalibrationStyles;
  HDErrorInfo error;

  hdGetIntegerv(HD_CALIBRATION_STYLE, &supportedCalibrationStyles);
  if (supportedCalibrationStyles & HD_CALIBRATION_ENCODER_RESET) {
    calibrationStyle = HD_CALIBRATION_ENCODER_RESET;
    ROS_INFO("HD_CALIBRATION_ENCODER_RESET..");
  }
  if (supportedCalibrationStyles & HD_CALIBRATION_INKWELL) {
    calibrationStyle = HD_CALIBRATION_INKWELL;
    ROS_INFO("HD_CALIBRATION_INKWELL..");
  }
  if (supportedCalibrationStyles & HD_CALIBRATION_AUTO) {
    calibrationStyle = HD_CALIBRATION_AUTO;
    ROS_INFO("HD_CALIBRATION_AUTO..");
  }
  if (calibrationStyle == HD_CALIBRATION_ENCODER_RESET) {
    do {
      hdUpdateCalibration(calibrationStyle);
      ROS_INFO("Calibrating.. (put stylus in well)");
      if (HD_DEVICE_ERROR(error = hdGetError())) {
        hduPrintError(stderr, &error, "Reset encoders reset failed.");
        break;
      }
    } while (hdCheckCalibration() != HD_CALIBRATION_OK);
    ROS_INFO("Calibration complete.");
  }
  while(hdCheckCalibration() != HD_CALIBRATION_OK) {
    usleep(1e6);
    if (hdCheckCalibration() == HD_CALIBRATION_NEEDS_MANUAL_INPUT)
      ROS_INFO("Please place the device into the inkwell for calibration");
    else if (hdCheckCalibration() == HD_CALIBRATION_NEEDS_UPDATE) {
      ROS_INFO("Calibration updated successfully");
      hdUpdateCalibration(calibrationStyle);
    }
    else
      ROS_FATAL("Unknown calibration status");
  }
}

void *ros_publish(void *ptr) {
  PhantomROS *omni_ros = (PhantomROS *) ptr;
  int publish_rate;
  ros::param::param(std::string("~publish_rate"), publish_rate, 1000);
  ROS_INFO("Publishing PHaNTOM state at [%d] Hz", publish_rate);
  ros::Rate loop_rate(publish_rate);
  ros::AsyncSpinner spinner(2);
  spinner.start();

  while (ros::ok()) {
    omni_ros->publish_omni_state(); // 将预定义的消息发布出去
    loop_rate.sleep();
  }
  return NULL;
}

int main(int argc, char** argv) {
  //************* Init ROS *************//
  ros::init(argc, argv, "omni_haptic_node");
  OmniState state; // 初始化一个结构体
  PhantomROS omni_ros; // // 初始化前边定义的 PhantomROS Class


  //************* Init Phantom *************//
  HDErrorInfo error;
  HHD hHD;
  // HDstring target_dev = HD_DEFAULT_DEVICE;
  // string dev_string;
  ros::NodeHandle nh("~");
  std::string device_name;
  nh.getParam("device_name", device_name);
  HDstring target_dev = device_name.c_str();
  hHD = hdInitDevice(target_dev);
  if (HD_DEVICE_ERROR(error = hdGetError())) {
    ROS_ERROR("Failed to initialize haptic device");
    return -1;
  }
  ROS_INFO("Found %s.", hdGetString(HD_DEVICE_MODEL_TYPE));
  hdEnable(HD_FORCE_OUTPUT); //启用设备的力输出，所有电机都已打开
  hdStartScheduler(); // 启动计划程序。调度器管理要在伺服循环线程内执行的回调。
  if (HD_DEVICE_ERROR(error = hdGetError())) {
    ROS_ERROR("Failed to start the scheduler");
    return -1;
  }
  HHD_Auto_Calibration();

  omni_ros.init(&state); // 初始化状态与消息的订阅与发布器state
  // 将主手真实的状态写入到state，通过指针; 向设备发送笛卡尔力；HD_MAX_SCHEDULER_PRIORITY，最高优先级
  hdScheduleAsynchronous(omni_state_callback, &state, HD_MAX_SCHEDULER_PRIORITY);


  //********** Loop and publish *************//
  // 创建一个 POSIX 线程
  // pthread_create (myThread, attr, start_routine, arg)
  // myThread 指向线程标识符指针；attr 属性对象被用来设置线程属性。可以指定线程属性对象也可以使用默认值 NULL 
  // start_routine 线程运行函数起始地址，一旦线程被创建就会执行； arg	运行函数的参数，必须通过把引用作为指针强制转换为 void 类型进行传递。没有传递参数则使用 NULL
  pthread_t publish_thread;
  // 创建了一个线程，通过调用ros_publish将omni_ros表示的真实状态发布出去（调用omni_ros->publish_omni_state()）
  pthread_create(&publish_thread, NULL, ros_publish, (void*) &omni_ros);  
  pthread_join(publish_thread, NULL); // 主线程需要等待子线程执行完成之后再结束（有用）

  ROS_INFO("Ending Session....");
  hdStopScheduler();
  hdDisableDevice(hHD);

  return 0;
}
