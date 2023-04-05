#include <xarm_msgs/RobotMsg.h>
#include <xarm_msgs/ClearErr.h>
#include <xarm_msgs/SetAxis.h>
#include <xarm_msgs/SetInt16.h>
#include <xarm_msgs/Move.h>

#include <ros/ros.h>

std::string id;

xarm_msgs::RobotMsg robotMsg;

#define CLEAR_ERR_NUM 11
#define MOTION_CTRL_NUM 12
#define SET_MODE_NUM 13
#define SET_STATE_NUM 14
#define GO_HOME_NUM 15
#define MOVE_JOINT_NUM 16
#define MOVE_LINE_TOOL_NUM 17
#define MOVE_LINE_NUM 18

bool clearErrCallback(xarm_msgs::ClearErr::Request &req, xarm_msgs::ClearErr::Response &res) {
    ROS_INFO("[%s] %s", id.c_str(), __FUNCTION__);
    
    robotMsg.mode = CLEAR_ERR_NUM;
    robotMsg.cmdnum = CLEAR_ERR_NUM;
    robotMsg.mt_brake = CLEAR_ERR_NUM;
    robotMsg.mt_able = CLEAR_ERR_NUM;
    robotMsg.err = CLEAR_ERR_NUM;
    robotMsg.warn = CLEAR_ERR_NUM;
    for (int i=0; i<sizeof(robotMsg.pose)/sizeof(float); i++) {
        robotMsg.angle.push_back(CLEAR_ERR_NUM);
        robotMsg.pose[i] = CLEAR_ERR_NUM;
        robotMsg.offset[i] = CLEAR_ERR_NUM;
    }
    
#define SRV_SUCCESS	true
    res.ret = SRV_SUCCESS;
    res.message = __FUNCTION__;

    return true;
}

bool motionCtrlCallback(xarm_msgs::SetAxis::Request &req, xarm_msgs::SetAxis::Response &res) {
    ROS_INFO("[%s] %s", id.c_str(), __FUNCTION__);
    
    robotMsg.mode = MOTION_CTRL_NUM;
    robotMsg.cmdnum = MOTION_CTRL_NUM;
    robotMsg.mt_brake = MOTION_CTRL_NUM;
    robotMsg.mt_able = MOTION_CTRL_NUM;
    robotMsg.err = MOTION_CTRL_NUM;
    robotMsg.warn = MOTION_CTRL_NUM;
    for (int i=0; i<sizeof(robotMsg.pose)/sizeof(float); i++) {
        robotMsg.angle.push_back(MOTION_CTRL_NUM);
        robotMsg.pose[i] = MOTION_CTRL_NUM;
        robotMsg.offset[i] = MOTION_CTRL_NUM;
    }
    
#define SRV_SUCCESS	true
    res.ret = SRV_SUCCESS;
    res.message = __FUNCTION__;

    return true;
}

bool setModeCallback(xarm_msgs::SetInt16::Request &req, xarm_msgs::SetInt16::Response &res) {
    ROS_INFO("[%s] %s", id.c_str(), __FUNCTION__);
    
    robotMsg.mode = SET_MODE_NUM;
    robotMsg.cmdnum = SET_MODE_NUM;
    robotMsg.mt_brake = SET_MODE_NUM;
    robotMsg.mt_able = SET_MODE_NUM;
    robotMsg.err = SET_MODE_NUM;
    robotMsg.warn = SET_MODE_NUM;
    for (int i=0; i<sizeof(robotMsg.pose)/sizeof(float); i++) {
        robotMsg.angle.push_back(SET_MODE_NUM);
        robotMsg.pose[i] = SET_MODE_NUM;
        robotMsg.offset[i] = SET_MODE_NUM;
    }
    
#define SRV_SUCCESS	true
    res.ret = SRV_SUCCESS;
    res.message = __FUNCTION__;

    return true;
}

bool setStateCallback(xarm_msgs::SetInt16::Request &req, xarm_msgs::SetInt16::Response &res) {
    ROS_INFO("[%s] %s", id.c_str(), __FUNCTION__);
    
    robotMsg.mode = SET_STATE_NUM;
    robotMsg.cmdnum = SET_STATE_NUM;
    robotMsg.mt_brake = SET_STATE_NUM;
    robotMsg.mt_able = SET_STATE_NUM;
    robotMsg.err = SET_STATE_NUM;
    robotMsg.warn = SET_STATE_NUM;
    for (int i=0; i<sizeof(robotMsg.pose)/sizeof(float); i++) {
        robotMsg.angle.push_back(SET_STATE_NUM);
        robotMsg.pose[i] = SET_STATE_NUM;
        robotMsg.offset[i] = SET_STATE_NUM;
    }
    
#define SRV_SUCCESS	true
    res.ret = SRV_SUCCESS;
    res.message = __FUNCTION__;

    return true;
}

bool goHomeCallback(xarm_msgs::Move::Request &req, xarm_msgs::Move::Response &res) {
    ROS_INFO("[%s] %s", id.c_str(), __FUNCTION__);
    
    robotMsg.mode = GO_HOME_NUM;
    robotMsg.cmdnum = GO_HOME_NUM;
    robotMsg.mt_brake = GO_HOME_NUM;
    robotMsg.mt_able = GO_HOME_NUM;
    robotMsg.err = GO_HOME_NUM;
    robotMsg.warn = GO_HOME_NUM;
    for (int i=0; i<sizeof(robotMsg.pose)/sizeof(float); i++) {
        robotMsg.angle.push_back(GO_HOME_NUM);
        robotMsg.pose[i] = GO_HOME_NUM;
        robotMsg.offset[i] = GO_HOME_NUM;
    }
    
#define SRV_SUCCESS	true
    res.ret = SRV_SUCCESS;
    res.message = __FUNCTION__;

    return true;
}

bool moveJointCallback(xarm_msgs::Move::Request &req, xarm_msgs::Move::Response &res) {
    ROS_INFO("[%s] %s", id.c_str(), __FUNCTION__);
    
    robotMsg.mode = MOVE_JOINT_NUM;
    robotMsg.cmdnum = MOVE_JOINT_NUM;
    robotMsg.mt_brake = MOVE_JOINT_NUM;
    robotMsg.mt_able = MOVE_JOINT_NUM;
    robotMsg.err = MOVE_JOINT_NUM;
    robotMsg.warn = MOVE_JOINT_NUM;
    for (int i=0; i<sizeof(robotMsg.pose)/sizeof(float); i++) {
        robotMsg.angle.push_back(MOVE_JOINT_NUM);
        robotMsg.pose[i] = MOVE_JOINT_NUM;
        robotMsg.offset[i] = MOVE_JOINT_NUM;
    }
    
#define SRV_SUCCESS	true
    res.ret = SRV_SUCCESS;
    res.message = __FUNCTION__;

    return true;
}

bool moveLineToolCallback(xarm_msgs::Move::Request &req, xarm_msgs::Move::Response &res) {
    ROS_INFO("[%s] %s", id.c_str(), __FUNCTION__);
    
    robotMsg.mode = MOVE_LINE_TOOL_NUM;
    robotMsg.cmdnum = MOVE_LINE_TOOL_NUM;
    robotMsg.mt_brake = MOVE_LINE_TOOL_NUM;
    robotMsg.mt_able = MOVE_LINE_TOOL_NUM;
    robotMsg.err = MOVE_LINE_TOOL_NUM;
    robotMsg.warn = MOVE_LINE_TOOL_NUM;
    for (int i=0; i<sizeof(robotMsg.pose)/sizeof(float); i++) {
        robotMsg.angle.push_back(MOVE_LINE_TOOL_NUM);
        robotMsg.pose[i] = MOVE_LINE_TOOL_NUM;
        robotMsg.offset[i] = MOVE_LINE_TOOL_NUM;
    }
    
#define SRV_SUCCESS	true
    res.ret = SRV_SUCCESS;
    res.message = __FUNCTION__;

    return true;
}

bool moveLineCallback(xarm_msgs::Move::Request &req, xarm_msgs::Move::Response &res) {
    ROS_INFO("[%s] %s", id.c_str(), __FUNCTION__);
    
    robotMsg.mode = MOVE_LINE_NUM;
    robotMsg.cmdnum = MOVE_LINE_NUM;
    robotMsg.mt_brake = MOVE_LINE_NUM;
    robotMsg.mt_able = MOVE_LINE_NUM;
    robotMsg.err = MOVE_LINE_NUM;
    robotMsg.warn = MOVE_LINE_NUM;
    for (int i=0; i<sizeof(robotMsg.pose)/sizeof(float); i++) {
        robotMsg.angle.push_back(MOVE_LINE_NUM);
        robotMsg.pose[i] = MOVE_LINE_NUM;
        robotMsg.offset[i] = MOVE_LINE_NUM;
    }
    
#define SRV_SUCCESS	true
    res.ret = SRV_SUCCESS;
    res.message = __FUNCTION__;

    return true;
}

int main(int argc, char* argv[]) {
    id = "xarm6_dummy";
    ros::init(argc, argv, id.c_str());
    ros::NodeHandle nh("~");

    int main_hz;
    double PUB_SEC_PERIOD;

    ros::param::get("~id", id);
    ROS_INFO("get param id : [%s]", id.c_str());
    ros::param::get("~main_hz", main_hz);
    ros::param::get("~PUB_SEC_PERIOD", PUB_SEC_PERIOD);

    // publisher
    ros::Publisher pub_state = nh.advertise<xarm_msgs::RobotMsg>("/" + id + "/xarm_states", 100);

    // service
    ros::ServiceServer srv_clearErr = nh.advertiseService("/" + id + "/clear_err", clearErrCallback);
    ros::ServiceServer srv_motionCtrl = nh.advertiseService("/" + id + "/motion_ctrl", motionCtrlCallback);
    ros::ServiceServer srv_setMode = nh.advertiseService("/" + id + "/set_mode", setModeCallback);
    ros::ServiceServer srv_setState = nh.advertiseService("/" + id + "/set_state", setStateCallback);
    ros::ServiceServer srv_goHome = nh.advertiseService("/" + id + "/go_home", goHomeCallback);
    ros::ServiceServer srv_moveJoint = nh.advertiseService("/" + id + "/move_joint", moveJointCallback);
    ros::ServiceServer srv_moveLineTool = nh.advertiseService("/" + id + "/move_line_tool", moveLineToolCallback);
    ros::ServiceServer srv_moveLine = nh.advertiseService("/" + id + "/move_line", moveLineCallback);

    double time_cur = ros::Time::now().toSec();
    double time_pre = time_cur;
    double time_diff;

    ros::Rate r(main_hz);

    while(ros::ok())
    {
        time_cur = ros::Time::now().toSec();

        time_diff = time_cur - time_pre;
        if (time_diff > PUB_SEC_PERIOD) {
            time_pre = time_cur;

            static ros::Time ts_now;
            ts_now = ros::Time::now();

            robotMsg.header.stamp = ts_now;
            robotMsg.state++;
            
            pub_state.publish(robotMsg);
        }

        ros::spinOnce();
        r.sleep();
    }

    return 0;
}
