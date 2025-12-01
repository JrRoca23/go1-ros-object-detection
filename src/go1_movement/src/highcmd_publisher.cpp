#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <unitree_legged_msgs/HighCmd.h>
#include <unitree_legged_msgs/HighState.h>
#include <unitree_legged_msgs/LowCmd.h>
#include <unitree_legged_msgs/LowState.h>
#include <unitree_legged_sdk/unitree_legged_sdk.h>
#include "convert.h"

using namespace UNITREE_LEGGED_SDK;

// Clase que encapsula comunicación con el robot GO1 usando el SDK
class Go1Bridge {
public:
    Go1Bridge()
        : udp_low_(LOWLEVEL, 8091, "192.168.123.10", 8007),
          udp_high_(8090, "192.168.123.161", 8082, sizeof(HighCmd), sizeof(HighState)) {
        udp_high_.InitCmdData(high_cmd_);
        udp_low_.InitCmdData(low_cmd_);
    }

    void sendHighCommand() {
        udp_high_.SetSend(high_cmd_);
        udp_high_.Send();
    }

    void receiveHighState() {
        udp_high_.Recv();
        udp_high_.GetRecv(high_state_);
    }

    // Accesores
    HighCmd& getHighCmd() { return high_cmd_; }
    const HighState& getHighState() const { return high_state_; }

private:
    UDP udp_low_;
    UDP udp_high_;

    HighCmd high_cmd_{};
    HighState high_state_{};
    LowCmd low_cmd_{};
    LowState low_state_{};
};

Go1Bridge go1;

ros::Publisher state_pub;
ros::Subscriber velocity_sub;

long twist_msg_count = 0;

// Callback para recibir comandos de velocidad y convertirlos en HighCmd
void twistCallback(const geometry_msgs::Twist::ConstPtr &msg) {
    ROS_INFO_STREAM("Recibido Twist #" << twist_msg_count++);

    go1.getHighCmd() = rosMsg2Cmd(msg);

    const auto& state = go1.getHighState();
    unitree_legged_msgs::HighState state_msg = state2rosMsg(state);
    state_pub.publish(state_msg);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "go1_motion_node");
    ros::NodeHandle nh;

    velocity_sub = nh.subscribe("cmd_vel", 1, twistCallback);
    state_pub = nh.advertise<unitree_legged_msgs::HighState>("go1/high_state", 1);

    LoopFunc send_loop("send_high_cmd", 0.002, 3, boost::bind(&Go1Bridge::sendHighCommand, &go1));
    LoopFunc recv_loop("recv_high_state", 0.002, 3, boost::bind(&Go1Bridge::receiveHighState, &go1));

    send_loop.start();
    recv_loop.start();

    ros::spin();
    return 0;
}

