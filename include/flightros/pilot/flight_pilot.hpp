
#pragma once

#include <memory>

// ros
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <tf/tf.h>
#include "tf/transform_broadcaster.h"
#include "tf/transform_listener.h"
// rpg quadrotor
#include <autopilot/autopilot_helper.h>
#include <autopilot/autopilot_states.h>
#include <quadrotor_common/parameter_helper.h>
#include <quadrotor_msgs/AutopilotFeedback.h>

// flightlib
#include "flightlib/bridges/unity_bridge.hpp"
#include "flightlib/bridges/unity_message_types.hpp"
#include "flightlib/common/quad_state.hpp"
#include "flightlib/common/types.hpp"
#include "flightlib/objects/quadrotor.hpp"
#include "flightlib/sensors/rgb_camera.hpp"

using namespace flightlib;

namespace flightros {

  class FlightPilot {
    public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    FlightPilot(const ros::NodeHandle& nh, const ros::NodeHandle& pnh);
    ~FlightPilot();

    // callbacks
    void mainLoopCallback(const ros::TimerEvent& event);
    void timmerCallback (const ros::TimerEvent& event);
    void poseCallback(const geometry_msgs::PoseStamped& msg);
    bool setUnity(const bool render);
    bool connectUnity(void);
    bool loadParams(void);

    private:
    // ros nodes
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    // publisher
    image_transport::Publisher rgb_pub;
    image_transport::Publisher rgb2_pub;
    image_transport::Publisher depth_pub;
    image_transport::Publisher segmentation_pub;
    image_transport::Publisher opticalflow_pub;

    cv::Mat img_rgb;
    ros::Time timestamp_rgb;
    ros::Time timeStamp_rgb_end;
    sensor_msgs::ImagePtr rgb_msg;
    sensor_msgs::ImagePtr rgb2_msg;
    sensor_msgs::ImagePtr depth_msg;
    // subscriber
    ros::Subscriber sub_state_est_;
    ros::Subscriber sub_state_est_2;
    // main loop timer
    ros::Timer timer_main_loop_;
    ros::Timer rgb_loop;
    // unity quadrotor
    std::shared_ptr<Quadrotor> quad_ptr_;
    std::shared_ptr<RGBCamera> rgb_camera_;
    std::shared_ptr<RGBCamera> rgb2_camera_;
    QuadState quad_state_;

    // Flightmare(Unity3D)
    std::shared_ptr<UnityBridge> unity_bridge_ptr_;
    SceneID scene_id_{UnityScene::WAREHOUSE};
    bool unity_ready_{false};
    bool unity_render_{false};
    RenderMessage_t unity_output_;
    uint16_t receive_id_{0};

    // auxiliary variables
    Scalar main_loop_freq_{100.0};
  };
}  // namespace flightros