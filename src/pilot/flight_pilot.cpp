#include "flightros/pilot/flight_pilot.hpp"

namespace flightros {

FlightPilot::FlightPilot(const ros::NodeHandle &nh, const ros::NodeHandle &pnh)
  : nh_(nh),
    pnh_(pnh),
    scene_id_(0),
    unity_ready_(false),
    unity_render_(false),
    receive_id_(0),
    main_loop_freq_(10) {
  // load parameters
  if (!loadParams()) {
    ROS_WARN("[%s] Could not load all parameters.",
             pnh_.getNamespace().c_str());
  } else {
    ROS_INFO("[%s] Loaded all parameters.", pnh_.getNamespace().c_str());
  }

  // quad initialization
  quad_ptr_ = std::make_shared<Quadrotor>();

  // add mono camera
  rgb_camera_ = std::make_shared<RGBCamera>();
  rgb2_camera_ = std::make_shared<RGBCamera>();

  image_transport::ImageTransport it(pnh);
  rgb_pub = it.advertise("/rgb", 1);
  rgb2_pub = it.advertise("/rgb2", 1);
  depth_pub = it.advertise("/depth", 1);

  rgb_info_pub = nh_.advertise<sensor_msgs::CameraInfo>("/rgb_info",1);

  //when changing camera placement, please also change the camera link transformation in the launch file to match the depth map with global map
  Vector<3> B_r_BC(0,-0.05,0.8);
  Vector<3> B_r_BC2(0,0.05,0.8);
  Matrix<3, 3> R_BC = Quaternion(0.7071068, 0, 0, -0.7071068).toRotationMatrix();
  std::cout << R_BC << std::endl;
  Matrix<3, 3> R_BC2 = Quaternion(0.7071068, 0, 0, -0.7071068).toRotationMatrix();

  rgb2_camera_->setFOV(90);
  rgb2_camera_->setWidth(720);
  rgb2_camera_->setHeight(480);
  rgb2_camera_->setRelPose(B_r_BC2, R_BC2);

  rgb_camera_->setFOV(90);
  rgb_camera_->setWidth(720);
  rgb_camera_->setHeight(480);
  rgb_camera_->setRelPose(B_r_BC, R_BC);
  rgb_camera_->setPostProcesscing(
  std::vector<bool>{true, false, false});

  quad_ptr_->addRGBCamera(rgb_camera_);
  quad_ptr_->addRGBCamera(rgb2_camera_);
  // initialization
  quad_state_.setZero();
  quad_ptr_->reset(quad_state_);


  // initialize subscriber call backs
  sub_state_est_ = nh_.subscribe("/mavros/local_position/pose", 1,
  //sub_state_est_ = nh_.subscribe("/gazebo_groundtruth_posestamped", 1,
                                 &FlightPilot::poseCallback, this);
  // wait until the gazebo and unity are loaded
  ros::Duration(5.0).sleep();

  // connect unity
  setUnity(unity_render_);
  connectUnity();
  ros::spin();
}

FlightPilot::~FlightPilot() {}

void FlightPilot::poseCallback(const geometry_msgs::PoseStamped &msg) {
  ros::Time timestamp = ros::Time::now();

  quad_state_.x[QS::POSX] = (Scalar)msg.pose.position.x;
  quad_state_.x[QS::POSY] = (Scalar)msg.pose.position.y;
  quad_state_.x[QS::POSZ] = (Scalar)msg.pose.position.z;
  quad_state_.x[QS::ATTW] = (Scalar)msg.pose.orientation.w;
  quad_state_.x[QS::ATTX] = (Scalar)msg.pose.orientation.x;
  quad_state_.x[QS::ATTY] = (Scalar)msg.pose.orientation.y;
  quad_state_.x[QS::ATTZ] = (Scalar)msg.pose.orientation.z;
  quad_ptr_->setState(quad_state_);
  if (unity_render_ && unity_ready_) {
    unity_bridge_ptr_->getRender(0);
    unity_bridge_ptr_->handleOutput();
  }

  cv::Mat img;
  rgb_camera_->getRGBImage(img);
  sensor_msgs::ImagePtr rgb_msg =
    cv_bridge::CvImage(std_msgs::Header(), "bgr8", img).toImageMsg();
  rgb_msg->header.stamp = timestamp;
  rgb_pub.publish(rgb_msg);

  rgb2_camera_->getRGBImage(img);
  sensor_msgs::ImagePtr rgb2_msg =
    cv_bridge::CvImage(std_msgs::Header(), "bgr8", img).toImageMsg();
  rgb2_msg->header.stamp = timestamp;
  rgb2_pub.publish(rgb2_msg);

  rgb_camera_->getDepthMap(img);
  sensor_msgs::ImagePtr depth_msg =
    cv_bridge::CvImage(std_msgs::Header(), "32FC1", img).toImageMsg();
  depth_msg->header.stamp = timestamp;
  depth_msg->header.frame_id = "/camera_link";
  depth_pub.publish(depth_msg);

  rgb_info.header.frame_id="rgb",
  rgb_info.header.stamp = timestamp;
  rgb_info.width = 720;
  rgb_info.height = 480;
  rgb_info.distortion_model = "plumb_bob";
  rgb_info.D = std::vector<double>(0);

  //boost::array<double,9> K_array = {360.0, 0.0, 240, 0.0, 360.0, 360.0, 0.0, 0.0, 1.0};
  //boost::array<double,9> R_array = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
  //boost::array<double,12> P_array = {360.0, 0.0, 240, 0.0, 0.0, 360, 360, 0.0, 0.0, 0.0, 1.0, 0.0};
  
  boost::array<double,9> K_array = {240.0, 0.0, 360, 0.0, 240.0, 240.0, 0.0, 0.0, 1.0};
  boost::array<double,9> R_array = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
  boost::array<double,12> P_array = {240.0, 0.0, 360, 0.0, 0.0, 240, 240, 0.0, 0.0, 0.0, 1.0, 0.0};
  rgb_info.K = K_array;
  rgb_info.R = R_array;
  rgb_info.P = P_array;
  rgb_info_pub.publish(rgb_info);
}

bool FlightPilot::setUnity(const bool render) {
  unity_render_ = render;
  if (unity_render_ && unity_bridge_ptr_ == nullptr) {
    // create unity bridge
    unity_bridge_ptr_ = UnityBridge::getInstance();
    unity_bridge_ptr_->addQuadrotor(quad_ptr_);
    ROS_INFO("[%s] Unity Bridge is created.", pnh_.getNamespace().c_str());
  }
  return true;
}

bool FlightPilot::connectUnity() {
  if (!unity_render_ || unity_bridge_ptr_ == nullptr) return false;
  unity_ready_ = unity_bridge_ptr_->connectUnity(scene_id_);
  return unity_ready_;
}

bool FlightPilot::loadParams(void) {
  // load parameters
  quadrotor_common::getParam("main_loop_freq", main_loop_freq_, pnh_);
  quadrotor_common::getParam("unity_render", unity_render_, pnh_);

  return true;
}

}  // namespace flightros
