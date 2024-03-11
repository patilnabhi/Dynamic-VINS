/* Author: Abhishek Patil */

#include <rclcpp/rclcpp.hpp>

#include "utility/parameters.h"
#include "estimator/estimator.h"

namespace estimator_node_ns
{

class EstimatorNode : public rclcpp::Node
{
public:

  EstimatorNode(rclcpp::NodeOptions options)
  : Node("estimator_node", options.use_intra_process_comms(true)),
  logger_(this->get_logger())
  {
    ;
  }

private:
  void onInit() //override
  {
    // ros::NodeHandle &pn = getPrivateNodeHandle();
    // ros::NodeHandle &nh = getMTNodeHandle();

    // ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);
    std::string config_file = this->declare_parameter("config_file", config_file);
    this->get_parameter("config_file", config_file);
    readParameters(logger_, config_file);

    estimator_->setParameter();
#ifdef EIGEN_DONT_PARALLELIZE
    RCLCPP_DEBUG(logger_, "EIGEN_DONT_PARALLELIZE");
#endif
    RCLCPP_WARN(logger_, "waiting for image, semantic and imu...");

    // registerPub(nh);

    sub_image_ = this->create_subscription<sensor_msgs::msg::Image>(
      IMAGE_TOPIC, 100, std::bind(&EstimatorNode::image_callback, this, std::placeholders::_1));
    sub_depth_ = this->create_subscription<sensor_msgs::msg::Image>(
      DEPTH_TOPIC, 100, std::bind(&EstimatorNode::depth_callback, this, std::placeholders::_1));

//       // sub_semantic =
//       //     nh.subscribe("/untracked_info", 100, &EstimatorNodelet::semantic_callback, this);
//       if (USE_IMU)
//           sub_imu = nh.subscribe(IMU_TOPIC, 100, &EstimatorNodelet::imu_callback, this,
//                                  ros::TransportHints().tcpNoDelay());
//       // topic from pose_graph, notify if there's relocalization
//       sub_relo_points = nh.subscribe("/pose_graph/match_points", 10,
//                                      &EstimatorNodelet::relocalization_callback, this);

//       dura = std::chrono::milliseconds(2);

//       trackThread   = std::thread(&EstimatorNodelet::process_tracker, this);
//       processThread = std::thread(&EstimatorNodelet::process, this);
  }

  void image_callback(const sensor_msgs::msg::Image::ConstSharedPtr &color_msg)
  {
    m_buf_.lock();
    img_buf_.emplace(color_msg);
    m_buf_.unlock();
    con_tracker_.notify_one();
  }

  void depth_callback(const sensor_msgs::msg::Image::ConstSharedPtr &depth_msg)
  {
    m_buf_.lock();
    depth_buf_.emplace(depth_msg);
    m_buf_.unlock();
    con_tracker_.notify_one();
  }

private:
  rclcpp::Logger logger_;

  std::shared_ptr<Estimator> estimator_;

  std::shared_ptr<rclcpp::Subscription<sensor_msgs::msg::Image>> sub_image_;
  std::shared_ptr<rclcpp::Subscription<sensor_msgs::msg::Image>> sub_depth_;

  std::mutex m_buf_;

  std::condition_variable con_tracker_;
  std::queue<sensor_msgs::msg::Image::ConstSharedPtr> img_buf_;
  std::queue<sensor_msgs::msg::Image::ConstSharedPtr> depth_buf_;

};

} // namespace estimator_node_ns

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(estimator_node_ns::EstimatorNode)