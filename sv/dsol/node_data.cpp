#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rosgraph_msgs/msg/clock.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include "sv/dsol/extra.h"
#include "sv/dsol/node_util.h"
#include "sv/ros/msg_conv.h"
#include "sv/util/dataset.h"
#include "sv/util/logging.h"
#include "sv/util/ocv.h"

namespace sv::dsol {

using SE3d = Sophus::SE3d;
namespace gm = geometry_msgs::msg;
namespace sm = sensor_msgs::msg;
namespace vm = visualization_msgs::msg;

struct NodeData : public rclcpp::Node {
  explicit NodeData();

  void InitOdom();
  void InitRosIO();
  void InitDataset();

  void PublishOdom(const std_msgs::msg::Header& header, const Sophus::SE3d& Twc);
  void PublishCloud(const std_msgs::msg::Header& header) const;
  void SendTransform(const gm::PoseStamped& pose_msg,
                     const std::string& child_frame);
  void Run();

  bool reverse_{false};
  double freq_{10.0};
  double data_max_depth_{0};
  double cloud_max_depth_{100};
  cv::Range data_range_{0, 0};

  Dataset dataset_;
  MotionModel motion_;
  TumFormatWriter writer_;
  DirectOdometry odom_;

  KeyControl ctrl_;
  std::string frame_{"fixed"};
  tf2_ros::TransformBroadcaster tfbr_;

  rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr clock_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr pose_array_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr align_marker_pub_;
  PosePathPublisher gt_pub_;
  PosePathPublisher kf_pub_;
  PosePathPublisher odom_pub_;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr points_pub_;
};

NodeData::NodeData() : rclcpp::Node("dsol_node_data") {
  InitRosIO();
  InitDataset();
  InitOdom();

  const int wait_ms = this->declare_parameter("wait_ms", 0);
  RCLCPP_INFO_STREAM(rclcpp::get_logger("DEFAULT_LOGGER"), "wait_ms: " << wait_ms);
  ctrl_ = KeyControl(wait_ms);

  const auto save = this->declare_parameter("save", "");
  writer_ = TumFormatWriter(save);
  if (!writer_.IsDummy()) {
    RCLCPP_WARN_STREAM(rclcpp::get_logger("DEFAULT_LOGGER"), "Writing results to: " << writer_.filename());
  }

  const auto alpha = this->declare_parameter("motion_alpha", 0.5);
  motion_ = MotionModel(alpha);
  RCLCPP_INFO_STREAM(rclcpp::get_logger("DEFAULT_LOGGER"), "motion_alpha: " << motion_.alpha());

}

void NodeData::InitRosIO() {
  clock_pub_ = this->create_publisher<rosgraph_msgs::msg::Clock>("/clock", 1);

  gt_pub_ = PosePathPublisher(this, "gt", frame_);
  kf_pub_ = PosePathPublisher(this, "kf", frame_);
  odom_pub_ = PosePathPublisher(this, "odom", frame_);
  points_pub_ = this->create_publisher<sm::PointCloud2>("points", 1);
  pose_array_pub_ = this->create_publisher<gm::PoseArray>("poses", 1);
  align_marker_pub_ = this->create_publisher<vm::Marker>("align_graph", 1);
}

void NodeData::InitDataset() {
  const auto data_dir = this->declare_parameter("data_dir", {});
  dataset_ = CreateDataset(data_dir);
  if (!dataset_) {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("DEFAULT_LOGGER"), "Invalid dataset at: " << data_dir);
    ros::shutdown();
  }
  RCLCPP_INFO_STREAM(rclcpp::get_logger("DEFAULT_LOGGER"), dataset_.Repr());

  this->getParam("start", data_range_.start);
  this->getParam("end", data_range_.end);
  this->getParam("reverse", reverse_);

  if (data_range_.end <= 0) {
    data_range_.end += dataset_.size();
  }
  RCLCPP_INFO(rclcpp::get_logger("DEFAULT_LOGGER"), "Data range: [%d, %d)", data_range_.start, data_range_.end);
  RCLCPP_INFO(rclcpp::get_logger("DEFAULT_LOGGER"), "Reverse: %s", reverse_ ? "true" : "false");

  this->getParam("freq", freq_);
  this->getParam("data_max_depth", data_max_depth_);
  this->getParam("cloud_max_depth", cloud_max_depth_);

  RCLCPP_INFO_STREAM(rclcpp::get_logger("DEFAULT_LOGGER"), "Freq: " << freq_);
  RCLCPP_INFO_STREAM(rclcpp::get_logger("DEFAULT_LOGGER"), "Max depth: " << data_max_depth_);
}

void NodeData::InitOdom() {
  {
    auto cfg = ReadOdomCfg({this, "odom"});
    this->getParam("tbb", cfg.tbb);
    this->getParam("log", cfg.log);
    this->getParam("vis", cfg.vis);
    odom_.Init(cfg);
  }
  odom_.selector = PixelSelector(ReadSelectCfg({this, "select"}));
  odom_.matcher = StereoMatcher(ReadStereoCfg({this, "stereo"}));
  odom_.aligner = FrameAligner(ReadDirectCfg({this, "align"}));
  odom_.adjuster = BundleAdjuster(ReadDirectCfg({this, "adjust"}));
  odom_.cmap = GetColorMap(this->declare_parameter("cm", "jet"));

  RCLCPP_INFO_STREAM(rclcpp::get_logger("DEFAULT_LOGGER"), odom_.Repr());
}

void NodeData::PublishCloud(const std_msgs::msg::Header& header) const {
  if (points_pub_.getNumSubscribers() == 0) return;

  static sensor_msgs::msg::PointCloud2 cloud;
  cloud.header = header;
  cloud.point_step = 16;
  cloud.fields = MakePointFields("xyzi");

  RCLCPP_DEBUG_STREAM(rclcpp::get_logger("DEFAULT_LOGGER"), odom_.window.MargKf().status().Repr());
  Keyframe2Cloud(odom_.window.MargKf(), cloud, cloud_max_depth_);
  points_pub_->publish(cloud);
}

void NodeData::SendTransform(const geometry_msgs::msg::PoseStamped& pose_msg,
                             const std::string& child_frame) {
  gm::TransformStamped tf_msg;
  tf_msg.header = pose_msg.header;
  tf_msg.child_frame_id = child_frame;
  Ros2Ros(pose_msg.pose, tf_msg.transform);
  tfbr_.sendTransform(tf_msg);
}

void NodeData::Run() {
  ros::Time time{};
  const auto dt = 1.0 / freq_;
  const rclcpp::Duration dtime{rclcpp::Rate{freq_}};

  bool init_tf{false};
  SE3d T_c0_w_gt;
  SE3d dT_pred;

  int start_ind = reverse_ ? data_range_.end - 1 : data_range_.start;
  int end_ind = reverse_ ? data_range_.start - 1 : data_range_.end;
  const int delta = reverse_ ? -1 : 1;

  vm::Marker align_marker;

  for (int ind = start_ind, cnt = 0; ind != end_ind; ind += delta, ++cnt) {
    if (!rclcpp::ok() || !ctrl_.Wait()) break;

    RCLCPP_INFO(rclcpp::get_logger("DEFAULT_LOGGER"), "=== %d ===", ind);
    rosgraph_msgs::msg::Clock clock;
    clock.clock = time;
    clock_pub_->publish(clock);

    auto image_l = dataset_.Get(DataType::kImage, ind, 0);
    auto image_r = dataset_.Get(DataType::kImage, ind, 1);

    if (!odom_.camera.Ok()) {
      const auto intrin = dataset_.Get(DataType::kIntrin, ind);
      const auto camera = Camera::FromMat({image_l.cols, image_l.rows}, intrin);
      odom_.SetCamera(camera);
      RCLCPP_INFO_STREAM(rclcpp::get_logger("DEFAULT_LOGGER"), camera);
    }

    auto depth = dataset_.Get(DataType::kDepth, ind, 0);

    if (!depth.empty()) {
      if (data_max_depth_ > 0) {
        cv::threshold(depth, depth, data_max_depth_, 0, cv::THRESH_TOZERO_INV);
      }
    }

    const auto pose_gt = dataset_.Get(DataType::kPose, ind, 0);

    if (!init_tf) {
      T_c0_w_gt = SE3dFromMat(pose_gt).inverse();
      RCLCPP_INFO_STREAM(rclcpp::get_logger("DEFAULT_LOGGER"), "T_c0_w:\n" << T_c0_w_gt.matrix());
      init_tf = true;
    }

    const auto T_c0_c_gt = T_c0_w_gt * SE3dFromMat(pose_gt);

    if (!motion_.Ok()) {
      motion_.Init(T_c0_c_gt);
    } else {
      dT_pred = motion_.PredictDelta(dt);
    }

    const auto T_pred = odom_.frame.Twc() * dT_pred;

    const auto status = odom_.Estimate(image_l, image_r, dT_pred, depth);
    RCLCPP_INFO_STREAM(rclcpp::get_logger("DEFAULT_LOGGER"), status.Repr());

    if (status.track.ok && ind != start_ind) {
      motion_.Correct(status.Twc(), dt);
    } else {
      RCLCPP_WARN_STREAM(rclcpp::get_logger("DEFAULT_LOGGER"), "Tracking failed (or 1st frame), slow motion model");
      motion_.Scale(0.5);
    }

    writer_.Write(cnt, status.Twc());

    RCLCPP_DEBUG_STREAM(rclcpp::get_logger("DEFAULT_LOGGER"), "trans gt:   " << T_c0_c_gt.translation().transpose());
    RCLCPP_DEBUG_STREAM(rclcpp::get_logger("DEFAULT_LOGGER"), "trans pred: " << T_pred.translation().transpose());
    RCLCPP_DEBUG_STREAM(rclcpp::get_logger("DEFAULT_LOGGER"), "trans odom: " << status.Twc().translation().transpose());
    RCLCPP_DEBUG_STREAM(rclcpp::get_logger("DEFAULT_LOGGER"), "trans ba:   "
                     << odom_.window.CurrKf().Twc().translation().transpose());
    RCLCPP_DEBUG_STREAM(rclcpp::get_logger("DEFAULT_LOGGER"), "aff_l: " << odom_.frame.state().affine_l.ab.transpose());
    RCLCPP_DEBUG_STREAM(rclcpp::get_logger("DEFAULT_LOGGER"), "aff_r: " << odom_.frame.state().affine_r.ab.transpose());

    std_msgs::msg::Header header;
    header.frame_id = frame_;
    header.stamp = time;

    gt_pub_.Publish(time, T_c0_c_gt);
    PublishOdom(header, status.Twc());

    if (status.map.remove_kf) {
      PublishCloud(header);
    }


    time += dtime;
  }
}

void NodeData::PublishOdom(const std_msgs::msg::Header& header,
                           const Sophus::SE3d& Twc) {
  const auto odom_pose_msg = odom_pub_.Publish(header.stamp, Twc);
  SendTransform(odom_pose_msg, "camera");

  const auto poses = odom_.window.GetAllPoses();
  gm::PoseArray pose_array_msg;
  pose_array_msg.header = header;
  pose_array_msg.poses.resize(poses.size());
  for (size_t i = 0; i < poses.size(); ++i) {
    Sophus2Ros(poses.at(i), pose_array_msg.poses.at(i));
  }
  pose_array_pub_->publish(pose_array_msg);
}

}  // namespace sv::dsol

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  sv::dsol::NodeData node{rclcpp::Node{"~"}};
  node.Run();
}
