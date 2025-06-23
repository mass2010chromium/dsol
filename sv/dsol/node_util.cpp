#include "sv/dsol/node_util.h"

#include "sv/ros/msg_conv.h"
#include "sv/util/logging.h"

namespace sv::dsol {

namespace gm = geometry_msgs::msg;
namespace vm = visualization_msgs::msg;
static constexpr auto kNaNF = std::numeric_limits<float>::quiet_NaN();

SelectCfg ReadSelectCfg(const std::shared_ptr<rclcpp::Node> node) {
  SelectCfg cfg;
  node->get_parameter("sel_level");
  node->get_parameter("cell_size", cfg.cell_size);
  node->get_parameter("min_grad", cfg.min_grad);
  node->get_parameter("max_grad", cfg.max_grad);
  node->get_parameter("nms_size", cfg.nms_size);
  node->get_parameter("min_ratio", cfg.min_ratio);
  node->get_parameter("max_ratio", cfg.max_ratio);
  node->get_parameter("reselect", cfg.reselect);
  return cfg;
}

DirectCfg ReadDirectCfg(const std::shared_ptr<rclcpp::Node> node) {
  DirectCfg cfg;

  node->get_parameter("init_level", cfg.optm.init_level);
  node->get_parameter("max_iters", cfg.optm.max_iters);
  node->get_parameter("max_xs", cfg.optm.max_xs);

  node->get_parameter("affine", cfg.cost.affine);
  node->get_parameter("stereo", cfg.cost.stereo);
  node->get_parameter("c2", cfg.cost.c2);
  node->get_parameter("dof", cfg.cost.dof);
  node->get_parameter("max_outliers", cfg.cost.max_outliers);
  node->get_parameter("grad_factor", cfg.cost.grad_factor);
  node->get_parameter("min_depth", cfg.cost.min_depth);

  return cfg;
}

StereoCfg ReadStereoCfg(const std::shared_ptr<rclcpp::Node> node) {
  StereoCfg cfg;
  node->get_parameter("half_rows", cfg.half_rows);
  node->get_parameter("half_cols", cfg.half_cols);
  node->get_parameter("match_level", cfg.match_level);
  node->get_parameter("refine_size", cfg.refine_size);
  node->get_parameter("min_zncc", cfg.min_zncc);
  node->get_parameter("min_depth", cfg.min_depth);
  return cfg;
}

OdomCfg ReadOdomCfg(const std::shared_ptr<rclcpp::Node> node) {
  OdomCfg cfg;
  node->get_parameter("marg", cfg.marg);
  node->get_parameter("num_kfs", cfg.num_kfs);
  node->get_parameter("num_levels", cfg.num_levels);
  node->get_parameter("min_track_ratio", cfg.min_track_ratio);
  node->get_parameter("vis_min_depth", cfg.vis_min_depth);

  node->get_parameter("reinit", cfg.reinit);
  node->get_parameter("init_depth", cfg.init_depth);
  node->get_parameter("init_stereo", cfg.init_stereo);
  node->get_parameter("init_align", cfg.init_align);
  return cfg;
}

Camera MakeCamera(const sensor_msgs::msg::CameraInfo& cinfo_msg) {
  const cv::Size size(cinfo_msg.width, cinfo_msg.height);
  const auto& P = cinfo_msg.p;
  CHECK_GT(P[0], 0);
  Eigen::Array4d fc;
  fc << P[0], P[5], P[2], P[6];
  return {size, fc, -P[3] / P[0]};
}

void Keyframe2Cloud(const Keyframe& keyframe,
                    sensor_msgs::msg::PointCloud2& cloud,
                    double max_depth,
                    int offset) {
  const auto& points = keyframe.points();
  const auto& patches = keyframe.patches().front();
  const auto grid_size = points.cvsize();

  const auto total_size = offset + grid_size.area();
  cloud.data.resize(total_size * cloud.point_step);
  cloud.height = 1;
  cloud.width = total_size;

  for (int gr = 0; gr < points.rows(); ++gr) {
    for (int gc = 0; gc < points.cols(); ++gc) {
      const auto i = offset + gr * grid_size.width + gc;
      auto* ptr =
          reinterpret_cast<float*>(cloud.data.data() + i * cloud.point_step);

      const auto& point = points.at(gr, gc);
      if (!point.InfoMax() || (1.0 / point.idepth()) > max_depth) {
        ptr[0] = ptr[1] = ptr[2] = kNaNF;
        continue;
      }
      CHECK(point.PixelOk());
      CHECK(point.DepthOk());

      const Eigen::Vector3f p_w = (keyframe.Twc() * point.pt()).cast<float>();
      const auto& patch = patches.at(gr, gc);

      ptr[0] = p_w.x();
      ptr[1] = p_w.y();
      ptr[2] = p_w.z();
      ptr[3] = static_cast<float>(patch.vals[0] / 255.0);
    }
  }
}

void Keyframes2Cloud(const KeyframePtrConstSpan& keyframes,
                     sensor_msgs::msg::PointCloud2& cloud,
                     double max_depth) {
  if (keyframes.empty()) return;

  const auto num_kfs = static_cast<int>(keyframes.size());
  const auto grid_size = keyframes[0]->points().cvsize();

  const auto total_size = num_kfs * grid_size.area();
  cloud.data.reserve(total_size * cloud.point_step);
  cloud.height = 1;
  cloud.width = total_size;

  for (int k = 0; k < num_kfs; ++k) {
    Keyframe2Cloud(
        GetKfAt(keyframes, k), cloud, max_depth, grid_size.area() * k);
  }
}


void DrawAlignGraph(const Eigen::Vector3d& frame_pos,
                    const Eigen::Matrix3Xd& kfs_pos,
                    const std::vector<int>& tracks,
                    const cv::Scalar& color,
                    double scale,
                    vm::Marker& marker) {
  CHECK_EQ(tracks.size(), kfs_pos.cols());
  marker.ns = "align";
  marker.id = 0;
  marker.type = vm::Marker::LINE_LIST;
  marker.action = vm::Marker::ADD;
  marker.color.b = static_cast<float>(color[0]);
  marker.color.g = static_cast<float>(color[1]);
  marker.color.r = static_cast<float>(color[2]);
  marker.color.a = 1.0F;

  marker.scale.x = scale;
  marker.pose.orientation.w = 1.0;
  const auto num_kfs = tracks.size();
  marker.points.clear();
  marker.points.reserve(num_kfs * 2);

  gm::Point p0;
  p0.x = frame_pos.x();
  p0.y = frame_pos.y();
  p0.z = frame_pos.z();

  gm::Point p1;
  for (int i = 0; i < num_kfs; ++i) {
    if (tracks[i] <= 0) continue;
    p1.x = kfs_pos.col(i).x();
    p1.y = kfs_pos.col(i).y();
    p1.z = kfs_pos.col(i).z();
    marker.points.push_back(p0);
    marker.points.push_back(p1);
  }
}

PosePathPublisher::PosePathPublisher(std::shared_ptr<rclcpp::Node> node,
                                     const std::string& name,
                                     const std::string& frame_id)
    : node_(node), frame_id_(frame_id),
      pose_pub_(node->create_publisher<gm::PoseStamped>("pose_" + name, 1)),
      path_pub_(node->create_publisher<nav_msgs::msg::Path>("path_" + name, 1)) {
  path_msg_.poses.reserve(1024);
}

gm::PoseStamped PosePathPublisher::Publish(const rclcpp::Time& time,
                                           const Sophus::SE3d& tf) {
  gm::PoseStamped pose_msg;
  pose_msg.header.stamp = time;
  pose_msg.header.frame_id = frame_id_;
  Sophus2Ros(tf, pose_msg.pose);
  pose_pub_->publish(pose_msg);

  path_msg_.header = pose_msg.header;
  path_msg_.poses.push_back(pose_msg);
  path_pub_->publish(path_msg_);
  return pose_msg;
}

}  // namespace sv::dsol
