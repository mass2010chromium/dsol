#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/msg/pose_array.hpp>
#include <message_filters/subscriber.hpp>
#include <message_filters/time_synchronizer.h>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <tf2_ros/transform_broadcaster.h>

//#include <boost/circular_buffer.hpp>

#include "sv/dsol/extra.h"
#include "sv/dsol/node_util.h"
#include "sv/dsol/odom.h"
#include "sv/ros/msg_conv.h"

namespace sv::dsol {

namespace cb = cv_bridge;
namespace sm = sensor_msgs::msg;
namespace gm = geometry_msgs::msg;
namespace mf = message_filters;

template<typename T>
class circular_buffer {
    public:
        circular_buffer(size_t n) {
            _data.reserve(n);
            _start = 0;
            _end = 0;
            _size = 0;
            _buffer_size = n;
        }

        void push_back(const T& obj) {
            _data[_end] = obj;
            if (_size < _buffer_size) { _size += 1; }
            else {
                _start += 1;
                if (_start == _buffer_size) { _start = 0; }
            }
            _end += 1;
            if (_end == _buffer_size) { _end = 0; }
        }

        size_t normalize(size_t index) {
            size_t res = index + _start;
            if (res < _buffer_size) { return res; }
            return res - _buffer_size;
        }

        size_t size() { return _size; }
        // front/back
        T& front() { return _data[_start]; }
        T& back() { return _data[_end]; }
        // indexing
        T& at(size_t i) { return _data[normalize(i)]; }
        T& operator[](size_t i) { return at(i); }

    private:
        size_t _start;
        size_t _end;
        size_t _size;
        size_t _buffer_size;
        std::vector<T> _data;
};

struct NodeOdom : public rclcpp::Node {
  explicit NodeOdom();

  void InitOdom();
  void InitRosIO();

  void Cinfo1Cb(const sm::CameraInfo& cinfo1_msg);
  void StereoCb(const std::shared_ptr<sm::Image const>& image0_ptr,
                const std::shared_ptr<sm::Image const>& image1_ptr);
  void StereoDepthCb(const std::shared_ptr<sm::Image const>& image0_ptr,
                     const std::shared_ptr<sm::Image const>& image1_ptr,
                     const std::shared_ptr<sm::Image const>& depth0_ptr);

  void TfCamCb(const gm::Transform& tf_cam_msg);
  void TfImuCb(const gm::Transform& tf_imu_msg);

  void AccCb(const sm::Imu& acc_msg);
  void GyrCb(const sm::Imu& gyr_msg);

  void PublishOdom(const std_msgs::msg::Header& header, const Sophus::SE3d& tf);
  void PublishCloud(const std_msgs::msg::Header& header);

  using SyncStereo = mf::TimeSynchronizer<sm::Image, sm::Image>;
  using SyncStereoDepth = mf::TimeSynchronizer<sm::Image, sm::Image, sm::Image>;

  //boost::circular_buffer<sm::Imu> gyrs_;
  circular_buffer<sm::Imu> gyrs_;
  mf::Subscriber<sm::Image> sub_image0_;
  mf::Subscriber<sm::Image> sub_image1_;
  mf::Subscriber<sm::Image> sub_depth0_;

  std::optional<SyncStereo> sync_stereo_;
  std::optional<SyncStereoDepth> sync_stereo_depth_;

  rclcpp::Subscription<sm::CameraInfo>::SharedPtr sub_cinfo1_;
  rclcpp::Subscription<sm::Imu>::SharedPtr sub_gyr_;

  rclcpp::Publisher<sm::PointCloud2>::SharedPtr pub_points_;
  rclcpp::Publisher<gm::PoseArray>::SharedPtr pub_parray_;
  PosePathPublisher pub_odom_;

  MotionModel motion_;
  DirectOdometry odom_;

  std::string frame_{"fixed"};
  sm::PointCloud2 cloud_;
};

NodeOdom::NodeOdom()
    : rclcpp::Node("dsol_node_odom"),
      gyrs_(50) {

  // Old version of message_filters requires raw rmw_qos_profile_t
  rmw_qos_profile_t qos = {};
  qos.depth = 5;
  std::shared_ptr<rclcpp::Node> ptr(this);
  std::shared_ptr<rclcpp::Node> node_p = shared_from_this();
  sub_image0_.subscribe(node_p, "image0", qos);
  sub_image1_.subscribe(node_p, "image1", qos);
  sub_depth0_.subscribe(node_p, "depth0", qos);
  InitOdom();
  InitRosIO();
}

void NodeOdom::InitOdom() {
  std::shared_ptr<rclcpp::Node> node_p = shared_from_this();
  {
    auto cfg = ReadOdomCfg(node_p, "odom");
    this->get_parameter("tbb", cfg.tbb);
    this->get_parameter("log", cfg.log);
    this->get_parameter("vis", cfg.vis);
    odom_.Init(cfg);
  }
  odom_.selector = PixelSelector(ReadSelectCfg(node_p, "select"));
  odom_.matcher = StereoMatcher(ReadStereoCfg(node_p, "stereo"));
  odom_.aligner = FrameAligner(ReadDirectCfg(node_p, "align"));
  odom_.adjuster = BundleAdjuster(ReadDirectCfg(node_p, "adjust"));
  odom_.cmap = GetColorMap(this->declare_parameter("cm", "jet"));
  RCLCPP_INFO_STREAM(rclcpp::get_logger("DEFAULT_LOGGER"), odom_.Repr());

  motion_.Init();
}

void NodeOdom::InitRosIO() {
  bool use_depth = this->declare_parameter("use_depth", false);
  if (use_depth) {
    sync_stereo_depth_.emplace(sub_image0_, sub_image1_, sub_depth0_, 5);
    sync_stereo_depth_->registerCallback(
        std::bind(&NodeOdom::StereoDepthCb, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
  } else {
    sync_stereo_.emplace(sub_image0_, sub_image1_, 5);
    sync_stereo_->registerCallback(
        std::bind(&NodeOdom::StereoCb, this, std::placeholders::_1, std::placeholders::_2));
  }
  sub_cinfo1_ = this->create_subscription<sm::CameraInfo>("cinfo1", 1, std::bind(&NodeOdom::Cinfo1Cb, this, std::placeholders::_1));
  sub_gyr_ = this->create_subscription<sm::Imu>("gyr", 200, std::bind(&NodeOdom::GyrCb, this, std::placeholders::_1));

  std::shared_ptr<rclcpp::Node> node_p = shared_from_this();
  pub_odom_ = PosePathPublisher(node_p, "odom", frame_);
  pub_points_ = this->create_publisher<sm::PointCloud2>("points", 1);
  pub_parray_ = this->create_publisher<gm::PoseArray>("parray", 1);
}

void NodeOdom::Cinfo1Cb(const sm::CameraInfo& cinfo1_msg) {
  odom_.camera = MakeCamera(cinfo1_msg);
  RCLCPP_INFO_STREAM(rclcpp::get_logger("DEFAULT_LOGGER"), odom_.camera.Repr());
  // TODO: destroy subscription
  // sub_cinfo1_.shutdown();
}

void NodeOdom::AccCb(const sm::Imu& acc_msg) {}

void NodeOdom::GyrCb(const sm::Imu& gyr_msg) {

  gyrs_.push_back(gyr_msg);
}

void NodeOdom::StereoCb(const std::shared_ptr<sm::Image const>& image0_ptr,
                        const std::shared_ptr<sm::Image const>& image1_ptr) {
  StereoDepthCb(image0_ptr, image1_ptr, nullptr);
}

void NodeOdom::StereoDepthCb(const std::shared_ptr<sm::Image const>& image0_ptr,
                             const std::shared_ptr<sm::Image const>& image1_ptr,
                             const std::shared_ptr<sm::Image const>& depth0_ptr) {
  const auto curr_header = image0_ptr->header;
  const auto image0 = cb::toCvShare(image0_ptr)->image;
  const auto image1 = cb::toCvShare(image1_ptr)->image;

  cv::Mat depth0;
  if (depth0_ptr) {
    depth0 = cb::toCvCopy(depth0_ptr)->image;
    depth0.convertTo(depth0, CV_32FC1, 0.001);  // 16bit in millimeters
  }

  static rclcpp::Time prev_stamp;
  rclcpp::Time curr_time(curr_header.stamp.sec, curr_header.stamp.nanosec);
  const rclcpp::Duration delta_duration =
      prev_stamp.nanoseconds() == 0 ? rclcpp::Duration(0, 0) : curr_time - prev_stamp;
  const auto dt = delta_duration.seconds();
  RCLCPP_INFO_STREAM(rclcpp::get_logger("DEFAULT_LOGGER"), "dt: " << dt * 1000);

  Sophus::SE3d dtf_pred;
  if (dt > 0) {
    dtf_pred = motion_.PredictDelta(dt);

    RCLCPP_INFO_STREAM(rclcpp::get_logger("DEFAULT_LOGGER"), 
        fmt::format("prev: {}, curr: {}, first_imu: {}, last_imu: {}",
                    prev_stamp.seconds(),
                    curr_header.stamp.sec,
                    gyrs_.front().header.stamp.sec,
                    gyrs_.back().header.stamp.sec));
    Sophus::SO3d dR{};
    int n_imus = 0;
    for (size_t i = 0; i < gyrs_.size(); ++i) {
      const auto& imu = gyrs_[i];
      const rclcpp::Time imu_stamp(imu.header.stamp.sec, imu.header.stamp.nanosec);
      if (imu_stamp <= prev_stamp) continue;
      if (imu_stamp > curr_time) continue;

      rclcpp::Time prev_imu_stamp;
      if (i == 0) {
        prev_imu_stamp = prev_stamp;
      }
      else {
        const auto prev_header_stamp = gyrs_.at(i-1).header.stamp;
        prev_imu_stamp = rclcpp::Time(prev_header_stamp.sec, prev_header_stamp.nanosec);
      }
      const double dt_imu = (imu_stamp - prev_imu_stamp).seconds();
      CHECK_GT(dt_imu, 0);
      Eigen::Map<const Eigen::Vector3d> w(&imu.angular_velocity.x);
      dR *= Sophus::SO3d::exp(w * dt_imu);
      ++n_imus;
    }
    RCLCPP_INFO_STREAM(rclcpp::get_logger("DEFAULT_LOGGER"), "n_imus: " << n_imus);
    if (n_imus > 0) dtf_pred.so3() = dR;
  }

  const auto status = odom_.Estimate(image0, image1, dtf_pred, depth0);
  RCLCPP_INFO_STREAM(rclcpp::get_logger("DEFAULT_LOGGER"), status.Repr());

  if (status.track.ok) {
    motion_.Correct(status.Twc(), dt);
  } else {
    RCLCPP_WARN_STREAM(rclcpp::get_logger("DEFAULT_LOGGER"), "Tracking failed (or 1st frame), slow motion model");
  }

  std_msgs::msg::Header header;
  header.frame_id = "fixed";
  header.stamp = curr_header.stamp;

  PublishOdom(header, status.Twc());
  if (status.map.remove_kf) {
    PublishCloud(header);
  }

  prev_stamp = curr_header.stamp;
}

void NodeOdom::PublishOdom(const std_msgs::msg::Header& header,
                           const Sophus::SE3d& tf) {
  const auto pose_msg = pub_odom_.Publish(header.stamp, tf);

  const auto poses = odom_.window.GetAllPoses();
  gm::PoseArray parray_msg;
  parray_msg.header = header;
  parray_msg.poses.resize(poses.size());
  for (size_t i = 0; i < poses.size(); ++i) {
    Sophus2Ros(poses.at(i), parray_msg.poses.at(i));
  }
  pub_parray_->publish(parray_msg);
}

void NodeOdom::PublishCloud(const std_msgs::msg::Header& header) {
  if (this->count_subscribers("points") == 0) return;

  cloud_.header = header;
  cloud_.point_step = 16;
  cloud_.fields = MakePointFields("xyzi");

  RCLCPP_DEBUG_STREAM(rclcpp::get_logger("DEFAULT_LOGGER"), odom_.window.MargKf().status().Repr());
  Keyframe2Cloud(odom_.window.MargKf(), cloud_, 50.0);
  pub_points_->publish(cloud_);
}



}  // namespace sv::dsol

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  cv::setNumThreads(4);
  auto node = std::make_shared<sv::dsol::NodeOdom>();
  rclcpp::spin(node);
}
