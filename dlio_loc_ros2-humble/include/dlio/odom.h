#include "dlio/dlio.h"
#include "dlio/utils.h"

// ROS
#include <tf2_ros/transform_broadcaster.h>

#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "rclcpp/rclcpp.hpp"

// BOOST
#include <boost/algorithm/string.hpp>
#include <boost/circular_buffer.hpp>
#include <boost/format.hpp>
#include <boost/range/adaptor/adjacent_filtered.hpp>
#include <boost/range/adaptor/indexed.hpp>

// PCL
#include <pcl/filters/crop_box.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/surface/concave_hull.h>
#include <pcl/surface/convex_hull.h>
#include <pcl_conversions/pcl_conversions.h>

// #include "lifecycle_msgs/msg/transition.hpp"
// #include "rclcpp_lifecycle/lifecycle_node.hpp"
// #include "rclcpp_lifecycle/lifecycle_publisher.hpp"
// #include "nav_msgs/msg/odometry.
#include <tf2/utils.h>
#include <tf2_ros/buffer.h>
// #include <tf2_ros/transform_listener.h>
#include <tf2_eigen/tf2_eigen.h>

#include <geometry_msgs/msg/pose2_d.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>

#include "threadpool/hasync.h"

using namespace std::chrono_literals;

namespace dlio {

class OdomNode : public rclcpp::Node {
 public:
  OdomNode();
  ~OdomNode();

 private:
  struct ImuMeas {
    double stamp;
    double dt;  // defined as the difference between the current and the
                // previous measurement
    Eigen::Vector3f ang_vel;
    Eigen::Vector3f lin_accel;
  };
  ImuMeas imu_meas;
  void initialization();
  void declare_parameters();
  void declare_global_param();
  void declare_odom_param();
  void declare_extrinsics_param();
  void declare_imu_param();
  void declare_frame_param();
  void initializeParameters();
  void initializePubSub();
  void createPublishers();
  void createSubscribers();
  void loadMap();
  void loadPoseFromServer();
  void savePoseToServer(const geometry_msgs::msg::Pose2D& pose);
  void setInitialPose();
  bool isFileExist(const std::string& filename);
  // void initializeRegistration();

  void pointTimeCallback(
      std::function<bool(const PointType&, const PointType&)>& point_time_cmp,
      std::function<bool(boost::range::index_value<PointType&, long>,
                         boost::range::index_value<PointType&, long>)>&
          point_time_neq,
      std::function<double(boost::range::index_value<PointType&, long>)>&
          extract_point_time);
  std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>>
  integrateImu(double start_time, Eigen::Quaternionf q_init,
               Eigen::Vector3f p_init, Eigen::Vector3f v_init,
               const std::vector<double>& sorted_timestamps);
  std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>>
  integrateImuInternal(
      Eigen::Quaternionf q_init, Eigen::Vector3f p_init, Eigen::Vector3f v_init,
      const std::vector<double>& sorted_timestamps,
      boost::circular_buffer<ImuMeas>::reverse_iterator begin_imu_it,
      boost::circular_buffer<ImuMeas>::reverse_iterator end_imu_it);
  void initialPoseReceived(
      geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
  void mapReceived(sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void odomReceived(nav_msgs::msg::Odometry::ConstSharedPtr msg);
  void propagateState();

  void imuReceived(sensor_msgs::msg::Imu::ConstSharedPtr msg);
  void getScanFromROS(sensor_msgs::msg::PointCloud2::ConstSharedPtr msg);
  void deskewPointcloud();
  void setInputSource();
  void initializeInputTarget();
  void cloudReceived(sensor_msgs::msg::PointCloud2::ConstSharedPtr msg);
  // void gnssReceived();
  void computeSpaciousness();
  void computeDensity();
  void computeMetrics();
  void propagateGICP();
  void updateState();
  void resetImu();
  void getNextPose();
  bool imuMeasFromTimeRange(
      double start_time, double end_time,
      boost::circular_buffer<ImuMeas>::reverse_iterator& begin_imu_it,
      boost::circular_buffer<ImuMeas>::reverse_iterator& end_imu_it);

  sensor_msgs::msg::Imu::SharedPtr transformImu(
      const sensor_msgs::msg::Imu::ConstPtr& imu_raw);
  // void publishToROS(Eigen::Matrix4f curr_pose,
  //                   std_msgs::msg::Header msg_header);
  void publishPose();
  void publishPoseThread();
  void publishCloud(
      const pcl::PointCloud<PointType>::ConstPtr& published_cloud);
  // tf2_ros::TransformBroadcaster broadcaster_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> broadcaster_;
  rclcpp::Clock clock_;
  // tf2_ros::Buffer tfbuffer_;
  // tf2_ros::TransformListener tflistener_;
  rclcpp::TimerBase::SharedPtr publish_timer;


  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::
      ConstSharedPtr initial_pose_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
      pose_pub_;
  // rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr initial_map_pub_;
  // rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr aligned_pub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::ConstSharedPtr map_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::ConstSharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::ConstSharedPtr
      cloud_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::ConstSharedPtr imu_sub_;

  rclcpp::CallbackGroup::SharedPtr lidar_cb_group, imu_cb_group, odom_cb_group,
      initial_pose_cb_group, map_cb_group;

  // pcl::Registration<PointType, PointType>::Ptr registration_;
  // pcl::VoxelGrid<PointType> voxel_grid_filter_;
  geometry_msgs::msg::PoseWithCovarianceStamped corrent_pose_stamped_;
  // nav_msgs::msg::Path path_;

  // Point Clouds
  pcl::PointCloud<PointType>::ConstPtr original_scan;
  pcl::PointCloud<PointType>::ConstPtr deskewed_scan;
  pcl::PointCloud<PointType>::ConstPtr current_scan;
  // pcl::PointCloud<PointType>::ConstPtr aligned_scan;

  pcl::PointCloud<PointType>::Ptr global_map_;
  std::shared_ptr<const nano_gicp::CovarianceList> global_map_cov;
  std::atomic<bool> map_recieved_{false};
  std::atomic<bool> initialpose_recieved_{false};
  std::atomic<bool> first_scan_{false};
  // parameters
  std::string global_frame_id_;
  std::string odom_frame_id_;
  std::string base_frame_id_;
  std::string imu_frame_id_;
  std::string laser_frame_id_;
  std::string registration_method_;
  double scan_max_range_;
  double scan_min_range_;
  double scan_period_;
  double ndt_resolution_;
  double ndt_step_size_;
  double transform_epsilon_;
  double voxel_leaf_size_;
  bool use_pcd_map_{false};
  std::string map_path_;
  std::string init_pose_path_;
  bool set_initial_pose_{false};
  double initial_pose_x_;
  double initial_pose_y_;
  double initial_pose_z_;
  double initial_pose_qx_;
  double initial_pose_qy_;
  double initial_pose_qz_;
  double initial_pose_qw_;
  double initial_pose_yaw_;
  double initial_pose_roll_;
  double initial_pose_pitch_;

  bool use_odom_{false};
  double last_odom_received_time_;
  bool enable_debug_{false};

  // Flags
  std::atomic<bool> dlio_initialized;
  std::atomic<bool> first_valid_scan;
  std::atomic<bool> first_imu_received_;
  std::atomic<bool> imu_calibrated;
  std::atomic<bool> gicp_hasConverged;
  std::atomic<bool> deskew_status;
  std::atomic<int> deskew_size;
  std::atomic<bool> initial_pose_input;
  double sweep_ref_time = {0.0};

  bool vf_use_;
  double vf_res_;

  bool adaptive_params_;

  // IMU
  rclcpp::Time imu_stamp;
  double first_imu_stamp;
  double prev_imu_stamp;
  double imu_dp, imu_dq_deg;

  bool imu_calibrate_;
  bool calibrate_gyro_;
  bool calibrate_accel_;

  // bool gravity_align_;
  double imu_calib_time_;
  int imu_buffer_size_;
  Eigen::Matrix3f imu_accel_sm_;
  double gravity_;

  int gicp_min_num_points_;
  int gicp_k_correspondences_;
  double gicp_max_corr_dist_;
  int gicp_max_iter_;
  double gicp_transformation_ep_;
  double gicp_rotation_ep_;
  double gicp_init_lambda_factor_;

  double keyframe_thresh_dist_;
  double keyframe_thresh_rot_;

  int submap_knn_;
  int submap_kcv_;
  int submap_kcc_;
  double submap_concave_alpha_;

  bool densemap_filtered_;
  bool wait_until_move_;

  double crop_size_;

  bool deskew_;

  int num_threads_;

  // Threads
  std::thread publish_pose_thread;
  std::thread publish_frame_thread;
  std::thread metrics_thread;
  std::thread debug_thread;

  // Trajectory
  std::vector<std::pair<Eigen::Vector3f, Eigen::Quaternionf>> trajectory;
  // Frames
  std::string odom_frame;
  std::string baselink_frame;
  std::string lidar_frame;
  std::string imu_frame;

  // Preprocessing
  pcl::CropBox<PointType> crop;
  pcl::VoxelGrid<PointType> voxel;

  Eigen::Matrix4f T, T_prior, T_corr, T_init, T_corr_prev;
  Eigen::Quaternionf q_final;

  Eigen::Vector3f origin;

  boost::circular_buffer<ImuMeas> imu_buffer;
  std::mutex mtx_imu;
  std::condition_variable cv_imu_stamp;

  static bool comparatorImu(ImuMeas m1, ImuMeas m2) {
    return (m1.stamp < m2.stamp);
  };

  // Geometric Observer
  struct Geo {
    bool first_opt_done;
    std::mutex mtx;
    double dp;
    double dq_deg;
    Eigen::Vector3f prev_p;
    Eigen::Quaternionf prev_q;
    Eigen::Vector3f prev_vel;
  };
  Geo geo;

  // State Vector
  struct ImuBias {
    Eigen::Vector3f gyro;
    Eigen::Vector3f accel;
  };

  struct Frames {
    Eigen::Vector3f b;
    Eigen::Vector3f w;
  };

  struct Velocity {
    Frames lin;
    Frames ang;
  };

  struct State {
    Eigen::Vector3f p;     // position in world frame
    Eigen::Quaternionf q;  // orientation in world frame
    Velocity v;
    ImuBias b;  // imu biases in body frame
  };
  State state;

  struct Pose {
    Eigen::Vector3f p;     // position in world frame
    Eigen::Quaternionf q;  // orientation in world frame
  };
  Pose lidarPose;
  Pose imuPose;

  // Metrics
  struct Metrics {
    std::vector<float> spaciousness;
    std::vector<float> density;
  };
  Metrics metrics;

  struct Extrinsics {
    struct SE3 {
      Eigen::Vector3f t;
      Eigen::Matrix3f R;
    };
    SE3 baselink2imu;
    SE3 baselink2lidar;
    Eigen::Matrix4f baselink2imu_T;
    Eigen::Matrix4f baselink2lidar_T;
  };
  Extrinsics extrinsics;

  // Timestamps
  rclcpp::Time scan_header_stamp;
  double scan_stamp;
  double prev_scan_stamp;
  double scan_dt;
  std::vector<double> comp_times;
  std::vector<double> imu_rates;
  std::vector<double> lidar_rates;

  double first_scan_stamp;
  double elapsed_time;
  geometry_msgs::msg::Pose2D last_save_pose;
  rclcpp::Time last_save_stamp;

  // GICP
  nano_gicp::NanoGICP<PointType, PointType> gicp;
  // nano_gicp::NanoGICP<PointType, PointType> gicp_temp;

  SensorType sensor;

  double geo_Kp_;
  double geo_Kv_;
  double geo_Kq_;
  double geo_Kab_;
  double geo_Kgb_;
  double geo_abias_max_;
  double geo_gbias_max_;
};
}  // namespace dlio
