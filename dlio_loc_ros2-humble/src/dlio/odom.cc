#include <dlio/odom.h>
using namespace dlio;

OdomNode::OdomNode() : Node("dlio_loc_node") { this->initialization(); }

OdomNode::~OdomNode() { hv::async::cleanup(); }
void OdomNode::setInitialPose() {
  if (set_initial_pose_) {
    auto msg =
        std::make_shared<geometry_msgs::msg::PoseWithCovarianceStamped>();
    msg->header.stamp = now();
    msg->header.frame_id = global_frame_id_;
    msg->pose.pose.position.x = initial_pose_x_;
    msg->pose.pose.position.y = initial_pose_y_;
    msg->pose.pose.position.z = initial_pose_z_;
    msg->pose.pose.orientation.x = initial_pose_qx_;
    msg->pose.pose.orientation.y = initial_pose_qy_;
    msg->pose.pose.orientation.z = initial_pose_qz_;
    msg->pose.pose.orientation.w = initial_pose_qw_;
    initialPoseReceived(msg);
  }
}

void OdomNode::declare_parameters() {
  RCLCPP_INFO(get_logger(), "declare_parameters");
  declare_global_param();
  declare_odom_param();
  declare_extrinsics_param();
  declare_imu_param();
  declare_frame_param();
}
void OdomNode::initializeParameters() {
  RCLCPP_INFO(get_logger(), "initializeParameters");
  this->num_threads_ = omp_get_max_threads();
  this->dlio_initialized = false;
  this->first_valid_scan = false;
  this->first_imu_received_ = false;
  if (this->imu_calibrate_) {
    this->imu_calibrated = false;
  } else {
    this->imu_calibrated = true;
  }
  this->deskew_status = false;
  this->deskew_size = 0;
  scan_stamp = 0.0;
  prev_scan_stamp = 0.0;
  scan_dt = 0.1;
  last_save_stamp = get_clock()->now();
  //
  tf2::Quaternion qua;
  qua.setRPY(initial_pose_roll_, initial_pose_pitch_, initial_pose_yaw_);
  initial_pose_qx_ = qua.x();
  initial_pose_qy_ = qua.y();
  initial_pose_qz_ = qua.z();
  initial_pose_qw_ = qua.w();

  //
  this->T = Eigen::Matrix4f::Identity();
  this->T_prior = Eigen::Matrix4f::Identity();
  this->T_corr = Eigen::Matrix4f::Identity();
  this->T_corr_prev = Eigen::Matrix4f::Identity();

  this->origin = Eigen::Vector3f(0., 0., 0.);
  this->state.p = Eigen::Vector3f(0., 0., 0.);
  this->state.q = Eigen::Quaternionf(1., 0., 0., 0.);
  this->state.v.lin.b = Eigen::Vector3f(0., 0., 0.);
  this->state.v.lin.w = Eigen::Vector3f(0., 0., 0.);
  this->state.v.ang.b = Eigen::Vector3f(0., 0., 0.);
  this->state.v.ang.w = Eigen::Vector3f(0., 0., 0.);
  this->lidarPose.p = Eigen::Vector3f(0., 0., 0.);
  this->lidarPose.q = Eigen::Quaternionf(1., 0., 0., 0.);

  this->imu_meas.stamp = 0.;
  this->imu_meas.ang_vel[0] = 0.;
  this->imu_meas.ang_vel[1] = 0.;
  this->imu_meas.ang_vel[2] = 0.;
  this->imu_meas.lin_accel[0] = 0.;
  this->imu_meas.lin_accel[1] = 0.;
  this->imu_meas.lin_accel[2] = 0.;

  this->imu_buffer.set_capacity(this->imu_buffer_size_);
  this->first_imu_stamp = 0.;
  this->prev_imu_stamp = 0.;

  this->original_scan = std::make_shared<const pcl::PointCloud<PointType>>();
  this->deskewed_scan = std::make_shared<const pcl::PointCloud<PointType>>();
  this->current_scan = std::make_shared<const pcl::PointCloud<PointType>>();

  this->first_scan_stamp = 0.;
  this->elapsed_time = 0.;

  this->gicp.setCorrespondenceRandomness(this->gicp_k_correspondences_);
  this->gicp.setMaxCorrespondenceDistance(this->gicp_max_corr_dist_);
  this->gicp.setMaximumIterations(this->gicp_max_iter_);
  this->gicp.setTransformationEpsilon(this->gicp_transformation_ep_);
  this->gicp.setRotationEpsilon(this->gicp_rotation_ep_);
  this->gicp.setInitialLambdaFactor(this->gicp_init_lambda_factor_);

  pcl::Registration<PointType, PointType>::KdTreeReciprocalPtr temp;
  this->gicp.setSearchMethodSource(temp, true);
  this->gicp.setSearchMethodTarget(temp, true);
  this->geo.first_opt_done = false;
  this->geo.prev_vel = Eigen::Vector3f(0., 0., 0.);
  pcl::console::setVerbosityLevel(pcl::console::L_ERROR);
  this->crop.setNegative(true);
  this->crop.setMin(Eigen::Vector4f(-this->crop_size_, -this->crop_size_,
                                    -this->crop_size_, 1.0));
  this->crop.setMax(Eigen::Vector4f(this->crop_size_, this->crop_size_,
                                    this->crop_size_, 1.0));

  this->voxel.setLeafSize(this->vf_res_, this->vf_res_, this->vf_res_);
  this->metrics.spaciousness.push_back(0.);
  this->metrics.density.push_back(this->gicp_max_corr_dist_);

}

void OdomNode::loadMap() {
  // this->global_map_.reset(new pcl::PointCloud<PointType>());
  // this->global_map_->clear();
  this->global_map_ = pcl::PointCloud<PointType>::Ptr(
                  new pcl::PointCloud<PointType>());
  pcl::io::loadPCDFile(map_path_, *global_map_);

  this->gicp.setInputTarget(this->global_map_);
  this->global_map_cov = this->gicp.getTargetCovariances();
  this->gicp.registerInputTarget(this->global_map_);
  RCLCPP_INFO(get_logger(), "***************map size: %lu  *************\n",
              global_map_->size());
  sensor_msgs::msg::PointCloud2::SharedPtr map_msg_ptr(
      new sensor_msgs::msg::PointCloud2);
  pcl::toROSMsg(*global_map_, *map_msg_ptr);
  map_msg_ptr->header.frame_id = this->global_frame_id_;
  initial_map_pub_->publish(*map_msg_ptr);
  RCLCPP_INFO(get_logger(), "Initil Map Published");

  map_recieved_ = true;
}

void OdomNode::loadPoseFromServer() {
  try {
    YAML::Node doc = YAML::LoadFile(init_pose_path_);
    initial_pose_x_ = yaml_get_value<double>(doc, "initial_pose_x");
    initial_pose_y_ = yaml_get_value<double>(doc, "initial_pose_y");
    initial_pose_yaw_ = yaml_get_value<double>(doc, "initial_pose_yaw");
    tf2::Quaternion qua;
    qua.setRPY(0.0, 0.0, initial_pose_yaw_);
    initial_pose_qx_ = qua.x();
    initial_pose_qy_ = qua.y();
    initial_pose_qz_ = qua.z();
    initial_pose_qw_ = qua.w();
    set_initial_pose_ = true;
  } catch (YAML::ParserException& e) {
    RCLCPP_ERROR(get_logger(), "Failed to parse YAML for reason: %s",
                 e.msg.c_str());
  } catch (YAML::BadFile& e) {
    RCLCPP_ERROR(get_logger(), "Failed to load YAML for reason: %s",
                 e.msg.c_str());
  } catch (YAML::Exception& e) {
    RCLCPP_ERROR(get_logger(), "Failed to exeption YAML for reason: %s",
                 e.msg.c_str());
  }
}
void OdomNode::savePoseToServer(const geometry_msgs::msg::Pose2D& pose) {
  if (abs(pose.x - last_save_pose.x) > 0.06 ||
      abs(pose.y - last_save_pose.y) > 0.06 ||
      abs(pose.theta - last_save_pose.theta) > 0.06 ||
      (get_clock()->now() - last_save_stamp) >
          rclcpp::Duration::from_seconds(10.0)) {
    YAML::Emitter e;
    e << YAML::Precision(5);
    e << YAML::BeginMap;
    e << YAML::Key << "initial_pose_x" << YAML::Value << pose.x;
    e << YAML::Key << "initial_pose_y" << YAML::Value << pose.y;
    e << YAML::Key << "initial_pose_yaw" << YAML::Value << pose.theta;
    if (!e.good()) {
      RCLCPP_WARN_STREAM(get_logger(), "YAML writer failed with an error "
                                           << e.GetLastError()
                                           << ". The pose may be invalid.");
    }
    std::ofstream(init_pose_path_) << e.c_str();
    last_save_pose = pose;
    last_save_stamp = get_clock()->now();
  }
}

void OdomNode::createPublishers() {
  pose_pub_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "pcl_pose", 1);

  // path_pub_ = create_publisher<nav_msgs::msg::Path>("path", 1);

  initial_map_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      "initial_map",
      rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());

  // aligned_pub_ =
  //     create_publisher<sensor_msgs::msg::PointCloud2>("aligned_points", 1);
}
void OdomNode::createSubscribers() {
  this->lidar_cb_group =
      this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  auto lidar_sub_opt = rclcpp::SubscriptionOptions();
  lidar_sub_opt.callback_group = this->lidar_cb_group;
  this->imu_cb_group =
      this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  auto imu_sub_opt = rclcpp::SubscriptionOptions();
  imu_sub_opt.callback_group = this->imu_cb_group;
  this->initial_pose_cb_group =
      this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  auto initial_pose_sub_opt = rclcpp::SubscriptionOptions();
  initial_pose_sub_opt.callback_group = this->initial_pose_cb_group;
  this->map_cb_group =
      this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  auto map_sub_opt = rclcpp::SubscriptionOptions();
  map_sub_opt.callback_group = this->map_cb_group;

  initial_pose_sub_ =
      create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
          "initialpose", rclcpp::SystemDefaultsQoS(),
          std::bind(&OdomNode::initialPoseReceived, this,
                    std::placeholders::_1),
          initial_pose_sub_opt);

  map_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      "map", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(),
      std::bind(&OdomNode::mapReceived, this, std::placeholders::_1),
      map_sub_opt);

  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "odom", rclcpp::SensorDataQoS(),
      std::bind(&OdomNode::odomReceived, this, std::placeholders::_1));

  cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      "cloud", rclcpp::SensorDataQoS(),
      std::bind(&OdomNode::cloudReceived, this, std::placeholders::_1),
      lidar_sub_opt);

  imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      "imu", rclcpp::SensorDataQoS(),
      std::bind(&OdomNode::imuReceived, this, std::placeholders::_1),
      imu_sub_opt);
}

void OdomNode::initializePubSub() {
  RCLCPP_INFO(get_logger(), "initializePubSub");
  createPublishers();
  createSubscribers();
  this->broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(*this);
  this->publish_timer = this->create_wall_timer(std::chrono::duration<double>(0.02), 
      std::bind(&OdomNode::publishPoseThread, this));
}

void OdomNode::initialization() {
  declare_parameters();
  initializeParameters();
  initializePubSub();
  loadMap();
  loadPoseFromServer();
  setInitialPose();
  hv::async::startup();
  RCLCPP_INFO(get_logger(), "initialized!!!");
}

void OdomNode::resetImu() {
  this->first_imu_received_ = false;
  this->first_imu_stamp = 0.;
  this->imu_calibrated = false;
  this->calibrate_accel_ = false;
  this->calibrate_gyro_ = false;
  RCLCPP_INFO(get_logger(), "resetImu...");
}

void OdomNode::setInputSource() {
  this->gicp.setInputSource(this->current_scan);
  this->gicp.calculateSourceCovariances();
}

void OdomNode::propagateState() {
  // Lock thread to prevent state from being accessed by UpdateState
  std::lock_guard<std::mutex> lock(this->geo.mtx);
  // double dt = this->imu_meas.dt;
  double dt = this->imu_meas.dt > 0.012f ? 0.01f : this->imu_meas.dt;
  // RCLCPP_INFO("dt in propagateState: %.8f", dt);
  Eigen::Quaternionf qhat = this->state.q, omega;
  Eigen::Vector3f world_accel;

  // Transform accel from body to world frame
  // 这个imu_meas是否是imu转到lidar下的？
  world_accel = qhat._transformVector(this->imu_meas.lin_accel);

  // Accel propogation
  // float px = this->state.v.lin.w[0] * dt + 0.5 * dt * dt * world_accel[0];
  // float py = this->state.v.lin.w[1] * dt + 0.5 * dt * dt * world_accel[1];
  // float pz = this->state.v.lin.w[2] * dt +  0.5 * dt * dt * (world_accel[2] - this->gravity_);
  // this->state.p[0] +=
  //     px;
  // this->state.p[1] +=
  //     py;
  // this->state.p[2] += 
  //     pz;

  this->state.p[0] +=
      this->state.v.lin.w[0] * dt + 0.5 * dt * dt * world_accel[0];
  this->state.p[1] +=
      this->state.v.lin.w[1] * dt + 0.5 * dt * dt * world_accel[1];
  this->state.p[2] += 
      this->state.v.lin.w[2] * dt +  0.5 * dt * dt * (world_accel[2] - this->gravity_);
  // RCLCPP_INFO(this->get_logger(), "px, py, pz, dt in propagateState:[%.8f,%.8f,%.8f,%.8f]", 
    // this->state.p[0], this->state.p[1], this->state.p[2], dt);
  this->state.v.lin.w[0] += world_accel[0] * dt;
  this->state.v.lin.w[1] += world_accel[1] * dt;
  this->state.v.lin.w[2] += (world_accel[2] - this->gravity_) * dt;
  this->state.v.lin.b =
      this->state.q.toRotationMatrix().inverse() * this->state.v.lin.w;
  // Gyro propogation
  omega.w() = 0;
  omega.vec() = this->imu_meas.ang_vel;
  Eigen::Quaternionf tmp = qhat * omega;
  this->state.q.w() += 0.5 * dt * tmp.w();
  this->state.q.vec() += 0.5 * dt * tmp.vec();

  // Ensure quaternion is properly normalized
  this->state.q.normalize();

  this->state.v.ang.b = this->imu_meas.ang_vel;
  this->state.v.ang.w = this->state.q.toRotationMatrix() * this->state.v.ang.b;
}

void OdomNode::deskewPointcloud() {
  pcl::PointCloud<PointType>::Ptr deskewed_scan_ =
      std::make_shared<pcl::PointCloud<PointType>>();
  deskewed_scan_->points.resize(this->original_scan->points.size());

  // individual point timestamps should be relative to this time
  sweep_ref_time = rclcpp::Time(this->scan_header_stamp).seconds();
  // sort points by timestamp and build list of timestamps
  std::function<bool(const PointType&, const PointType&)> point_time_cmp;
  std::function<bool(boost::range::index_value<PointType&, long>,
                     boost::range::index_value<PointType&, long>)>
      point_time_neq;
  std::function<double(boost::range::index_value<PointType&, long>)>
      extract_point_time;
  pointTimeCallback(point_time_cmp, point_time_neq, extract_point_time);
  // copy points into deskewed_scan_ in order of timestamp
  std::partial_sort_copy(this->original_scan->points.begin(),
                         this->original_scan->points.end(),
                         deskewed_scan_->points.begin(),
                         deskewed_scan_->points.end(), point_time_cmp);

  // filter unique timestamps
  auto points_unique_timestamps =
      deskewed_scan_->points | boost::adaptors::indexed() |
      boost::adaptors::adjacent_filtered(point_time_neq);

  // extract timestamps from points and put them in their own list
  std::vector<double> timestamps;
  std::vector<int> unique_time_indices;
  for (auto it = points_unique_timestamps.begin();
       it != points_unique_timestamps.end(); it++) {
    timestamps.push_back(extract_point_time(*it));
    unique_time_indices.push_back(it->index());
  }
  unique_time_indices.push_back(deskewed_scan_->points.size());

  int median_pt_index = timestamps.size() / 2;
  this->scan_stamp =
      timestamps[median_pt_index];  // set this->scan_stamp to the timestamp of
                                    // the median point

  // don't process scans until IMU data is present
  if (!this->first_valid_scan) {
    if (this->imu_buffer.empty() ||
        this->scan_stamp <= this->imu_buffer.back().stamp) {
      RCLCPP_WARN_ONCE(get_logger(), "Waiting First Scan Valid....");
      return;
    }

    RCLCPP_INFO(get_logger(), "First Scan is Valid");
    this->first_valid_scan = true;
    this->T_prior = this->T;  // assume no motion for the first scan
    pcl::transformPointCloud(*deskewed_scan_, *deskewed_scan_,
                             this->T_prior * this->extrinsics.baselink2lidar_T);
    this->deskewed_scan = deskewed_scan_;
    this->deskew_status = true;
    return;
  }

  // IMU prior & deskewing for second scan onwards
  std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>>
      frames;
  frames = this->integrateImu(this->prev_scan_stamp, this->lidarPose.q,
                              this->lidarPose.p,
                              this->geo.prev_vel.cast<float>(), timestamps);
  // if there are no frames between the start and end of the sweep
  // that probably means that there's a sync issue
  if (frames.size() != timestamps.size()) {
    RCLCPP_FATAL(this->get_logger(), "Bad time sync between LiDAR and IMU! frames: %d, timestamp: %d",
       frames.size(), timestamps.size());    
    this->T_prior = this->T;
    pcl::transformPointCloud(*deskewed_scan_, *deskewed_scan_,
                             this->T_prior * this->extrinsics.baselink2lidar_T);
    this->deskewed_scan = deskewed_scan_;
    this->deskew_status = false;
    return;
  }

  // update prior to be the estimated pose at the median time of the scan
  // (corresponds to this->scan_stamp)
  this->T_prior = frames[median_pt_index];

#pragma omp parallel for num_threads(this->num_threads_)
  for (int i = 0; i < timestamps.size(); i++) {
    Eigen::Matrix4f T = frames[i] * this->extrinsics.baselink2lidar_T;
    // transform point to world frame
    for (int k = unique_time_indices[i]; k < unique_time_indices[i + 1]; k++) {
      auto& pt = deskewed_scan_->points[k];
      pt.getVector4fMap()[3] = 1.;
      pt.getVector4fMap() = T * pt.getVector4fMap();
    }
  }
  this->deskewed_scan = deskewed_scan_;
  this->deskew_status = true;
}

void OdomNode::computeMetrics() {
  this->computeSpaciousness();
  this->computeDensity();
}

void OdomNode::computeSpaciousness() {
  // compute range of points
  std::vector<float> ds;

  for (int i = 0; i <= this->original_scan->points.size(); i++) {
    float d = std::sqrt(pow(this->original_scan->points[i].x, 2) +
                        pow(this->original_scan->points[i].y, 2));
    ds.push_back(d);
  }

  // median
  std::nth_element(ds.begin(), ds.begin() + ds.size() / 2, ds.end());
  float median_curr = ds[ds.size() / 2];
  static float median_prev = median_curr;
  float median_lpf = 0.95 * median_prev + 0.05 * median_curr;
  median_prev = median_lpf;

  // push
  this->metrics.spaciousness.push_back(median_lpf);
}

void OdomNode::computeDensity() {
  float density;

  if (!this->geo.first_opt_done) {
    density = 0.;
  } else {
    density = this->gicp.source_density_;
  }

  static float density_prev = density;
  float density_lpf = 0.95 * density_prev + 0.05 * density;
  density_prev = density_lpf;

  this->metrics.density.push_back(density_lpf);
}

void OdomNode::propagateGICP() {
  this->lidarPose.p << this->T(0, 3), this->T(1, 3), this->T(2, 3);

  Eigen::Matrix3f rotSO3;
  rotSO3 << this->T(0, 0), this->T(0, 1), this->T(0, 2), this->T(1, 0),
      this->T(1, 1), this->T(1, 2), this->T(2, 0), this->T(2, 1), this->T(2, 2);

  Eigen::Quaternionf q(rotSO3);

  // Normalize quaternion
  double norm =
      sqrt(q.w() * q.w() + q.x() * q.x() + q.y() * q.y() + q.z() * q.z());
  q.w() /= norm;
  q.x() /= norm;
  q.y() /= norm;
  q.z() /= norm;
  this->lidarPose.q = q;
}

void OdomNode::updateState() {
  // Lock thread to prevent state from being accessed by PropagateState
  std::lock_guard<std::mutex> lock(this->geo.mtx);

  Eigen::Vector3f pin = this->lidarPose.p;
  Eigen::Quaternionf qin = this->lidarPose.q;
  double dt = this->scan_stamp - this->prev_scan_stamp;
  if (dt < 1e-6) {
    RCLCPP_ERROR(get_logger(), "scan dt is invalid: %f", dt);
    return;
  }
  if (dt > 0.065) {
    RCLCPP_INFO(get_logger(), "scan dt is too large: %f", dt);
    dt = 0.05;
  }
  Eigen::Quaternionf qe, qhat, qcorr;
  qhat = this->state.q;

  // Constuct error quaternion
  qe = qhat.conjugate() * qin;

  double sgn = 1.;
  if (qe.w() < 0) {
    sgn = -1;
  }

  // Construct quaternion correction
  qcorr.w() = 1 - abs(qe.w());
  qcorr.vec() = sgn * qe.vec();
  qcorr = qhat * qcorr;

  Eigen::Vector3f err = pin - this->state.p;
  Eigen::Vector3f err_body;

  err_body = qhat.conjugate()._transformVector(err);

  double abias_max = this->geo_abias_max_;
  double gbias_max = this->geo_gbias_max_;

  // Update accel bias
  this->state.b.accel -= dt * this->geo_Kab_ * err_body;
  this->state.b.accel =
      this->state.b.accel.array().min(abias_max).max(-abias_max);

  // Update gyro bias
  this->state.b.gyro[0] -= dt * this->geo_Kgb_ * qe.w() * qe.x();
  this->state.b.gyro[1] -= dt * this->geo_Kgb_ * qe.w() * qe.y();
  this->state.b.gyro[2] -= dt * this->geo_Kgb_ * qe.w() * qe.z();
  this->state.b.gyro =
      this->state.b.gyro.array().min(gbias_max).max(-gbias_max);

  // Update state
  this->state.p += dt * this->geo_Kp_ * err;
  this->state.v.lin.w += dt * this->geo_Kv_ * err;
  this->state.q.w() += dt * this->geo_Kq_ * qcorr.w();
  this->state.q.x() += dt * this->geo_Kq_ * qcorr.x();
  this->state.q.y() += dt * this->geo_Kq_ * qcorr.y();
  this->state.q.z() += dt * this->geo_Kq_ * qcorr.z();
  this->state.q.normalize();

  // store previous pose, orientation, and velocity
  this->geo.prev_p = this->state.p;
  this->geo.prev_q = this->state.q;
  this->geo.prev_vel = this->state.v.lin.w;

  this->geo.first_opt_done = true;
}

void OdomNode::getNextPose() {
  // rclcpp::Time start = this->now();
  
  pcl::PointCloud<PointType>::Ptr aligned =
      std::make_shared<pcl::PointCloud<PointType>>();
  this->gicp.align(*aligned);
  aligned->header.frame_id = this->global_frame_id_;
  // hv::async(std::bind(&OdomNode::publishCloud, this, aligned));
  this->T_corr = this->gicp.getFinalTransformation();
  // this->T = this->T_corr * this->T_prior;
  
  // rclcpp::Time end = this->now();
  // rclcpp::Duration duration = end - start;
  // RCLCPP_INFO(this->get_logger(), "*****Time interval: %.6f s",
  //             duration.seconds());
  this->gicp_hasConverged = this->gicp.hasConverged();

  // //在这里添加发布对齐之后的点云
  if (!this->gicp_hasConverged) {
     RCLCPP_WARN(this->get_logger(), "The registration didn't converge.");
     // return;
     this->T = this->T_corr_prev * this->T_prior;
  } else {
     this->geo.first_opt_done = true;
     this->T = this->T_corr * this->T_prior;
     this->T_corr_prev = this->T_corr;
    //  RCLCPP_INFO(get_logger(),"after lidar propogate: x %.8f, y %.8f, z %.8f", 
    //     this->T(0,3), this->T(1,3), this->T(2,3));
  }

  this->propagateGICP();

  this->updateState();
}
