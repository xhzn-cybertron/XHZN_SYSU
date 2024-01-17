#include <dlio/odom.h>
using namespace dlio;
void OdomNode::initialPoseReceived(
    geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
  RCLCPP_INFO(get_logger(), "initialPoseReceived");
  if (msg->header.frame_id != this->global_frame_id_) {
    RCLCPP_WARN(this->get_logger(),
                "initialpose_frame_id does not match　global_frame_id");
    return;
  }

  initialpose_recieved_ = true;
  geometry_msgs::msg::PoseWithCovarianceStamped msg_pose = *msg;
  pose_pub_->publish(msg_pose);

  Eigen::Quaternionf q_init(msg_pose.pose.pose.orientation.w,
                            msg_pose.pose.pose.orientation.x,
                            msg_pose.pose.pose.orientation.y,
                            msg_pose.pose.pose.orientation.z);
  q_init.normalize();

  this->T_init.setIdentity();
  this->T_init.block<3, 3>(0, 0) = q_init.toRotationMatrix();
  this->T_init.block<3, 1>(0, 3) =
      Eigen::Vector3f(msg_pose.pose.pose.position.x,
                      msg_pose.pose.pose.position.y,
                      msg_pose.pose.pose.position.z);
  // pose_pub_->publish(corrent_pose_stamped_);

  this->first_valid_scan = false;
  //让激光的回调去执行首祯配准 直接将当前的定位结果改成这个初始位姿
  this->T = this->T_init;
  this->T_prior = this->T_init;
  this->geo.mtx.lock();
  this->lidarPose.q = Eigen::Quaternionf(this->T_init.block<3, 3>(0, 0));
  this->lidarPose.p = Eigen::Vector3f(this->T_init.block<3, 1>(0, 3));
  // this->lidarPose = this->T_init;
  //如何清空geo的状态？
  this->geo.first_opt_done = false;
  //状态重传播
  this->state.q = lidarPose.q;
  this->state.p = lidarPose.p;
  this->state.q.normalize();
  // this->state.p.v = lidarPose.p;
  this->state.v.lin.b = Eigen::Vector3f(0., 0., 0.);
  this->state.v.lin.w = Eigen::Vector3f(0., 0., 0.);
  this->state.v.ang.b = Eigen::Vector3f(0., 0., 0.);
  this->state.v.ang.w = Eigen::Vector3f(0., 0., 0.);
  this->geo.mtx.unlock();
  //速度和角速度也要乘坐标变换吧？
  // this->state.v = this->state
  this->resetImu();
}

void OdomNode::mapReceived(sensor_msgs::msg::PointCloud2::SharedPtr msg) {
  RCLCPP_INFO(get_logger(), "mapReceived");
  // pcl::PointCloud<PointType>::Ptr map_cloud_ptr(new
  // pcl::PointCloud<PointType>);

  if (msg->header.frame_id != global_frame_id_) {
    RCLCPP_WARN(this->get_logger(),
                "map_frame_id does not match　global_frame_id");
    return;
  }

  map_recieved_ = true;
}

void OdomNode::odomReceived(nav_msgs::msg::Odometry::ConstSharedPtr msg) {
  if (!use_odom_) {
    return;
  }
  RCLCPP_INFO(get_logger(), "odomReceived");

  double current_odom_received_time =
      msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;
  double dt_odom = current_odom_received_time - last_odom_received_time_;
  last_odom_received_time_ = current_odom_received_time;
  if (dt_odom > 1.0 /* [sec] */) {
    RCLCPP_WARN(this->get_logger(), "odom time interval is too large");
    return;
  }
  if (dt_odom < 0.0 /* [sec] */) {
    RCLCPP_WARN(this->get_logger(), "odom time interval is negative");
    return;
  }

  tf2::Quaternion previous_quat_tf;
  double roll, pitch, yaw;
  tf2::fromMsg(corrent_pose_stamped_.pose.pose.orientation, previous_quat_tf);
  tf2::Matrix3x3(previous_quat_tf).getRPY(roll, pitch, yaw);

  //-------因此，当前地roll,pitch,yaw是相对于开始时刻的，也就是相对于map坐标系下的
  roll += msg->twist.twist.angular.x * dt_odom;
  pitch += msg->twist.twist.angular.y * dt_odom;
  yaw += msg->twist.twist.angular.z * dt_odom;

  //使用欧拉角转换为四元数
  Eigen::Quaterniond quat_eig =
      Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()) *
      Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ());

  geometry_msgs::msg::Quaternion quat_msg = tf2::toMsg(quat_eig);

  Eigen::Vector3d odom{msg->twist.twist.linear.x, msg->twist.twist.linear.y,
                       msg->twist.twist.linear.z};

  //旋转*平移 得到平移增量的适量矢量(时间上正确吗？不应该用中值吗？)
  Eigen::Vector3d delta_position = quat_eig.matrix() * dt_odom * odom;

  //加上增量，得到当前的position
  corrent_pose_stamped_.pose.pose.position.x += delta_position.x();
  corrent_pose_stamped_.pose.pose.position.y += delta_position.y();
  corrent_pose_stamped_.pose.pose.position.z += delta_position.z();
  corrent_pose_stamped_.pose.pose.orientation = quat_msg;
}

void OdomNode::imuReceived(sensor_msgs::msg::Imu::ConstSharedPtr msg) {
  this->first_imu_received_ = true;
  sensor_msgs::msg::Imu::SharedPtr imu = transformImu(msg);
  imu_stamp = imu->header.stamp;
  double imu_stamp_secs = rclcpp::Time(imu->header.stamp).seconds();

  Eigen::Vector3f lin_accel;
  Eigen::Vector3f ang_vel;

  // Get IMU samples
  ang_vel[0] = imu->angular_velocity.x;
  ang_vel[1] = imu->angular_velocity.y;
  ang_vel[2] = imu->angular_velocity.z;

  lin_accel[0] = imu->linear_acceleration.x;
  lin_accel[1] = imu->linear_acceleration.y;
  lin_accel[2] = imu->linear_acceleration.z;

  if (this->first_imu_stamp == 0.) {
    this->first_imu_stamp = imu_stamp_secs;
  }

  // IMU calibration procedure - do for three seconds
  if (!this->imu_calibrated) {
    static int num_samples = 0;
    static Eigen::Vector3f gyro_avg(0., 0., 0.);
    static Eigen::Vector3f accel_avg(0., 0., 0.);
    static bool print = true;

    if ((imu_stamp_secs - this->first_imu_stamp) < this->imu_calib_time_) {
      num_samples++;

      gyro_avg[0] += ang_vel[0];
      gyro_avg[1] += ang_vel[1];
      gyro_avg[2] += ang_vel[2];

      accel_avg[0] += lin_accel[0];
      accel_avg[1] += lin_accel[1];
      accel_avg[2] += lin_accel[2];

      if (print) {
        std::cout << std::endl
                  << " Calibrating IMU for " << this->imu_calib_time_
                  << " seconds... ";
        std::cout.flush();
        print = false;
      }

    } else {
      std::cout << "done" << std::endl << std::endl;

      gyro_avg /= num_samples;
      accel_avg /= num_samples;

      Eigen::Vector3f grav_vec(0., 0., this->gravity_);

      if (this->calibrate_accel_) {
        // subtract gravity from avg accel to get bias
        this->state.b.accel = accel_avg - grav_vec;

        std::cout << " Accel biases [xyz]: "
                  << to_string_with_precision(this->state.b.accel[0], 8) << ", "
                  << to_string_with_precision(this->state.b.accel[1], 8) << ", "
                  << to_string_with_precision(this->state.b.accel[2], 8)
                  << std::endl;
      }

      if (this->calibrate_gyro_) {
        this->state.b.gyro = gyro_avg;

        std::cout << " Gyro biases  [xyz]: "
                  << to_string_with_precision(this->state.b.gyro[0], 8) << ", "
                  << to_string_with_precision(this->state.b.gyro[1], 8) << ", "
                  << to_string_with_precision(this->state.b.gyro[2], 8)
                  << std::endl;
      }

      this->imu_calibrated = true;
    }

  } else {
    double dt = imu_stamp_secs - this->prev_imu_stamp;
    if (dt == 0) {
      dt = 1.0 / 200.0;
    }
    this->imu_rates.push_back(1. / dt);

    // Apply the calibrated bias to the new IMU measurements
    this->imu_meas.stamp = imu_stamp_secs;
    this->imu_meas.dt = dt;
    // RCLCPP_INFO(get_logger, "   imu dt is : %.8f", dt);
    this->prev_imu_stamp = this->imu_meas.stamp;

    Eigen::Vector3f lin_accel_corrected =
        (this->imu_accel_sm_ * lin_accel) - this->state.b.accel;
    Eigen::Vector3f ang_vel_corrected = ang_vel - this->state.b.gyro;

    this->imu_meas.lin_accel = lin_accel_corrected;
    this->imu_meas.ang_vel = ang_vel_corrected;

    // Store calibrated IMU measurements into imu buffer for manual integration
    // later. std::cout << "imu_buffer->push_front a imu mess" << std::endl;
    this->mtx_imu.lock();
    this->imu_buffer.push_front(this->imu_meas);
    // std::cout << "****imu_buffer size:" << imu_buffer.size() << std::endl;
    this->mtx_imu.unlock();

    // Notify the callbackPointCloud thread that IMU data exists for this time
    this->cv_imu_stamp.notify_one();
    // RCLCPP_INFO(get_logger(), "notify the imu waiter...");

    //可以改成位姿是否初始化完成
    if (this->geo.first_opt_done) {
      // Geometric Observer: Propagate State
      //使用每祯IMU更新状态
      this->propagateState();

      // publishPoseThread();
    }
  }
}

// void OdomNode::publishCloud(
//     const pcl::PointCloud<PointType>::ConstPtr& published_cloud) {
//   sensor_msgs::msg::PointCloud2 pt_msg;
//   pcl::toROSMsg(*published_cloud, pt_msg);
//   this->aligned_pub_->publish(pt_msg);
// }

void OdomNode::publishPoseThread() {
  // rclcpp::Time start = this->now();

  geometry_msgs::msg::PoseWithCovarianceStamped pose_msg;
  pose_msg.header.stamp = this->imu_stamp;
  pose_msg.pose.pose.position.x =
      static_cast<double>(this->state.p.x());
  pose_msg.pose.pose.position.y =
      static_cast<double>(this->state.p.y());
  pose_msg.pose.pose.position.z =
      static_cast<double>(this->state.p.z());
  Eigen::Quaterniond quat_eig(this->state.q);
  geometry_msgs::msg::Quaternion quat_msg = tf2::toMsg(quat_eig);
  pose_msg.pose.pose.orientation = quat_msg;
  this->pose_pub_->publish(pose_msg);


  geometry_msgs::msg::Pose2D pose;
  pose.x = pose_msg.pose.pose.position.x;
  pose.y = pose_msg.pose.pose.position.y;
  pose.theta = tf2::getYaw(pose_msg.pose.pose.orientation);
  // RCLCPP_INFO(this->get_logger(), "******Output position: x: %.6f, y: %.6f, yaw: %.6f", 
  // pose.x,pose.y,pose.theta);
  savePoseToServer(pose);

  geometry_msgs::msg::TransformStamped transform_stamped;
  transform_stamped.header.stamp = this->imu_stamp;
  transform_stamped.header.frame_id = global_frame_id_;
  transform_stamped.child_frame_id = base_frame_id_;
  transform_stamped.transform.translation.x =
      static_cast<double>(this->state.p.x());
  transform_stamped.transform.translation.y =
      static_cast<double>(this->state.p.y());
  transform_stamped.transform.translation.z =
      static_cast<double>(this->state.p.z());
  transform_stamped.transform.rotation = quat_msg;
  this->broadcaster_->sendTransform(transform_stamped);

  // geometry_msgs::msg::PoseStamped curr_pose_stamped;
  // curr_pose_stamped.pose.orientation =
  //     this->corrent_pose_stamped_.pose.pose.orientation;
  // curr_pose_stamped.pose.position =
  //     this->corrent_pose_stamped_.pose.pose.position;
  // curr_pose_stamped.header = this->corrent_pose_stamped_.header;
  // this->path_.poses.push_back(curr_pose_stamped);
  // this->path_pub_->publish(path_);

  // rclcpp::Time end = this->now();
  // rclcpp::Duration duration = end - start;
  // RCLCPP_INFO(this->get_logger(), "*****PubPose time interval: %.8f s *****",
  //             duration.seconds());
}

void OdomNode::cloudReceived(
    sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
  if (!map_recieved_ || !initialpose_recieved_) {
    return;
  }
  //检查IMU是否完成初始化
  if (!this->first_imu_received_ || !this->imu_calibrated) {
    return;
  }
  //初始化第一帧雷达时间
  if (this->prev_scan_stamp == 0.) {
    this->prev_scan_stamp = rclcpp::Time(msg->header.stamp).seconds();
    //需要直接返回，保证后续的dt为非零值
    return;
  }
  //第一帧有效雷达时间
  if (this->first_scan_stamp == 0.) {
    this->first_scan_stamp = rclcpp::Time(msg->header.stamp).seconds();
  }
  // rclcpp::Time start = this->now();

  this->getScanFromROS(msg);
  //畸变补偿
  this->deskewPointcloud();
  //是否开启体素滤波
  if (this->vf_use_) {
    pcl::PointCloud<PointType>::Ptr current_scan_ =
        std::make_shared<pcl::PointCloud<PointType>>(*this->deskewed_scan);
    this->voxel.setInputCloud(current_scan_);
    this->voxel.filter(*current_scan_);
    this->current_scan = current_scan_;
  } else {
    this->current_scan = this->deskewed_scan;
  }

  //如果IMU数据始终晚于激光数据，则不进行配准
  if (!this->first_valid_scan) {
    return;
  }

  hv::async(std::bind(&OdomNode::computeMetrics, this));
  this->setInputSource();
  this->getNextPose();
  // RCLCPP_INFO_STREAM(get_logger(), "pose matrix carried out: \n " <<
  // to_string_with_precision(this->T));

  // Update time stamps
  this->lidar_rates.push_back(1. / (this->scan_stamp - this->prev_scan_stamp));
  this->prev_scan_stamp = this->scan_stamp;
  // this->elapsed_time = this->scan_stamp - this->first_scan_stamp;

  //  this->gicp_hasConverged = this->gicp.hasConverged();

  //  this->geo.first_opt_done = true;
  // // //在这里添加发布对齐之后的点云
  //  if (!this->gicp_hasConverged) {
  //    RCLCPP_WARN(this->get_logger(), "The registration didn't converge.");
  //    // return;
  //  }
  
  // hv::async(std::bind(&OdomNode::publishToROS, this, this->T, msg->header));
  // int* ptr = nullptr;
  // *ptr = 10;
  // rclcpp::Time end = this->now();
  // rclcpp::Duration duration = end - start;
  // RCLCPP_INFO(this->get_logger(), "*****Scan Time interval: %.6f s",
  //             duration.seconds());
}
