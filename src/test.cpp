
#include <vector>
#include <random>
#include <chrono>

#include <ur_kinematics/ur_moveit_plugin.h>
#include <ur_kinematics/ur_kin.h>

#include <Eigen/Geometry>

geometry_msgs::Pose convertPose(const Eigen::Isometry3d& tf)
{
  Eigen::Vector3d p = tf.translation();
  Eigen::Quaterniond R{tf.rotation()};

  geometry_msgs::Pose pose;

  pose.position.x = p.x();
  pose.position.y = p.y();
  pose.position.z = p.z();

  pose.orientation.w = R.w();
  pose.orientation.x = R.x();
  pose.orientation.y = R.y();
  pose.orientation.z = R.z();

  return pose;
}

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "universal_robot_test");

  ros::NodeHandle nh;

  ur_kinematics::URKinematicsPlugin test;
  test.initialize("/robot_description", "manipulator",
                  "base_link", "ee_link", 0.1);

  std::vector<double> q;
  q.assign(6, 0.0);

  Eigen::Isometry3d tf_orig = Eigen::Isometry3d::Identity();
  tf_orig.translation() = Eigen::Vector3d(0.30, 0.18, 0.18);
  tf_orig.rotate(Eigen::Quaterniond(0.0, 0.0, 1.0/std::sqrt(2.0), 1.0/std::sqrt(2.0)));

  std::vector<double> solution;
  moveit_msgs::MoveItErrorCodes error_code;

  std::random_device rd;
  std::mt19937 mt(rd());
  std::uniform_real_distribution<double> dist(-0.01, 0.01);

  // Pre-generate randomized values
  const std::size_t N = 100000;
  std::array<std::array<double, 3>, N> randomized_values;
  for(std::size_t i=0; i < N; ++i)
  {
    for(std::size_t j=0; j < 3; ++j)
    {
      randomized_values[i][j] = dist(mt);
    }
  }

  bool solved = false;
  ROS_INFO(" ==== Beginning performance test iterations ==== ");
  auto start = std::chrono::high_resolution_clock::now();
  for(size_t i=0; i < N; ++i)
  {
    Eigen::Isometry3d tf = tf_orig;
    tf.translation().x() += randomized_values[i][0];
    tf.translation().y() += randomized_values[i][1];
    tf.translation().z() += randomized_values[i][2];

    geometry_msgs::Pose pose = convertPose(tf);

    solved = test.getPositionIK(pose, q, solution, error_code);
  }
  auto end = std::chrono::high_resolution_clock::now();

  std::chrono::duration<double> diff = end-start;
  ROS_INFO_STREAM("Average time per query: " << diff.count()/N * 1e6 << " us");
}
