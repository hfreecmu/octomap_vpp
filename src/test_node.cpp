#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/cache.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/PointCloud2.h>
#include <octomap/OcTree.h>
#include <octomap/ColorOcTree.h>
#include <octomap_ros/conversions.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/message_filter.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
//#include <moveit_msgs/PlanningScene.h>
#include <sensor_msgs/CameraInfo.h>
#include <image_geometry/pinhole_camera_model.h>

#include <instance_segmentation_msgs/Detections.h>
#include <pointcloud_roi_msgs/PointcloudWithRoi.h>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_interface/planning_interface.h>

#include <eigen_conversions/eigen_msg.h>

#include <boost/thread/mutex.hpp>

#include "roioctree.h"
#include "roioctree_utils.h"

RoiOcTree testTree(0.02);
tf2_ros::Buffer tfBuffer(ros::Duration(30));
ros::Publisher octomapPub;
ros::Publisher inflatedOctomapPub;
ros::Publisher pcGlobalPub;
//ros::Publisher planningScenePub;

boost::mutex tree_mtx;

const double OCCUPANCY_THRESH = 0.7;
const double FREE_THRESH = 0.3;

const std::string MAP_FRAME = "world";
const std::string PC_TOPIC = "/camera/depth/points";
const std::string PC_GLOBAL = "/points_global";

/*void publishOctomapToPlanningScene(const octomap_msgs::Octomap &map_msg)
{
  moveit_msgs::PlanningScene scene;
  scene.world.octomap.header = map_msg.header;
  scene.world.octomap.octomap = map_msg;
  scene.world.octomap.octomap.id = "OcTree";
  scene.is_diff = true;
  planningScenePub.publish(scene);
}*/

void publishMap()
{
  octomap_msgs::Octomap map_msg;
  map_msg.header.frame_id = MAP_FRAME;
  map_msg.header.stamp = ros::Time::now();
  tree_mtx.lock();
  bool msg_generated = octomap_msgs::fullMapToMsg(testTree, map_msg);
  tree_mtx.unlock();
  if (msg_generated)
  {
    octomapPub.publish(map_msg);
    //publishOctomapToPlanningScene(map_msg);
  }

  //if (octomap_msgs::binaryMapToMsg(testTree, map_msg))
  //    octomapPub.publish(map_msg);

  tree_mtx.lock();
  ros::Time inflationStart = ros::Time::now();
  std::shared_ptr<InflatedRoiOcTree> inflatedTree = testTree.computeInflatedRois();
  ros::Time inflationDone = ros::Time::now();
  tree_mtx.unlock();

  ROS_INFO_STREAM("Time for inflation: " << inflationDone - inflationStart);
  if (inflatedTree != nullptr)
  {
    octomap_msgs::Octomap inflated_map;
    inflated_map.header.frame_id = MAP_FRAME;
    inflated_map.header.stamp = ros::Time::now();
    if (octomap_msgs::fullMapToMsg(*inflatedTree, inflated_map))
    {
      inflatedOctomapPub.publish(inflated_map);
    }
  }
}

void registerNewScan(const sensor_msgs::PointCloud2ConstPtr &pc_msg)
{
  ros::Time cbStartTime = ros::Time::now();
  geometry_msgs::TransformStamped pcFrameTf;

  try
  {
    pcFrameTf = tfBuffer.lookupTransform(MAP_FRAME, pc_msg->header.frame_id, pc_msg->header.stamp);
  }
  catch (const tf2::TransformException &e)
  {
    ROS_ERROR_STREAM("Couldn't find transform to map frame: " << e.what());
    return;
  }
  //ROS_INFO_STREAM("Transform for time " << pc_msg->header.stamp << " successful");

  const geometry_msgs::Vector3 &pcfOrig = pcFrameTf.transform.translation;
  octomap::point3d scan_orig(pcfOrig.x, pcfOrig.y, pcfOrig.z);

  ros::Time tfTime = ros::Time::now();

  sensor_msgs::PointCloud2 pc_glob;
  tf2::doTransform(*pc_msg, pc_glob, pcFrameTf);

  //pcGlobalPub.publish(pc_glob);

  ros::Time doTFTime = ros::Time::now();

  octomap::Pointcloud pc;
  octomap::pointCloud2ToOctomap(pc_glob, pc);

  ros::Time toOctoTime = ros::Time::now();

  tree_mtx.lock();
  testTree.insertPointCloud(pc, scan_orig);
  tree_mtx.unlock();

  ros::Time insertTime = ros::Time::now();

  ROS_INFO_STREAM("Timings - TF: " << tfTime - cbStartTime << "; doTF: " << doTFTime - tfTime << "; toOct: " << toOctoTime - doTFTime << "; insert: " << insertTime - toOctoTime);
}

void pointCloud2ToOctomapByIndices(const sensor_msgs::PointCloud2 &cloud, const std::unordered_set<size_t> &indices,  octomap::Pointcloud &inlierCloud, octomap::Pointcloud &outlierCloud)
{
   inlierCloud.reserve(indices.size());
   outlierCloud.reserve(cloud.data.size() / cloud.point_step - indices.size());

   sensor_msgs::PointCloud2ConstIterator<float> iter_x(cloud, "x");
   sensor_msgs::PointCloud2ConstIterator<float> iter_y(cloud, "y");
   sensor_msgs::PointCloud2ConstIterator<float> iter_z(cloud, "z");

   size_t invalid_points = 0;
   for (size_t i = 0; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z, ++i){
     // Check if the point is invalid
     if (std::isfinite (*iter_x) && std::isfinite (*iter_y) && std::isfinite (*iter_z))
     {
       if (indices.find(i) != indices.end())
         inlierCloud.push_back(*iter_x, *iter_y, *iter_z);
       else
         outlierCloud.push_back(*iter_x, *iter_y, *iter_z);
     }
     else
       invalid_points++;
   }
   //ROS_INFO_STREAM("Number of invalid points: " << invalid_points);
}

// indices must be ordered!
void pointCloud2ToOctomapByIndices(const sensor_msgs::PointCloud2 &cloud, const std::vector<uint32_t> &indices,  octomap::Pointcloud &inlierCloud, octomap::Pointcloud &outlierCloud)
{
   inlierCloud.reserve(indices.size());
   outlierCloud.reserve(cloud.data.size() / cloud.point_step - indices.size());

   sensor_msgs::PointCloud2ConstIterator<float> iter_x(cloud, "x");
   sensor_msgs::PointCloud2ConstIterator<float> iter_y(cloud, "y");
   sensor_msgs::PointCloud2ConstIterator<float> iter_z(cloud, "z");

   size_t invalid_points = 0;
   std::vector<uint32_t>::const_iterator it = indices.begin();
   for (uint32_t i = 0; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z, ++i){
     // Check if the point is invalid
     if (std::isfinite (*iter_x) && std::isfinite (*iter_y) && std::isfinite (*iter_z))
     {
       if (i == *it)
       {
         inlierCloud.push_back(*iter_x, *iter_y, *iter_z);
         it++;
       }
       else
       {
         outlierCloud.push_back(*iter_x, *iter_y, *iter_z);
       }
     }
     else
       invalid_points++;
   }
   //ROS_INFO_STREAM("Number of invalid points: " << invalid_points);
}

void registerRoiPCL(const pointcloud_roi_msgs::PointcloudWithRoi &roi)
{
  octomap::Pointcloud inlierCloud, outlierCloud;
  pointCloud2ToOctomapByIndices(roi.cloud, roi.roi_indices, inlierCloud, outlierCloud);
  //ROS_INFO_STREAM("Cloud sizes: " << inlierCloud.size() << ",  " << outlierCloud.size());

  tree_mtx.lock();
  testTree.insertRegionScan(inlierCloud, outlierCloud);
  tree_mtx.unlock();

  //std::vector<octomap::OcTreeKey> roi_keys = testTree.getRoiKeys();
  //ROS_INFO_STREAM("Found " << testTree.getRoiSize() << " ROI keys (" << testTree.getAddedRoiSize() << " added, " << testTree.getDeletedRoiSize() << " removed)");
}

void registerRoi(const sensor_msgs::PointCloud2ConstPtr &pc_msg, const instance_segmentation_msgs::DetectionsConstPtr &dets_msg)
{
  //ROS_INFO_STREAM("Register ROI called, " << dets_msg->detections.size() << " detections");
  std::unordered_set<size_t> inlier_indices;
  for (const auto &det : dets_msg->detections)
  {
    if (det.class_name != "capsicum") // not region of interest
      continue;

    for (int y = det.box.y1; y < det.box.y2; y++)
    {
      for (int x = det.box.x1; x < det.box.x2; x++)
      {
        if (det.mask.mask[(y - det.box.y1) * det.mask.width + (x - det.box.x1)])
        {
          inlier_indices.insert(y * pc_msg->width + x);
        }
      }
    }
  }

  //ROS_INFO_STREAM("Number of inliers: " << inlier_indices.size());

  octomap::Pointcloud inlierCloud, outlierCloud;
  pointCloud2ToOctomapByIndices(*pc_msg, inlier_indices, inlierCloud, outlierCloud);
  //ROS_INFO_STREAM("Cloud sizes: " << inlierCloud.size() << ",  " << outlierCloud.size());

  tree_mtx.lock();
  testTree.insertRegionScan(inlierCloud, outlierCloud);
  tree_mtx.unlock();
}

void testRayTrace(const octomap::point3d &orig, const octomap::point3d &end)
{
  octomap::KeyRay ray;
  testTree.computeRayKeys(orig, end, ray);

  for (const octomap::OcTreeKey &key : ray)
  {
    octomap::OcTreeNode *node = testTree.search(key);
    if (node != NULL)
    {
      double occ = node->getOccupancy();
    }

  }
}

robot_state::RobotStatePtr sampleNextRobotState(const robot_state::JointModelGroup *joint_model_group, const robot_state::RobotState &current_state)
{
  double maxValue = 0;
  robot_state::RobotStatePtr maxState(new robot_state::RobotState(current_state));
  robot_state::RobotStatePtr curSample(new robot_state::RobotState(current_state));
  for (size_t i = 0; i < 10; i++)
  {
    curSample->setToRandomPositionsNearBy(joint_model_group, current_state, 0.2);
    const Eigen::Affine3d& stateTf = curSample->getGlobalLinkTransform("camera_link");
    auto translation = stateTf.translation();
    auto quaternion = Eigen::Quaterniond(stateTf.linear()).coeffs();
    octomap::pose6d viewpoint(octomap::point3d(translation[0], translation[1], translation[2]), octomath::Quaternion(quaternion[0], quaternion[1], quaternion[2], quaternion[3]));
    tree_mtx.lock();
    double curValue = testTree.computeViewpointValue(viewpoint, 80.0 * M_PI / 180.0, 8, 6, 5.0);
    tree_mtx.unlock();
    if (curValue > maxValue)
    {
      *maxState = *curSample;
      maxValue = curValue;
    }
  }
  return maxState;
}

bool moveToState(moveit::planning_interface::MoveGroupInterface &manipulator_group, const robot_state::RobotState &goal_state)
{
  manipulator_group.setJointValueTarget(goal_state);

  moveit::planning_interface::MoveGroupInterface::Plan plan;
  bool success = (manipulator_group.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

  if (success)
  {
    manipulator_group.asyncExecute(plan);
  }
  else
  {
    ROS_INFO("Can't execute plan");
  }
  return success;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "test_node");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(4);
  spinner.start();
  tf2_ros::TransformListener tfListener(tfBuffer);

  octomapPub = nh.advertise<octomap_msgs::Octomap>("octomap", 1);
  inflatedOctomapPub = nh.advertise<octomap_msgs::Octomap>("inflated_octomap", 1);
  //pcGlobalPub = nh.advertise<sensor_msgs::PointCloud2>(PC_GLOBAL, 1);
  //planningScenePub = nh.advertise<moveit_msgs::PlanningScene>("/planning_scene", 1);

  message_filters::Subscriber<sensor_msgs::PointCloud2> depthCloudSub(nh, PC_TOPIC, 1);
  //tf2_ros::MessageFilter<sensor_msgs::PointCloud2> tfCloudFilter(depthCloudSub, tfBuffer, MAP_FRAME, 1, nh);
  //message_filters::Cache<sensor_msgs::PointCloud2> cloudCache(tfCloudFilter, 1);

  depthCloudSub.registerCallback(registerNewScan);
  //tfCloudFilter.registerCallback(registerNewScan);
  //cloudCache.registerCallback(registerNewScan);

  //message_filters::Subscriber<sensor_msgs::PointCloud2> pcGlobalSub(nh, PC_GLOBAL, 1);
  //message_filters::Subscriber<instance_segmentation_msgs::Detections> detectionsSub(nh, "/mask_rcnn/detections", 1);

  //typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, instance_segmentation_msgs::Detections> DetsSyncPolicy;
  //message_filters::Synchronizer<DetsSyncPolicy> syncDets(DetsSyncPolicy(50), pcGlobalSub, detectionsSub);

  //syncDets.registerCallback(registerRoi);

  ros::Subscriber roiSub = nh.subscribe("/pointcloud_roi", 1, registerRoiPCL);

  moveit::planning_interface::MoveGroupInterface manipulator_group("manipulator");

  robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
  robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
  const robot_state::JointModelGroup* joint_model_group = kinematic_model->getJointModelGroup("manipulator");
  robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(kinematic_model));

  for (ros::Rate rate(1); ros::ok(); rate.sleep())
  {
    /*std::vector<double> mg_joint_values = manipulator_group.getCurrentJointValues();
    kinematic_state->setJointGroupPositions(joint_model_group, mg_joint_values);
    kinematic_state->setToRandomPositionsNearBy(joint_model_group, *kinematic_state, 0.2);
    const Eigen::Affine3d& stateTf = kinematic_state->getGlobalLinkTransform("camera_link");
    auto translation = stateTf.translation();
    auto quaternion = Eigen::Quaterniond(stateTf.linear()).coeffs();
    octomap::pose6d viewpoint(octomap::point3d(translation[0], translation[1], translation[2]), octomath::Quaternion(quaternion[0], quaternion[1], quaternion[2], quaternion[3]));*/

    publishMap();

    /*tree_mtx.lock();
    ros::Time vpEvalStart = ros::Time::now();
    double value = testTree.computeViewpointValue(viewpoint, 80.0 * M_PI / 180.0, 8, 6, 5.0);
    double value_no_weighting = testTree.computeViewpointValue(viewpoint, 80.0 * M_PI / 180.0, 8, 6, 5.0, false) / 2;
    ros::Time vpEvalDone = ros::Time::now();
    tree_mtx.unlock();

    ROS_INFO_STREAM("Viewpoint value: " << value << " / " << value_no_weighting << "; Computation time: " << (vpEvalDone - vpEvalStart));*/

    std::vector<double> mg_joint_values = manipulator_group.getCurrentJointValues();
    kinematic_state->setJointGroupPositions(joint_model_group, mg_joint_values);
    kinematic_state = sampleNextRobotState(joint_model_group, *kinematic_state);
    moveToState(manipulator_group, *kinematic_state);

  }
}
