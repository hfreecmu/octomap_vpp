#include "octomap_vpp/TsdfOcTree.h"
#include <algorithm>

namespace octomap_vpp
{

void TsdfOcTreeNode::updateTsdfVoxel(const float w, const float sdf, 
                                    const float defaultTruncationDistance,
                                    const float dropoffEpsilon,
                                    bool useWeightDropoff,
                                    float maxWeight
)
{
    float updated_weight = w;

    if ((useWeightDropoff) && (sdf < -dropoffEpsilon))
    {
        updated_weight = w * (defaultTruncationDistance + sdf) / 
            (defaultTruncationDistance - dropoffEpsilon);
        
        if (updated_weight < 0.0)
            updated_weight = 0;
    }

    const float new_weight = this->weight + updated_weight;

    //prevent nans when dividing
    //TODO should I still set distance?
    if (new_weight < 1e-6)
        return;

    this->distance = (sdf*updated_weight + this->distance*this->weight) / new_weight;
    
    if (this->distance < -defaultTruncationDistance)
        this->distance = -defaultTruncationDistance;

    if (this->distance > defaultTruncationDistance)
        this->distance = defaultTruncationDistance;

    if (new_weight > maxWeight)
        this->weight = maxWeight;
    else
        this->weight = new_weight;

}

TsdfOcTree::TsdfOcTree(double resolution,
                       float truncationDistance,
                       float maxrange,
                       bool useConstWeight,
                       bool useWeightDropoff,
                       float dropOffEpsilon,
                       float maxWeight) 
: octomap::OccupancyOcTreeBase <TsdfOcTreeNode>(resolution),
  truncationDistance(truncationDistance),
  maxrange(maxrange),
  useConstWeight(useConstWeight),
  useWeightDropoff(useWeightDropoff),
  dropOffEpsilon(dropOffEpsilon),
  maxWeight(maxWeight)
{
  ocTreeMemberInit.ensureLinking();
}

TsdfOcTree::StaticMemberInitializer TsdfOcTree::ocTreeMemberInit;

//Needs to be in camera frame
void TsdfOcTree::insertPointCloud(const octomap::Pointcloud& scan, 
                                  const octomap::point3d& sensor_origin,
                                  octomath::Pose6D world2cam,
                                  bool discretize)
{
    
    
    if (discretize)
        computeDiscreteUpdate(scan, sensor_origin, world2cam);
    else
        computeUpdate(scan, sensor_origin, world2cam);
}

void TsdfOcTree::computeDiscreteUpdate(const octomap::Pointcloud& scan, 
                                       const octomap::point3d& sensor_origin,
                                       octomath::Pose6D world2cam)
{
   octomap::Pointcloud discretePC;
   discretePC.reserve(scan.size());
   octomap::KeySet endpoints;

   for (int i = 0; i < (int)scan.size(); ++i) {
     octomap::OcTreeKey k = this->coordToKey(scan[i]);
     std::pair<octomap::KeySet::iterator,bool> ret = endpoints.insert(k);
     if (ret.second){ // insertion took place => k was not in set
       discretePC.push_back(this->keyToCoord(k));
     }
   }

   computeUpdate(discretePC, sensor_origin, world2cam);
}

void TsdfOcTree::computeUpdate(const octomap::Pointcloud& scan, 
                               const octomap::point3d& sensor_origin,
                               octomath::Pose6D world2cam)
{
    unsigned threadIdx = 0;
    octomap::KeyRay* keyray = &(this->keyrays.at(threadIdx));
    octomap::point3d dirVec;
    octomap::point3d rayEnd;
    octomap::point3d rayStart = sensor_origin;
    octomap::point3d cameraFramePoint;
    octomap::point3d voxelCenter;
    for (int i = 0; i < (int)scan.size(); ++i)
    {
        const octomap::point3d& p = scan[i];
        dirVec = (p - sensor_origin).normalize();
        float pointDistance = (p - sensor_origin).norm();

        if ((maxrange < 0.0) || (p - sensor_origin).norm() <= maxrange)
        {
            rayEnd = p + dirVec*truncationDistance; 
        }
        else
        {
            //TODO why do they substract truncation distance from ray length?
            //maybe just if it is a free insert?
            float rayLength = std::min(std::max(pointDistance - truncationDistance,
                                         static_cast<float>(0.0)), static_cast<float>(maxrange));

            rayEnd = sensor_origin + dirVec*rayLength;
        }

        if (!this->computeRayKeys(rayStart, rayEnd, *keyray))
            continue;

        octomap::OcTreeKey endkey;
        bool keyCheck = this->coordToKeyChecked(rayEnd, endkey);
        if (keyCheck)
            keyray->addKey(endkey);
        
        cameraFramePoint = world2cam.transform(p);
        const float weight = getVoxelWeight(cameraFramePoint, useConstWeight); 

        for(octomap::KeyRay::iterator it=keyray->begin(); it != keyray->end(); it++)
        {
            voxelCenter = this->keyToCoord(*it);
            const float sdf = computeDistance(sensor_origin, p, voxelCenter);
            updateNode(*it, weight, sdf, truncationDistance, dropOffEpsilon,
                        useWeightDropoff, maxWeight);
        }
    }
}

//TODO assuming here that sphere is within max distance, like others
void TsdfOcTree::computeUpdate(const octomap::Pointcloud& scan,
                               const octomap::Pointcloud& origins,
                               const octomap::Pointcloud& endPoints,
                               octomath::Pose6D world2cam)
{
    unsigned threadIdx = 0;
    octomap::KeyRay* keyray = &(this->keyrays.at(threadIdx));
    octomap::point3d cameraFramePoint;
    octomap::point3d voxelCenter;

    for (int i = 0; i < (int)scan.size(); ++i)
    {
        const octomap::point3d& point = scan[i];
        const octomap::point3d& origin = origins[i];
        const octomap::point3d& endPoint = endPoints[i];

        if (!this->computeRayKeys(origin, endPoint, *keyray))
            continue;

        octomap::OcTreeKey endkey;
        bool keyCheck = this->coordToKeyChecked(endPoint, endkey);
        if (keyCheck)
            keyray->addKey(endkey);
        
        cameraFramePoint = world2cam.transform(point);
        const float weight = getVoxelWeight(cameraFramePoint, useConstWeight); 

        for(octomap::KeyRay::iterator it=keyray->begin(); it != keyray->end(); it++)
        {
            voxelCenter = this->keyToCoord(*it);
            const float sdf = computeDistance(origin, point, voxelCenter);
            updateNode(*it, weight, sdf, truncationDistance, dropOffEpsilon,
                        useWeightDropoff, maxWeight);
        }
    }
}

//Needs to be in camera frame
float TsdfOcTree::getVoxelWeight(const octomap::point3d& point, bool use_const_weight) const
{
    if (use_const_weight)
        return 1.0;

    const float dist_z = std::abs(point.z());

    if (dist_z < 1e-6)
        return 0.0;

    return 1.0 / (dist_z*dist_z);
}

float TsdfOcTree::computeDistance(const octomap::point3d& origin,
                                  const octomap::point3d& point,
                                  const octomap::point3d& voxel_center)
{
    const octomap::point3d v_voxel_origin = voxel_center - origin;
    const octomap::point3d v_point_origin = point - origin;
    
    const float dist = v_point_origin.norm();

    const float dist_v = v_voxel_origin.dot(v_point_origin) / dist;

    const float sdf = dist - dist_v;

    return sdf;
}

TsdfOcTreeNode* TsdfOcTree::updateNode(const octomap::OcTreeKey& key,
                                       const float w, const float sdf, 
                                       const float defaultTruncationDistance,
                                       const float dropoffEpsilon,
                                       bool useWeightDropoff,
                                       float maxWeight)
{
    bool createdRoot = false;
    if (this->root == NULL){
      this->root = new TsdfOcTreeNode();
      this->tree_size++;
      createdRoot = true;
    }

    return updateNodeRecurs(this->root, createdRoot, key, 0, w, sdf);
}

TsdfOcTreeNode* TsdfOcTree::updateNodeRecurs(TsdfOcTreeNode* node, 
                                             bool node_just_created,
                                             const octomap::OcTreeKey& key,
                                             unsigned int depth,
                                             const float w, const float sdf)
{
    bool created_node = false;

    assert(node);

    // follow down to last level
    if (depth < this->tree_depth) {
      unsigned int pos = computeChildIdx(key, this->tree_depth -1 - depth);
      if (!this->nodeChildExists(node, pos)) {
        // child does not exist, but maybe it's a pruned node?
        if (!this->nodeHasChildren(node) && !node_just_created ) {
          // current node does not have children AND it is not a new node 
          // -> expand pruned node
          this->expandNode(node);
        }
        else {
          // not a pruned node, create requested child
          this->createNodeChild(node, pos);
          created_node = true;
        }
      }

        //always lazy eval
        return updateNodeRecurs(this->getNodeChild(node, pos), created_node, key, depth+1, w, sdf);

    }

    // at last level, update node, end of recursion
    else {
      node->updateTsdfVoxel(w, sdf, truncationDistance,
                            dropOffEpsilon, useWeightDropoff, maxWeight);
      return node;
    }
}

void TsdfOcTree::extractSurfacePontCloud(pcl::PointCloud<pcl::PointXYZ> &cloud)
{
    const float surface_distance_thresh = resolution*0.75;

    float kMinWeight = 0;

    cloud.clear();

    for(TsdfOcTree::leaf_iterator it = this->begin_leafs(), end=this->end_leafs(); it!= end; ++it)
    {
        octomap::OcTreeKey key = it.getKey();

        TsdfOcTreeNode* node = this->search(key);
        //should not happen
        if (node == nullptr)
            continue;

        if (node->getWeight() <= kMinWeight)
            continue;
        
        if (std::abs(node->getDistance()) >= surface_distance_thresh)
            continue;

        octomap::point3d octoPoint = this->keyToCoord(key);
        pcl::PointXYZ point(octoPoint.x(), octoPoint.y(), octoPoint.z());

        cloud.push_back(point);
    }
}

}