#include "octomap_vpp/RoiTsdfOcTree.h"
#include <algorithm>
#include <boost/make_shared.hpp>
#include <ros/ros.h>


namespace octomap_vpp
{

void RoiTsdfOcTreeNode::updateRoiTsdfVoxel(const float w, const float sdf, 
                                    const float defaultTruncationDistance,
                                    const float dropoffEpsilon,
                                    bool useWeightDropoff,
                                    float maxWeight,
                                    bool isRoi)
{
    updateTsdfVoxel(w, sdf, defaultTruncationDistance,
                    dropoffEpsilon, useWeightDropoff,
                    maxWeight);

    if (isRoi)
        updateVoxel(w, sdf, defaultTruncationDistance, 
                    dropoffEpsilon, useWeightDropoff,
                    maxWeight, this->roiDistance, this->roiWeight);
}

RoiTsdfOcTree::RoiTsdfOcTree(double resolution) 
: octomap::OccupancyOcTreeBase <RoiTsdfOcTreeNode>(resolution),
maxFruitletId(0)
{
  ocTreeMemberInit.ensureLinking();
}

RoiTsdfOcTree::StaticMemberInitializer RoiTsdfOcTree::ocTreeMemberInit;

RoiTsdfOcTreeNode* RoiTsdfOcTree::updateRoiNode(const octomap::OcTreeKey& key,
                                       const float w, const float sdf, 
                                       const float defaultTruncationDistance,
                                       const float dropoffEpsilon,
                                       const bool useWeightDropoff,
                                       const float maxWeight,
                                       const bool isRoi)
{
    bool createdRoot = false;
    if (this->root == NULL){
      this->root = new RoiTsdfOcTreeNode();
      this->tree_size++;
      createdRoot = true;
    }

    return updateRoiNodeRecurs(this->root, createdRoot, key, 0, w, sdf,
                            defaultTruncationDistance, dropoffEpsilon,
                            useWeightDropoff, maxWeight, isRoi);
}

RoiTsdfOcTreeNode* RoiTsdfOcTree::updateRoiNodeRecurs(RoiTsdfOcTreeNode* node, 
                                             bool node_just_created,
                                             const octomap::OcTreeKey& key,
                                             unsigned int depth,
                                             const float w, const float sdf,
                                             const float defaultTruncationDistance,
                                             const float dropoffEpsilon,
                                             const bool useWeightDropoff,
                                             const float maxWeight,
                                             const bool isRoi)
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
        return updateRoiNodeRecurs(this->getNodeChild(node, pos), created_node, key, depth+1, w, sdf,
                                defaultTruncationDistance, dropoffEpsilon, useWeightDropoff,
                                maxWeight, isRoi);

    }

    // at last level, update node, end of recursion
    else {
      node->updateRoiTsdfVoxel(w, sdf, defaultTruncationDistance,
                            dropoffEpsilon, useWeightDropoff, maxWeight, isRoi);
      return node;
    }
}

void RoiTsdfOcTree::extractRoiSurfacePontCloud(pcl::PointCloud<pcl::PointXYZRGB> &cloud)
{
    const float surface_distance_thresh = resolution*0.75;

    float kMinWeight = 0;

    cloud.clear();

    for(RoiTsdfOcTree::leaf_iterator it = this->begin_leafs(), end=this->end_leafs(); it!= end; ++it)
    {
        octomap::OcTreeKey key = it.getKey();

        RoiTsdfOcTreeNode* node = this->search(key);
        //should not happen
        if (node == nullptr)
            continue;

        if (node->getWeight() <= kMinWeight)
            continue;
        
        if (std::abs(node->getDistance()) >= surface_distance_thresh)
            continue;

        octomap::point3d octoPoint = this->keyToCoord(key);
        pcl::PointXYZRGB point;
        point.x = octoPoint.x();
        point.y = octoPoint.y();
        point.z = octoPoint.z();

        if ((node->getRoiWeight() <= kMinWeight) ||
             std::abs(node->getRoiDistance()) >= surface_distance_thresh)
        {
            point.r = 0;
            point.g = 255;
            point.b = 0;
        }
        else{
            point.r = 255;
            point.g = 0;
            point.b = 0;
        }

        cloud.push_back(point);
    }
}

}