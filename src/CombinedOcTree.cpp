#include "octomap_vpp/CombinedOcTree.h"
#include "octomap_vpp/TsdfOcTree.h"
#include <algorithm>

#include <boost/make_shared.hpp>
#include <ros/ros.h>

namespace octomap_vpp
{

void CombinedOcTreeNode::updateTsdfVoxel(const float w, const float sdf, 
                                    const float defaultTruncationDistance,
                                    const float dropoffEpsilon,
                                    bool useWeightDropoff,
                                    float maxWeight)
{
    TsdfOcTreeNode::updateVoxel(w, sdf, defaultTruncationDistance,
                    dropoffEpsilon, useWeightDropoff,
                    maxWeight, this->distance, this->weight);
}

CombinedOcTree::CombinedOcTree(double resolution) 
: octomap::OccupancyOcTreeBase <CombinedOcTreeNode>(resolution),
roi_prob_thres_log(FLT_MIN),
maxFruitletId(0)
{
  ocTreeMemberInit.ensureLinking();
}

CombinedOcTree::StaticMemberInitializer CombinedOcTree::ocTreeMemberInit;

CombinedOcTreeNode* CombinedOcTree::updateCombinedNode(const octomap::OcTreeKey& key,
                                       const float w, const float sdf, 
                                       const float defaultTruncationDistance,
                                       const float dropoffEpsilon,
                                       const bool useWeightDropoff,
                                       const float maxWeight)
{

    bool createdRoot = false;
    if (this->root == NULL){
      this->root = new CombinedOcTreeNode();
      this->tree_size++;
      createdRoot = true;
    }

    return updateComnbinedNodeRecurs(this->root, createdRoot, key, 0, w, sdf,
                            defaultTruncationDistance, dropoffEpsilon,
                            useWeightDropoff, maxWeight);
}

CombinedOcTreeNode* CombinedOcTree::updateComnbinedNodeRecurs(CombinedOcTreeNode* node, 
                                             bool node_just_created,
                                             const octomap::OcTreeKey& key,
                                             unsigned int depth,
                                             const float w, const float sdf,
                                             const float defaultTruncationDistance,
                                             const float dropoffEpsilon,
                                             const bool useWeightDropoff,
                                             const float maxWeight)
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
        return updateComnbinedNodeRecurs(this->getNodeChild(node, pos), created_node, key, depth+1, w, sdf,
                                defaultTruncationDistance, dropoffEpsilon, useWeightDropoff,
                                maxWeight);

    }

    // at last level, update node, end of recursion
    else {
      //lastly, we can update our tsdf
      node->updateTsdfVoxel(w, sdf, defaultTruncationDistance,
                      dropoffEpsilon, useWeightDropoff, maxWeight);
      return node;
    }
}

void CombinedOcTree::extractRoiSurfacePontCloud(pcl::PointCloud<pcl::PointXYZRGB> &cloud)
{
    const float surface_distance_thresh = resolution*0.75;

    float kMinWeight = 0;

    cloud.clear();

    for(CombinedOcTree::leaf_iterator it = this->begin_leafs(), end=this->end_leafs(); it!= end; ++it)
    {
        octomap::OcTreeKey key = it.getKey();

        CombinedOcTreeNode* node = this->search(key);
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

        //ROI
        if (node->getVPRoiLogOdds() > roi_prob_thres_log)
        {
            point.r = 255;
            point.g = 0;
            point.b = 0;
        }
        //Not roi
        else{
            point.r = 0;
            point.g = 255;
            point.b = 0;
        }

        cloud.push_back(point);
    }
}

void CombinedOcTree::updateNodeRoiLogOdds(CombinedOcTreeNode* node, const float& update) const
{
  node->addRoiValue(update);
  if (node->getRoiLogOdds() < this->clamping_thres_min) {
    node->setRoiLogOdds(this->clamping_thres_min);
    return;
  }
  if (node->getRoiLogOdds() > this->clamping_thres_max) {
    node->setRoiLogOdds(this->clamping_thres_max);
  }
}

}