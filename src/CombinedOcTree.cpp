#include "octomap_vpp/CombinedOcTree.h"
#include "octomap_vpp/TsdfOcTree.h"
#include <algorithm>

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
roi_prob_thres_log(FLT_MIN)
{
  ocTreeMemberInit.ensureLinking();
}

CombinedOcTree::StaticMemberInitializer CombinedOcTree::ocTreeMemberInit;

CombinedOcTreeNode* CombinedOcTree::updateCombinedNode(const octomap::OcTreeKey& key,
                                       const float w, const float sdf, 
                                       const float defaultTruncationDistance,
                                       const float dropoffEpsilon,
                                       const bool useWeightDropoff,
                                       const float maxWeight,
                                       const bool isOccupied,
                                       const bool isRoi,
                                       const bool shouldUpdateOcc)
{
    //idealy not occupied means not roi but leaving that up to caller
    float log_odds_update = isOccupied ? this->prob_hit_log : this->prob_miss_log;
    float roi_log_odds_update = isRoi ? this->prob_hit_log : this->prob_miss_log;


    bool createdRoot = false;
    if (this->root == NULL){
      this->root = new CombinedOcTreeNode();
      this->tree_size++;
      createdRoot = true;
    }

    return updateComnbinedNodeRecurs(this->root, createdRoot, key, 0, w, sdf,
                            defaultTruncationDistance, dropoffEpsilon,
                            useWeightDropoff, maxWeight, log_odds_update, roi_log_odds_update, shouldUpdateOcc);
}

CombinedOcTreeNode* CombinedOcTree::updateComnbinedNodeRecurs(CombinedOcTreeNode* node, 
                                             bool node_just_created,
                                             const octomap::OcTreeKey& key,
                                             unsigned int depth,
                                             const float w, const float sdf,
                                             const float defaultTruncationDistance,
                                             const float dropoffEpsilon,
                                             const bool useWeightDropoff,
                                             const float maxWeight,
                                             const float log_odds_update,
                                             const float roi_log_odds_update,
                                             const bool shouldUpdateOcc)
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
                                maxWeight, log_odds_update, roi_log_odds_update, shouldUpdateOcc);

    }

    // at last level, update node, end of recursion
    else {
      if (shouldUpdateOcc)
      {
        //first, update loggodds (OccupancyOcTreeBase.hxx)
        updateNodeLogOdds(node, log_odds_update);

        //then update roiodds (RoiOcTree.cpp)
        //note that if not occupied, should not be roi
        //but we are going to let the caller determine that. not us
        updateNodeRoiLogOdds(node, roi_log_odds_update);
      }

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