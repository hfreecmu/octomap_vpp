#include "octomap_vpp/TsdfOcTree.h"
#include <algorithm>

namespace octomap_vpp
{

void TsdfOcTreeNode::updateTsdfVoxel(const float w, const float sdf, 
                                    const float defaultTruncationDistance,
                                    const float dropoffEpsilon,
                                    bool useWeightDropoff,
                                    float maxWeight)
{
    updateVoxel(w, sdf, defaultTruncationDistance,
                    dropoffEpsilon, useWeightDropoff,
                    maxWeight, this->distance, this->weight);
}

//static
void TsdfOcTreeNode::updateVoxel(const float w, const float sdf, 
                                    const float defaultTruncationDistance,
                                    const float dropoffEpsilon,
                                    bool useWeightDropoff,
                                    float maxWeight,
                                    float &voxelDistance,
                                    float &voxelWeight)
{
    float updated_weight = w;

    if ((useWeightDropoff) && (sdf < -dropoffEpsilon))
    {
        updated_weight = w * (defaultTruncationDistance + sdf) / 
            (defaultTruncationDistance - dropoffEpsilon);
        
        if (updated_weight < 0.0)
            updated_weight = 0;
    }

    const float new_weight = voxelWeight + updated_weight;

    //prevent nans when dividing
    //TODO should I still set distance?
    if (new_weight < 1e-6)
        return;

    voxelDistance = (sdf*updated_weight + voxelDistance*voxelWeight) / new_weight;
    
    if (voxelDistance < -defaultTruncationDistance)
        voxelDistance = -defaultTruncationDistance;

    if (voxelDistance > defaultTruncationDistance)
        voxelDistance = defaultTruncationDistance;

    if (new_weight > maxWeight)
        voxelWeight = maxWeight;
    else
        voxelWeight = new_weight;
}


TsdfOcTree::TsdfOcTree(double resolution) 
: octomap::OccupancyOcTreeBase <TsdfOcTreeNode>(resolution)
{
  ocTreeMemberInit.ensureLinking();
}

TsdfOcTree::StaticMemberInitializer TsdfOcTree::ocTreeMemberInit;

TsdfOcTreeNode* TsdfOcTree::updateNode(const octomap::OcTreeKey& key,
                                       const float w, const float sdf, 
                                       const float defaultTruncationDistance,
                                       const float dropoffEpsilon,
                                       const bool useWeightDropoff,
                                       const float maxWeight)
{
    bool createdRoot = false;
    if (this->root == NULL){
      this->root = new TsdfOcTreeNode();
      this->tree_size++;
      createdRoot = true;
    }

    return updateNodeRecurs(this->root, createdRoot, key, 0, w, sdf,
                            defaultTruncationDistance, dropoffEpsilon,
                            useWeightDropoff, maxWeight);
}

TsdfOcTreeNode* TsdfOcTree::updateNodeRecurs(TsdfOcTreeNode* node, 
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
        return updateNodeRecurs(this->getNodeChild(node, pos), created_node, key, depth+1, w, sdf,
                                defaultTruncationDistance, dropoffEpsilon, useWeightDropoff,
                                maxWeight);

    }

    // at last level, update node, end of recursion
    else {
      node->updateTsdfVoxel(w, sdf, defaultTruncationDistance,
                            dropoffEpsilon, useWeightDropoff, maxWeight);
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