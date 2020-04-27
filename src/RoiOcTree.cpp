#include "octomap_vpp/RoiOcTree.h"

namespace octomap_vpp
{

double RoiOcTreeNode::getMeanChildRoiLogOdds() const
{
    double mean = 0;
    uint8_t c = 0;
    if (children !=NULL){
      for (unsigned int i=0; i<8; i++) {
        if (children[i] != NULL) {
          mean += static_cast<RoiOcTreeNode*>(children[i])->getRoiProb(); // TODO check if works generally
          ++c;
        }
      }
    }

    if (c > 0)
      mean /= (double) c;

    return log(mean/(1-mean));
}

float RoiOcTreeNode::getMaxChildRoiLogOdds() const
{
    float max = -std::numeric_limits<float>::max();

    if (children !=NULL){
      for (unsigned int i=0; i<8; i++) {
        if (children[i] != NULL) {
          float l = static_cast<RoiOcTreeNode*>(children[i])->getRoiLogOdds(); // TODO check if works generally
          if (l > max)
            max = l;
        }
      }
    }
    return max;
}

void RoiOcTreeNode::addRoiValue(const float& logOdds)
{
  roiValue += logOdds;
}

RoiOcTree::RoiOcTree(double resolution) : octomap::OccupancyOcTreeBase <RoiOcTreeNode>(resolution), roi_prob_thres_log(1)
{
  ocTreeMemberInit.ensureLinking();
}

RoiOcTree::StaticMemberInitializer RoiOcTree::ocTreeMemberInit;

void RoiOcTree::insertRegionScan(const octomap::Pointcloud &regionPoints, const octomap::Pointcloud &offRegionPoints)
{
  //std::unordered_set<RoiOcTreeNode*> regionNodes;
  //std::unordered_set<RoiOcTreeNode*> offRegionNodes;
  octomap::KeySet regionNodes;
  octomap::KeySet offRegionNodes;
  for (size_t i = 0; i < regionPoints.size(); i++)
  {
    const octomap::point3d &p = regionPoints[i];
    //RoiOcTreeNode *node = this->search(p);
    //if (node != NULL)
    //  regionNodes.insert(node);

    octomap::OcTreeKey key;
    if (coordToKeyChecked(p, key))
      regionNodes.insert(key);
  }

  for (size_t i = 0; i < offRegionPoints.size(); i++)
  {
    const octomap::point3d &p = offRegionPoints[i];
    //RoiOcTreeNode *node = this->search(p);
    //if (node != NULL)
    //  offRegionNodes.insert(node);

    octomap::OcTreeKey key;
    if (coordToKeyChecked(p, key) && (regionNodes.find(key) == regionNodes.end())) // regionNodes have priority over offRegionNodes
      offRegionNodes.insert(key);
  }

  //ROS_INFO_STREAM("Key set sizes: " << regionNodes.size() << ", " << offRegionNodes.size());
  //ROS_INFO_STREAM("Hit/miss prob: " << this->prob_miss_log << ", " << this->prob_hit_log);

  for (const octomap::OcTreeKey &key : regionNodes)
  {
    updateNodeRoi(key, true, false);
  }

  for (const octomap::OcTreeKey &key : offRegionNodes)
  {
    updateNodeRoi(key, false, false);
  }
}

RoiOcTreeNode* RoiOcTree::updateNodeRoi(const octomap::OcTreeKey& key, float log_odds_update, bool lazy_eval) {
   // early abort (no change will happen).
   // may cause an overhead in some configuration, but more often helps
   RoiOcTreeNode* leaf = this->search(key);
   // no change: node already at threshold
   if (leaf
       && ((log_odds_update >= 0 && leaf->getRoiLogOdds() >= this->clamping_thres_max)
       || ( log_odds_update <= 0 && leaf->getRoiLogOdds() <= this->clamping_thres_min)))
   {
     return leaf;
   }

   bool createdRoot = false;
   if (this->root == NULL){
     //return leaf; // don't create new nodes
     this->root = new RoiOcTreeNode();
     this->tree_size++;
     createdRoot = true;
   }

   return updateNodeRoiRecurs(this->root, createdRoot, key, 0, log_odds_update, lazy_eval);
 }

RoiOcTreeNode* RoiOcTree::updateNodeRoi(const octomap::OcTreeKey& key, bool isRoi, bool lazy_eval)
{
  float logOdds = this->prob_miss_log;
  if (isRoi)
    logOdds = this->prob_hit_log;

  return updateNodeRoi(key, logOdds, lazy_eval);
}

RoiOcTreeNode* RoiOcTree::updateNodeRoiRecurs(RoiOcTreeNode* node, bool node_just_created, const octomap::OcTreeKey& key, unsigned int depth, const float& log_odds_update, bool lazy_eval)
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

      if (lazy_eval)
        return updateNodeRoiRecurs(this->getNodeChild(node, pos), created_node, key, depth+1, log_odds_update, lazy_eval);
      else {
        RoiOcTreeNode* retval = updateNodeRoiRecurs(this->getNodeChild(node, pos), created_node, key, depth+1, log_odds_update, lazy_eval);
        // prune node if possible, otherwise set own probability
        // note: combining both did not lead to a speedup!
        if (this->pruneNode(node)){
          // return pointer to current parent (pruned), the just updated node no longer exists
          retval = node;
        } else{
          node->updateRoiValChildren();
        }

        return retval;
      }
    }

    // at last level, update node, end of recursion
    else {
      //if (use_change_detection) {
      bool roiBefore = this->isNodeROI(node);
      updateNodeRoiLogOdds(node, log_odds_update);
      bool roiAfter = this->isNodeROI(node);

      if (roiAfter && !roiBefore) // roi added
      {
        octomap::KeySet::iterator it = deleted_rois.find(key);
        if (it != deleted_rois.end())
        {
          deleted_rois.erase(it);
        }
        added_rois.insert(key);
        roi_keys.insert(key);
      }
      else if (!roiAfter && roiBefore) // roi deleted
      {
        octomap::KeySet::iterator it = added_rois.find(key);
        if (it != added_rois.end())
        {
          added_rois.erase(it);
        }
        deleted_rois.insert(key);
        it = roi_keys.find(key);
        if (it != roi_keys.end())
        {
          roi_keys.erase(it);
        }
      }

      /*if (node_just_created){  // new node
        changed_keys.insert(std::pair<octomap::OcTreeKey,bool>(key, true));
      } else if (roiBefore != this->isNodeROI(node)) {  // occupancy changed, track it
        octomap::KeyBoolMap::iterator it = changed_keys.find(key);
        if (it == changed_keys.end())
          changed_keys.insert(std::pair<octomap::OcTreeKey,bool>(key, false));
        else if (it->second == false)
          changed_keys.erase(it);
      }
      } else {
        //float lo_before = node->getRoiLogOdds();
        updateNodeRoiLogOdds(node, log_odds_update);
        //if (log_odds_update > 0)
        //  ROS_INFO_STREAM("Updated LOs from " << lo_before << " to " << node->getRoiLogOdds() << " (Prob: " << node->getRoiProb() << "CV: " << log_odds_update << ")");
      }*/
      return node;
    }
}

std::shared_ptr<InflatedRoiOcTree> RoiOcTree::computeInflatedRois(double resolution, double inflation_radius)
{
  //std::vector<octomap::OcTreeKey> roiKeys = getRoiKeys();
  //if (roi_keys.size() == 0)
  //  return NULL;

  double previous_resolution = inflated_rois ? inflated_rois->getResolution() : 0.0;
  double previous_inflation_radius = inflated_rois ? inflated_rois->getInfluenceRadius() : 0.0;

  if (added_rois.empty() && deleted_rois.empty() && (resolution == previous_resolution) && (inflation_radius == previous_inflation_radius)) // no changes, return existing tree
    return inflated_rois;

  bool full_construct = false;
  if (inflated_rois == nullptr || !deleted_rois.empty() || resolution != previous_resolution || inflation_radius != previous_inflation_radius)
  {
    inflated_rois.reset(new InflatedRoiOcTree(resolution, inflation_radius));
    inflated_roi_keys.clear();
    full_construct = true;
  }

  //InflatedRoiOcTree *inflatedTree = new InflatedRoiOcTree(this->resolution);

  typedef octomap::unordered_ns::unordered_map<octomap::OcTreeKey, float, octomap::OcTreeKey::KeyHash> KeyFloatMap;

  /*struct cmp_fkey_pair {
    bool operator() (const std::pair<float, octomap::OcTreeKey>& lhs, const std::pair<float, octomap::OcTreeKey>& rhs) const
    {return lhs.first > rhs.first;}
  };
  typedef std::set<std::pair<float, octomap::OcTreeKey>, cmp_fkey_pair> SortedFloatKeySet;*/

  KeyFloatMap keyMap;
  //SortedFloatKeySet floatKeySet;
  octomap::KeySet processedKeys;

  float maxVal = inflated_rois->getMaxRoiVal();
  float stepReduction = maxVal / inflated_rois->getInfluenceRadius() * inflated_rois->getResolution() ;
  //ROS_INFO_STREAM("Max Val: " << maxVal << "; Step reduction: " << stepReduction);
  const octomap::KeySet &keysToAdd = full_construct ? roi_keys : added_rois;
  for (const octomap::OcTreeKey &key : keysToAdd)
  {
    if (this->getResolution() == inflated_rois->getResolution())
      keyMap[key] = maxVal;
      //floatKeySet.insert(std::make_pair(maxVal, key));
    else
      keyMap[inflated_rois->coordToKey(this->keyToCoord(key))] = maxVal;
  }
  while(!keyMap.empty())
  {
    KeyFloatMap::iterator maxIt = std::max_element
    (
        keyMap.begin(), keyMap.end(),
        [] (const KeyFloatMap::value_type &p1, const KeyFloatMap::value_type &p2) {
            return p1.second < p2.second;
        }
    );
    octomap::OcTreeKey curKey = maxIt->first;
    float curVal = maxIt->second;
    keyMap.erase(maxIt);
    inflated_rois->updateNodeVal(curKey, curVal, true, !full_construct);
    processedKeys.insert(curKey);
    /*SortedFloatKeySet::iterator maxIt = floatKeySet.begin();
    octomap::OcTreeKey curKey = maxIt->second;
    float curVal = maxIt->first;
    floatKeySet.erase(maxIt);
    keyMap.erase(curKey);
    inflated_rois->updateNodeVal(curKey, curVal, true, !full_construct);
    processedKeys.insert(curKey);*/

    #pragma omp parallel for collapse(3)
    for (int i = -1; i <= 1; i++)
    {
      for (int j = -1; j <= 1; j++)
      {
        for (int k = -1; k <= 1; k++)
        {
          int coordDiff = std::abs(i) + std::abs(j) + std::abs(k);
          float newNeighbourVal = 0;
          if (coordDiff == 0) continue; // only process neighbours, not the node itself
          else if (coordDiff == 1) newNeighbourVal = curVal - stepReduction;
          else if (coordDiff == 2) newNeighbourVal = curVal - stepReduction * sqrt(2);
          else /* coordDiff == 3*/ newNeighbourVal = curVal - stepReduction * sqrt(3);
          if (newNeighbourVal <= 0) continue; // new value is out of influence radius

          octomap::OcTreeKey neighbourKey(curKey[0] + i, curKey[1] + j, curKey[2] + k);
          if (processedKeys.find(neighbourKey) != processedKeys.end()) continue; // already processed keys can be ignored

          KeyFloatMap::iterator it = keyMap.find(neighbourKey);
          if (it != keyMap.end()) // key already known, check if update needed
          {
            if (newNeighbourVal > it->second) // only update if new value would be higher
            {
              //SortedFloatKeySet::iterator set_it = floatKeySet.find(std::make_pair(it->second, it->first));
              //floatKeySet.erase(set_it);
              it->second = newNeighbourVal;
              //floatKeySet.insert(std::make_pair(it->second, it->first));
            }
          }
          else // otherwise, enter new key to map
          {
            keyMap[neighbourKey] = newNeighbourVal;
            //floatKeySet.insert(std::make_pair(newNeighbourVal, neighbourKey));
          }
        }
      }
    }
  }
  inflated_rois->updateInnerVals();
  inflated_roi_keys.insert(processedKeys.begin(), processedKeys.end());
  added_rois.clear();
  deleted_rois.clear();
  return inflated_rois;
}

void RoiOcTree::updateNodeRoiLogOdds(RoiOcTreeNode* node, const float& update) const
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
