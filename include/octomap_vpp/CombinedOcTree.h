#ifndef COMBINED_H
#define COMBINED_H

#include "RoiOcTree.h"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace octomap_vpp
{

class CombinedOcTreeNode : public RoiOcTreeNode
{
public:
    CombinedOcTreeNode() : RoiOcTreeNode(), distance(0), weight(0) {}

    CombinedOcTreeNode(const CombinedOcTreeNode& rhs) : RoiOcTreeNode(rhs), distance(rhs.distance), weight(rhs.weight) {}

    bool operator==(const CombinedOcTreeNode& rhs) const{
        return (rhs.value == value && rhs.roiValue == roiValue && rhs.distance == distance && rhs.weight == weight);
    }

    void copyData(const CombinedOcTreeNode& from){
        RoiOcTreeNode::copyData(from);
        distance = from.distance;
        weight = from.weight;
    }

    std::ostream& writeData(std::ostream &s) const
    {
        s.write((const char*) &value, sizeof(value));
        s.write((const char*) &roiValue, sizeof(roiValue));
        s.write((const char*) &distance, sizeof(distance)); 
        s.write((const char*) &weight, sizeof(weight));
        return s;
    }

    std::istream& readData(std::istream &s)
    {
        s.read((char*) &value, sizeof(value));
        s.read((char*) &roiValue, sizeof(roiValue));
        s.read((char*) &distance, sizeof(distance));
        s.read((char*) &weight, sizeof(weight));

        return s;
    }

    inline float getDistance() const
    {
        return distance;
    }

    inline float getWeight() const
    {
        return weight;
    }

    inline void setDistance(float d)
    {
        distance = d;
    }

    inline void setWeight(float w)
    {
        weight = w;
    }

    //getVPOccupancy covered by parent
    //isHardUnknown covered by parent
    //getVPRoiLogOdds covered by parent

    void updateTsdfVoxel(const float w, const float sdf, 
                        const float defaultTruncationDistance,
                        const float dropoffEpsilon,
                        bool useWeightDropoff,
                        float maxWeight);

protected:
    float distance;
    float weight;
};

class CombinedOcTree : public octomap::OccupancyOcTreeBase <CombinedOcTreeNode>
{
public:
    CombinedOcTree(double resolution);

    virtual CombinedOcTree* create() const {return new CombinedOcTree(resolution); }

    virtual std::string getTreeType() const {return "CombinedOcTree";}

    CombinedOcTreeNode* updateCombinedNode(const octomap::OcTreeKey& key,
                               const float w, const float sdf, 
                               const float defaultTruncationDistance,
                               const float dropoffEpsilon,
                               const bool useWeightDropoff,
                               const float maxWeight,
                               const octomap::point3d &cameraPosition);

    CombinedOcTreeNode* updateComnbinedNodeRecurs(CombinedOcTreeNode* node,
                                    bool node_just_created,
                                    const octomap::OcTreeKey& key,
                                    unsigned int depth,
                                    const float w, const float sdf,
                                    const float defaultTruncationDistance,
                                    const float dropoffEpsilon,
                                    const bool useWeightDropoff,
                                    const float maxWeight,
                                    const octomap::point3d &cameraPosition);

    void extractRoiSurfacePontCloud(pcl::PointCloud<pcl::PointXYZRGB> &cloud);

    void updateNodeRoiLogOdds(CombinedOcTreeNode* node, const float& update) const;

    /// queries whether a node is occupied according to the tree's parameter for "occupancy"
    inline bool isNodeROI(const RoiOcTreeNode* node) const{
        return (node->getRoiLogOdds() >= this->roi_prob_thres_log);
    }

    /// queries whether a node is occupied according to the tree's parameter for "occupancy"
    inline bool isNodeROI(const RoiOcTreeNode& node) const{
        return (node.getRoiLogOdds() >= this->roi_prob_thres_log);
    }

    void extractFruitletClusters(std::unordered_map<uint8_t, pcl::PointCloud<pcl::PointXYZ>::Ptr> &fruitletClouds);
    void updateAssociations(std::vector<int> &fruitletIds, std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> &fruitletClouds);

protected:
   /**
   * Static member object which ensures that this OcTree's prototype
   * ends up in the classIDMapping only once. You need this as a
   * static member in any derived octree class in order to read .ot
   * files through the AbstractOcTree factory. You should also call
   * ensureLinking() once from the constructor.
   */
  class StaticMemberInitializer{
  public:
    StaticMemberInitializer() {
      CombinedOcTree* tree = new CombinedOcTree(0.1);
      tree->clearKeyRays();
      AbstractOcTree::registerTreeType(tree);
    }

    /**
     * Dummy function to ensure that MSVC does not drop the
     * StaticMemberInitializer, causing this tree failing to register.
     * Needs to be called from the constructor of this octree.
     */
    void ensureLinking() {}
  };

  /// to ensure static initialization (only once)
  static StaticMemberInitializer ocTreeMemberInit;
protected:
  float roi_prob_thres_log;
  uint8_t maxFruitletId;
};

}

#endif // COMBINED_H