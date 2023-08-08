#ifndef ROITSDFOCTREE_H
#define ROITSDFOCTREE_H

#include "VPBaseNode.h"
#include "TsdfOcTree.h"
#include <octomap/OccupancyOcTreeBase.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <geometry_msgs/Point.h>

namespace octomap_vpp
{

class RoiTsdfOcTreeNode : public TsdfOcTreeNode
{
public:
  RoiTsdfOcTreeNode() : TsdfOcTreeNode(), roiDistance(0), roiWeight(0) {}

  RoiTsdfOcTreeNode(const RoiTsdfOcTreeNode& rhs) : TsdfOcTreeNode(rhs), roiDistance(rhs.roiDistance), roiWeight(rhs.roiWeight) {}

  bool operator==(const RoiTsdfOcTreeNode& rhs) const{
    return (rhs.distance == distance && rhs.weight == weight && rhs.roiDistance == roiDistance && rhs.roiWeight == roiWeight);
  }

  void copyData(const RoiTsdfOcTreeNode& from){
    TsdfOcTreeNode::copyData(from);
    roiDistance = from.roiDistance;
    roiWeight = from.roiWeight;
  }

  std::ostream& writeData(std::ostream &s) const
  {
    s.write((const char*) &distance, sizeof(distance)); 
    s.write((const char*) &weight, sizeof(weight));
    s.write((const char*) &roiDistance, sizeof(roiDistance));
    s.write((const char*) &roiWeight, sizeof(roiWeight));
    return s;
  }

  std::istream& readData(std::istream &s)
  {
    s.read((char*) &distance, sizeof(distance));
    s.read((char*) &weight, sizeof(weight));
    s.read((char*) &roiDistance, sizeof(roiDistance));
    s.read((char*) &roiWeight, sizeof(roiWeight));

    return s;
  }

  inline float getRoiDistance() const
  {
    return roiDistance;
  }

  inline void setRoiDistance(float rd)
  {
    roiDistance = rd;
  }

  inline float getRoiWeight() const
  {
    return roiWeight;
  }

  inline void setRoiWeight(float rw)
  {
    roiWeight = rw;
  }

  virtual float getVPRoiLogOdds() const override
  {
    if (std::abs(roiWeight) < 1e-6)
      return 0.0; 

    if (distance <= 0)
      return FLT_MAX;

    return -FLT_MAX;
  }

  void updateRoiTsdfVoxel(const float w, const float sdf, 
                          const float defaultTruncationDistance,
                          const float dropoffEpsilon,
                          bool useWeightDropoff,
                          float maxWeight,
                          bool isRoi);

protected:
  float roiDistance;
  float roiWeight;
};

class RoiTsdfOcTree : public octomap::OccupancyOcTreeBase <RoiTsdfOcTreeNode>
{
public:
    RoiTsdfOcTree(double resolution);

    virtual RoiTsdfOcTree* create() const {return new RoiTsdfOcTree(resolution); }

    virtual std::string getTreeType() const {return "RoiTsdfOcTree";}

    RoiTsdfOcTreeNode* updateRoiNode(const octomap::OcTreeKey& key,
                               const float w, const float sdf, 
                               const float defaultTruncationDistance,
                               const float dropoffEpsilon,
                               const bool useWeightDropoff,
                               const float maxWeight,
                               const bool isRoi);

    RoiTsdfOcTreeNode* updateRoiNodeRecurs(RoiTsdfOcTreeNode* node,
                                    bool node_just_created,
                                    const octomap::OcTreeKey& key,
                                    unsigned int depth,
                                    const float w, const float sdf,
                                    const float defaultTruncationDistance,
                                    const float dropoffEpsilon,
                                    const bool useWeightDropoff,
                                    const float maxWeight,
                                    const bool isRoi);

    void extractRoiSurfacePontCloud(pcl::PointCloud<pcl::PointXYZRGB> &cloud);

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
      RoiTsdfOcTree* tree = new RoiTsdfOcTree(0.1);
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

  uint8_t maxFruitletId;
};

}

#endif // ROITSDFOCTREE_H