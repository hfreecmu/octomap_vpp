#ifndef TSDFOCTREE_H
#define TSDFOCTREE_H

#include "VPBaseNode.h"
#include <octomap/OccupancyOcTreeBase.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <geometry_msgs/Point.h>

namespace octomap_vpp
{

class TsdfOcTreeNode : public VPBaseNode
{
public:
  TsdfOcTreeNode() : VPBaseNode(), distance(0), weight(0) {}

  TsdfOcTreeNode(const TsdfOcTreeNode& rhs) : VPBaseNode(rhs), distance(rhs.distance), weight(rhs.weight) {}

  bool operator==(const TsdfOcTreeNode& rhs) const{
    return (rhs.distance == distance && rhs.weight == rhs.weight);
  }

  void copyData(const TsdfOcTreeNode& from){
    VPBaseNode::copyData(from);
    distance = from.distance;
    weight = from.weight;
  }

  std::ostream& writeData(std::ostream &s) const
  {
    s.write((const char*) &distance, sizeof(distance)); // occupancy
    s.write((const char*) &weight, sizeof(weight)); // roi value

    return s;
  }

  std::istream& readData(std::istream &s)
  {
    s.read((char*) &distance, sizeof(distance)); // occupancy
    s.read((char*) &weight, sizeof(weight)); // roi value

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

  virtual double getVPOccupancy() const override
  { 
    if (std::abs(weight) < 1e-6)
      return 0.5; 

    if (distance <= 0)
      return 1.0;

    return 0.0;
  }

  virtual bool isHardUnknown() const override
  {
     return (std::abs(weight) < 1e-6);
  }

  virtual float getVPRoiLogOdds() const override
  {
    return -FLT_MAX;
  }


  void updateTsdfVoxel(const float w, const float sdf, 
                       const float defaultTruncationDistance,
                       const float dropoffEpsilon,
                       bool useWeightDropoff,
                       float maxWeight);

  static void updateVoxel(const float w, const float sdf, 
                              const float defaultTruncationDistance,
                              const float dropoffEpsilon,
                              bool useWeightDropoff,
                              float maxWeight,
                              float &voxelDistance,
                              float &voxelWeight);

protected:
  float distance;
  float weight;
};

class TsdfOcTree : public octomap::OccupancyOcTreeBase <TsdfOcTreeNode>
{
public:
    TsdfOcTree(double resolution);

    virtual TsdfOcTree* create() const {return new TsdfOcTree(resolution); }

    virtual std::string getTreeType() const {return "TsdfOcTree";}

    TsdfOcTreeNode* updateNode(const octomap::OcTreeKey& key,
                               const float w, const float sdf, 
                               const float defaultTruncationDistance,
                               const float dropoffEpsilon,
                               const bool useWeightDropoff,
                               const float maxWeight);

    TsdfOcTreeNode* updateNodeRecurs(TsdfOcTreeNode* node,
                                    bool node_just_created,
                                    const octomap::OcTreeKey& key,
                                    unsigned int depth,
                                    const float w, const float sdf,
                                    const float defaultTruncationDistance,
                                    const float dropoffEpsilon,
                                    const bool useWeightDropoff,
                                    const float maxWeight);

    void extractSurfacePontCloud(pcl::PointCloud<pcl::PointXYZRGB> &cloud);
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
      TsdfOcTree* tree = new TsdfOcTree(0.1);
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
};

}

#endif // TSDFOCTREE_H