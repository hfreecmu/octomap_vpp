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

  /// adds p to the node's logOdds value (with no boundary / threshold checking!)
  void updateTsdfVoxel(const float w, const float sdf, 
                       const float defaultTruncationDistance,
                       const float dropoffEpsilon,
                       bool useWeightDropoff,
                       float maxWeight
                      );

protected:
  float distance;
  float weight;
};

class TsdfOcTree : public octomap::OccupancyOcTreeBase <TsdfOcTreeNode>
{
public:
    TsdfOcTree(double resolution,
               float truncationDistance,
               float maxrange,
               bool useConstWeight,
               bool useWeightDropoff,
               float dropOffEpsilon,
               float maxWeight);

    virtual TsdfOcTree* create() const {return new TsdfOcTree(resolution,
                                                               truncationDistance,
                                                               maxrange,
                                                               useConstWeight,
                                                               useWeightDropoff,
                                                               dropOffEpsilon,
                                                               maxWeight); }

    virtual std::string getTreeType() const {return "TsdfOcTree";}

    virtual void insertPointCloud(const octomap::Pointcloud& scan, 
                                  const octomap::point3d& sensor_origin,
                                  octomath::Pose6D world2cam,
                                  bool discretize);

    void computeDiscreteUpdate(const octomap::Pointcloud& scan, 
                              const octomap::point3d& sensor_origin,
                              octomath::Pose6D world2cam);
    
    void computeUpdate(const octomap::Pointcloud& scan, 
                       const octomap::point3d& sensor_origin,
                       octomath::Pose6D world2cam);

    void computeUpdate(const octomap::Pointcloud& scan,
                       const octomap::Pointcloud& origins,
                       const octomap::Pointcloud& endPoints,
                       octomath::Pose6D world2cam);

    float getVoxelWeight(const octomap::point3d& point, bool use_const_weight) const;

    float computeDistance(const octomap::point3d& origin,
                          const octomap::point3d& point,
                          const octomap::point3d& voxel_center);

    TsdfOcTreeNode* updateNode(const octomap::OcTreeKey& key,
                               const float w, const float sdf, 
                               const float defaultTruncationDistance,
                               const float dropoffEpsilon,
                               bool useWeightDropoff,
                               float maxWeight);

    TsdfOcTreeNode* updateNodeRecurs(TsdfOcTreeNode* node,
                                    bool node_just_created,
                                    const octomap::OcTreeKey& key,
                                    unsigned int depth,
                                    const float w, const float sdf);

    void extractSurfacePontCloud(pcl::PointCloud<pcl::PointXYZ> &cloud);
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
      TsdfOcTree* tree = new TsdfOcTree(0.1, 0.004, 1.0, false, true, 0.001, 10000.0);
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

   float truncationDistance;
   float maxrange;
   bool useConstWeight;
   bool useWeightDropoff;
   float dropOffEpsilon;
   float maxWeight;
};

}

#endif // TSDFOCTREE_H