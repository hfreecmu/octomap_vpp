#ifndef OCCOCTREE_H
#define OCCOCTREE_H

#include "VPBaseNode.h"
#include <octomap/OccupancyOcTreeBase.h>

namespace octomap_vpp
{
class OccOcTreeNode : public VPBaseNode
{
public:
  OccOcTreeNode() : VPBaseNode() {}

  OccOcTreeNode(const OccOcTreeNode& rhs) : VPBaseNode(rhs) {}

  bool operator==(const OccOcTreeNode& rhs) const{
    return (rhs.value == value);
  }

  void copyData(const OccOcTreeNode& from){
    VPBaseNode::copyData(from);
  }

  std::ostream& writeData(std::ostream &s) const
  {
    s.write((const char*) &value, sizeof(value)); // occupancy

    return s;
  }

  std::istream& readData(std::istream &s)
  {
    s.read((char*) &value, sizeof(value)); // occupancy

    return s;
  }

  virtual double getVPOccupancy() const override
  { 
    return getOccupancy();
  }

  virtual bool isHardUnknown() const override
  {
    //if in map we know it
    return false;
  }
};

class OccOcTree : public octomap::OccupancyOcTreeBase <OccOcTreeNode>
{
public:
    OccOcTree(double resolution);

    virtual OccOcTree* create() const {return new OccOcTree(resolution); }

    virtual std::string getTreeType() const {return "OcTree";}

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
      OccOcTree* tree = new OccOcTree(0.1);
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

#endif // OCCOCTREE_H