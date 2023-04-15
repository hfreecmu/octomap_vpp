#ifndef VPOCTREE_H
#define VPOCTREE_H

#include <octomap/OcTreeNode.h>

namespace octomap_vpp
{

class VPBaseNode : public octomap::OcTreeNode
{
public:
  VPBaseNode() : OcTreeNode() {}

  virtual double getVPOccupancy() const = 0;

  virtual bool isHardUnknown() const = 0;

  bool isUnknown(float minThresh, float maxThresh) const
  {
    if (isHardUnknown())
      return true;
      
    double occ = getVPOccupancy();
    return (occ < maxThresh) && (occ > minThresh);
  }

  bool isOccupied(float thresh) const
  {
    if (isHardUnknown())
      return false;

    return getVPOccupancy() > thresh;
  }

};

}

#endif // VPOCTREE_H
