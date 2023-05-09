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

  virtual double getVPLogOdds() const = 0;

  virtual bool isHardUnknown() const = 0;

  virtual float getVPRoiLogOdds() const = 0;

  // float getVPLogOdds() const
  // {
  //   //TODO
  //   //this is actually going to call prob then logodd so fix
  //   return octomap::logodds(getVPOccupancy());
  // }

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
