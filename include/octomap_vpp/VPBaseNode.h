#ifndef VPOCTREE_H
#define VPOCTREE_H

#include <octomap/OcTreeNode.h>
#include <octomap/OcTreeKey.h>
#include <unordered_map>

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

  bool getFruitletId(uint8_t &fruitletId) const
  {
    if (fruitletIds.size() == 0)
      return false;
    
    float maxWeight = 0;
    for (auto entry : fruitletIds)
    {
      if (entry.second > maxWeight)
      {
        maxWeight = entry.second;
        fruitletId = entry.first;
      }
    }

    return (maxWeight != 0);
  }

  void updateFruitletId(uint8_t fruitletId, float weight)
  {
    if (fruitletIds.find(fruitletId) == fruitletIds.end())
    {
      std::pair<uint8_t, float> entry(fruitletId, 0);
      fruitletIds.insert(entry);
    }

    fruitletIds.at(fruitletId) = fruitletIds.at(fruitletId) + weight;
  }

  void addCameraPosition(const octomap::OcTreeKey &key)
  {
    cameraPositions.insert(key);
  }

  octomap::KeySet& getCameraPositions() {return cameraPositions;}

public:
  //fruitletIds stores weight
  std::unordered_map<uint8_t, float> fruitletIds;
  octomap::KeySet cameraPositions;
};

}

#endif // VPOCTREE_H
