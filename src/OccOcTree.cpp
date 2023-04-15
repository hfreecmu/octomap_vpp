#include "octomap_vpp/OccOcTree.h"

namespace octomap_vpp
{

OccOcTree::OccOcTree(double resolution) : octomap::OccupancyOcTreeBase <OccOcTreeNode>(resolution)
{
  ocTreeMemberInit.ensureLinking();
}

OccOcTree::StaticMemberInitializer OccOcTree::ocTreeMemberInit;

}