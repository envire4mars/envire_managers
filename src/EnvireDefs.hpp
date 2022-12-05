#ifndef MARS_PLUGINS_ENVIRE_MANAGERS_MARSDEFS_H
#define MARS_PLUGINS_ENVIRE_MANAGERS_MARSDEFS_H
#include <string>

#include <envire_core/graph/EnvireGraph.hpp>
#include <envire_core/items/Item.hpp>

namespace mars {

  namespace sim {
    class SimEntity;
    class SimNode;
    class SimJoint;
  }

  namespace plugins {
    namespace envire_managers {

      typedef envire::core::GraphTraits::vertex_descriptor VertexDesc;

      // SimEntity
      typedef std::shared_ptr<mars::sim::SimEntity> SimEntityPtr;
      typedef envire::core::Item<SimEntityPtr> SimEntityItem;
      typedef SimEntityItem::Ptr SimEntityItemPtr;
      typedef envire::core::EnvireGraph::ItemIterator<SimEntityItem> SimEntityItemItr;

      // SimNode
      typedef std::shared_ptr<mars::sim::SimNode> SimNodePtr;
      typedef envire::core::Item<SimNodePtr> SimNodeItem;
      typedef SimNodeItem::Ptr SimNodeItemPtr;
      typedef envire::core::EnvireGraph::ItemIterator<SimNodeItem> SimNodeItemItr;

      // SimJoint
      typedef std::shared_ptr<mars::sim::SimJoint> SimJointPtr;
      typedef envire::core::Item<SimJointPtr> SimJointItem;
      typedef SimJointItem::Ptr SimJointItemPtr;
      typedef envire::core::EnvireGraph::ItemIterator<SimJointItem> SimJointItemItr;


// TODO: should be asked to node manager
#define SIM_CENTER_FRAME_NAME std::string("center")
#define MLS_FRAME_NAME std::string("mls_01")
//#define DUMPED_MLS_FRAME_NAME std::string("mls_map") // The name of the frame in which to find a mls in a .graph file
#define DUMPED_MLS_FRAME_NAME std::string("map") // The name of the frame in which to find a mls in a .graph file
#define ENV_AUTOPROJ_ROOT "AUTOPROJ_CURRENT_ROOT"
//#define DEBUG_ENVIRE_MANAGERS 1


/*// TODO: should be set over config
#define MLS_FRAME_NAME std::string("mls_01")
// TODO: should be set over config
#define ROBOT_NAME std::string("Asguard_v4")
// TODO: should be set over config
#define ROBOT_ROOT_LINK_NAME std::string("body")
// TODO: do we need this?
#define ASGUARD_PATH std::string("/models/robots/asguard_v4/smurf/asguard_v4.smurf")
// This is the name of the mls frame in the serialized graph that can be loaded
// by mars
#define DUMPED_MLS_FRAME_NAME std::string("mls_map")
//#define DEBUG_WORLD_PHYSICS 1 // Comment in to have logs from the physics simulator controller
#define DRAW_MLS_CONTACTS 1 // Comment in to have logs from the physics simulator controller
*/


    }
  }
}

#endif // MARS_PLUGINS_ENVIRE_MANAGERS_MARSDEFS_H


