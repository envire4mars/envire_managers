/*
 *  Copyright 2011, 2012, DFKI GmbH Robotics Innovation Center
 *
 *  This file is part of the MARS simulation framework.
 *
 *  MARS is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU Lesser General Public License
 *  as published by the Free Software Foundation, either version 3
 *  of the License, or (at your option) any later version.
 *
 *  MARS is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Lesser General Public License for more details.
 *
 *   You should have received a copy of the GNU Lesser General Public License
 *   along with MARS.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

/**
 * \file EnvireEnvireNodeManager.cpp
 * \author Malte Roemmermann
 * \brief "EnvireEnvireNodeManager" is the class that manage all nodes and their
 * operations and communication between the different modules of the simulation.
 */

#include "EnvireNodeManager.hpp"

#include <mars/sim/SimNode.h>
#include <mars/sim/SimJoint.h>
#include <mars/sim/PhysicsMapper.h>

#include <mars/interfaces/sim/LoadCenter.h>
#include <mars/interfaces/sim/SimulatorInterface.h>
#include <mars/interfaces/sim/JointManagerInterface.h>
#include <mars/interfaces/graphics/GraphicsManagerInterface.h>
#include <mars/interfaces/terrainStruct.h>
#include <mars/interfaces/Logging.hpp>

#include <lib_manager/LibManager.hpp>

#include <mars/interfaces/utils.h>
#include <mars/utils/mathUtils.h>
#include <mars/utils/misc.h>

#include <stdexcept>

#include <mars/utils/MutexLocker.h>

#include "EnvireDefs.hpp"

#include "EnvireStorageManager.hpp"

#ifdef _MSC_VER
#define __PRETTY_FUNCTION__  __FUNCTION__ __FUNCSIG__
#endif

namespace mars {
  namespace plugins {
        namespace envire_managers {


    /**
     *\brief Initialization of a new EnvireNodeManager
     *
     * pre:
     *     - a pointer to a ControlCenter is needed
     * post:
     *     - next_node_id should be initialized to one
     */
    EnvireNodeManager::EnvireNodeManager(mars::interfaces::ControlCenter *c,
                             lib_manager::LibManager *theManager) :
                                                 next_node_id(1),
                                                 update_all_nodes(false),
                                                 visual_rep(1),
                                                 maxGroupID(0),
                                                 libManager(theManager),
                                                 control(c)
    {
      //if(control->graphics) {
      //  mars::interfaces::GraphicsUpdateInterface *gui = static_cast<mars::interfaces::GraphicsUpdateInterface*>(this);
      //  control->graphics->addGraphicsUpdateInterface(gui);
      //}

        // keep updating tree
      EnvireStorageManager::instance()->getGraph()->getTree(SIM_CENTER_FRAME_NAME, true, &graphTreeView);
    }


    mars::interfaces::NodeId EnvireNodeManager::createPrimitiveNode(const std::string &name,
                                            mars::interfaces::NodeType type,
                                            bool moveable,
                                            const mars::utils::Vector &pos,
                                            const mars::utils::Vector &extension,
                                            double mass,
                                            const mars::utils::Quaternion &orientation,
                                            bool disablePhysics) {
      mars::interfaces::NodeData s;
      s.initPrimitive(type, extension, mass);
      s.name = name;
      s.pos = pos;
      s.rot = orientation;
      s.movable = moveable;
      s.noPhysical = disablePhysics;
      return addNode(&s);
    }

    mars::interfaces::NodeId EnvireNodeManager::addNode(mars::interfaces::NodeData *nodeS, bool reload,
                                bool loadGraphics) {

        // FIX: if frameID is not set
        // FIX: add source and target frame id in node data
        // so we can specify where the node should be placed
        if (nodeS->frameID.empty())
            nodeS->frameID = nodeS->name;

        iMutex.lock();
        nodeS->index = next_node_id;
        next_node_id++;
        iMutex.unlock();

        // ------ NOT RELOADED OBJECTS -> TERRAIN
        if (reload == false) {
            LOG_INFO(("EnvireNodeManager::addNode: non reloaded object Terrain: " + nodeS->name).c_str());
            iMutex.lock();

            //TODO: check if we can take out the mars_graphics

            mars::interfaces::NodeData reloadNode = *nodeS;
            if((nodeS->physicMode == mars::interfaces::NODE_TYPE_TERRAIN) && nodeS->terrain ) {

                if(!control->loadCenter){
                    LOG_ERROR("EnvireNodeManager:: loadCenter is missing, can not create terrain Node");
                    return INVALID_ID;
                }

                if (!control->loadCenter->loadHeightmap){
                    mars::interfaces::GraphicsManagerInterface *g = libManager->getLibraryAs<mars::interfaces::GraphicsManagerInterface>("mars_graphics");
                    if(!g) {
                        libManager->loadLibrary("mars_graphics", NULL, false, true);
                        g = libManager->getLibraryAs<mars::interfaces::GraphicsManagerInterface>("mars_graphics");
                    }
                    if(g) {
                        reloadNode.terrain = new(mars::interfaces::terrainStruct);
                        *(reloadNode.terrain) = *(nodeS->terrain);
                        control->loadCenter->loadHeightmap = g->getLoadHeightmapInterface();
                        control->loadCenter->loadHeightmap->readPixelData(reloadNode.terrain);
                        libManager->releaseLibrary("mars_graphics");
                        LOG_INFO("EnvireNodeManager:: mars_graphics was just released");
                        if(!reloadNode.terrain->pixelData) {
                            LOG_ERROR("EnvireNodeManager::addNode: could not load image for terrain");
                            return INVALID_ID;
                        }
                    } else {
                        LOG_ERROR("EnvireNodeManager:: loadHeightmap is missing, can not create Node");
                    }
                }
            }

            //simNodesReload.push_back(reloadNode);

            if (nodeS->c_params.friction_direction1) {
                mars::utils::Vector *tmp = new mars::utils::Vector();
                *tmp = *(nodeS->c_params.friction_direction1);

                //if(simNodesReload.back().index == nodeS->index) {
                //    simNodesReload.back().c_params.friction_direction1 = tmp;
                //}
            }
            iMutex.unlock();
        }

        // to check some preconditions
        if (nodeS->groupID < 0) {
            nodeS->groupID = 0;
        } else if (nodeS->groupID > maxGroupID) {
            maxGroupID = nodeS->groupID;
        }

        // ------ NODE_TYPE_MESH
        // convert obj to ode mesh
        if((nodeS->physicMode == mars::interfaces::NODE_TYPE_MESH) && (nodeS->terrain == 0) ) {
            LOG_INFO(("EnvireNodeManager::addNode: NODE_TYPE_MESH: " + nodeS->name).c_str());
            if(!control->loadCenter) {
                LOG_ERROR("EnvireNodeManager:: loadCenter is missing, can not create Node");
                return INVALID_ID;
            }
            if(!control->loadCenter->loadMesh) {
                mars::interfaces::GraphicsManagerInterface *g = libManager->getLibraryAs<mars::interfaces::GraphicsManagerInterface>("mars_graphics");
                if(!g) {
                    libManager->loadLibrary("mars_graphics", NULL, false, true);
                    g = libManager->getLibraryAs<mars::interfaces::GraphicsManagerInterface>("mars_graphics");
                }
                if(g) {
                    control->loadCenter->loadMesh = g->getLoadMeshInterface();
                }
                else {
                    LOG_ERROR("EnvireNodeManager:: loadMesh is missing, can not create Node");
                }
            }
            control->loadCenter->loadMesh->getPhysicsFromMesh(nodeS);
        }

        // ------ NODE_TYPE_TERRAIN
        if((nodeS->physicMode == mars::interfaces::NODE_TYPE_TERRAIN) && nodeS->terrain ) {
            LOG_DEBUG(("EnvireNodeManager::addNode: NODE_TYPE_TERRAIN and nodeS->terrain: " + nodeS->name).c_str());
            if(!nodeS->terrain->pixelData) {
                if(!control->loadCenter) {
                    LOG_ERROR("EnvireNodeManager:: loadCenter is missing, can not create Node");
                    return INVALID_ID;
                }

                bool release_graphics = false;
                if(!control->loadCenter->loadHeightmap) {
                    mars::interfaces::GraphicsManagerInterface *g = libManager->getLibraryAs<mars::interfaces::GraphicsManagerInterface>("mars_graphics");
                    release_graphics = true;
                    if(!g) {
                        libManager->loadLibrary("mars_graphics", NULL, false, true);
                        g = libManager->getLibraryAs<mars::interfaces::GraphicsManagerInterface>("mars_graphics");
                    }
                    if(g) {
                        control->loadCenter->loadHeightmap = g->getLoadHeightmapInterface();
                    } else {
                        LOG_ERROR("EnvireNodeManager:: loadHeightmap is missing, can not create Node");
                        return INVALID_ID;
                    }
                }

                control->loadCenter->loadHeightmap->readPixelData(nodeS->terrain);
                if (release_graphics){
                    libManager->releaseLibrary("mars_graphics");
                    LOG_INFO("EnvireNodeManager:: mars_graphics was just released");
                } else {
                    LOG_INFO("EnvireNodeManager:: mars_graphics was not released");
                }
                if(!nodeS->terrain->pixelData) {
                    LOG_ERROR("EnvireNodeManager::addNode: could not load image for terrain");
                    return INVALID_ID;
                }
            }
        }

        // this should be done somewhere else
        // if we have a relative position, we have to calculate the absolute
        // position here
        if(nodeS->relative_id != 0) {
            setNodeStructPositionFromRelative(nodeS);
        }

        // create a node object
        std::shared_ptr<mars::sim::SimNode> newNode(new mars::sim::SimNode(control, *nodeS));

        // ------ PHYSICAL NODE
        if(nodeS->noPhysical == false) {
            LOG_INFO(("EnvireNodeManager::addNode: physical: " + nodeS->name).c_str());
            // create an interface object to the physics
            std::shared_ptr<mars::interfaces::NodeInterface> newNodeInterface = mars::sim::PhysicsMapper::newNodePhysics(control->sim->getPhysics());

            // can not create physics
            if (!newNodeInterface->createNode(nodeS)) {
                // if no node was created in physics
                // delete the objects
                newNode.reset();
                newNodeInterface.reset();
                // and return false
                LOG_ERROR("EnvireNodeManager::addNode: No node was created in physics.");
                return INVALID_ID;
            }

            // put all data to the correct place
            //      newNode->setSNode(*nodeS);
            newNode->setInterface(newNodeInterface);

            // if frame is not in the graph, create one
            if (EnvireStorageManager::instance()->getGraph()->containsFrame(nodeS->frameID) == false)
            {
                LOG_DEBUG(("[EnvireNodeManager::addNode] create new transformation between center and " + nodeS->frameID).c_str());
                envire::core::Transform nodeTransf(nodeS->pos, nodeS->rot);
                nodeTransf.time = base::Time::now();
                EnvireStorageManager::instance()->getGraph()->addTransform(SIM_CENTER_FRAME_NAME, nodeS->frameID, nodeTransf);
            }

            iMutex.lock();
            // add node into the graph
            SimNodeItemPtr newNodeItemPtr( new SimNodeItem(newNode));
            EnvireStorageManager::instance()->getGraph()->addItemToFrame(nodeS->frameID, newNodeItemPtr);

            // add node into the node map
            simNodes[nodeS->index] = newNodeItemPtr;
            //if (nodeS->movable)
            //    simNodesDyn[nodeS->index] = newNodeItemPtr;
            iMutex.unlock();

            control->sim->sceneHasChanged(false);
            mars::interfaces::NodeId id;
        } else {  // ------ NONE PHYSICAL NODE
            LOG_INFO(("EnvireNodeManager::addNode: nonPhysical: " + nodeS->name).c_str());

            iMutex.lock();
            // if frame is not in the graph, create one
            if (EnvireStorageManager::instance()->getGraph()->containsFrame(nodeS->frameID) == false)
            {
                LOG_DEBUG(("[EnvireNodeManager::addNode] create new transformation between center and " + nodeS->frameID).c_str());
                envire::core::Transform nodeTransf(nodeS->pos, nodeS->rot);
                nodeTransf.time = base::Time::now();
                EnvireStorageManager::instance()->getGraph()->addTransform(SIM_CENTER_FRAME_NAME, nodeS->frameID, nodeTransf);
            }

            // add node into the graph
            SimNodeItemPtr newNodeItemPtr( new SimNodeItem(newNode));
            EnvireStorageManager::instance()->getGraph()->addItemToFrame(nodeS->frameID, newNodeItemPtr);
            LOG_DEBUG(("[EnvireNodeManager::addNode] non physical " + nodeS->frameID + " " + nodeS->name).c_str());

            simNodes[nodeS->index] = newNodeItemPtr;
            //if (nodeS->movable)
              //simNodesDyn[nodeS->index] = newNodeItemPtr;

            iMutex.unlock();

            control->sim->sceneHasChanged(false);
        }
        return nodeS->index;
    }

  //   /**
  //    *\brief This function maps a mars::interfaces::terrainStruct to a node struct and adds
  //    * that node to the simulation.
  //    *
  //    */
  mars::interfaces::NodeId EnvireNodeManager::addTerrain(mars::interfaces::terrainStruct* terrain) {
          printf("not implemented : %s\n", __PRETTY_FUNCTION__);
  //     mars::interfaces::NodeData newNode;
  //     mars::interfaces::terrainStruct *newTerrain = new mars::interfaces::terrainStruct(*terrain);
  //     mars::utils::sRotation trot = {0, 0, 0};

  //     newNode.name = terrain->name;
  //     newNode.filename = terrain->srcname;
  //     newNode.terrain = newTerrain;
  //     newNode.movable = false;
  //     newNode.groupID = 0;
  //     newNode.relative_id = 0;
  //     newNode.physicMode = mars::interfaces::NODE_TYPE_TERRAIN;
  //     newNode.pos = mars::utils::Vector(0.0, 0.0, 0.0);
  //     newNode.rot = eulerToQuaternion(trot);
  //     newNode.density = 1;
  //     newNode.mass = 0;
  //     newNode.ext = mars::utils::Vector(terrain->targetWidth, terrain->targetHeight,
  //                          terrain->scale);
  //     newNode.material = terrain->material;
  //     control->sim->sceneHasChanged(false);
  //     return addNode(&newNode, true);
          return 0;
   }

  //   /**
  //    *\brief This function adds an std::vector of nodes to the factory.
  //    * The functionality is implemented in the GUI, but should
  //    * move to the node factory soon.
  //    *
  //    */
  std::vector<mars::interfaces::NodeId> EnvireNodeManager::addNode(std::vector<mars::interfaces::NodeData> v_NodeData) {
          printf("not implemented : %s\n", __PRETTY_FUNCTION__);
  //     std::vector<mars::interfaces::NodeId> tmp;
  //     std::vector<mars::interfaces::NodeData>::iterator iter;

  //     control->sim->sceneHasChanged(false);
  //     for(iter=v_NodeData.begin(); iter!=v_NodeData.end(); iter++)
  //       tmp.push_back(addNode(&(*iter)));
  //     return tmp;
         return std::vector<mars::interfaces::NodeId>();
   }

  //   /**
  //    *\brief This function adds an primitive to the simulation.
  //    * The functionality is implemented in the GUI, but should
  //    * move to the node factory soon.
  //    *
  //    */
    mars::interfaces::NodeId EnvireNodeManager::addPrimitive(mars::interfaces::NodeData *snode) {
            printf("not implemented : %s\n", __PRETTY_FUNCTION__);
  //     control->sim->sceneHasChanged(false);
  //     return addNode(snode);
            return 0;
    }

    /**
     *\brief returns true if the node with the given id exists
     *
     */
    bool EnvireNodeManager::exists(mars::interfaces::NodeId id) const {
        printf("not implemented : %s\n", __PRETTY_FUNCTION__);
    //   NodeMap::const_iterator iter = simNodes.find(id);
    //   if(iter != simNodes.end()) {
    //     return true;
    //   }
      return false;
    }

    /**
     *\brief returns the number of nodes added to the simulation
     *
     */
    int EnvireNodeManager::getNodeCount() const {
        mars::utils::MutexLocker locker(&iMutex);
        return simNodes.size();
         return 0;
    }

   mars::interfaces::NodeId EnvireNodeManager::getNextNodeID() const {
          printf("not implemented : %s\n", __PRETTY_FUNCTION__);
  //     return next_node_id;
          return 0;
     }

  //   /**
  //    * \brief Change a node. The simulation must be updated in here.
  //    * doc has to be written
  //    */
     void EnvireNodeManager::editNode(mars::interfaces::NodeData *nodeS, int changes) {
            printf("not implemented : %s\n", __PRETTY_FUNCTION__);
  //     NodeMap::iterator iter;
  //     std::vector<mars::sim::SimJoint*>::iterator jter;
  //     std::vector<int> gids;
  //     NodeMap::iterator it;
  //     mars::utils::Vector offset;
  //     mars::utils::Vector rotation_point;

  //     //cout << "EnvireNodeManager::editNode !!!" << endl;
  //     // first lock all core functions
  //     iMutex.lock();

  //     iter = simNodes.find(nodeS->index);
  //     if(iter == simNodes.end()) {
  //       iMutex.unlock();
  //       LOG_ERROR("EnvireNodeManager::editNode: node id not found!");
  //       return;
  //     }

  //     mars::sim::SimNode *editedNode = iter->second;
  //     mars::interfaces::NodeData sNode = editedNode->getSNode();
  //     if(changes & mars::interfaces::EDIT_NODE_POS) {
  //       if(changes & mars::interfaces::EDIT_NODE_MOVE_ALL) {
  //         // first move the node an all nodes of the group
  //         offset = editedNode->setPosition(nodeS->pos, true);
  //         // then move recursive all nodes that are connected through
  //         // joints to the node
  //         std::vector<mars::sim::SimJoint*> joints = control->joints->getSimJoints();
  //         if(editedNode->getGroupID())
  //           gids.push_back(editedNode->getGroupID());
  //         NodeMap nodes = simNodes;
  //         std::vector<mars::sim::SimJoint*> jointsj = joints;
  //         nodes.erase(nodes.find(editedNode->getID()));
  //         moveNodeRecursive(nodeS->index, offset, &joints, &gids, &nodes);
  //       } else {
  //         if(nodeS->relative_id) {
  //           iMutex.unlock();
  //           setNodeStructPositionFromRelative(nodeS);
  //           iMutex.lock();
  //         }
  //         mars::utils::Vector diff = nodeS->pos - editedNode->getPosition();
  //         editedNode->setPosition(nodeS->pos, false);

  //         // new implementation in jointManager?
  //         NodeMap nodes = simNodes;
  //         NodeMap nodesj = simNodes;
  //         std::vector<mars::sim::SimJoint*> jointsj = control->joints->getSimJoints();
  //         nodes.erase(nodes.find(editedNode->getID()));
  //         moveRelativeNodes(*editedNode, &nodes, diff);

  //         if(sNode.groupID != 0) {
  //           for(it=simNodes.begin(); it!=simNodes.end(); ++it) {
  //             if(it->second->getGroupID() == sNode.groupID) {
  //               control->joints->reattacheJoints(it->second->getID());
  //             }
  //           }
  //         }
  //         else {
  //           control->joints->reattacheJoints(nodeS->index);
  //         }
  //         iMutex.unlock();
  //         resetRelativeJoints(*editedNode, &nodesj, &jointsj);
  //         iMutex.lock();
  //       }
  //       update_all_nodes = true;
  //     }
  //     if(changes & mars::interfaces::EDIT_NODE_ROT) {
  //       mars::utils::Quaternion q(mars::utils::Quaternion::Identity());
  //       if(changes & mars::interfaces::EDIT_NODE_MOVE_ALL) {
  //         // first move the node an all nodes of the group
  //         rotation_point = editedNode->getPosition();
  //         // the first node have to be rotated normal, not at a point
  //         // and should return the relative rotation it executes
  //         q = editedNode->setRotation(nodeS->rot, true);
  //         // then rotate recursive all nodes that are connected through
  //         // joints to the node
  //         std::vector<mars::sim::SimJoint*> joints = control->joints->getSimJoints();
  //         if(editedNode->getGroupID())
  //           gids.push_back(editedNode->getGroupID());
  //         NodeMap nodes = simNodes;
  //         std::vector<mars::sim::SimJoint*> jointsj = control->joints->getSimJoints();
  //         nodes.erase(nodes.find(editedNode->getID()));
  //         rotateNodeRecursive(nodeS->index, rotation_point, q, &joints,
  //                             &gids, &nodes);
  //       } else {
  //         if(nodeS->relative_id) {
  //           iMutex.unlock();
  //           setNodeStructPositionFromRelative(nodeS);
  //           iMutex.lock();
  //           mars::interfaces::NodeData da = editedNode->getSNode();
  //           da.rot = nodeS->rot;
  //           editedNode->setRelativePosition(da);
  //         }
  //         rotation_point = editedNode->getPosition();
  //         //if(nodeS->relative_id && !load_actual)
  //         //setNodeStructPositionFromRelative(nodeS);
  //         q = editedNode->setRotation(nodeS->rot, 0);

  //         //(*iter)->rotateAtPoint(&rotation_point, &nodeS->rot, false);

  //         NodeMap nodes = simNodes;
  //         NodeMap nodesj = simNodes;
  //         std::vector<mars::sim::SimJoint*> jointsj = control->joints->getSimJoints();
  //         nodes.erase(nodes.find(editedNode->getID()));
  //         rotateRelativeNodes(*editedNode, &nodes, rotation_point, q);

  //         if(sNode.groupID != 0) {
  //           for(it=simNodes.begin(); it!=simNodes.end(); ++it) {
  //             if(it->second->getGroupID() == sNode.groupID) {
  //               control->joints->reattacheJoints(it->second->getID());
  //             }
  //           }
  //         }
  //         else {
  //           control->joints->reattacheJoints(nodeS->index);
  //         }

  //         iMutex.unlock(); // is this desired???
  //         resetRelativeJoints(*editedNode, &nodesj, &jointsj);
  //         iMutex.lock();
  //       }
  //       update_all_nodes = true;
  //     }
  //     if ((changes & mars::interfaces::EDIT_NODE_SIZE) || (changes & mars::interfaces::EDIT_NODE_TYPE) || (changes & mars::interfaces::EDIT_NODE_CONTACT) ||
  //         (changes & mars::interfaces::EDIT_NODE_MASS) || (changes & mars::interfaces::EDIT_NODE_NAME) ||
  //         (changes & mars::interfaces::EDIT_NODE_GROUP) || (changes & mars::interfaces::EDIT_NODE_PHYSICS)) {
  //       //cout << "mars::interfaces::EDIT_NODE_SIZE !!!" << endl;
  //       mars::interfaces::NodeData sNode = editedNode->getSNode();
  //       if(control->graphics) {
  //         mars::utils::Vector scale;
  //         if(sNode.filename == "PRIMITIVE") {
  //           scale = nodeS->ext;
  //           if(sNode.physicMode == mars::interfaces::NODE_TYPE_SPHERE) {
  //             scale.x() *= 2;
  //             scale.y() = scale.z() = scale.x();
  //           }
  //           // todo: set scale for cylinder and capsule
  //         } else {
  //           scale = sNode.visual_size-sNode.ext;
  //           scale += nodeS->ext;
  //           nodeS->visual_size = scale;
  //         }
  //         control->graphics->setDrawObjectScale(editedNode->getGraphicsID(), scale);
  //         control->graphics->setDrawObjectScale(editedNode->getGraphicsID2(), nodeS->ext);
  //       }
  //       editedNode->changeNode(nodeS);
  //       if(nodeS->groupID > maxGroupID) {
  //         maxGroupID = nodeS->groupID;
  //       }
  //       /*
  //         if (changes & mars::interfaces::EDIT_NODE_SIZE) {
  //         NodeMap nodes = simNodes;
  //         NodeMap nodesj = simNodes;
  //         std::vector<mars::sim::SimJoint*> jointsj = control->joints->getSimJoints();
  //         nodes.erase(nodes.find(editedNode->getID()));
  //         resetRelativeNodes(*editedNode, &nodes);
  //         iMutex.unlock(); // is this desired???
  //         resetRelativeJoints(*editedNode, &nodesj, &jointsj);
  //         iMutex.lock();
  //         }
  //       */
  //     }
  //     if ((changes & mars::interfaces::EDIT_NODE_MATERIAL)) {
  //       editedNode->setMaterial(nodeS->material);
  //       if(control->graphics)
  //         control->graphics->setDrawObjectMaterial(editedNode->getGraphicsID(),
  //                                                  nodeS->material);
  //     }

  //     // vs: updateNodesFromPhysics();
  //     iMutex.unlock();
  //     updateDynamicNodes(0, false);
   }

    void EnvireNodeManager::changeGroup(mars::interfaces::NodeId id, int group) {
            printf("not implemented : %s\n", __PRETTY_FUNCTION__);
  //     CPP_UNUSED(id);
  //     CPP_UNUSED(group);
     }

    /**
     * \brief Fills a list of core_object_exchange objects with node
     * iformations.
     */
  void EnvireNodeManager::getListNodes(std::vector<mars::interfaces::core_objects_exchange>* nodeList) const {
    mars::interfaces::core_objects_exchange obj;
    NodeMap::const_iterator iter;
    mars::utils::MutexLocker locker(&iMutex);
    nodeList->clear();
    for (iter = simNodes.begin(); iter != simNodes.end(); iter++) {
        iter->second->getData()->getCoreExchange(&obj);
        nodeList->push_back(obj);
    }
   }

  //   /** \brief
  //    * Fills one core_object_exchange object with node information
  //    * of the node with the given id.
  //    */
  void EnvireNodeManager::getNodeExchange(mars::interfaces::NodeId id, mars::interfaces::core_objects_exchange* obj) const {
          printf("not implemented : %s\n", __PRETTY_FUNCTION__);
  //     mars::utils::MutexLocker locker(&iMutex);
  //     NodeMap::const_iterator iter = simNodes.find(id);
  //     if (iter != simNodes.end())
  //       iter->second->getCoreExchange(obj);
  //     else
  //       obj = NULL;
   }

    /**
     * \brief get the full struct of a Node for editing purposes
     * \throw std::runtime_error if the node cannot be found
     */
    const mars::interfaces::NodeData EnvireNodeManager::getFullNode(mars::interfaces::NodeId id) const {
        mars::utils::MutexLocker locker(&iMutex);
        NodeMap::const_iterator iter = simNodes.find(id);
        if (iter != simNodes.end())
            return iter->second->getData()->getSNode();
        else {
            char msg[128];
            sprintf(msg, "could not find node with id: %lu", id);
            throw std::runtime_error(msg);
        }
    }

  //   /**
  //    * \brief removes the node with the corresponding id.
  //    *
  //    * Ok, first we have to check if the node is an element of a composite object.
  //    * If so, we have to cases:
  //    *
  //    * first case:
  //    *       - other nodes exist, which are element of the composite object.
  //    *       -> we can delete the node and keep the group
  //    *
  //    * second case:
  //    *       - this node is the only (last) one of the composite object
  //    *       -> we have to delete both, the node and the group
  //    *
  //    * At this moment we only the physical implementation handle the groups,
  //    * so the tow cases can be handled here in the same way:
  //    * -> tell the physics to destroy the object and remove the node from
  //    *    the core.
  //    *
  //    * What about joints?
  //    */
     void EnvireNodeManager::removeNode(mars::interfaces::NodeId id, bool clearGraphics) {
            printf("not implemented : %s\n", __PRETTY_FUNCTION__);
  //     removeNode(id, true, clearGraphics);
    }

  void EnvireNodeManager::removeNode(mars::interfaces::NodeId id, bool lock, bool clearGraphics) {
          printf("not implemented : %s\n", __PRETTY_FUNCTION__);
  //     NodeMap::iterator iter; //NodeMap is a map containing an id and a mars::sim::SimNode
  //     mars::sim::SimNode *tmpNode = 0;

  //     if(lock) iMutex.lock();

  //     iter = simNodes.find(id);
  //     if (iter != simNodes.end()) {
  //       tmpNode = iter->second; //iter->second is a pointer to the mars::sim::SimNode associated with the map
  //       simNodes.erase(iter);
  //     }

  //     iter = nodesToUpdate.find(id);
  //     if (iter != nodesToUpdate.end()) {
  //       nodesToUpdate.erase(iter);
  //     }

  //     if (tmpNode && tmpNode->isMovable()) {
  //       iter = simNodesDyn.find(id);
  //       if (iter != simNodesDyn.end()) {
  //         simNodesDyn.erase(iter);
  //       }
  //     }

  //     iMutex.unlock();
  //     if(!lock) iMutex.lock();
  //     if (tmpNode) {
  //       clearRelativePosition(id, lock);
  //       if(control->graphics && clearGraphics) {
  //         control->graphics->removeDrawObject(tmpNode->getGraphicsID());
  //         control->graphics->removeDrawObject(tmpNode->getGraphicsID2());
  //       }
  //       delete tmpNode;
  //     }
  //     control->sim->sceneHasChanged(false);
   }

  //   /**
  //    *\brief Set physical dynamic values for the node with the given id.
  //    */
    void EnvireNodeManager::setNodeState(mars::interfaces::NodeId id, const mars::interfaces::nodeState &state) {
            printf("not implemented : %s\n", __PRETTY_FUNCTION__);
  //     mars::utils::MutexLocker locker(&iMutex);
  //     NodeMap::iterator iter = simNodes.find(id);
  //     if (iter != simNodes.end())
  //       iter->second->setPhysicalState(state);
  }

  //   /**
  //    *\brief Get physical dynamic values for the node with the given id.
  //    */
  void EnvireNodeManager::getNodeState(mars::interfaces::NodeId id, mars::interfaces::nodeState *state) const {
          printf("not implemented : %s\n", __PRETTY_FUNCTION__);
  //     mars::utils::MutexLocker locker(&iMutex);
  //     NodeMap::const_iterator iter = simNodes.find(id);
  //     if (iter != simNodes.end())
  //       iter->second->getPhysicalState(state);
   }

  //   /**
  //    *\brief Return the center of mass for the nodes corresponding to
  //    * the id's from the given std::vector.
  //    */
   const mars::utils::Vector EnvireNodeManager::getCenterOfMass(const std::vector<mars::interfaces::NodeId> &ids) const {
          printf("not implemented : %s\n", __PRETTY_FUNCTION__);
          return mars::utils::Vector();

  //     std::vector<mars::interfaces::NodeId>::const_iterator iter;
  //     NodeMap::const_iterator nter;
  //     std::vector<mars::interfaces::NodeInterface*> pNodes;

  //     mars::utils::MutexLocker locker(&iMutex);

  //     for (iter = ids.begin(); iter != ids.end(); iter++) {
  //       nter = simNodes.find(*iter);
  //       if (nter != simNodes.end())
  //         pNodes.push_back(nter->second->getInterface());
  //     }

  //     return control->sim->getPhysics()->getCenterOfMass(pNodes);
   }

  //   /**
  //    *\brief Sets the position of the node with the given id.
  //    */
   void EnvireNodeManager::setPosition(mars::interfaces::NodeId id, const mars::utils::Vector &pos) {
       mars::utils::MutexLocker locker(&iMutex);
       printf("not implemented : %s\n", __PRETTY_FUNCTION__);
       // Find in which frame the simNode with the given Id and set its transformation to the center to be equal to the given position.
       // This is a bit more tricky, as you have to change the last transformation only to match the total tf to the center.
  //     NodeMap::iterator iter = simNodes.find(id);
  //     if (iter != simNodes.end()) {
  //       iter->second->setPosition(pos, 1);
  //       nodesToUpdate[id] = iter->second;
  //     }
   }



   const mars::utils::Vector EnvireNodeManager::getPosition(mars::interfaces::NodeId id) const {
       mars::utils::Vector pos(0.0,0.0,0.0);
       mars::utils::MutexLocker locker(&iMutex);
       NodeMap::const_iterator iter = simNodes.find(id);
       if (iter != simNodes.end())
            pos = iter->second->getData()->getPosition();
       return pos;
   }


   const mars::utils::Quaternion EnvireNodeManager::getRotation(mars::interfaces::NodeId id) const {
       mars::utils::Quaternion q(mars::utils::Quaternion::Identity());
       mars::utils::MutexLocker locker(&iMutex);
       NodeMap::const_iterator iter = simNodes.find(id);
       if (iter != simNodes.end())
         q = iter->second->getData()->getRotation();
       return q;
   }


   const mars::utils::Vector EnvireNodeManager::getLinearVelocity(mars::interfaces::NodeId id) const {
          printf("not implemented : %s\n", __PRETTY_FUNCTION__);
          return mars::utils::Vector();
  //     mars::utils::Vector vel(0.0,0.0,0.0);
  //     mars::utils::MutexLocker locker(&iMutex);
  //     NodeMap::const_iterator iter = simNodes.find(id);
  //     if (iter != simNodes.end())
  //       vel = iter->second->getLinearVelocity();
  //     return vel;
    }

  const mars::utils::Vector EnvireNodeManager::getAngularVelocity(mars::interfaces::NodeId id) const {
          printf("not implemented : %s\n", __PRETTY_FUNCTION__);
          return mars::utils::Vector();
  //     mars::utils::Vector avel(0.0,0.0,0.0);
  //     mars::utils::MutexLocker locker(&iMutex);
  //     NodeMap::const_iterator iter = simNodes.find(id);
  //     if (iter != simNodes.end())
  //       avel = iter->second->getAngularVelocity();
  //     return avel;
    }


   const mars::utils::Vector EnvireNodeManager::getLinearAcceleration(mars::interfaces::NodeId id) const {
          printf("not implemented : %s\n", __PRETTY_FUNCTION__);
          return mars::utils::Vector();
  //     mars::utils::Vector acc(0.0,0.0,0.0);
  //     mars::utils::MutexLocker locker(&iMutex);
  //     NodeMap::const_iterator iter = simNodes.find(id);
  //     if (iter != simNodes.end())
  //       acc = iter->second->getLinearAcceleration();
  //     return acc;
   }

   const mars::utils::Vector EnvireNodeManager::getAngularAcceleration(mars::interfaces::NodeId id) const {
          printf("not implemented : %s\n", __PRETTY_FUNCTION__);
          return mars::utils::Vector();
  //     mars::utils::Vector aacc(0.0,0.0,0.0);
  //     mars::utils::MutexLocker locker(&iMutex);
  //     NodeMap::const_iterator iter = simNodes.find(id);
  //     if (iter != simNodes.end())
  //       aacc = iter->second->getAngularAcceleration();
  //     return aacc;
    }




  //   /**
  //    *\brief Sets the rotation of the node with the given id.
  //    */
    void EnvireNodeManager::setRotation(mars::interfaces::NodeId id, const mars::utils::Quaternion &rot) {
            printf("not implemented : %s\n", __PRETTY_FUNCTION__);
  //     mars::utils::MutexLocker locker(&iMutex);
  //     NodeMap::iterator iter = simNodes.find(id);
  //     if (iter != simNodes.end())
  //       iter->second->setRotation(rot, 1);
   }

    /**
     *\brief Adds a off-center Force to the node with the given id.
     */
    void EnvireNodeManager::applyForce(mars::interfaces::NodeId id, const mars::utils::Vector &force, const mars::utils::Vector &pos) {
        mars::utils::MutexLocker locker(&iMutex);
        NodeMap::iterator iter = simNodes.find(id);
        if (iter != simNodes.end())
            iter->second->getData()->applyForce(force, pos);
  }
    /**
     *\brief Adds a Force to the node with the given id.
     */
   void EnvireNodeManager::applyForce(mars::interfaces::NodeId id, const mars::utils::Vector &force) {
        mars::utils::MutexLocker locker(&iMutex);
        NodeMap::iterator iter = simNodes.find(id);
        if (iter != simNodes.end())
            iter->second->getData()->applyForce(force);
    }

    /**
     *\brief Adds a Torque to the node with the given id.
     */
  void EnvireNodeManager::applyTorque(mars::interfaces::NodeId id, const mars::utils::Vector &torque) {
    mars::utils::MutexLocker locker(&iMutex);
    NodeMap::iterator iter = simNodes.find(id);
    if (iter != simNodes.end())
        iter->second->getData()->applyTorque(torque);
    }


  //   /**
  //    *\brief Sets the contact parameter motion1 for the node with the given id.
  //    */
   void EnvireNodeManager::setContactParamMotion1(mars::interfaces::NodeId id, mars::interfaces::sReal motion) {
          printf("not implemented : %s\n", __PRETTY_FUNCTION__);
  //     mars::utils::MutexLocker locker(&iMutex);
  //     NodeMap::iterator iter = simNodes.find(id);
  //     if (iter != simNodes.end())
  //       iter->second->setContactMotion1(motion);
   }


  //   /**
  //    *\brief Adds a physical sensor to the node with the given id.
  //    */
     void EnvireNodeManager::addNodeSensor(mars::interfaces::BaseNodeSensor *sensor){
       mars::utils::MutexLocker locker(&iMutex);
       NodeMap::iterator iter = simNodes.find(sensor->getAttachedNode());
       if (iter != simNodes.end()) {
         iter->second->getData()->addSensor(sensor);
         NodeMap::iterator kter = simNodesDyn.find(sensor->getAttachedNode());
         if (kter == simNodesDyn.end())
           simNodesDyn[iter->first] = iter->second;
       }
       else
         {
           std::cerr << "Could not find node id " << sensor->getAttachedNode() << " in simNodes and did not call addSensors on the node." << std::endl;
         }
   }

    void EnvireNodeManager::reloadNodeSensor(mars::interfaces::BaseNodeSensor* sensor) {
            printf("not implemented : %s\n", __PRETTY_FUNCTION__);
  //     mars::utils::MutexLocker locker(&iMutex);
  //     NodeMap::iterator iter = simNodes.find(sensor->getAttachedNode());
  //     if (iter != simNodes.end())
  //       iter->second->reloadSensor(sensor);
     }

  //   /**
  //    *\brief Returns a pointer to the mars::sim::SimNode Object.
  //    */

    std::shared_ptr<mars::sim::SimNode> EnvireNodeManager::getSimNode(mars::interfaces::NodeId id) {
        mars::utils::MutexLocker locker(&iMutex);
        NodeMap::const_iterator iter = simNodes.find(id);
        if (iter != simNodes.end())
            return iter->second->getData();
        else
            return std::shared_ptr<mars::sim::SimNode>();
     }


     void EnvireNodeManager::setNodeStructPositionFromRelative(mars::interfaces::NodeData *node) const {
            printf("not implemented : %s\n", __PRETTY_FUNCTION__);
  //     mars::utils::MutexLocker locker(&iMutex);
  //     NodeMap::const_iterator iter = simNodes.find(node->relative_id);
  //     if (iter != simNodes.end()) {
  //       mars::interfaces::NodeData tmpNode = iter->second->getSNode();
  //       getAbsFromRel(tmpNode, node);
  //     }
     }

    void EnvireNodeManager::resetRelativeNodes(const mars::sim::SimNode &node,
                                         NodeMap *nodes,
                                         const mars::utils::Quaternion *rotate) {
            printf("not implemented : %s\n", __PRETTY_FUNCTION__);

  //     NodeMap::iterator iter;
  //     mars::interfaces::NodeData tmpNode, tmpNode2;
  //     mars::sim::SimNode* nextNode;

  //     // TODO: doesn't this function need locking? no
  //     tmpNode = node.getSNode();
  //     for (iter = nodes->begin(); iter != nodes->end(); iter++) {
  //       if (iter->second->getSNode().relative_id == node.getID()) {
  //         nextNode = iter->second;
  //         tmpNode2 = iter->second->getSNode();
  //         if(rotate)
  //           tmpNode2.rot = *rotate * tmpNode2.rot;
  //         getAbsFromRel(tmpNode, &tmpNode2);
  //         iter->second->setPosition(tmpNode2.pos, false);
  //         iter->second->setRotation(tmpNode2.rot, 0);
  //         nodes->erase(iter);
  //         resetRelativeNodes(node, nodes, rotate);
  //         resetRelativeNodes(*nextNode, nodes, rotate);
  //         break;
  //       }
  //     }
    }

    void EnvireNodeManager::resetRelativeJoints(const mars::sim::SimNode &node,
                                          NodeMap *nodes,
                                          std::vector<mars::sim::SimJoint*> *joints,
                                          const mars::utils::Quaternion *rotate) {
      printf("not implemented : %s\n", __PRETTY_FUNCTION__);
  //     NodeMap::iterator iter;
  //     std::vector<mars::sim::SimJoint*>::iterator jter;
  //     std::vector<mars::sim::SimJoint*>::iterator jter2;
  //     mars::sim::SimNode* nextNode;

  //     iMutex.lock();
  //     for (iter = nodes->begin(); iter != nodes->end(); iter++) {
  //       if (iter->second->getSNode().relative_id == node.getID()) {
  //         nextNode = iter->second;
  //         for (jter = joints->begin(); jter != joints->end();) {
  //           if ((*jter)->getSJoint().nodeIndex1 == iter->first ||
  //               (*jter)->getSJoint().nodeIndex2 == iter->first) {
  //             if(rotate) (*jter)->rotateAxis(*rotate);
  //             (*jter)->reattachJoint();
  //             jter2 = jter;
  //             if(jter != joints->begin()) jter--;
  //             else jter = joints->begin();
  //             joints->erase(jter2);
  //           }
  //           else jter++;
  //         }

  //         nodes->erase(iter);
  //         iMutex.unlock();
  //         resetRelativeJoints(node, nodes, joints, rotate);
  //         resetRelativeJoints(*nextNode, nodes, joints, rotate);
  //         iMutex.lock();
  //         break;
  //       }
  //     }
  //     iMutex.unlock();
     }


    void EnvireNodeManager::recursiveHelper(mars::interfaces::NodeId id, const Params *params,
                                      std::vector<std::shared_ptr<mars::sim::SimJoint>> *joints,
                                      std::vector<int> *gids,
                                      NodeMap *nodes,
                                      void (*applyFunc)(mars::sim::SimNode *, const Params *)) {
      std::vector<std::shared_ptr<mars::sim::SimJoint>>::iterator iter;
      std::vector<int>::iterator jter;
      NodeMap::iterator nter;
      mars::interfaces::NodeId id2;
      bool found = false;

      for(jter = gids->begin(); jter != gids->end(); jter++) {
        for(nter = nodes->begin(); nter != nodes->end(); nter++) {
          if(nter->second->getData()->getGroupID() == (*jter)) {
            id2 = nter->first;
            nodes->erase(nter);
            recursiveHelper(id, params, joints, gids, nodes, applyFunc);
            recursiveHelper(id2, params, joints, gids, nodes, applyFunc);
            return;
          }
        }
      }

      for (iter = joints->begin(); iter != joints->end(); iter++) {
        if ((*iter)->getAttachedNode() &&
            (*iter)->getAttachedNode()->getID() == id) {
          for (jter = gids->begin(); jter != gids->end(); jter++) {
            if ((*iter)->getAttachedNode(2) &&
                (*jter) == (*iter)->getAttachedNode(2)->getGroupID()) {
              found = true;
              break;
            }
          }
          if ((*iter)->getAttachedNode(2) &&
              nodes->find((*iter)->getAttachedNode(2)->getID()) != nodes->end()) {
            id2 = (*iter)->getAttachedNode(2)->getID();
            if (!found) {
              if ((*iter)->getAttachedNode(2)->getGroupID())
                gids->push_back((*iter)->getAttachedNode(2)->getGroupID());
              applyFunc((*iter)->getAttachedNode(2).get(), params);
            }
            nodes->erase(nodes->find((*iter)->getAttachedNode(2)->getID()));
            joints->erase(iter);
            recursiveHelper(id, params, joints, gids, nodes, applyFunc);
            recursiveHelper(id2, params, joints, gids, nodes, applyFunc);
            return;
          }
          else found = false;
        } else if ((*iter)->getAttachedNode(2) &&
                   (*iter)->getAttachedNode(2)->getID() == id) {
          for (jter = gids->begin(); jter != gids->end(); jter++) {
            if ((*iter)->getAttachedNode() &&
                (*jter) == (*iter)->getAttachedNode()->getGroupID()) {
              found = true;
              break;
            }
          }
          if(nodes->find((*iter)->getAttachedNode()->getID()) != nodes->end()) {
            id2 = (*iter)->getAttachedNode()->getID();
            if (!found) {
              if ((*iter)->getAttachedNode()->getGroupID())
                gids->push_back((*iter)->getAttachedNode()->getGroupID());
              applyFunc((*iter)->getAttachedNode().get(), params);
            }
            nodes->erase(nodes->find((*iter)->getAttachedNode()->getID()));
            joints->erase(iter);
            recursiveHelper(id, params, joints, gids, nodes, applyFunc);
            recursiveHelper(id2, params, joints, gids, nodes, applyFunc);
            return;
          }
          else found = false;
        }
      }
     }

     void EnvireNodeManager::applyMove(mars::sim::SimNode *node, const Params *params)
     {
      printf("not implemented : %s\n", __PRETTY_FUNCTION__);
  //     const mars::utils::Vector offset = dynamic_cast<const MoveParams*>(params)->offset;
  //     node->setPositionOffset(offset);
     }

    void EnvireNodeManager::applyRotation(mars::sim::SimNode *node, const Params *params)
    {
        const RotationParams *p = dynamic_cast<const RotationParams*>(params);
        node->rotateAtPoint(p->rotation_point, p->rotation, true);
    }

    void EnvireNodeManager::moveNodeRecursive(mars::interfaces::NodeId id, const mars::utils::Vector &offset,
                                        std::vector<mars::sim::SimJoint*> *joints,
                                        std::vector<int> *gids,
                                        NodeMap *nodes) {
      printf("not implemented : %s\n", __PRETTY_FUNCTION__);
  //     MoveParams params;
  //     params.offset = offset;
  //     recursiveHelper(id, &params, joints, gids, nodes, &applyMove);
     }

     void EnvireNodeManager::rotateNode(mars::interfaces::NodeId id, mars::utils::Vector pivot, mars::utils::Quaternion q,
                                  unsigned long excludeJointId, bool includeConnected) {
        std::vector<int> gids;
        NodeMap::iterator iter = simNodes.find(id);
        if(iter == simNodes.end()) {
            iMutex.unlock();
            LOG_ERROR("EnvireNodeManager::rotateNode: node id not found!");
            return;
        }

        std::shared_ptr<mars::sim::SimNode> editedNode = iter->second->getData();
        editedNode->rotateAtPoint(pivot, q, true);

        if (includeConnected) {
            std::vector<std::shared_ptr<mars::sim::SimJoint>> joints = control->joints->getSimJoints();
            std::vector<std::shared_ptr<mars::sim::SimJoint>>::iterator jter;
            for(jter=joints.begin(); jter!=joints.end(); ++jter) {
                if((*jter)->getIndex() == excludeJointId) {
                    joints.erase(jter);
                    break;
                }
            }

            if(editedNode->getGroupID())
                gids.push_back(editedNode->getGroupID());

            NodeMap nodes = simNodes;
            nodes.erase(nodes.find(editedNode->getID()));

            rotateNodeRecursive(id, pivot, q, &joints,
                                   &gids, &nodes);
        }
        update_all_nodes = true;
        updateDynamicNodes(0, false);
     }

     void EnvireNodeManager::positionNode(mars::interfaces::NodeId id, mars::utils::Vector pos,
                                    unsigned long excludeJointId) {

      printf("not implemented : %s\n", __PRETTY_FUNCTION__);
  //     std::vector<int> gids;
  //     NodeMap::iterator iter = simNodes.find(id);
  //     if(iter == simNodes.end()) {
  //       iMutex.unlock();
  //       LOG_ERROR("EnvireNodeManager::rotateNode: node id not found!");
  //       return;
  //     }

  //     mars::sim::SimNode *editedNode = iter->second;
  //     mars::utils::Vector offset = pos - editedNode->getPosition();
  //     editedNode->setPosition(pos, true);

  //     std::vector<mars::sim::SimJoint*> joints = control->joints->getSimJoints();
  //     std::vector<mars::sim::SimJoint*>::iterator jter;
  //     for(jter=joints.begin(); jter!=joints.end(); ++jter) {
  //       if((*jter)->getIndex() == excludeJointId) {
  //         joints.erase(jter);
  //         break;
  //       }
  //     }

  //     if(editedNode->getGroupID())
  //       gids.push_back(editedNode->getGroupID());

  //     NodeMap nodes = simNodes;
  //     nodes.erase(nodes.find(editedNode->getID()));

  //     moveNodeRecursive(id, offset, &joints, &gids, &nodes);

  //     update_all_nodes = true;
  //     updateDynamicNodes(0, false);
     }

     void EnvireNodeManager::rotateNodeRecursive(mars::interfaces::NodeId id,
                                           const mars::utils::Vector &rotation_point,
                                           const mars::utils::Quaternion &rotation,
                                           std::vector<std::shared_ptr<mars::sim::SimJoint>> *joints,
                                           std::vector<int> *gids,
                                           NodeMap *nodes) {
        RotationParams params;
        params.rotation_point = rotation_point;
        params.rotation = rotation;
        recursiveHelper(id, &params, joints, gids, nodes, &applyRotation);
     }

     void EnvireNodeManager::clearRelativePosition(mars::interfaces::NodeId id, bool lock) {
      printf("not implemented : %s\n", __PRETTY_FUNCTION__);
  //     NodeMap::iterator iter;
  //     if(lock) iMutex.lock();
  //     for(iter = simNodes.begin(); iter != simNodes.end(); iter++) {
  //       if(iter->second->getSNode().relative_id == id) {
  //         iter->second->clearRelativePosition();
  //       }
  //     }
  //     if(lock) iMutex.unlock();
     }


  //   /**
  //    *\brief Reloads all nodes in the simulation.
  //    */
     void EnvireNodeManager::reloadNodes(bool reloadGrahpics) {
      printf("not implemented : %s\n", __PRETTY_FUNCTION__);
  //     std::list<mars::interfaces::NodeData>::iterator iter;
  //     mars::interfaces::NodeData tmp;
  //     mars::utils::Vector* friction;

  //     iMutex.lock();
  //     for(iter = simNodesReload.begin(); iter != simNodesReload.end(); iter++) {
  //       tmp = *iter;
  //       if(tmp.c_params.friction_direction1) {
  //         friction = new mars::utils::Vector(0.0, 0.0, 0.0);
  //         *friction = *(tmp.c_params.friction_direction1);
  //         tmp.c_params.friction_direction1 = friction;
  //       }
  //       if(tmp.terrain) {
  //         tmp.terrain = new(mars::interfaces::terrainStruct);
  //         *(tmp.terrain) = *(iter->terrain);
  //         tmp.terrain->pixelData = (double*)calloc((tmp.terrain->width*
  //                                                    tmp.terrain->height),
  //                                                   sizeof(double));
  //         memcpy(tmp.terrain->pixelData, iter->terrain->pixelData,
  //                (tmp.terrain->width*tmp.terrain->height)*sizeof(double));
  //       }
  //       iMutex.unlock();
  //       addNode(&tmp, true, reloadGrahpics);
  //       iMutex.lock();
  //     }
  //     iMutex.unlock();
  //     updateDynamicNodes(0);
     }

     std::list<mars::interfaces::NodeData>::iterator EnvireNodeManager::getReloadNode(mars::interfaces::NodeId id) {

      printf("not implemented : %s\n", __PRETTY_FUNCTION__);
  //     std::list<mars::interfaces::NodeData>::iterator iter = simNodesReload.begin();
  //     for(;iter!=simNodesReload.end(); ++iter) {
  //       if(iter->index == id) break;
  //     }
  //     return iter;
     }

  //   /**
  //    *\brief set the size for the node with the given id.
  //    */
     const mars::utils::Vector EnvireNodeManager::setReloadExtent(mars::interfaces::NodeId id, const mars::utils::Vector &ext) {
      printf("not implemented : %s\n", __PRETTY_FUNCTION__);
      return mars::utils::Vector();
  //     mars::utils::Vector x(0.0,0.0,0.0);
  //     mars::utils::MutexLocker locker(&iMutex);
  //     std::list<mars::interfaces::NodeData>::iterator iter = getReloadNode(id);
  //     if (iter != simNodesReload.end()) {
  //       if(iter->filename != "PRIMITIVE") {
  //         x.x() = ext.x() / iter->ext.x();
  //         x.y() = ext.y() / iter->ext.y();
  //         x.z() = ext.z() / iter->ext.z();
  //       }
  //       iter->ext = ext;
  //     }
  //     return x;
     }


     void EnvireNodeManager::setReloadFriction(mars::interfaces::NodeId id, mars::interfaces::sReal friction1,
                                         mars::interfaces::sReal friction2) {
      printf("not implemented : %s\n", __PRETTY_FUNCTION__);
  //     mars::utils::MutexLocker locker(&iMutex);

  //     std::list<mars::interfaces::NodeData>::iterator iter = getReloadNode(id);
  //     if (iter != simNodesReload.end()) {
  //       iter->c_params.friction1 = friction1;
  //       iter->c_params.friction2 = friction2;
  //     }
     }


  //   /**
  //    *\brief set the position for the node with the given id.
  //    */
     void EnvireNodeManager::setReloadPosition(mars::interfaces::NodeId id, const mars::utils::Vector &pos) {
      printf("not implemented : %s\n", __PRETTY_FUNCTION__);
  //     mars::utils::MutexLocker locker(&iMutex);
  //     std::list<mars::interfaces::NodeData>::iterator iter = getReloadNode(id);
  //     if (iter != simNodesReload.end()) {
  //       iter->pos = pos;
  //     }
     }


    /**
     *\brief Updates the Node values of dynamical nodes from the physics.
     */
    void EnvireNodeManager::updateDynamicNodes(mars::interfaces::sReal calc_ms, bool physics_thread) {
        mars::utils::MutexLocker locker(&iMutex);

        // FIX: update Graph
        if(graphTreeView.crossEdges.size() > 0)
        {
            const envire::core::GraphTraits::vertex_descriptor source = EnvireStorageManager::instance()->getGraph()->getSourceVertex(graphTreeView.crossEdges[0].edge);
            const envire::core::GraphTraits::vertex_descriptor target = EnvireStorageManager::instance()->getGraph()->getTargetVertex(graphTreeView.crossEdges[0].edge);
            const envire::core::FrameId sourceId = EnvireStorageManager::instance()->getGraph()->getFrameId(source);
            const envire::core::FrameId targetId = EnvireStorageManager::instance()->getGraph()->getFrameId(target);
            const std::string msg = "Loop in tree detected: " + sourceId + " --> " + targetId +
                               ". The physics plugin cannot handle loops in the graph";
            throw std::runtime_error(msg);
        }

        // update the graph from top to bottom
        // starts with the parent and go to children
        const envire::core::GraphTraits::vertex_descriptor originDesc = EnvireStorageManager::instance()->getGraph()->vertex(SIM_CENTER_FRAME_NAME);
        updateChildPositions(originDesc, base::TransformWithCovariance::Identity(), calc_ms, physics_thread);
    }

    void EnvireNodeManager::updateChildPositions(const envire::core::GraphTraits::vertex_descriptor vertex,
                                                const base::TransformWithCovariance& frameToRoot,
                                                mars::interfaces::sReal calc_ms, bool physics_thread)
    {
        if(graphTreeView.tree.find(vertex) != graphTreeView.tree.end())
        {
            const std::unordered_set<envire::core::GraphTraits::vertex_descriptor>& children = graphTreeView.tree[vertex].children;
            for(const envire::core::GraphTraits::vertex_descriptor child : children)
            {
                updatePositions(vertex, child, frameToRoot, calc_ms, physics_thread);
            }
        }
    }

    /*void EnvireNodeManager::setTfToCenter(envire::core::FrameId frameId, const envire::core::Transform tf){
        mars::utils::MutexLocker locker(&iMutex);
        EnvireStorageManager::instance()->getGraph()->updateTransform(SIM_CENTER_FRAME_NAME, frameId, tf);
        updatePositionsFromGraph();
    }

    void EnvireNodeManager::updatePositionsFromGraph(){

        //update positions in sim nodes

        std::pair<envire::core::EnvireGraph::vertex_iterator, envire::core::EnvireGraph::vertex_iterator> vertices = EnvireStorageManager::instance()->getGraph()->getVertices();

        for (auto vertex = vertices.first; vertex != vertices.second; vertex++){

            envire::core::GraphTraits::vertex_descriptor center = EnvireStorageManager::instance()->getGraph()->getVertex(SIM_CENTER_FRAME_NAME);
            base::TransformWithCovariance targetPos = EnvireStorageManager::instance()->getGraph()->getTransform(center,*vertex).transform;

            if (EnvireStorageManager::instance()->getGraph()->containsItems<envire::core::Item<std::shared_ptr<mars::sim::SimNode>>>(*vertex)){
                // Update simulation node

                using IteratorSimNode = envire::core::EnvireGraph::ItemIterator<SimNodeItem>;
                IteratorSimNode begin_sim, end_sim;
                boost::tie(begin_sim, end_sim) = EnvireStorageManager::instance()->getGraph()->getItems<SimNodeItem>(*vertex);
                for (;begin_sim!=end_sim; begin_sim++)
                {
                    const std::shared_ptr<mars::sim::SimNode> sim_node = begin_sim->getData();

                    utils::Vector oldpos = sim_node->getPosition();

                    utils::Vector pos = targetPos.translation;
                    sim_node->setPosition(pos,true);

                    utils::Quaternion rot = targetPos.orientation;
                    sim_node->setRotation(rot,true);
        //                    }
                    std::string name = EnvireStorageManager::instance()->getGraph()->getFrameId(*vertex);

                    LOG_WARN("node %s (%.2f %.2f %.2f) to (%.2f %.2f %.2f)\n",name.c_str(),oldpos.x(),oldpos.y(),oldpos.z(),pos.x(),pos.y(),pos.z());

                }
            }
        }

    }*/

    void EnvireNodeManager::updatePositions( const envire::core::GraphTraits::vertex_descriptor origin,
                                        const envire::core::GraphTraits::vertex_descriptor target,
                                        const base::TransformWithCovariance& originToRoot,
                                        mars::interfaces::sReal calc_ms, bool physics_thread)
    {

        envire::core::Transform tf = EnvireStorageManager::instance()->getGraph()->getTransform(origin, target);

        if (EnvireStorageManager::instance()->getGraph()->containsItems<envire::core::Item<std::shared_ptr<mars::sim::SimNode>>>(target))
        {
            // Update simulation node
            using IteratorSimNode = envire::core::EnvireGraph::ItemIterator<SimNodeItem>;
            IteratorSimNode begin_sim, end_sim;
            boost::tie(begin_sim, end_sim) = EnvireStorageManager::instance()->getGraph()->getItems<SimNodeItem>(target);
            for (;begin_sim!=end_sim; begin_sim++)
            {
                const std::shared_ptr<mars::sim::SimNode> sim_node = begin_sim->getData();
                mars::interfaces::NodeData node_data = sim_node->getSNode();

                // update the pose of node only if it is dynamic
                // don't update the position of inertial, collision and visuals
                // since these simnode has static transformation to the frame
                // the frame updates its pose
                if (sim_node->isMovable() == true)
                {
                    // update the physic of sim node
                    sim_node->update(calc_ms, physics_thread);

                    if ( node_data.simNodeType != mars::interfaces::SimNodeType::INERTIA
                        && node_data.simNodeType != mars::interfaces::SimNodeType::COLLISION
                        && node_data.simNodeType != mars::interfaces::SimNodeType::VISUAL) {

                        // update graph: update the pose of sim node in graph
                        base::TransformWithCovariance absolutTransform;
                        absolutTransform.translation = sim_node->getPosition();
                        absolutTransform.orientation = sim_node->getRotation();

                        // FIX: do we need update time in transformation?

                        tf.setTransform(originToRoot * absolutTransform);
                        tf.time = base::Time::now();
                        EnvireStorageManager::instance()->getGraph()->updateTransform(origin, target, tf);
                    }
                }
            }
        }

        const envire::core::Transform invTf = EnvireStorageManager::instance()->getGraph()->getTransform(target, origin);
        updateChildPositions(target, invTf.transform * originToRoot, calc_ms, physics_thread);
    }

     void EnvireNodeManager::preGraphicsUpdate() {
      printf("not implemented : %s\n", __PRETTY_FUNCTION__);
  // //  printf("...preGraphicsUpdate...\n");
  //     NodeMap::iterator iter;
  //     if(!control->graphics)
  //       return;

  //     iMutex.lock();
  //     if(update_all_nodes) {
  //       update_all_nodes = false;
  //       for(iter = simNodes.begin(); iter != simNodes.end(); iter++) {
  //         control->graphics->setDrawObjectPos(iter->second->getGraphicsID(),
  //                                             iter->second->getVisualPosition());
  //         control->graphics->setDrawObjectRot(iter->second->getGraphicsID(),
  //                                             iter->second->getVisualRotation());
  //         control->graphics->setDrawObjectPos(iter->second->getGraphicsID2(),
  //                                             iter->second->getPosition());
  //         control->graphics->setDrawObjectRot(iter->second->getGraphicsID2(),
  //                                             iter->second->getRotation());
  //       }
  //     }
  //     else {
  //       for(iter = simNodesDyn.begin(); iter != simNodesDyn.end(); iter++) {
  //         control->graphics->setDrawObjectPos(iter->second->getGraphicsID(),
  //                                             iter->second->getVisualPosition());
  //         control->graphics->setDrawObjectRot(iter->second->getGraphicsID(),
  //                                             iter->second->getVisualRotation());
  //         control->graphics->setDrawObjectPos(iter->second->getGraphicsID2(),
  //                                             iter->second->getPosition());
  //         control->graphics->setDrawObjectRot(iter->second->getGraphicsID2(),
  //                                             iter->second->getRotation());
  //       }
  //       for(iter = nodesToUpdate.begin(); iter != nodesToUpdate.end(); iter++) {
  //         control->graphics->setDrawObjectPos(iter->second->getGraphicsID(),
  //                                             iter->second->getVisualPosition());
  //         control->graphics->setDrawObjectRot(iter->second->getGraphicsID(),
  //                                             iter->second->getVisualRotation());
  //         control->graphics->setDrawObjectPos(iter->second->getGraphicsID2(),
  //                                             iter->second->getPosition());
  //         control->graphics->setDrawObjectRot(iter->second->getGraphicsID2(),
  //                                             iter->second->getRotation());
  //       }
  //       nodesToUpdate.clear();
  //     }
  //     iMutex.unlock();
     }

  //   /**
  //    *\brief Removes all nodes from the simulation to clear the world.
  //    */
     void EnvireNodeManager::clearAllNodes(bool clear_all, bool clearGraphics) {
      printf("not implemented : %s\n", __PRETTY_FUNCTION__);
  //     mars::utils::MutexLocker locker(&iMutex);
  //     NodeMap::iterator iter;
  //     while (!simNodes.empty())
  //       removeNode(simNodes.begin()->first, false, clearGraphics);
  //     simNodes.clear();
  //     simNodesDyn.clear();
  //     if(clear_all) simNodesReload.clear();
  //     next_node_id = 1;
  //     iMutex.unlock();
     }

  //   /**
  //    *\brief Set the reload orientation of a node.
  //    */
     void EnvireNodeManager::setReloadAngle(mars::interfaces::NodeId id, const mars::utils::sRotation &angle) {
      printf("not implemented : %s\n", __PRETTY_FUNCTION__);
  //     setReloadQuaternion(id, eulerToQuaternion(angle));
     }


  //   /**
  //    *\brief Set the reload orientation of a node by using a quaternion.
  //    */
     void EnvireNodeManager::setReloadQuaternion(mars::interfaces::NodeId id, const mars::utils::Quaternion &q) {
      printf("not implemented : %s\n", __PRETTY_FUNCTION__);
  //     mars::utils::MutexLocker locker(&iMutex);
  //     std::list<mars::interfaces::NodeData>::iterator iter = getReloadNode(id);
  //     if (iter != simNodesReload.end()) {
  //       iter->rot = q;
  //     }
     }

     /**
      *\brief Set the contact parameter of a node.
      */
     void EnvireNodeManager::setContactParams(mars::interfaces::NodeId id, const mars::interfaces::contact_params &cp) {
      printf("not implemented : %s\n", __PRETTY_FUNCTION__);
  //     mars::utils::MutexLocker locker(&iMutex);
  //     NodeMap::iterator iter = simNodes.find(id);
  //     if (iter != simNodes.end())
  //       iter->second->setContactParams(cp);
     }


    void EnvireNodeManager::setVelocity(mars::interfaces::NodeId id, const mars::utils::Vector& vel) {
      printf("not implemented : %s\n", __PRETTY_FUNCTION__);
  //     mars::utils::MutexLocker locker(&iMutex);
  //     NodeMap::iterator iter = simNodesDyn.find(id);
  //     if (iter != simNodesDyn.end())
  //       iter->second->setLinearVelocity(vel);
     }


     void EnvireNodeManager::setAngularVelocity(mars::interfaces::NodeId id, const mars::utils::Vector& vel) {
      printf("not implemented : %s\n", __PRETTY_FUNCTION__);
  //     mars::utils::MutexLocker locker(&iMutex);
  //     NodeMap::iterator iter = simNodesDyn.find(id);
  //     if (iter != simNodesDyn.end())
  //       iter->second->setAngularVelocity(vel);
     }


     /**
      *\brief Scales the nodes to reload.
      */
     void EnvireNodeManager::scaleReloadNodes(mars::interfaces::sReal factor_x, mars::interfaces::sReal factor_y,
                                        mars::interfaces::sReal factor_z) {
      printf("not implemented : %s\n", __PRETTY_FUNCTION__);
  //     std::list<mars::interfaces::NodeData>::iterator iter;

  //     iMutex.lock();
  //     for(iter = simNodesReload.begin(); iter != simNodesReload.end(); iter++) {
  //       iter->pos.x() *= factor_x;
  //       iter->pos.y() *= factor_y;
  //       iter->pos.z() *= factor_z;
  //       iter->ext.x() *= factor_x;
  //       iter->ext.y() *= factor_y;
  //       iter->ext.z() *= factor_z;
  //       iter->visual_offset_pos.x() *= factor_x;
  //       iter->visual_offset_pos.y() *= factor_y;
  //       iter->visual_offset_pos.z() *= factor_z;
  //       iter->visual_size.x() *= factor_x;
  //       iter->visual_size.y() *= factor_y;
  //       iter->visual_size.z() *= factor_z;

  //     }
  //     iMutex.unlock();
     }

     void EnvireNodeManager::getNodeMass(mars::interfaces::NodeId id, mars::interfaces::sReal *mass,
                                   mars::interfaces::sReal* inertia) const {
      printf("not implemented : %s\n", __PRETTY_FUNCTION__);
  //     mars::utils::MutexLocker locker(&iMutex);
  //     NodeMap::const_iterator iter = simNodes.find(id);
  //     if (iter != simNodes.end())
  //       iter->second->getMass(mass, inertia);
     }


     void EnvireNodeManager::setAngularDamping(mars::interfaces::NodeId id, mars::interfaces::sReal damping) {
      printf("not implemented : %s\n", __PRETTY_FUNCTION__);
  //     mars::utils::MutexLocker locker(&iMutex);
  //     NodeMap::iterator iter = simNodes.find(id);
  //     if (iter != simNodes.end())
  //       iter->second->setAngularDamping(damping);
     }


     void EnvireNodeManager::addRotation(mars::interfaces::NodeId id, const mars::utils::Quaternion &q) {
      printf("not implemented : %s\n", __PRETTY_FUNCTION__);
  //     mars::utils::MutexLocker locker(&iMutex);
  //     NodeMap::iterator iter = simNodes.find(id);
  //     if (iter != simNodes.end())
  //       iter->second->addRotation(q);
     }


     const mars::interfaces::contact_params EnvireNodeManager::getContactParams(mars::interfaces::NodeId id) const {
      printf("not implemented : %s\n", __PRETTY_FUNCTION__);
      return mars::interfaces::contact_params();
  //     mars::utils::MutexLocker locker(&iMutex);
  //     NodeMap::const_iterator iter = simNodes.find(id);
  //     if (iter != simNodes.end())
  //       return iter->second->getContactParams();
  //     mars::interfaces::contact_params a;
  //     return a;
     }




     void EnvireNodeManager::exportGraphicNodesByID(const std::string &folder) const {
      printf("not implemented : %s\n", __PRETTY_FUNCTION__);
  //     if(control->graphics) {
  //       char text[255];
  //       std::string filename;

  //       NodeMap::const_iterator iter;
  //       iMutex.lock();
  //       for(iter=simNodes.begin(); iter!=simNodes.end(); ++iter) {
  //         sprintf(text, "/%lu.stl", iter->first);
  //         filename = folder+std::string(text);
  //         control->graphics->exportDrawObject(iter->second->getGraphicsID(), filename);
  //         sprintf(text, "/%lu.obj", iter->first);
  //         filename = folder+std::string(text);
  //         control->graphics->exportDrawObject(iter->second->getGraphicsID(), filename);
  //       }
  //       iMutex.unlock();
  //     }
     }

    void EnvireNodeManager::getContactPoints(std::vector<mars::interfaces::NodeId> *ids,
                                            std::vector<mars::utils::Vector> *contact_points) const {
        NodeMap::const_iterator iter;
        std::vector<mars::utils::Vector>::const_iterator lter;
        std::vector<mars::utils::Vector> points;

        iMutex.lock();
        for(iter=simNodes.begin(); iter!=simNodes.end(); ++iter) {
            iter->second->getData()->getContactPoints(&points);
            for(lter=points.begin(); lter!=points.end(); ++lter) {
                ids->push_back(iter->first);
                contact_points->push_back((*lter));
            }
        }
        iMutex.unlock();
    }

    void EnvireNodeManager::getContactIDs(const mars::interfaces::NodeId &id,
                                    std::list<mars::interfaces::NodeId> *ids) const {
        mars::utils::MutexLocker locker(&iMutex);
        NodeMap::const_iterator iter = simNodes.find(id);
        if (iter != simNodes.end()) {
            iter->second->getData()->getContactIDs(ids);
        }
    }

     void EnvireNodeManager::updateRay(mars::interfaces::NodeId id) {
  //    printf("not implemented : %s\n", __PRETTY_FUNCTION__);
  //     mars::utils::MutexLocker locker(&iMutex);
  //     NodeMap::iterator iter = simNodes.find(id);
  //     if (iter != simNodes.end())
  //       iter->second->updateRay();
    }



    mars::interfaces::NodeId EnvireNodeManager::getDrawID(mars::interfaces::NodeId id) const {
        // FIX: move this into envire graph viz
        // Take care of the nodeid 0, it is part of sim
        // TODO_A: introduce draw interface manager to separate node from visualisation
        mars::utils::MutexLocker locker(&iMutex);
        NodeMap::const_iterator iter = simNodes.find(id);
        if (iter != simNodes.end())
            return iter->second->getData()->getGraphicsID();
        else
            return INVALID_ID;
    }


    const mars::utils::Vector EnvireNodeManager::getContactForce(mars::interfaces::NodeId id) const {
        mars::utils::MutexLocker locker(&iMutex);
        NodeMap::const_iterator iter = simNodes.find(id);
        if (iter != simNodes.end())
            return iter->second->getData()->getContactForce();
        else
            return mars::utils::Vector(0.0, 0.0, 0.0);
    }


    double EnvireNodeManager::getCollisionDepth(mars::interfaces::NodeId id) const {
        mars::utils::MutexLocker locker(&iMutex);
        NodeMap::const_iterator iter = simNodes.find(id);
        if (iter != simNodes.end())
            return iter->second->getData()->getCollisionDepth();
        else
            return 0.0;
    }


    void EnvireNodeManager::setVisualRep(mars::interfaces::NodeId id, int val) {
        // FIX: move this into envire graph viz
        // Take care of the nodeid 0, it is part of sim
        // TODO_A: introduce draw interface manager to separate node from visualisation

        if(!(control->graphics))
            return;
        visual_rep = val;
        NodeMap::iterator iter;
        int current;

        iMutex.lock();
        for(iter = simNodes.begin(); iter != simNodes.end(); iter++) {
            if(id == 0 || iter->first == id) {
                current = iter->second->getData()->getVisualRep();
                std::cout << "[EnvireNodeManager::setVisualRep] current: " << iter->second->getData()->getName() << " " << current << std::endl;
                if(val & 1 && !(current & 1))
                    control->graphics->setDrawObjectShow(iter->second->getData()->getGraphicsID(), true);
                else if(!(val & 1) && current & 1)
                    control->graphics->setDrawObjectShow(iter->second->getData()->getGraphicsID(), false);
                if(val & 2 && !(current & 2))
                    control->graphics->setDrawObjectShow(iter->second->getData()->getGraphicsID2(), true);
                else if(!(val & 2) && current & 2)
                    control->graphics->setDrawObjectShow(iter->second->getData()->getGraphicsID2(), false);

                iter->second->getData()->setVisualRep(val);
                if(id != 0) break;
            }
        }
        iMutex.unlock();
     }

    mars::interfaces::NodeId EnvireNodeManager::getID(const std::string& node_name) const {
        iMutex.lock();
        NodeMap::const_iterator iter;
        for(iter = simNodes.begin(); iter != simNodes.end(); iter++) {
            if (iter->second->getData()->getName() == node_name)  {
                iMutex.unlock();
                return iter->first;
            }
        }
        iMutex.unlock();
        return INVALID_ID;
     }

    /**
     * Retrieve the ids of the node which contain str_in_name in there name.
     * \param str_in_name String in the name of the nodes to get the ids for
     * \return Vector filled with the ids of the node if it exists, otherwise emtpy vector
     */
    std::vector<mars::interfaces::NodeId> EnvireNodeManager::getNodeIDs(const std::string& str_in_name) const {
         printf("not implemented : %s\n", __PRETTY_FUNCTION__);
    //   iMutex.lock();
    //   NodeMap::const_iterator iter;
    //   std::vector<interfaces::NodeId> out;
    //   for(iter = simNodes.begin(); iter != simNodes.end(); iter++) {
    //     if (iter->second->getName().find(str_in_name) != std::string::npos)  {
    //       out.push_back(iter->first);
    //     }
    //   }
    //   iMutex.unlock();
    //   return out;
        return std::vector<mars::interfaces::NodeId>();
    }

     void EnvireNodeManager::pushToUpdate(mars::sim::SimNode* node) {
      printf("not implemented : %s\n", __PRETTY_FUNCTION__);
  //     mars::utils::MutexLocker locker(&iMutex);
  //     NodeMap::iterator iter = nodesToUpdate.find(node->getID());
  //     if (iter == nodesToUpdate.end())
  //       nodesToUpdate[node->getID()] = node;
     }



    std::vector<mars::interfaces::NodeId> EnvireNodeManager::getConnectedNodes(mars::interfaces::NodeId id) {
        //TODO_A: check how we can simplify the functionality by using graph
        std::vector<mars::interfaces::NodeId> connected;
        mars::utils::MutexLocker locker(&iMutex);

        // find the node with given NodeID
        NodeMap::iterator iter = simNodes.find(id);
        if (iter == simNodes.end())
            return connected;

        std::shared_ptr<mars::sim::SimNode> currentNode = iter->second->getData();
        std::vector<std::shared_ptr<mars::sim::SimJoint>> simJoints = control->joints->getSimJoints();

        // find all nodes that belong to the same group as the current node
        if (currentNode->getGroupID() != 0){
            for (iter = simNodes.begin(); iter != simNodes.end(); iter++)
                if (iter->second->getData()->getGroupID() == currentNode->getGroupID())
                    connected.push_back(iter->first);
        }

        for (size_t i = 0; i < simJoints.size(); i++) {
            if (simJoints[i]->getAttachedNode() &&
                simJoints[i]->getAttachedNode()->getID() == id &&
                simJoints[i]->getAttachedNode(2)) {
                connected.push_back(simJoints[i]->getAttachedNode(2)->getID());
                /*    currentNode = simNodes.find(connected.back())->second;
                    if (currentNode->getGroupID() != 0)
                    for (iter = simNodes.begin(); iter != simNodes.end(); iter++)
                    if (iter->second->getGroupID() == currentNode->getGroupID())
                    connected.push_back(iter->first);*/
            }

            if (simJoints[i]->getAttachedNode(2) &&
                simJoints[i]->getAttachedNode(2)->getID() == id &&
                simJoints[i]->getAttachedNode()) {
                connected.push_back(simJoints[i]->getAttachedNode()->getID());
                /*      currentNode = simNodes.find(connected.back())->second;
                        if (currentNode->getGroupID() != 0)
                        for (iter = simNodes.begin(); iter != simNodes.end(); iter++)
                        if (iter->second->getGroupID() == currentNode->getGroupID())
                        connected.push_back(iter->first);*/
            }
        }

        return connected;

    }


     bool EnvireNodeManager::getDataBrokerNames(mars::interfaces::NodeId id, std::string *groupName,
                                          std::string *dataName) const {
        mars::utils::MutexLocker locker(&iMutex);
        NodeMap::const_iterator iter = simNodes.find(id);
        if (iter == simNodes.end())
            return false;
        iter->second->getData()->getDataBrokerNames(groupName, dataName);
        return true;
     }

     void EnvireNodeManager::setVisualQOffset(mars::interfaces::NodeId id, const mars::utils::Quaternion &q) {
      printf("not implemented : %s\n", __PRETTY_FUNCTION__);
  //     NodeMap::const_iterator iter = simNodes.find(id);
  //     if (iter != simNodes.end())
  //       iter->second->setVisQOffset(q);
     }

     void EnvireNodeManager::updatePR(mars::interfaces::NodeId id, const mars::utils::Vector &pos,
                                const mars::utils::Quaternion &rot,
                                const mars::utils::Vector &visOffsetPos,
                                const mars::utils::Quaternion &visOffsetRot,
                                bool doLock) {
      printf("not implemented : %s\n", __PRETTY_FUNCTION__);
  //     NodeMap::const_iterator iter = simNodes.find(id);

  //     if (iter != simNodes.end()) {
  //       iter->second->updatePR(pos, rot, visOffsetPos, visOffsetRot);
  //       if(doLock) mars::utils::MutexLocker locker(&iMutex);
  //       nodesToUpdate[id] = iter->second;
  //     }
     }

     bool EnvireNodeManager::getIsMovable(mars::interfaces::NodeId id) const {
      printf("not implemented : %s\n", __PRETTY_FUNCTION__);
      return true;
  //     NodeMap::const_iterator iter = simNodes.find(id);
  //     if(iter != simNodes.end())
  //       return iter->second->isMovable();
  //     return false;
     }

     void EnvireNodeManager::setIsMovable(mars::interfaces::NodeId id, bool isMovable) {
      printf("not implemented : %s\n", __PRETTY_FUNCTION__);
  //     NodeMap::iterator iter = simNodes.find(id);
  //     if(iter != simNodes.end())
  //       iter->second->setMovable(isMovable);
     }

     void EnvireNodeManager::moveRelativeNodes(const mars::sim::SimNode &node, NodeMap *nodes,
                                         mars::utils::Vector v) {
      printf("not implemented : %s\n", __PRETTY_FUNCTION__);
  //     NodeMap::iterator iter;
  //     mars::sim::SimNode* nextNode;

  //     // TODO: doesn't this function need locking? no
  //     for (iter = nodes->begin(); iter != nodes->end(); iter++) {
  //       if (iter->second->getParentID() == node.getID()) {
  //         nextNode = iter->second;
  //         mars::utils::Vector newPos = nextNode->getPosition() + v;
  //         nextNode->setPosition(newPos, false);
  //         nodes->erase(iter);
  //         moveRelativeNodes(node, nodes, v);
  //         moveRelativeNodes(*nextNode, nodes, v);
  //         break;
  //       }
  //     }
     }

     void EnvireNodeManager::rotateRelativeNodes(const mars::sim::SimNode &node, NodeMap *nodes,
                                           mars::utils::Vector pivot, mars::utils::Quaternion rot) {
      printf("not implemented : %s\n", __PRETTY_FUNCTION__);
  //     NodeMap::iterator iter;
  //     mars::sim::SimNode* nextNode;

  //     // TODO: doesn't this function need locking? no
  //     for (iter = nodes->begin(); iter != nodes->end(); iter++) {
  //       if (iter->second->getParentID() == node.getID()) {
  //         nextNode = iter->second;
  //         nextNode->rotateAtPoint(pivot, rot, false);
  //         nodes->erase(iter);
  //         rotateRelativeNodes(node, nodes, pivot, rot);
  //         rotateRelativeNodes(*nextNode, nodes, pivot, rot);
  //         break;
  //       }
  //     }
     }

     void EnvireNodeManager::printNodeMasses(bool onlysum) {
      printf("not implemented : %s\n", __PRETTY_FUNCTION__);
  //     NodeMap::iterator it;
  //     double masssum = 0;
  //     for(it=simNodes.begin(); it!=simNodes.end(); ++it) {
  //       if (!onlysum)
  //         fprintf(stderr, "%s: %f\n", it->second->getName().c_str(), it->second->getMass());
  //       masssum+=it->second->getMass();
  //     }
  //     fprintf(stderr, "Sum of masses of imported model: %f\n", masssum);
     }

     void EnvireNodeManager::edit(mars::interfaces::NodeId id, const std::string &key,
                            const std::string &value) {
      printf("not implemented : %s\n", __PRETTY_FUNCTION__);
  //     mars::interfaces::NodeData nd = getFullNode(id);
  //     if(mars::utils::matchPattern("*/position/*", key)) {
  //       double v = atof(value.c_str());
  //       if(key[key.size()-1] == 'x') nd.pos.x() = v;
  //       else if(key[key.size()-1] == 'y') nd.pos.y() = v;
  //       else if(key[key.size()-1] == 'z') nd.pos.z() = v;
  //       control->nodes->editNode(&nd, (mars::interfaces::EDIT_NODE_POS | mars::interfaces::EDIT_NODE_MOVE_ALL));
  //     }
  //     else if(mars::utils::matchPattern("*/extend/*", key)) {
  //       double v = atof(value.c_str());
  //       if(key[key.size()-1] == 'x') nd.ext.x() = v;
  //       else if(key[key.size()-1] == 'y') nd.ext.y() = v;
  //       else if(key[key.size()-1] == 'z') nd.ext.z() = v;
  //       control->nodes->editNode(&nd, mars::interfaces::EDIT_NODE_SIZE);
  //     }
  //     else if(mars::utils::matchPattern("*/material", key)) {
  //       if(control->graphics) {
  //         std::vector<interfaces::MaterialData> mList;
  //         std::vector<interfaces::MaterialData>::iterator it;
  //         mList = control->graphics->getMaterialList();
  //         for(it=mList.begin(); it!=mList.end(); ++it) {
  //           if(it->name == value) {
  //             unsigned long drawID = getDrawID(id);
  //             control->graphics->setDrawObjectMaterial(drawID, *it);
  //             break;
  //           }
  //         }
  //       }
  //     }
  //     else if(mars::utils::matchPattern("*/c*", key)) {
  //       mars::utils::MutexLocker locker(&iMutex);
  //       NodeMap::iterator iter;
  //       // todo: cfdir1 is a std::vector
  //       iter = simNodes.find(id);
  //       if(iter == simNodes.end()) return;
  //       mars::interfaces::contact_params c = iter->second->getContactParams();
  //       if(mars::utils::matchPattern("*/cmax_num_contacts", key)) {
  //         c.max_num_contacts = atoi(value.c_str());;
  //       }
  //       else if(mars::utils::matchPattern("*/cerp", key)) c.erp = atof(value.c_str());
  //       else if(mars::utils::matchPattern("*/ccfm", key)) c.cfm = atof(value.c_str());
  //       else if(mars::utils::matchPattern("*/cfriction1", key)) c.friction1 = atof(value.c_str());
  //       else if(mars::utils::matchPattern("*/cfriction2", key)) c.friction2 = atof(value.c_str());
  //       else if(mars::utils::matchPattern("*/cmotion1", key)) c.motion1 = atof(value.c_str());
  //       else if(mars::utils::matchPattern("*/cmotion2", key)) c.motion2 = atof(value.c_str());
  //       else if(mars::utils::matchPattern("*/cfds1", key)) c.fds1 = atof(value.c_str());
  //       else if(mars::utils::matchPattern("*/cfds2", key)) c.fds2 = atof(value.c_str());
  //       else if(mars::utils::matchPattern("*/cbounce", key)) c.bounce = atof(value.c_str());
  //       else if(mars::utils::matchPattern("*/cbounce_vel", key)) c.bounce_vel = atof(value.c_str());
  //       else if(mars::utils::matchPattern("*/capprox", key)) {
  //         if(value == "true" || value == "True") c.approx_pyramid = true;
  //         else c.approx_pyramid = false;
  //       }
  //       else if(mars::utils::matchPattern("*/coll_bitmask", key)) c.coll_bitmask = atoi(value.c_str());
  //       else if(mars::utils::matchPattern("*/cfdir1*", key)) {
  //         double v = atof(value.c_str());
  //         if(!c.friction_direction1) c.friction_direction1 = new mars::utils::Vector(0,0,0);
  //         if(key[key.size()-1] == 'x') c.friction_direction1->x() = v;
  //         else if(key[key.size()-1] == 'y') c.friction_direction1->y() = v;
  //         else if(key[key.size()-1] == 'z') c.friction_direction1->z() = v;
  //         if(c.friction_direction1->norm() < 0.00000001) {
  //           delete c.friction_direction1;
  //           c.friction_direction1 = 0;
  //         }
  //       }
  //       iter->second->setContactParams(c);
  //     }
     }

  }
  } // end of namespace sim
} // end of namespace mars
