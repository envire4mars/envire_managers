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

#include "EnvireEntityManager.hpp"
#include <mars/sim/SimEntity.h>
#include <configmaps/ConfigData.h>
#include <mars/interfaces/graphics/GraphicsManagerInterface.h>
#include <mars/interfaces/sim/EntitySubscriberInterface.h>
#include <mars/utils/MutexLocker.h>
#include <mars/utils/misc.h>

#include <iostream>
#include <string>

namespace mars {
  namespace plugins {
    namespace envire_managers {

    EnvireEntityManager::EnvireEntityManager(mars::interfaces::ControlCenter* c) {

      control = c;
      next_entity_id = 1;
      if (control->graphics)
        control->graphics->addEventClient((mars::interfaces::GraphicsEventClient*) this);
    }

    unsigned long EnvireEntityManager::addEntity(const std::string &name) {
      unsigned long id = 0;
      mars::utils::MutexLocker locker(&iMutex);
      entities[id = getNextId()] = new mars::sim::SimEntity(control, name);
      notifySubscribers(entities[id]);
      return id;
    }

    unsigned long EnvireEntityManager::addEntity(mars::sim::SimEntity* entity) {
      unsigned long id = 0;
      mars::utils::MutexLocker locker(&iMutex);
      entities[id = getNextId()] = entity;
      notifySubscribers(entity);
      return id;
    }

    void EnvireEntityManager::removeEntity(const std::string &name, bool completeAssembly) {
      mars::sim::SimEntity* entity = getEntity(name);
      if (completeAssembly && entity->getAssembly() != "") {
        removeAssembly(entity->getAssembly());
      } else {
        //remove from entity map
        for (auto it = entities.begin(); it != entities.end(); ++it) {
          if (it->second == entity) {
            entities.erase(it);
            break;
          }
        }
        //delete entity
        entity->removeEntity();
        /*TODO we have to free the memory here, but we don't know if this entity
        is allocated on the heap or the stack*/
      }
    }

    void EnvireEntityManager::appendConfig(const std::string &name, configmaps::ConfigMap &map) {
      mars::sim::SimEntity *entity = 0;
      //iterate over all robots to find the robot with the given name
      for (std::map<unsigned long, mars::sim::SimEntity*>::iterator iter = entities.begin();
          iter != entities.end(); ++iter) {
        if (iter->second->getName() == name) {
          entity = iter->second;
          break;
        }
      }
      if (entity) {
        mars::utils::MutexLocker locker(&iMutex);
        entity->appendConfig(map);
      }
    }

    void EnvireEntityManager::removeAssembly(const std::string &assembly_name) {
      std::vector<mars::sim::SimEntity*> parts = getEntitiesOfAssembly(assembly_name);
      for (auto p: parts) {
        //remove from entity map
        for (auto it = entities.begin(); it != entities.end(); ++it) {
          if (it->second == p) {
            fprintf(stderr, "Deleting entity %s\n", p->getName().c_str());
            entities.erase(it);
            break;
          }
        }
        //delete entity
        p->removeEntity();
        // REVIEW Do we have to delete the anchor joints here?
      }
    }

    void EnvireEntityManager::notifySubscribers(mars::sim::SimEntity* entity) {
      for (std::vector<mars::interfaces::EntitySubscriberInterface*>::iterator it = subscribers.begin();
           it != subscribers.end(); ++it) {
             (*it)->registerEntity(entity);
           }
    }

    const std::map<unsigned long, mars::sim::SimEntity*>* EnvireEntityManager::subscribeToEntityCreation(mars::interfaces::EntitySubscriberInterface* newsub) {
      if (newsub!=nullptr) {
        subscribers.push_back(newsub);
      }
      return &entities;
    }

    void EnvireEntityManager::addNode(const std::string& entityName, long unsigned int nodeId,
        const std::string& nodeName) {
      mars::sim::SimEntity *entity = 0;
      //iterate over all robots to find the robot with the given name
      for (std::map<unsigned long, mars::sim::SimEntity*>::iterator iter = entities.begin();
          iter != entities.end(); ++iter) {
        if (iter->second->getName() == entityName) {
          entity = iter->second;
          break;
        }
      }
      if (entity) {
        mars::utils::MutexLocker locker(&iMutex);
        entity->addNode(nodeId, nodeName);
      }
    }

    void EnvireEntityManager::addMotor(const std::string& entityName, long unsigned int motorId,
        const std::string& motorName) {
      //iterate over all robots to find the robot with the given name
      for (std::map<unsigned long, mars::sim::SimEntity*>::iterator iter = entities.begin();
          iter != entities.end(); ++iter) {
        if (iter->second->getName() == entityName) {
          mars::utils::MutexLocker locker(&iMutex);
          iter->second->addMotor(motorId, motorName);
          break;
        }
      }
    }

    void EnvireEntityManager::addJoint(const std::string& entityName, long unsigned int jointId,
        const std::string& jointName) {
      //iterate over all robots to find the robot with the given name
      for (std::map<unsigned long, mars::sim::SimEntity*>::iterator iter = entities.begin();
          iter != entities.end(); ++iter) {
        if (iter->second->getName() == entityName) {
          mars::utils::MutexLocker locker(&iMutex);
          iter->second->addJoint(jointId, jointName);
          break;
        }
      }
    }

    void EnvireEntityManager::addController(const std::string& entityName,
        long unsigned int controllerId) {
      //iterate over all robots to find the robot with the given name
      for (std::map<unsigned long, mars::sim::SimEntity*>::iterator iter = entities.begin();
          iter != entities.end(); ++iter) {
        if (iter->second->getName() == entityName) {
          mars::utils::MutexLocker locker(&iMutex);
          iter->second->addController(controllerId);
          break;
        }
      }
    }

    void EnvireEntityManager::selectEvent(long unsigned int id, bool mode) {
      //the node was selected
      if (true == mode) {
        //go over all robots as we don't know which robot the node belongs to
        for (std::map<unsigned long, mars::sim::SimEntity*>::iterator iter = entities.begin();
            iter != entities.end(); ++iter) {
          //select returns true if the node belongs to the robot
          if (iter->second != NULL && iter->second->select(id)) {
            //TODO <jonas.peter@dfki.de> notify about selection change only if new selection
            std::cout << "robot has been selected: " << iter->second->getName() << std::endl;
            //TODO <jonas.peter@dfki.de> notify clients about selection change
          }
        }
      }
      //TODO <jonas.peter@dfki.de> handle deselection
    }

    mars::sim::SimEntity* EnvireEntityManager::getEntity(long unsigned int id) {
      //TODO replace with find
      for (std::map<unsigned long, mars::sim::SimEntity*>::iterator iter = entities.begin();
          iter != entities.end(); ++iter) {
        if (iter->first == id) {
          return iter->second;
        }
      }
      return 0;
    }

    mars::sim::SimEntity* EnvireEntityManager::getEntity(const std::string& name) {
      return getEntity(name, true);
    }

    mars::sim::SimEntity* EnvireEntityManager::getEntity(const std::string& name, bool verbose) {
      for (std::map<unsigned long, mars::sim::SimEntity*>::iterator iter = entities.begin();
          iter != entities.end(); ++iter) {
        if (iter->second->getName() == name) {
          return iter->second;
        }
      }
      if (verbose)
        fprintf(stderr, "ERROR: Entity with name %s not found!\n", name.c_str());
      return 0;
    }

    std::vector<mars::sim::SimEntity*> EnvireEntityManager::getEntities(const std::string &name) {
      std::vector<mars::sim::SimEntity*> out;
      for (std::map<unsigned long, mars::sim::SimEntity*>::iterator iter = entities.begin();
          iter != entities.end(); ++iter) {
        if (mars::utils::matchPattern(name, iter->second->getName())) {
          out.push_back(iter->second);
        }
      }
      return out;
    }

    std::vector<mars::sim::SimEntity*> EnvireEntityManager::getEntitiesOfAssembly(
      const std::string &assembly_name)
    {
      std::vector<mars::sim::SimEntity*> out;
      for (std::map<unsigned long, mars::sim::SimEntity*>::iterator iter = entities.begin();
          iter != entities.end(); ++iter) {
        if (mars::utils::matchPattern(assembly_name, iter->second->getAssembly())) {
          out.push_back(iter->second);
        }
      }
      return out;
    }

    mars::sim::SimEntity* EnvireEntityManager::getRootOfAssembly(
      const std::string &assembly_name)
    {
      std::vector<mars::sim::SimEntity*> out;
      for (std::map<unsigned long, mars::sim::SimEntity*>::iterator iter = entities.begin();
          iter != entities.end(); ++iter) {
        if (mars::utils::matchPattern(assembly_name, iter->second->getAssembly())) {
          configmaps::ConfigMap map = iter->second->getConfig();
          if (map.hasKey("root") && (bool) map["root"]) return iter->second;
        }
      }
      return 0;
    }

    mars::sim::SimEntity* EnvireEntityManager::getMainEntityOfAssembly(
      const std::string &assembly_name)
    {
      std::vector<mars::sim::SimEntity*> out;
      for (std::map<unsigned long, mars::sim::SimEntity*>::iterator iter = entities.begin();
          iter != entities.end(); ++iter) {
        if (mars::utils::matchPattern(assembly_name, iter->second->getAssembly())) {
          configmaps::ConfigMap map = iter->second->getConfig();
          if (map.hasKey("main_entity") && (bool) map["main_entity"]) return iter->second;
        }
      }
      return getRootOfAssembly(assembly_name);
    }

    long unsigned int EnvireEntityManager::getEntityNode(const std::string& entityName,
        const std::string& nodeName) {
      mars::utils::MutexLocker locker(&iMutex);
      mars::sim::SimEntity *entity = getEntity(entityName);
      unsigned long node = 0;
      if (entity) {
        node = entity->getNode(nodeName);
      }
      return node;
    }

    long unsigned int EnvireEntityManager::getEntityMotor(const std::string& entityName,
        const std::string& motorName) {
      //not sure if a mutex lock is needed here
      mars::utils::MutexLocker locker(&iMutex);
      mars::sim::SimEntity *entity = getEntity(entityName);
      unsigned long motor = 0;
      if (entity) {
        motor = entity->getMotor(motorName);
      }
      return motor;
    }

    std::vector<unsigned long> EnvireEntityManager::getEntityControllerList(
        const std::string &entityName) {
      //not sure if a mutex lock is needed here
      mars::utils::MutexLocker locker(&iMutex);
      mars::sim::SimEntity *entity = getEntity(entityName);
      if (entity) {
        return entity->getControllerList();
      }
      return std::vector<unsigned long>();
    }

    long unsigned int EnvireEntityManager::getEntityJoint(const std::string& entityName,
        const std::string& jointName) {
      //not sure if a mutex lock is needed here
      mars::utils::MutexLocker locker(&iMutex);
      mars::sim::SimEntity *entity = getEntity(entityName);
      unsigned long joint = 0;
      if (entity) {
        joint = entity->getJoint(jointName);
      }
      return joint;
    }

    void EnvireEntityManager::printEntityNodes(const std::string& entityName) {
      for (std::map<unsigned long, mars::sim::SimEntity*>::iterator iter = entities.begin();
          iter != entities.end(); ++iter) {
        if (iter->second->getName() == entityName) {
          std::cout << "printing entity with id: " << iter->first << std::endl;
          iter->second->printNodes();
          break;
        }
      }
    }

    void EnvireEntityManager::printEntityMotors(const std::string& entityName) {
      for (std::map<unsigned long, mars::sim::SimEntity*>::iterator iter = entities.begin();
          iter != entities.end(); ++iter) {
        if (iter->second->getName() == entityName) {
          std::cout << "printing entity with id: " << iter->first << std::endl;
          iter->second->printMotors();
          break;
        }
      }
    }

    void EnvireEntityManager::printEntityControllers(const std::string& entityName) {
      for (std::map<unsigned long, mars::sim::SimEntity*>::iterator iter = entities.begin();
          iter != entities.end(); ++iter) {
        if (iter->second->getName() == entityName) {
          std::cout << "printing entity with id: " << iter->first << std::endl;
          iter->second->printControllers();
          break;
        }
      }
    }

    void EnvireEntityManager::resetPose() {
      for (auto iter: entities) {
        iter.second->removeAnchor();
      }
      for (auto iter: entities) {
        iter.second->setInitialPose(true);
      }
    }

    } // end of namespace envire_managers
  } // end of namespace plugins
} // end of namespace mars
