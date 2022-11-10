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
 * \file EnvireEntityManager.h
 * \author Jonas Peter, Kai von Szadkowski
 * \brief "EntityManager" is the class that manages information about simulation entities
 *
 * The class can be used to access information about nodes and the position of the robot
 * to use pass robot name as parameter to the SimulatorInterface::loadScene function
 *
 * TODO delete robots after use
 * TODO handle node deletion (see NodeManager)
 * TODO allow to add nodes to robots via their ids instead of names;
 * TODO possible optimization store the index(iterator) of the last accessed robot as
 *      it is likely to be accessed again while nodes and motors are added
 */

#ifndef ENVIRE_ENTITY_MANAGER_H
#define ENVIRE_ENTITY_MANAGER_H

#include <map>
#include <mars/interfaces/sim/ControlCenter.h>
#include <mars/interfaces/graphics/GraphicsEventClient.h>
#include <mars/interfaces/sim/EntityManagerInterface.h>
#include <mars/utils/Mutex.h>
#include <configmaps/ConfigData.h>

#include <envire_core/graph/EnvireGraph.hpp>
#include <envire_core/items/Item.hpp>

namespace mars {
  namespace plugins {
    namespace envire_managers {

    class mars::sim::SimEntity;

    typedef std::shared_ptr<mars::sim::SimEntity> SimEntityPtr;
    typedef envire::core::Item<SimEntityPtr> SimEntityItem;
    typedef SimEntityItem::Ptr SimEntityItemPtr;

    typedef std::map<unsigned long, SimEntityItemPtr> EntityMap;
    typedef EntityMap::iterator EntityMapItr;

    /*get notifications
     * about selection changes*/
    class EnvireEntityManager: public interfaces::GraphicsEventClient,
        public interfaces::EntityManagerInterface {
    public:
      EnvireEntityManager(mars::interfaces::ControlCenter *c);
      /**creates a new entity with the given name and returns its id*/
      virtual unsigned long addEntity(const std::string &name);

      /**
       * @brief Add entity into the graph. The graph frame,
       * where the entity is stored, is given by the frame_id
       * in the config map of the entity.
       * 
       * Note for developer: The graph item of the entity will be stored 
       * in the private member entities, to get quick access to all entities. 
       * So we don't need to parse the whole graph to find all entities.
       * 
       * @param entity 
       * @return unsigned long - unique id of the added entity
       */
      virtual unsigned long addEntity(SimEntityPtr entity);

      virtual void removeEntity(const std::string &name, bool completeAssembly=false);

      virtual void removeAssembly(const std::string &assembly_name);

      virtual void appendConfig(const std::string &name, configmaps::ConfigMap &map);

      /**adds a node and maps the nodeId to the name*/
      virtual void addNode(const std::string &entityName, unsigned long nodeId,
          const std::string &nodeName);

      /**adds a motor to the entity and maps the motorId to its name*/
      virtual void addMotor(const std::string &entityName, unsigned long motorId,
          const std::string &motorName);
      /**adds a motor to the entity and maps the sensorId to its name*/
      virtual void addSensor(const std::string& entityName, long unsigned int sensorId,
        const std::string& sensorName);          

      /**adds a controller id to the controller list*/
      virtual void addController(const std::string &entityName, unsigned long controllerId);

      /**adds a joint to the entity and maps the jointId to its name*/
      virtual void addJoint(const std::string &entityName, unsigned long jointId,
          const std::string &jointName);

      // callback for entity creation
      virtual const std::map<unsigned long, mars::sim::SimEntity*>* subscribeToEntityCreation(interfaces::EntitySubscriberInterface* newsub);

      /**returns the entity with the given name
       */
      virtual mars::sim::SimEntity* getEntity(const std::string &name);
      virtual mars::sim::SimEntity* getEntity(const std::string &name, bool verbose);

      /**returns the entities that contain the given name string
       */
      virtual std::vector<mars::sim::SimEntity*> getEntities(const std::string &name);

      virtual void getListEntities(std::vector<mars::interfaces::core_objects_exchange>* entityList);

      /**
       * @brief return all entities
       */
      virtual std::vector<std::shared_ptr<mars::sim::SimEntity>> getEntities();

      /**returns the entities that belong to the assembly with the given name
       */
      virtual std::vector<mars::sim::SimEntity*> getEntitiesOfAssembly(
        const std::string &assembly_name);

      /**returns the root entity of the given assembly
       */
      virtual mars::sim::SimEntity* getRootOfAssembly(
        const std::string &assembly_name);

      /**returns the main entity of the given assembly if there is one, otherwise
       returns the root entity.
       @see getRootOfAssembly()
       */
      virtual mars::sim::SimEntity* getMainEntityOfAssembly(
        const std::string &assembly_name);

      /**returns the entity with the given id
       */
      virtual mars::sim::SimEntity* getEntity(unsigned long id);

      virtual unsigned long getEntityNode(const std::string &entityName,
          const std::string &nodeName);

      virtual unsigned long getEntityMotor(const std::string &entityName,
          const std::string &motorName);

      virtual unsigned long getEntitySensor(const std::string &entityName,
          const std::string &sensorName);          

      virtual std::vector<unsigned long> getEntityControllerList(const std::string &entityName);

      /**returns the node of the given entity; returns 0 if the entity or the node don't exist*/
      virtual unsigned long getEntityJoint(const std::string &entityName,
          const std::string &jointName);

      //from graphics event client
      virtual void selectEvent(unsigned long id, bool mode);

      //debug functions
      virtual void printEntityNodes(const std::string &entityName);
      virtual void printEntityMotors(const std::string &entityName);
      virtual void printEntityControllers(const std::string &entityName);
      virtual void resetPose();

    private:
      std::vector<mars::interfaces::EntitySubscriberInterface*> subscribers;
      void notifySubscribers(mars::sim::SimEntity* entity);
      mars::interfaces::ControlCenter *control;
      /**the id assigned to the next created entity; use getNextId function*/
      unsigned long next_entity_id;
      EntityMap entities;

      /**returns the id to be assigned to the next entity*/
      unsigned long getNextId() {
        return next_entity_id++;
      }

      // a mutex for the sensor containers
      mutable utils::Mutex iMutex;

    };

    } // end of namespace envire_managers
  } // end of namespace plugins
} // end of namespace mars

#endif // ENVIRE_ENTITY_MANAGER_H
