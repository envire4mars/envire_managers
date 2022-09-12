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
 * \file EnvireSensorManager.h
 * \author  Vladimir Komsiyski
 * \brief EnvireSensorManager implements SensorManagerInterface and
 * manages all sensors and all sensor
 * operations that are used for the communication between the simulation
 * modules.
 *
 * \version 1.3
 * Moved from the Simulator
 * \date 07.07.2011
 */

#include "EnvireSensorManager.hpp"

// sensor includes
#include <mars/sim/JointAVGTorqueSensor.h>
#include <mars/sim/JointLoadSensor.h>
#include <mars/sim/NodePositionSensor.h>
#include <mars/sim/NodeRotationSensor.h>
#include <mars/sim/NodeContactSensor.h>
#include <mars/sim/NodeIMUSensor.h>
#include <mars/sim/NodeContactForceSensor.h>
#include <mars/sim/NodeCOMSensor.h>
#include <mars/sim/JointPositionSensor.h>
#include <mars/sim/JointVelocitySensor.h>
#include <mars/sim/CameraSensor.h>
#include <mars/sim/NodeVelocitySensor.h>
#include <mars/sim/RaySensor.h>
#include <mars/sim/RotatingRaySensor.h>
#include <mars/sim/MultiLevelLaserRangeFinder.h>
//#include <mars/sim/RayGridSensor.h>
#include <mars/sim/NodeAngularVelocitySensor.h>
#include <mars/sim/HapticFieldSensor.h>
#include <mars/sim/Joint6DOFSensor.h>
#include <mars/sim/JointTorqueSensor.h>
#include <mars/sim/ScanningSonar.h>
#include <mars/sim/MotorCurrentSensor.h>

#include <mars/interfaces/sim/SimulatorInterface.h>
#include <mars/utils/MutexLocker.h>
#include <mars/interfaces/Logging.hpp>

#include <cstdio>
#include <stdexcept>

namespace mars {
  namespace plugins {
    namespace envire_managers {

    /**
     * \brief Constructor.
     *
     * \param c The pointer to the mars::interfaces::ControlCenter of the simulation.
     */
    EnvireSensorManager::EnvireSensorManager(mars::interfaces::ControlCenter *c)
    {
      control = c;
      next_sensor_id = 1;
      addSensorType("RaySensor",&mars::sim::RaySensor::instanciate);
      addSensorType("RotatingRaySensor",&mars::sim::RotatingRaySensor::instanciate);
      addSensorType("MultiLevelLaserRangeFinder",&mars::sim::MultiLevelLaserRangeFinder::instanciate);
      addSensorType("CameraSensor",&mars::sim::CameraSensor::instanciate);
      addSensorType("ScanningSonar",&mars::sim::ScanningSonar::instanciate);
      addSensorType("JointPosition",&mars::sim::JointPositionSensor::instanciate);
      addSensorType("JointVelocity",&mars::sim::JointVelocitySensor::instanciate);
      addSensorType("JointLoad",&mars::sim::JointLoadSensor::instanciate);
      addSensorType("JointTorque",&mars::sim::JointTorqueSensor::instanciate);
      addSensorType("JointAVGTorque",&mars::sim::JointAVGTorqueSensor::instanciate);
      addSensorType("Joint6DOF",&mars::sim::Joint6DOFSensor::instanciate);
      addSensorType("NodeContact",&mars::sim::NodeContactSensor::instanciate);
      addSensorType("NodeIMU", &mars::sim::NodeIMUSensor::instanciate);
      addSensorType("NodePosition",&mars::sim::NodePositionSensor::instanciate);
      addSensorType("NodeRotation",&mars::sim::NodeRotationSensor::instanciate);
      addSensorType("NodeContactForce",&mars::sim::NodeContactForceSensor::instanciate);
      addSensorType("NodeCOM",&mars::sim::NodeCOMSensor::instanciate);
      addSensorType("NodeVelocity",&mars::sim::NodeVelocitySensor::instanciate);
      addSensorType("NodeAngularVelocity",&mars::sim::NodeAngularVelocitySensor::instanciate);
      addSensorType("MotorCurrent",&mars::sim::MotorCurrentSensor::instanciate);
      addSensorType("HapticField",&mars::sim::HapticFieldSensor::instanciate);

      addMarsParser("RaySensor",&mars::sim::RaySensor::parseConfig);
      addMarsParser("RotatingRaySensor",&mars::sim::RotatingRaySensor::parseConfig);
      addMarsParser("MultiLevelLaserRangeFinder",&mars::sim::MultiLevelLaserRangeFinder::parseConfig);
      addMarsParser("CameraSensor",&mars::sim::CameraSensor::parseConfig);
      addMarsParser("ScanningSonar",&mars::sim::ScanningSonar::parseConfig);
      addMarsParser("JointPosition",&mars::sim::JointArraySensor::parseConfig);
      addMarsParser("JointVelocity",&mars::sim::JointArraySensor::parseConfig);
      addMarsParser("JointLoad",&mars::sim::JointArraySensor::parseConfig);
      addMarsParser("JointTorque",&mars::sim::JointArraySensor::parseConfig);
      addMarsParser("JointAVGTorque",&mars::sim::JointArraySensor::parseConfig);
      addMarsParser("Joint6DOF",&mars::sim::Joint6DOFSensor::parseConfig);
      addMarsParser("NodeContact",&mars::sim::NodeContactSensor::parseConfig);
      addMarsParser("NodeIMU", &mars::sim::NodeIMUSensor::parseConfig);
      addMarsParser("NodePosition",&mars::sim::NodeArraySensor::parseConfig);
      addMarsParser("NodeRotation",&mars::sim::NodeArraySensor::parseConfig);
      addMarsParser("NodeContactForce",&mars::sim::NodeArraySensor::parseConfig);
      addMarsParser("NodeCOM",&mars::sim::NodeArraySensor::parseConfig);
      addMarsParser("NodeVelocity",&mars::sim::NodeArraySensor::parseConfig);
      addMarsParser("NodeAngularVelocity",&mars::sim::NodeArraySensor::parseConfig);
      addMarsParser("MotorCurrent",&mars::sim::MotorCurrentSensor::parseConfig);
      addMarsParser("HapticField",&mars::sim::HapticFieldSensor::parseConfig);

      // missing sensors:
      //   RayGridSensor
    }

    /**
     *\brief Returns true, if the sensor with the given id exists.
     *
     * \param id The id of the sensor to look for.
     * \return boolean, whether the node exists.
     */
    bool EnvireSensorManager::exists(unsigned long index) const {
      std::map<unsigned long, mars::interfaces::BaseSensor*>::const_iterator iter = simSensors.find(index);
      if(iter != simSensors.end()) {
        return true;
      }
      return false;
    }

    /**
     * \brief Gives information about core exchange data for sensors.
     *
     * \param sensorList A pointer to a vector that is filled with a
     * core_objects_exchange struct for every sensor. The vector is cleared
     * in the beginning of this function.
     */
    void EnvireSensorManager::getListSensors(std::vector<mars::interfaces::core_objects_exchange> *sensorList) const {
      mars::interfaces::core_objects_exchange obj;
      std::map<unsigned long, mars::interfaces::BaseSensor*>::const_iterator iter;
      sensorList->clear();
      iMutex.lock();
      for (iter = simSensors.begin(); iter != simSensors.end(); iter++) {
        iter->second->getCoreExchange(&obj);
        sensorList->push_back(obj);
      }
      iMutex.unlock();
    }

    /**
     * \brief Gives all information of a certain sensor.
     *
     * \param index The unique id of the sensor to get information for.
     *
     * \return A pointer to the mars::interfaces::BaseSensor of the sensor with the given id.
     * \throw std::runtime_error if a motor with the given index does not exist.
     */
    const mars::interfaces::BaseSensor* EnvireSensorManager::getFullSensor(unsigned long index) const {
      mars::utils::MutexLocker locker(&iMutex);
      std::map<unsigned long, mars::interfaces::BaseSensor*>::const_iterator iter;

      iter = simSensors.find(index);
      if (iter != simSensors.end())
        return iter->second;
      else {
        char msg[128];
        sprintf(msg, "could not find sensor with index: %lu", index);
        throw std::runtime_error(msg);
      }
    }

    unsigned long EnvireSensorManager::getSensorID(std::string name) const {
      mars::utils::MutexLocker locker(&iMutex);
      std::map<unsigned long, mars::interfaces::BaseSensor*>::const_iterator it;
      for(it = simSensors.begin(); it != simSensors.end(); it++){
        if(it->second->name.compare(name) == 0){
          return it->first;
        }
      }
      printf("Cannot find Sensor with name: \"%s\"\n",name.c_str());
      return 0;
    }

    /**
     * \brief Removes a sensor from the simulation.
     *
     * \param index The unique id of the sensor to remove form the simulation.
     */
    void EnvireSensorManager::removeSensor(unsigned long index) {
      mars::interfaces::BaseSensor* tmpSensor = NULL;
      iMutex.lock();
      std::map<unsigned long, mars::interfaces::BaseSensor*>::iterator iter = simSensors.find(index);
      if (iter != simSensors.end()) {
        tmpSensor = iter->second;
        simSensors.erase(iter);
        if (tmpSensor)
          delete tmpSensor;
      }
      iMutex.unlock();

      control->sim->sceneHasChanged(false);
    }


    /**
     * \brief This function returns the SimSensor object for a given index.
     *
     * \param name The index of the sensor to get the core sensor object.
     *
     * \returns Returns a pointer to the corresponding sensor object.
     */
    mars::interfaces::BaseSensor* EnvireSensorManager::getSimSensor(unsigned long index) const {
      mars::utils::MutexLocker locker(&iMutex);
      std::map<unsigned long, mars::interfaces::BaseSensor*>::const_iterator iter = simSensors.find(index);

      if (iter != simSensors.end())
        return iter->second;
      else
        return NULL;
    }

    /**
     * \brief This function provides the sensor data for a given index.
     *
     * \param data The sensor data of the sensor.
     *
     * \param index The index of the sensor to get the data
     */
    int EnvireSensorManager::getSensorData(unsigned long id, mars::interfaces::sReal **data) const {
      mars::utils::MutexLocker locker(&iMutex);
      std::map<unsigned long, mars::interfaces::BaseSensor*>::const_iterator iter;

      iter = simSensors.find(id);
      if (iter != simSensors.end())
        return iter->second->getSensorData(data);

      LOG_DEBUG("Cannot Find Sensor wirh id: %lu\n",id);
      return 0;
    }


    /**
     *\brief Returns the number of sensors that are currently present in the simulation.
     *
     *\return The number of all sensors.
     */
    int EnvireSensorManager::getSensorCount() const {
      mars::utils::MutexLocker locker(&iMutex);
      return simSensors.size();
    }


    /**
     * \brief Destroys all sensors in the simulation.
     *
     * \details The \c clear_all flag indicates if the reload sensors should
     * be destroyed as well. If set to \c false they are left intact.
     *
     * \param clear_all Indicates if the reload sensors should
     * be destroyed as well. If set to \c false they are left intact.
     */
    void EnvireSensorManager::clearAllSensors(bool clear_all) {
      mars::utils::MutexLocker locker(&iMutex);
      std::map<unsigned long, mars::interfaces::BaseSensor*>::iterator iter;
      for(iter = simSensors.begin(); iter != simSensors.end(); iter++) {
        assert(iter->second);
        mars::interfaces::BaseSensor *sensor = iter->second;
        delete sensor;
      }
      simSensors.clear();
      if(clear_all) simSensorsReload.clear();
      next_sensor_id = 1;
    }


    /**
     * \brief This function reloads all sensors from a temporary sensor pool.
     *
     * \details All sensors that have been added with \c reload value as \c true
     * are added back to the simulation again with a \c reload value of \c true.
     */
    void EnvireSensorManager::reloadSensors(void) {

      std::vector<SensorReloadHelper>::iterator iter;
      iMutex.lock();
      for(iter=simSensorsReload.begin(); iter!=simSensorsReload.end(); ++iter) {
        iMutex.unlock();

        createAndAddSensor(iter->type, iter->config, true);
        iMutex.lock();
      }
      iMutex.unlock();
    }

    void EnvireSensorManager::addMarsParser(const std::string string,
				      mars::interfaces::BaseConfig* (*func)(mars::interfaces::ControlCenter*, ConfigMap*)){
      marsParser.insert(std::pair<const std::string, mars::interfaces::BaseConfig* (*)(mars::interfaces::ControlCenter*, configmaps::ConfigMap*)>(string,func));
    }

    void EnvireSensorManager::addSensorType(const std::string &name, mars::interfaces::BaseSensor* (*func)(mars::interfaces::ControlCenter*, mars::interfaces::BaseConfig*)){
      availableSensors.insert(std::pair<const std::string,mars::interfaces::BaseSensor* (*)(mars::interfaces::ControlCenter*, mars::interfaces::BaseConfig*)>(name,func));
    }

    mars::interfaces::BaseSensor* EnvireSensorManager::createAndAddSensor(const std::string &type_name,
                                                  mars::interfaces::BaseConfig *config,
						  bool reload){
      assert(config);
      std::map<const std::string,mars::interfaces::BaseSensor* (*)(mars::interfaces::ControlCenter*, mars::interfaces::BaseConfig*)>::iterator it = availableSensors.find(type_name);
      if(it == availableSensors.end()){
        std::cerr << "Could not load unknown Sensor with name: \"" << type_name.c_str()<< "\"" << std::endl;
        return 0;
      }

      int id = -1;
      iMutex.lock();
      id = next_sensor_id++;
      iMutex.unlock();

      if(config->name.empty()){
        std::stringstream str;
        str << "SENSOR-" << id;
        config->name = str.str();
      }
      config->id = id;
      mars::interfaces::BaseSensor *sensor = ((*it).second)(this->control,config);
      iMutex.lock();
      simSensors[id] = sensor;
      iMutex.unlock();

      if(!reload) {
        simSensorsReload.push_back(SensorReloadHelper(type_name, config));
      }

      return sensor;
    }

    mars::interfaces::BaseSensor* EnvireSensorManager::createAndAddSensor(configmaps::ConfigMap *config,
                                                  bool reload) {

      std::string type = (*config)["type"][0].getString();
      std::map<const std::string, mars::interfaces::BaseConfig* (*)(mars::interfaces::ControlCenter*, configmaps::ConfigMap*)>::iterator it = marsParser.find(type);

      if(it == marsParser.end()){
        std::cerr << "Could not find MarsParser for sensor with name: \"" << type.c_str()<< "\"" << std::endl;
        return 0;
      }
      //LOG_DEBUG("found sensor: %s", type.c_str());
      mars::interfaces::BaseConfig *cfg = ((*it).second)(control, config);
      cfg->name = (*config)["name"][0].getString();
      return createAndAddSensor(type, cfg);
    }

    } // end of namespace envire_managers
  } // end of namespace plugins
} // end of namespace mars
