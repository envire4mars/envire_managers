/*
 *  Copyright 2013, DFKI GmbH Robotics Innovation Center
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
 * \file EnvireMotors.h
 * \author Raul (raul.dominguez@dfki.de)
 * \brief Create and store simulated motors from their definition in the envire representation
 *
 * Version 0.1
 */

#ifndef MARS_PLUGINS_ENVIRE_MOTORMANAGER_H
#define MARS_PLUGINS_ENVIRE_MOTORMANAGER_H

#ifdef _PRINT_HEADER_
  #warning "EnvireMotorManager.hpp"
#endif

#include <mars/interfaces/sim/MotorManagerInterface.h>
#include <mars/interfaces/sim/ControlCenter.h>
#include <mars/utils/Mutex.h>

namespace mars {
  namespace plugins {
    namespace envire_managers {

      // inherit from MarsPluginTemplateGUI for extending the gui
      class EnvireMotorManager: public mars::interfaces::MotorManagerInterface
      {

      public:
         /**
       * \brief Constructor.
       *
       * \param c The pointer to the ControlCenter of the simulation.
       */ 
      EnvireMotorManager(mars::interfaces::ControlCenter *c);
  
      /**
       * \brief Destructor.
       */
      virtual ~EnvireMotorManager(){}
  
      /**
       * \brief Add a motor to the simulation.
       *
       * \param motorS A pointer to the MotorData that defines the new motor.
       *
       * \param reload Used internally by the simulation. The
       * default value is \c false. If this param is set to \c true the new motor
       * will not be reloaded after a reset of the simulation.
       *
       * \return The unique id of the newly added motor.
       */
      virtual unsigned long addMotor(mars::interfaces::MotorData *motorS, bool reload = false);

      /**
       *\brief Returns the number of motors that are currently present in the simulation.
       * 
       *\return The number of all motors.
       */
      virtual int getMotorCount() const;

      /**
       * \brief Change motor properties.
       *
       * \details The old struct is replaced 
       * by the new one completely, so prior to calling this function, one must 
       * ensure that all properties of this parameter are valid and as desired.
       *
       * \param motorS The id of the MotorData referred by this pointer must be the
       * same as the id of the motor that is to be edited. 
       */
      virtual void editMotor(const mars::interfaces::MotorData &motorS);
 
      /**
       * \brief Gives information about core exchange data for motors.
       *
       * \param motorList A pointer to a vector that is filled with a
       * core_objects_exchange struct for every motor and its index. The vector is cleared
       * in the beginning of this function.
       */
      virtual void getListMotors(std::vector<mars::interfaces::core_objects_exchange> *motorList) const;
 

      /**
       * \brief Gives all information of a certain motor.
       *
       * \param index The unique id of the motor to get information for.
       *
       * \return A pointer to the MotorData of the motor with the given id.
       */
      virtual const mars::interfaces::MotorData getFullMotor(unsigned long index) const;
 
      /**
       * \brief Removes a motor from the simulation.
       *
       * \param index The unique id of the motor to remove form the simulation.
       */
      virtual void removeMotor(unsigned long index);
 
      /**
       * \brief This function returns the SimMotor object for a given id.
       *
       * \warning This method is only internal used by the
       * MotorManager. Generally no other modules know the SimMotor class and
       * shouldn't use this method. All motor operations from outside the core
       * should be done over the MotorManager.
       *
       * \param index The id of the motor to get the core node object.
       *
       * \returns Returns a pointer to the corresponding motor object.
       */
      virtual mars::sim::SimMotor* getSimMotor(unsigned long id) const;

      /**
       * \brief This function returns the SimMotor object for a given name.
       *
       * \warning This method is only internal used by the
       * MotorManager. Generally no other modules know the SimMotor class and
       * shouldn't use this method. All motor operations from outside the core
       * should be done over the MotorManager.
       *
       * \param name The name of the motor to get the core node object.
       *
       * \returns Returns a pointer to the corresponding motor object.
       */
      virtual mars::sim::SimMotor* getSimMotorByName(const std::string &name) const;

      /**
       * \brief Sets the value of the motor with the given id to the given value.
       *
       * Essentially this function triggers the motor and moves the joint that is
       * attached to it.
       * Equivalent to \c moveMotor
       *
       * \param id The id of the motor whose value is to be changed.
       *
       * \param value The new value.
       */
      virtual void setMotorValue(unsigned long id, interfaces::sReal value);

      /**
       * \brief Sets the maximum torque of the motor with the given id to the given value.
       *
       * \param id The id of the motor whose value is to be changed.
       *
       * \param maxTorque The new maximum torque for the motor.
       */
      virtual void setMaxTorque(unsigned long id, mars::interfaces::sReal maxTorque);
      virtual void setMaxSpeed(unsigned long id, mars::interfaces::sReal maxSpeed);

      /**
       * \brief Sets the desired speed of a motor.
       * \param id The id of the motor whose value is to be changed.
       * \param value The new value in rad/s.
       */
      virtual void setMotorValueDesiredVelocity(unsigned long id, mars::interfaces::sReal velocity);
 
      /**
       * \brief Sets the proportional term of the motor with the given id to the given value.
       *
       * \details Only has effect on a PID motor. If the type of the motor with
       * the given id is different from PID, no effect is observed, although the 
       * P value of the motor object is still changed.
       *
       * \param id The id of the motor whose P value is to be changed.
       *
       * \param value The new P value.
       */
      virtual void setMotorP(unsigned long id, mars::interfaces::sReal value);
 
      /**
       * \brief Sets the integral term of the motor with the given id to the given value.
       *
       * \details Only has effect on a PID motor. If the type of the motor with
       * the given id is different from PID, no effect is observed, although the 
       * I value of the motor object is still changed.
       *
       * \param id The id of the motor whose I value is to be changed.
       *
       * \param value The new I value.
       */
      virtual void setMotorI(unsigned long id, mars::interfaces::sReal value);

      /**
       * \brief Sets the derivative term of the motor with the given id to the given value.
       *
       * \details Only has effect on a PID motor. If the type of the motor with
       * the given id is different from PID, no effect is observed, although the 
       * D value of the motor object is still changed.
       *
       * \param id The id of the motor whose D value is to be changed.
       *
       * \param value The new D value.
       */
      virtual void setMotorD(unsigned long id, mars::interfaces::sReal value);
 
      /** 
       * \brief Deactivates the motor with the given id.
       *
       * \param id The id of the motor that is to be deactivated.
       */
      virtual void deactivateMotor(unsigned long id);
 
      /** 
       * \brief Destroys all motors in the simulation.
       *
       * \details The \c clear_all flag indicates if the reload motors should
       * be destroyed as well. If set to \c false they are left intact.
       *
       * \param clear_all Indicates if the reload motors should
       * be destroyed as well. If set to \c false they are left intact.
       */
      virtual void clearAllMotors(bool clear_all = false);

      /**
       * \brief This function reloads all motors from a temporary motor pool.
       *
       * \details All motors that have been added with \c reload value as \c true
       * are added back to the simulation again with a \c reload value of \c true. 
       */
      virtual void reloadMotors(void);

      /**
       * \brief This function updates all motors with timing value \c calc_ms in miliseconds.
       *
       * \warning This function is only used internally and should not be called 
       * outside the core.
       *
       * \param calc_ms The timing value in miliseconds. 
       */
      virtual void updateMotors(mars::interfaces::sReal calc_ms);

      /**
       * \returns the actual position of the motor with the given Id.
       *          returns 0 if a motor with the given Id doesn't exist.
       */
      virtual mars::interfaces::sReal getActualPosition(unsigned long motorId) const;

      /**
       * \returns the torque excerted by the motor with the given Id. 
       *          returns 0 if a motor with the given Id doesn't exist.
       */
      virtual mars::interfaces::sReal getTorque(unsigned long motorId) const;

      /**
       * \brief Sets the value of the motor with the given id to the given value.
       *
       * Essentially this function triggers the motor and moves the joint that is
       * attached to it.
       * Equivalent to \c setMotorValue
       *
       * \param id The id of the motor whose value is to be changed.
       *
       * \param value The new value.
       */
      virtual void moveMotor(unsigned long index, mars::interfaces::sReal value);

      /**
       * \brief Detaches the joint with the given index from all motors that act on
       * it, if any. Used when a joint is destroyed.
       * 
       * \warning The detached motors are not destroyed and are still present in the 
       * simulation, although they do not have any effect on it. A call to 
       * \c removeMotor must be made to remove the motor completely.
       *
       * \param joint_index The id of the joint that is to be detached.
       */
      virtual void removeJointFromMotors(unsigned long joint_index);
  
      /**
       * \brief Retrieves the id of a motor by name
       *
       * \param motor_name Name of the motor to get the id for
       *
       * \return Id of the motor if it exists, otherwise 0
       */
      virtual unsigned long getID(const std::string& motor_name) const;

      virtual void getDataBrokerNames(unsigned long jointId, 
                                      std::string *groupName, 
                                      std::string *dataName) const;

      virtual void connectMimics();

      virtual void setOfflinePosition(interfaces::MotorId id,
                                      interfaces::sReal pos);

      virtual void updatePositionsFromGraph();

      virtual void edit(mars::interfaces::MotorId id, const std::string &key, const std::string &value);

    private:
      
      /*
       * This method attaches the a simMotor to the correspondent simulated joint
       * and stores it in the frame where the joint is.  
       *
       * NOTE Here we are assumming only a dynamic joint per frame.
       *
       * Steps: 
       *
       * 1. Search for the joint which name corresponds to the joint name in the
       * motor. 
       *  - Note that it might not be available a frame with the name of the
       * joint, because we store them in the source of the transformation. Thus
       * we have to search in the graph for the joint.  With this approach we
       * avoid assumming conventions like joint name = target/source link.
       *
       * 2. Link the  motor to that joint.  
       *
       * 3. Store the new motor in the frame where the joint was found.
       *
       */
      bool attachAndStoreMotor(std::shared_ptr<mars::sim::SimMotor> simMotor, const std::string & jointName);

      //! the id of the next motor that is added to the simulation
      unsigned long next_motor_id;

      //! a container for all motors currently present in the simulation
      std::map<unsigned long, std::shared_ptr<mars::sim::SimMotor> > simMotors;

      //! a containter for all motors that are reloaded after a reset of the simulation
      std::list<mars::interfaces::MotorData> simMotorsReload;

      //! a pointer to the control center
      mars::interfaces::ControlCenter *control;

      //! a mutex for the motor containters
      mutable utils::Mutex iMutex;

      // map of mimicmotors
      std::map<unsigned long, std::string> mimicmotors;

      }; // end of class definition EnvireMotors

    } // end of namespace envire_managers
  } // end of namespace plugins
} // end of namespace mars

#endif // MARS_PLUGINS_ENVIRE_MOTORS_H
