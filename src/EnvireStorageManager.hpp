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
 * \file EnvireManager.h
 * \author Raul (raul.dominguez@dfki.de)
 * \brief Create and store simulated motors from their definition in the envire representation
 *
 * Version 0.1
 */

#ifndef MARS_PLUGINS_ENVIRE_STORAGE_MANAGER_H
#define MARS_PLUGINS_ENVIRE_STORAGE_MANAGER_H

#ifdef _PRINT_HEADER_
  #warning "EnvireStorageManager.hpp"
#endif

#include "EnvireDefs.hpp"

#include <envire_core/graph/EnvireGraph.hpp>

namespace mars {
  namespace plugins {
    namespace envire_managers {

      class EnvireStorageManager
      {

      public:

        static std::shared_ptr<EnvireStorageManager> instance() {
        //NOTE it is important that instance is a local static variable because we need a way to 
        //     control static initialization/destruction order.
        //     If instance is global the static initialization order depends on the order in which 
        //     libraries are loaded which may lead to strange crashes          
            // Static variables are created and initialised when their
            // definition is run through for the first time. The variable and
            // its content are then retained until the end of the program.
            static std::shared_ptr<EnvireStorageManager> storage(new EnvireStorageManager()); 
            return storage;
        }

        EnvireStorageManager() {
          graph = std::shared_ptr<envire::core::EnvireGraph> (new envire::core::EnvireGraph());
          graph->addFrame(SIM_CENTER_FRAME_NAME);
        }

        std::shared_ptr<envire::core::EnvireGraph> getGraph() {
          return graph;
        }

      private:
        std::shared_ptr<envire::core::EnvireGraph> graph;

      }; // end of class definition EnvireStorageManager

    } // end of namespace envire_managers
  } // end of namespace plugins
} // end of namespace mars

#endif // MARS_PLUGINS_ENVIRE_STORAGE_MANAGER_H
