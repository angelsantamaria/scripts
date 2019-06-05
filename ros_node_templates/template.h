// Copyright (C)
// All rights reserved.
//
// This file is part of a free software package: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.
// 
// IMPORTANT NOTE: This code has been generated through a script. 
// Please do NOT delete any comments to guarantee the correctness
// of the scripts. ROS topics can be easly add by using those scripts. 

#ifndef _class_filename_h_
#define _class_filename_h_

#include "alg_filename.h"

// ROS
#include <ros/ros.h>
#include <ros/package.h>

// boost
#include <boost/bind.hpp>

// dynamic reconfigure server include
#include <dynamic_reconfigure/server.h>

// [publisher subscriber headers]

// [service client headers]

// [action server client headers]

namespace project_name
{

/**
 * \brief Algorithm Class
 *
 */
class Class
{
  private:
    // [publisher attributes]

    // [subscriber attributes]

    // [service attributes]

    // [client attributes]

    // [action server attributes]

    // [action client attributes]

   /**
    * \brief template algorithm class
    *
    * This template class refers to an implementation of an specific algorithm
    * interface. Will be used in the derivate class to define the common 
    * behaviour for all the different implementations from the same algorithm.
    */
    ClassAlg alg_;

   /**
    * \brief config variable
    *
    * This variable has all the driver parameters defined in the cfg config file.
    * Is updated everytime function config_update() is called.
    */
    Config config_;

  public:
   /**
    * \brief Constructor
    * 
    * This constructor initializes specific class attributes and all ROS
    * communications variables to enable message exchange.
    */
    Class(ros::NodeHandle &nh, dynamic_reconfigure::Server<Config> &dsrv);

   /**
    * \brief Destructor
    * 
    * This destructor frees all necessary dynamic memory allocated within this
    * this class.
    */
    ~Class(void);

   /**
    * \brief main node thread
    *
    * This is the main thread node function. Code written here will be executed
    * in every node loop while the algorithm is on running state. Loop frequency 
    * can be tuned by modifying loop rate attribute.
    */
    void mainNodeThread(void);

  protected:

   /**
    * \brief dynamic reconfigure server callback
    * 
    * This method is called whenever a new configuration is received through
    * the dynamic reconfigure. The derivated generic algorithm class must 
    * implement it.
    *
    * \param config an object with new configuration from all algorithm 
    *               parameters defined in the config file.
    * \param level  integer referring the level in which the configuration
    *               has been changed.
    */
    void DynRecCallback(Config &config, uint32_t level=0);

    // [diagnostic functions]
    
    // [test functions]
};

} //end of namespace

#endif
