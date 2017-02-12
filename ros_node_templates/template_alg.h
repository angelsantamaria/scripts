// Copyright (C) 2010-2011 Institut de Robotica i Informatica Industrial, CSIC-UPC.
// Author 
// All rights reserved.
//
// This file is part of iri-ros-pkg
// iri-ros-pkg is free software: you can redistribute it and/or modify
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
// IMPORTANT NOTE: This code has been generated through a script from the 
// iri_ros_scripts. Please do NOT delete any comments to guarantee the correctness
// of the scripts. ROS topics can be easly add by using those scripts. Please
// refer to the IRI wiki page for more information:
// http://wikiri.upc.es/index.php/Robotics_Lab

#ifndef _template_alg_h_
#define _template_alg_h_

#include <template_namespace/TemplateConfig.h>

#include <pthread.h>

//include template_alg main library

/**
 * \brief define config type
 *
 * Define a Config type with the TemplateConfig. 
 * All implementations will then use the same variable type Config.
 */
typedef template_namespace::TemplateConfig Config;

/**
 * \brief Mutex class
 *
 */
class CMutex
{  
  protected:

    /**
     * \brief define config type
     *
     * Define a Config type with the TemplateConfig. All driver implementations
     * will then use the same variable type Config.
     */
    pthread_mutex_t access_;
  
  public:

    CMutex()
    {
      pthread_mutex_init(&this->access_,NULL);
    };
    ~CMutex()
    {
      pthread_mutex_destroy(&this->access_);
    };
    /**
     * \brief Lock Algorithm
     *
     * Locks access to the Algorithm class
     */
    void lock(void) { pthread_mutex_lock(&this->access_); };
  
    /**
     * \brief Unlock Algorithm
     *
     * Unlocks access to the Algorithm class
     */
    void unlock(void) { pthread_mutex_unlock(&this->access_); };
  
    /**
     * \brief Tries Access to Algorithm
     *
     * Tries access to Algorithm
     * 
     * \return true if the lock was adquired, false otherwise
     */
    bool try_enter(void) 
    { 
      if(pthread_mutex_trylock(&this->access_)==0)
        return true;
      else
        return false;
    };  
};


/**
 * \brief Algorithm Class
 *
 *
 */
class TemplateAlg
{
  protected:

    // private attributes and methods

    CMutex alg_mutex_;

  public:

   /**
    * \brief config variable
    *
    * This variable has all the driver parameters defined in the cfg config file.
    * Is updated everytime function config_update() is called.
    */
    Config config_;

   /**
    * \brief constructor
    *
    * In this constructor parameters related to the specific driver can be
    * initalized. Those parameters can be also set in the openDriver() function.
    * Attributes from the main node driver class IriBaseDriver such as loop_rate,
    * may be also overload here.
    */
    TemplateAlg(void);

   /**
    * \brief Destructor
    *
    * This destructor is called when the object is about to be destroyed.
    *
    */
    ~TemplateAlg(void);

    /**
     * \brief Lock Algorithm
     *
     * Locks access to the Algorithm class
     */
    void lock(void) { this->alg_mutex_.lock(); };
  
    /**
     * \brief Unlock Algorithm
     *
     * Unlocks access to the Algorithm class
     */
    void unlock(void) { this->alg_mutex_.unlock(); };
  
    /**
     * \brief Tries Access to Algorithm
     *
     * Tries access to Algorithm
     * 
     * \return true if the lock was adquired, false otherwise
     */
    bool try_enter(void) { return this->alg_mutex_.try_enter(); }; 
  
    // here define all template_alg interface methods to retrieve and set
    // the driver parameters
};

#endif
