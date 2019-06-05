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

#ifndef _alg_filename_h_
#define _alg_filename_h_

#include <project_name/BasenameConfig.h>

#include <pthread.h>

//include alg_filename main library

namespace project_name
{

/**
 * \brief define config type
 *
 * Define a Config type with the BasenameConfig. 
 * All implementations will then use the same variable type Config.
 */
typedef project_name::BasenameConfig Config;

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
     * Define a Config type with the BasenameConfig. All driver implementations
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
class ClassAlg
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
    */
    ClassAlg(void);

   /**
    * \brief Destructor
    *
    * This destructor is called when the object is about to be destroyed.
    *
    */
    ~ClassAlg(void);

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
  
    // here define all alg_filename interface methods to retrieve and set
    // the driver parameters
};

} //end of namespace

#endif
