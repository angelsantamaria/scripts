#include "project_name/class_filename.h"

namespace project_name
{

ClassNode::ClassNode(ros::NodeHandle &nh, dynamic_reconfigure::Server<Config> &dsrv) 
{
  // Dynamic Reconfigure
  dynamic_reconfigure::Server<Config>::CallbackType dsrv_cb;
  dsrv_cb = boost::bind(&ClassNode::DynRecCallback, this, _1, _2);
  dsrv.setCallback(dsrv_cb);

  //init class attributes if necessary

  // Get launch parameters
  //double d_example;
  //nh.param<double>("d_example",d_example,0.5);

  // [init publishers]
  
  // [init subscribers]
  
  // [init services]
  
  // [init clients]
  
  // [init action servers]
  
  // [init action clients]
}

ClassNode::~ClassNode(void)
{
  // [free dynamic memory]
}

void ClassNode::mainNodeThread(void)
{
  // [fill msg structures]
  
  // [fill srv structure and make request to the server]
  
  // [fill action structure and make request to the action server]

  // [publish messages]
}

/*  [subscriber callbacks] */

/*  [service callbacks] */

/*  [action callbacks] */

/*  [action requests] */

void ClassNode::DynRecCallback(Config &config, uint32_t level)
{
  this->alg_.lock();
  this->config_=config;
  this->alg_.unlock();
}

} //end of namespace