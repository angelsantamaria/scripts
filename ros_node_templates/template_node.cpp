#include "template_node.h"

TemplateNode::TemplateNode(ros::NodeHandle &nh, dynamic_reconfigure::Server<Config> &dsrv) 
{
  // Dynamic Reconfigure
  dynamic_reconfigure::Server<Config>::CallbackType dsrv_cb;
  dsrv_cb = boost::bind(&TemplateNode::DynRecCallback, this, _1, _2);
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

TemplateNode::~TemplateNode(void)
{
  // [free dynamic memory]
}

void TemplateNode::mainNodeThread(void)
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

void TemplateNode::DynRecCallback(Config &config, uint32_t level)
{
  this->alg_.lock();
  this->config_=config;
  this->alg_.unlock();
}

/* main function */
int main(int argc,char *argv[])
{
  ros::init(argc, argv, "template");
  ros::NodeHandle nh(ros::this_node::getName());
  ros::Rate loop_rate(50);

  dynamic_reconfigure::Server<Config> dsrv;    

  TemplateNode node(nh, dsrv);

  while (ros::ok()) 
  {
    node.mainNodeThread();
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
