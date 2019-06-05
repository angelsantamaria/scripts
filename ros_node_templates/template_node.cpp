#include "template.h"

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
