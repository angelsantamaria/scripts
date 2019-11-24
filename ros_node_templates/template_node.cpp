#include "project_name/node_filename.h"

/* main function */
int main(int argc,char *argv[])
{
  ros::init(argc, argv, "project_name");
  ros::NodeHandle nh(ros::this_node::getName());
  ros::Rate loop_rate(50);

  dynamic_reconfigure::Server<project_name::Config> dsrv;    

  project_name::ClassNode node(nh, dsrv);

  while (ros::ok()) 
  {
    node.mainNodeThread();
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
