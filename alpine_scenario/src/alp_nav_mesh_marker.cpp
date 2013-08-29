/**
* \file
* \brief Publisher for the alpine environment mesh as rviz marker.
*
* \author Daniel Tietjen
*
*/
#include <alp_nav_init.h>
#include <visualization_msgs/Marker.h>

const char* g_LOG_NODE_PREFIX = "ALP-MESH-NODE:";

/**
 * \brief Publish the alpine environment mesh as rviz marker.
 *
 * Creates a new node, builds a visualization marker object with a reference
 * to alpine environment mesh (COLLADA format, .dae) and publishes this object every second.
 *
 * \author Daniel Tietjen
 */
int main(int argc, char** argv)
{
  // Initialize a new node
  ros::init(argc, argv, "alp_nav_mesh_marker");
  ros::NodeHandle node_handle_alp_mesh_marker;

  ROS_DEBUG("%s Starting alpine mesh marker node for visualization in rviz", g_LOG_NODE_PREFIX);

  // There is no need to puffer markers, because the marker message stays the same
  ros::Publisher marker_mesh_publisher = node_handle_alp_mesh_marker.
      advertise<visualization_msgs::Marker>("visualization_marker", 1);

  std::string mesh_frame_id;
  ros::param::param<std::string>(AlpineNavigation::ALP_NAV_ROS_NAMESPACE + "odom_combined_id",
                                 mesh_frame_id, "/odom_combined");

  // Build marker object
  visualization_msgs::Marker marker;
  marker.header.frame_id = mesh_frame_id;
  marker.ns = AlpineNavigation::ALP_NAV_ROS_NAMESPACE + "alp_mesh";
  marker.id = 0;
  marker.type = visualization_msgs::Marker::MESH_RESOURCE;
  marker.action = visualization_msgs::Marker::ADD;
  marker.scale.x = 1.0;
  marker.scale.y = 1.0;
  marker.scale.z = 1.0;
  marker.lifetime = ros::Duration();
  marker.mesh_resource = "package://alpine_scenario/models/alpine_landscape.dae";
  marker.mesh_use_embedded_materials = true;
  marker.pose.position.z = -3.884;

  // Publish mesh marker every second (hence there is no need to start rviz before this node)
  ros::Rate spin_rate(1);
  while (ros::ok())
  {
    marker.header.stamp = ros::Time::now();
    marker_mesh_publisher.publish(marker);
    spin_rate.sleep();
  }
}
