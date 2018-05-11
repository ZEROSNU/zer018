#include "path.h"
using namespace HybridAStar;

void Path::clear() {
  Node3D node;
  path.pathpoints.clear();
  path.headings.clear();
  path.header.stamp = ros::Time::now();
}

void Path::updatePath(std::vector<Node3D> nodePath) {
  //path.header.stamp = ros::Time::now();
  geometry_msgs::Vector3 vertex;
  vertex.z = 0;
  for (int i = 0; i < nodePath.size(); ++i) {
    vertex.x = nodePath[i].getY();
    vertex.y = nodePath[i].getX();
    path.pathpoints.push_back(vertex);
    path.headings.push_back((nodePath[i].getT()-M_PI)*180.f/M_PI);
  }
  return;
}
