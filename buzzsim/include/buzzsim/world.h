#ifndef SRC_WORLD_H
#define SRC_WORLD_H

#include <QPainter>

#include <buzzsim/turtle.h>

class World
{
 public:
  World();

  void init(const std::vector<turtle::Turtle::Options>& options);
  void update();
  void paint(QPainter* painter);

 private:
  std::vector<std::unique_ptr<turtle::Turtle>> turtles_;

  ros::NodeHandle nh;
  ros::NodeHandle pnh;
};

#endif //SRC_WORLD_H
