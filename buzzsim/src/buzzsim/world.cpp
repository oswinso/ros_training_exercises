#include <buzzsim/world.h>

World::World() : pnh{ "~" }
{
}

void World::init(const std::vector<turtle::Turtle::Options>& options)
{
  for (const auto& option : options)
  {
    turtles_.emplace_back(std::make_unique<turtle::Turtle>(option));
  }
}

void World::update()
{
  for (auto& turtle : turtles_)
  {
    turtle->updatePose();
  }
}

void World::paint(QPainter* painter)
{
  for (auto& turtle : turtles_)
  {
    turtle->paint(*painter);
  }
}
