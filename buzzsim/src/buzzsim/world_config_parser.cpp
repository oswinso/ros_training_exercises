
#include <buzzsim/world_config_parser.h>

WorldConfigParser::SpawnOptions WorldConfigParser::parseConfig(const std::string& config_path, const std::string& world_name) const
{
  try
  {
    YAML::Node config_node = YAML::LoadFile(config_path);

    YAML::Node world_nodes = config_node["worlds"];

    if (!world_nodes.IsDefined())
    {
      ROS_ERROR_STREAM("\"worlds\" key is not defined!");
      return getSimpleOptions();
    }

    std::string options;

    std::unordered_map<std::string, SpawnOptions> spawn_options;
    for (const auto& world_node : world_nodes)
    {
      auto name = world_node.first.as<std::string>();
      ROS_ERROR_STREAM("\n\n\nParsing world \"" << name << "\"...");
      spawn_options.insert(std::make_pair(name, parseWorld(world_node.second)));

      options += name + " ";
    }

    if (spawn_options.find(world_name) == spawn_options.end())
    {
      ROS_ERROR_STREAM("World " << world_name << " doesn not exist. Valid worlds are: " << options);
      return getSimpleOptions();
    }

    ROS_ERROR_STREAM("World name: " << world_name);

    return spawn_options.at(world_name);
  }
  catch (YAML::Exception& e)
  {
    ROS_ERROR_STREAM("[YAML Exception] Error parsing " << config_path << ":\n" << e.what());
  }
  catch (std::runtime_error& e)
  {
    ROS_ERROR_STREAM("Runtime error:\n" << e.what());
  }
  return getSimpleOptions();
}

WorldConfigParser::SpawnOptions WorldConfigParser::parseWorld(
    const YAML::Node& world) const
{
  const YAML::Node& turtles = world["turtles"];
  if (!turtles.IsDefined())
  {
    ROS_ERROR_STREAM("\"turtles\" key is not defined for world.");
    throw std::runtime_error("\"turtles\" key is not defined for world.");
  }

  SpawnOptions spawn_options;
  int image_num = 0;
  for (const auto& turtle : turtles)
  {
    ROS_ERROR_STREAM("\nParsing turtle " << turtle.first.as<std::string>() << " (" << image_num << ")...");
    auto option = turtle.second.as<turtle::Turtle::Options>();
    ROS_ERROR_STREAM("Options: " << option);
    option.name = turtle.first.as<std::string>();
    option.turtle_image = images_[image_num];

    spawn_options.emplace_back(option);

    image_num = (image_num + 1) % images_.size();
  }

  return spawn_options;
}

WorldConfigParser::SpawnOptions WorldConfigParser::getSimpleOptions() const
{
  SpawnOptions spawn_options;
  motion::State state{};
  turtle::Turtle::Options simple_option{ "oswin", images_[0], state, getSimpleLimits() };
  spawn_options.emplace_back(simple_option);

  return spawn_options;
}

motion::Limits WorldConfigParser::getSimpleLimits() const
{
  motion::AccelerationLimits acceleration_limits{ 5.0, 5.0 };
  motion::TwistLimits twist_limits{ 2.0, 2.0 };

  motion::Limits limits{ acceleration_limits, twist_limits };
  return limits;
}

void WorldConfigParser::setImages(std::vector<QImage>&& images)
{
  images_ = std::move(images);
}

namespace YAML
{
bool convert<motion::Position>::decode(const Node& node, motion::Position& rhs)
{
  if (!node.IsMap())
  {
    return false;
  }

  rhs.x = 0.0;
  if (node["x"].IsDefined())
  {
    if (!node["x"].IsScalar())
    {
      return false;
    }
    rhs.x = node["x"].as<double>();
  }

  rhs.y = 0.0;
  if (node["y"].IsDefined())
  {
    if (!node["y"].IsScalar())
    {
      return false;
    }
    rhs.y = node["y"].as<double>();
  }

  return true;
}

bool convert<motion::Pose>::decode(const Node& node, motion::Pose& rhs)
{
  if (!node.IsMap())
  {
    return false;
  }

  rhs.position = {};
  if (node["position"].IsDefined())
  {
    rhs.position = node["position"].as<motion::Position>();
  }

  rhs.orientation = 0.0;
  if (node["orientation"].IsDefined())
  {
    if (!node["orientation"].IsScalar())
    {
      return false;
    }
    rhs.orientation = node["orientation"].as<double>();
  }

  return true;
}

bool convert<motion::Twist>::decode(const Node& node, motion::Twist& rhs)
{
  if (!node.IsMap())
  {
    return false;
  }

  rhs.linear = 0.0;
  if (node["linear"].IsDefined())
  {
    if (!node["linear"].IsScalar())
    {
      return false;
    }
    rhs.linear = node["linear"].as<double>();
  }

  rhs.angular = 0.0;
  if (node["angular"].IsDefined())
  {
    if (!node["angular"].IsScalar())
    {
      return false;
    }
    rhs.angular = node["angular"].as<double>();
  }

  return true;
}

bool convert<motion::State>::decode(const Node& node, motion::State& rhs)
{
  if (!node.IsMap())
  {
    return false;
  }

  rhs.pose = {};
  if (node["pose"].IsDefined())
  {
    rhs.pose = node["pose"].as<motion::Pose>();
  }

  rhs.twist = {};
  if (node["twist"].IsDefined())
  {
    rhs.twist = node["twist"].as<motion::Twist>();
  }

  return true;
}

bool convert<motion::AccelerationLimits>::decode(const Node& node, motion::AccelerationLimits& rhs)
{
  if (!node.IsMap())
  {
    return false;
  }

  rhs.linear = std::numeric_limits<double>::max();
  if (node["linear"].IsDefined())
  {
    if (!node["linear"].IsScalar())
    {
      return false;
    }
    rhs.linear = node["linear"].as<double>();
  }

  rhs.angular = std::numeric_limits<double>::max();
  if (node["angular"].IsDefined())
  {
    if (!node["angular"].IsScalar())
    {
      return false;
    }
    rhs.angular = node["angular"].as<double>();
  }

  return true;
}

bool convert<motion::TwistLimits>::decode(const Node& node, motion::TwistLimits& rhs)
{
  if (!node.IsMap())
  {
    return false;
  }

  rhs.linear = std::numeric_limits<double>::max();
  if (node["linear"].IsDefined())
  {
    if (!node["linear"].IsScalar())
    {
      return false;
    }
    rhs.linear = node["linear"].as<double>();
  }

  rhs.angular = std::numeric_limits<double>::max();
  if (node["angular"].IsDefined())
  {
    if (!node["angular"].IsScalar())
    {
      return false;
    }
    rhs.angular = node["angular"].as<double>();
  }

  return true;
}

bool convert<motion::Limits>::decode(const Node& node, motion::Limits& rhs)
{
  if (!node.IsMap())
  {
    return false;
  }

  rhs.acceleration = {};
  if (node["acceleration"].IsDefined())
  {
    rhs.acceleration = node["acceleration"].as<motion::AccelerationLimits>();
  }

  rhs.twist = {};
  if (node["twist"].IsDefined())
  {
    rhs.twist = node["twist"].as<motion::TwistLimits>();
  }
  return true;
}

bool convert<turtle::Turtle::Options>::decode(const Node& node, turtle::Turtle::Options& rhs)
{
  if (!node.IsMap())
  {
    return false;
  }

  rhs.state = {};
  if (node["state"].IsDefined())
  {
    rhs.state = node["state"].as<motion::State>();
  }

  rhs.limits = {};
  if (node["limits"].IsDefined())
  {
    rhs.limits = node["limits"].as<motion::Limits>();
  }
  return true;
}
}  // namespace YAML
