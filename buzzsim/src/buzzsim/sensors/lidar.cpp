#include <buzzsim/sensors/cgal_types.h>
#include <buzzsim/sensors/lidar.h>
#include <buzzsim/sensors/print_utils.h>

namespace turtle
{
Lidar::Lidar(const Options& options) : options_{ options }
{
}

pcl::PointCloud<pcl::PointXY> Lidar::getLidarScan(const motion::Pose& pose, const std::vector<Obstacle>* obstacles)
{
  // Idea:
  // Sort each vertex by angle
  // Iterate through vertices in order
  // If distance is closer, then go calculate
  // If encounter end of segment, then look at what's currently on the stack?

  // Heap of segments (sorted by distance)
  // Heap of (vertices, start / end bool)
  // bimap of start vertex <-> end vertex?

  // Iterate through heap
  // if start vertex, add to b tree
  // if end vertex of current segment, then calculate all points and add them in
  // if end vertex of random segment, then remove that random one's start vertex,
  // if tree is empty, then keep going
  // if smaller than, then still add but don't add point
  // if larger than max angle, then terminate.

  // OR (probably this)
  // CGAL visibility polygon -> returns a polygon
  // Iterate through all angles, find intersection point.
  // If intersection point distance < range, then include it

  // TODO: CGAL, triangle thing with holes. Make a large rectangle that has width and length = range, then do the thing.
  return {};
}

std::vector<Lidar::LidarHit> Lidar::getLidarScan(const motion::Pose& pose, const Obstacle& obstacle)
{
  std::vector<LidarHit> hits;
  return hits;
}

void Lidar::test2(const motion::Position& position, const std::vector<Obstacle>& obstacles)
{
  auto polygon = getPolygonWithHoles(position, obstacles);
  auto visibility_polygon = computeVisibilityPolygon(position, polygon);
  ROS_INFO_STREAM("Result: " << visibility_polygon);
}

Polygon_2 Lidar::computeVisibilityPolygon(const motion::Position& position, const Polygon_with_holes_2& polygon) const
{
  Arrangement_2 env;
  std::vector<Segment_2> segments;

  for (auto it = polygon.outer_boundary().edges_begin(); it != polygon.outer_boundary().edges_end(); it++)
  {
    ROS_INFO_STREAM(*it);
    segments.emplace_back(*it);
  }

  ROS_INFO_STREAM("holes");
  for (auto it = polygon.holes_begin(); it != polygon.holes_end(); it++)
  {
    for (auto edge_it = it->edges_begin(); edge_it != it->edges_end(); edge_it++)
    {
      ROS_INFO_STREAM(*edge_it);
      segments.emplace_back(*edge_it);
    }
  }

  CGAL::insert_non_intersecting_curves(env, segments.begin(), segments.end());

  ROS_INFO_STREAM("here right?");

  // Find the face of the query point
  Point_2 q{position.x, position.y};
  Arrangement_2::Face_const_handle* face;
  CGAL::Arr_naive_point_location<Arrangement_2> pl(env);
  CGAL::Arr_point_location_result<Arrangement_2>::Type obj = pl.locate(q);

  // The query point locates in the interior of the a face
  face = boost::get<Arrangement_2::Face_const_handle>(&obj);

  // Compute regularized visibility area
  Arrangement_2 regular_output;
  TEV regular_visibility(env);

  regular_visibility.compute_visibility(q, *face, regular_output);

  Polygon_2 visibility_polygon;
  ROS_INFO_STREAM("Regularized visbility of region of q has " << regular_output.number_of_edges() << " edges: ");
  for (auto it = regular_output.edges_begin(); it != regular_output.edges_end(); it++)
  {
    ROS_INFO_STREAM("[" << it->source()->point() << " -> " << it->target()->point() << "]");
    visibility_polygon.push_back(it->source()->point());
  }

  ROS_INFO_STREAM("Done!");
  return visibility_polygon;
}

Polygon_with_holes_2 Lidar::getPolygonWithHoles(const motion::Position& position, const std::vector<Obstacle>& obstacles) const
{
  auto [x, y] = position;
  auto range = options_.range;

  Point_2 top{x + range, y};
  Point_2 bottom{x - range, y};
  Point_2 left{x, y + range};
  Point_2 right{x, y - range};

  Polygon_2 lidar_range_rect;
  lidar_range_rect.push_back(top);
  lidar_range_rect.push_back(left);
  lidar_range_rect.push_back(bottom);
  lidar_range_rect.push_back(right);

  Polygon_set_2 result;
  for (const auto& obstacle : obstacles)
  {
    Polygon_2 polygon_obstacle;
    for (const auto& point : obstacle.points)
    {
      Point_2 p{point.x, point.y};
      polygon_obstacle.push_back(p);
    }
    // Flip it if it's the wrong orientation
    if (polygon_obstacle.is_clockwise_oriented())
    {
      polygon_obstacle.reverse_orientation();
    }

    ROS_INFO_STREAM("Inserting " << polygon_obstacle);
    result.insert(polygon_obstacle);
  }

  result.complement();
  result.intersection(lidar_range_rect);

  std::list<Polygon_with_holes_2> result_list;
  result.polygons_with_holes(std::back_inserter(result_list));

  ROS_INFO_STREAM(result_list.front());

  return result_list.front();
}

void Lidar::test()
{
  Point_2 p1{ 0, 4 };
  Point_2 p2{ 0, 0 };
  Point_2 p3{ 3, 2 };
  Point_2 p4{ 4, 0 };
  Point_2 p5{ 4, 4 };
  Point_2 p6{ 1, 2 };

  std::vector<Segment_2> segments{ Segment_2{ p1, p2 }, Segment_2{ p2, p3 }, Segment_2{ p3, p4 },
                                   Segment_2{ p4, p5 }, Segment_2{ p5, p6 }, Segment_2{ p6, p1 } };

  Arrangement_2 env;
  CGAL::insert_non_intersecting_curves(env, segments.begin(), segments.end());

  // Find the face of the query point
  Point_2 q(0.5, 2);
  Arrangement_2::Face_const_handle* face;
  CGAL::Arr_naive_point_location<Arrangement_2> pl(env);
  CGAL::Arr_point_location_result<Arrangement_2>::Type obj = pl.locate(q);

  // The query point locates in the interior of the a face
  face = boost::get<Arrangement_2::Face_const_handle>(&obj);

  // Compute regularized visibility area
  using RSPV = CGAL::Simple_polygon_visibility_2<Arrangement_2, CGAL::Tag_true>;
  Arrangement_2 regular_output;
  RSPV regular_visibility(env);

  regular_visibility.compute_visibility(q, *face, regular_output);

  ROS_INFO_STREAM("Regularized visbility of region of q has " << regular_output.number_of_edges() << " edges: ");
  for (auto it = regular_output.edges_begin(); it != regular_output.edges_end(); it++)
  {
    ROS_INFO_STREAM("[" << it->source()->point() << " -> " << it->target()->point() << "]");
  }

  ROS_INFO_STREAM("Done!");
}

}  // namespace turtle
