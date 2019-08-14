#include <week_slam/g2o/landmark_measurement.h>

namespace g2o
{
LandmarkMeasurement::LandmarkMeasurement(int id, double x, double y) : id_{ id }, x_{ x }, y_{ y }
{
}


}  // namespace g2o
