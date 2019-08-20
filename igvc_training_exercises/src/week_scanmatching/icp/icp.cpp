#include <Eigen/StdVector>

#include <Eigen/src/Eigenvalues/EigenSolver.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <week_scanmatching/icp/icp.h>

namespace scanmatcher
{
Result ICP::scanmatch(const pcl::PointCloud<pcl::PointXYZ>& input, const pcl::PointCloud<pcl::PointXYZ>& target,
                      const Eigen::Isometry3d& prediction)
{
  // Transformation is sR(p_i) + t
  Eigen::Isometry3d total_transform = Eigen::Isometry3d::Identity();
  double error = std::numeric_limits<double>::infinity();

  // TODO(Oswin) Parametrize this
  int max_iterations = 10;
  double threshold = 1e-5;

  int num_input = input.points.size();

  Eigen::Matrix3Xd aligned_input(3, num_input);
  for (int i = 0; i < num_input; i++)
  {
    aligned_input.col(i) = input[i].getVector3fMap().cast<double>();
  }

  Eigen::Matrix3Xd target_corresp(3, num_input);

  // Construct KD tree for target for finding closest points
  pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
  pcl::PointCloud<pcl::PointXYZ>::Ptr target_ptr(new pcl::PointCloud<pcl::PointXYZ>);
  *target_ptr = target;
  kdtree.setInputCloud(target_ptr);

  for (int i = 0; i < max_iterations; i++)
  {
    // TODO(Oswin) Make this a KD tree
    for (int j = 0; j < num_input; j++)
    {
      pcl::PointXYZ pcl_point;
      pcl_point.getVector3fMap() = aligned_input.col(j).cast<float>();
      std::vector<int> closest_idx;
      std::vector<float> distance;
      kdtree.nearestKSearch(pcl_point, 1, closest_idx, distance);
      auto closest_point = target[closest_idx.front()];
      target_corresp.col(j) = closest_point.getVector3fMap().cast<double>();
    }

    Eigen::VectorXd weights = Eigen::VectorXd::Ones(num_input);
    Eigen::Isometry3d transform = getRigidTransform(weights, aligned_input, target_corresp);
    total_transform = transform * total_transform;

    error = 0.0;
    for (int j = 0; j < num_input; j++)
    {
      aligned_input.col(j) = transform * aligned_input.col(j);
      error += (target_corresp.col(j) - aligned_input.col(j)).squaredNorm();
    }
    error /= num_input;

    std::cout << "Error: " << error << std::endl;

    if (error < threshold)
    {
      break;
    }
  }

  Eigen::Affine3d affine_transform{ total_transform };
  return { affine_transform, error };
}

Eigen::Isometry3d ICP::getRigidTransform(const Eigen::VectorXd& weights, const Eigen::Matrix3Xd& input,
                                         const Eigen::Matrix3Xd& target) const
{
  const int dims = input.cols();

  // 1: Compute weighted centroids of input and target
  const double weights_sum_inverse = 1.0 / weights.sum();
  const Eigen::Vector3d input_centroid =
      (input.array().rowwise() * weights.array().transpose()).rowwise().sum() * weights_sum_inverse;
  const Eigen::Vector3d target_centroid =
      (target.array().rowwise() * weights.array().transpose()).rowwise().sum() * weights_sum_inverse;

  // 2: Subtract centroids from input and target
  const Eigen::Matrix3Xd input_prime = input.colwise() - input_centroid;
  const Eigen::Matrix3Xd target_prime = target.colwise() - target_centroid;

  // 3: Compute S = X W Y^T
  const Eigen::MatrixXd s = target_prime * weights.asDiagonal() * input_prime.transpose();
  const Eigen::JacobiSVD<Eigen::MatrixXd> svd(s, Eigen::ComputeThinU | Eigen::ComputeThinV);
  Eigen::Matrix3d rotation_matrix = svd.matrixV() * svd.matrixU().transpose();

  if (rotation_matrix.determinant() < 0)
  {
    Eigen::MatrixXd altered_v = svd.matrixV();
    altered_v.col(dims) *= -1;
    rotation_matrix = altered_v * svd.matrixU().transpose();
  }

  std::cout << "Rotation:\n" << rotation_matrix.transpose() << std::endl;
  Eigen::Isometry3d transform{ rotation_matrix.transpose() };
  transform.translation() = target_centroid - rotation_matrix * input_centroid;

  return transform;
}
}  // namespace scanmatcher
