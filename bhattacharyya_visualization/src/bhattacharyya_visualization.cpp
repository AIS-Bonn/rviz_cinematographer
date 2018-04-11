// Visualizes the Bhattacharyya distance between two (Gaussian) distributions
// Author: Jan Razlaw <jan.razlaw@gmx.de>

#include <bhattacharyya_visualization/bhattacharyya_visualization.h>

BhattacharyyaDistanceVisualizer::BhattacharyyaDistanceVisualizer()
 : nh_("~")
 , initial_cloud_1_(new PointCloud())
 , initial_cloud_2_(new PointCloud())

 , mean_1_x_("mean_1_x", -10.0f, 0.1f, 10.0f, 0.0f)
 , mean_1_y_("mean_1_y", -10.0f, 0.1f, 10.0f, 0.0f)
 , mean_1_z_("mean_1_z", -10.0f, 0.1f, 10.0f, 0.0f)
 , mean_2_x_("mean_2_x", -10.0f, 0.1f, 10.0f, 0.0f)
 , mean_2_y_("mean_2_y", -10.0f, 0.1f, 10.0f, 0.0f)
 , mean_2_z_("mean_2_z", -10.0f, 0.1f, 10.0f, 1.0f)

 , cov_1_xx_("cov_1_xx", 0.001, 0.001, 2.0, 1.0)
 , cov_1_yy_("cov_1_yy", 0.001, 0.001, 2.0, 1.0)
 , cov_1_zz_("cov_1_zz", 0.001, 0.001, 2.0, 1.0)
 , cov_1_xy_("cov_1_xy", -2.0, 0.001, 2.0, 0.0)
 , cov_1_xz_("cov_1_xz", -2.0, 0.001, 2.0, 0.0)
 , cov_1_yz_("cov_1_yz", -2.0, 0.001, 2.0, 0.0)

 , cov_2_xx_("cov_2_xx", 0.001, 0.001, 2.0, 1.0)
 , cov_2_yy_("cov_2_yy", 0.001, 0.001, 2.0, 1.0)
 , cov_2_zz_("cov_2_zz", 0.001, 0.001, 2.0, 1.0)
 , cov_2_xy_("cov_2_xy", -2.0, 0.001, 2.0, 0.0)
 , cov_2_xz_("cov_2_xz", -2.0, 0.001, 2.0, 0.0)
 , cov_2_yz_("cov_2_yz", -2.0, 0.001, 2.0, 0.0)

 , prev_dist_(0.f)
{
	pub_pointcloud_1_ = nh_.advertise<PointCloud>("visualization", 1);
	pub_pointcloud_2_ = nh_.advertise<PointCloud>("visualization2", 1);

	mean_1_x_.setCallback(boost::bind(&BhattacharyyaDistanceVisualizer::update, this));
	mean_1_y_.setCallback(boost::bind(&BhattacharyyaDistanceVisualizer::update, this));
	mean_1_z_.setCallback(boost::bind(&BhattacharyyaDistanceVisualizer::update, this));
	mean_2_x_.setCallback(boost::bind(&BhattacharyyaDistanceVisualizer::update, this));
	mean_2_y_.setCallback(boost::bind(&BhattacharyyaDistanceVisualizer::update, this));
	mean_2_z_.setCallback(boost::bind(&BhattacharyyaDistanceVisualizer::update, this));

  cov_1_xx_.setCallback(boost::bind(&BhattacharyyaDistanceVisualizer::update, this));
  cov_1_yy_.setCallback(boost::bind(&BhattacharyyaDistanceVisualizer::update, this));
  cov_1_zz_.setCallback(boost::bind(&BhattacharyyaDistanceVisualizer::update, this));
  cov_1_xy_.setCallback(boost::bind(&BhattacharyyaDistanceVisualizer::update, this));
  cov_1_xz_.setCallback(boost::bind(&BhattacharyyaDistanceVisualizer::update, this));
  cov_1_yz_.setCallback(boost::bind(&BhattacharyyaDistanceVisualizer::update, this));

  cov_2_xx_.setCallback(boost::bind(&BhattacharyyaDistanceVisualizer::update, this));
  cov_2_yy_.setCallback(boost::bind(&BhattacharyyaDistanceVisualizer::update, this));
  cov_2_zz_.setCallback(boost::bind(&BhattacharyyaDistanceVisualizer::update, this));
  cov_2_xy_.setCallback(boost::bind(&BhattacharyyaDistanceVisualizer::update, this));
  cov_2_xz_.setCallback(boost::bind(&BhattacharyyaDistanceVisualizer::update, this));
  cov_2_yz_.setCallback(boost::bind(&BhattacharyyaDistanceVisualizer::update, this));
}

BhattacharyyaDistanceVisualizer::~BhattacharyyaDistanceVisualizer()
{
}

void BhattacharyyaDistanceVisualizer::load()
{
	srand (time(NULL));

	std::default_random_engine generator;
	std::normal_distribution<double> distribution(0.0,1.0);

	// create initial Pointcloud randomly
	for(int index = 0; index < 1000; index++)
	{
		// get random values between -0.5 and 0.5
		pcl::PointXYZ random_point;
		random_point.x = distribution(generator);
		random_point.y = distribution(generator);
		random_point.z = distribution(generator);
		initial_cloud_1_->push_back(random_point);
	}

	initial_cloud_1_->header.frame_id = "base_link";

	pcl::copyPointCloud(*initial_cloud_1_, *initial_cloud_2_);

	update();
}

void BhattacharyyaDistanceVisualizer::update()
{
  // Set up means and covariance matrices from parameters
  Eigen::Vector3f mean_1(mean_1_x_(), mean_1_y_(), mean_1_z_());
  Eigen::Matrix3f cov_1;
  cov_1 << cov_1_xx_(), cov_1_xy_(), cov_1_xz_(),
           cov_1_xy_(), cov_1_yy_(), cov_1_yz_(),
           cov_1_xz_(), cov_1_yz_(), cov_1_zz_();

  Eigen::Vector3f mean_2(mean_2_x_(), mean_2_y_(), mean_2_z_());
  Eigen::Matrix3f cov_2;
  cov_2 << cov_2_xx_(), cov_2_xy_(), cov_2_xz_(),
           cov_2_xy_(), cov_2_yy_(), cov_2_yz_(),
           cov_2_xz_(), cov_2_yz_(), cov_2_zz_();

  // Check if matrices are positive definite
  Eigen::EigenSolver<Eigen::Matrix3f> eigen_solver(cov_1);
  auto eigen_values = eigen_solver.eigenvalues();
  if(eigen_values.col(0)[0].real() <= 0 || eigen_values.col(0)[1].real() <= 0 || eigen_values.col(0)[2].real() <= 0)
    ROS_ERROR("Covariance matrix 1 is not positive definite.");

  Eigen::EigenSolver<Eigen::Matrix3f> eigen_solver_2(cov_2);
  eigen_values = eigen_solver_2.eigenvalues();
  if(eigen_values.col(0)[0].real() <= 0 || eigen_values.col(0)[1].real() <= 0 || eigen_values.col(0)[2].real() <= 0)
    ROS_ERROR("Covariance matrix 2 is not positive definite.");

	// compute distance
	float dist = bhattacharyya(mean_1, mean_2, cov_1, cov_2);

  std::string sign = (prev_dist_ < dist) ? "+" : "-";
  std::cout << "dist = " << dist << " \t" << sign << std::endl;
  prev_dist_ = dist;

  // transform point clouds wrt. covariance matrices
  // as suggested here: http://www.visiondummy.com/2014/04/geometric-interpretation-covariance-matrix/
  // the covariance matrix can be decomposed into a transformation matrix using
  // the cholesky decomposition
	PointCloud::Ptr modified_cloud_1(new PointCloud());
	PointCloud::Ptr modified_cloud_2(new PointCloud());

	Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
//	LLT<MatrixXd> lltOfA(A); // compute the Cholesky decomposition of A
//	MatrixXd L = lltOfA.matrixL(); // retrieve factor L  in the decomposition
//	// The previous two lines can also be written as "L = A.llt().matrixL()"

	Eigen::Matrix3f L = cov_1.llt().matrixL();
	transform.block<3,3>(0,0) = L;
  transform.block<3,1>(0,3) = mean_1;

	pcl::transformPointCloud(*initial_cloud_1_, *modified_cloud_1, transform);

  L = cov_2.llt().matrixL();
	transform.block<3,3>(0,0) = L;
  transform.block<3,1>(0,3) = mean_2;
  pcl::transformPointCloud(*initial_cloud_2_, *modified_cloud_2, transform);

	modified_cloud_1->header.frame_id = "base_link";
	modified_cloud_2->header.frame_id = "base_link";
	pub_pointcloud_1_.publish(modified_cloud_1);
	pub_pointcloud_2_.publish(modified_cloud_2);
}

/// See: https://en.wikipedia.org/wiki/Mahalanobis_distance
double mahalanobis(const Eigen::Vector3f& dist, const Eigen::Matrix3f& cov)
{
   return (dist.transpose()*cov.inverse()*dist).eval()(0);
}

/// See: https://en.wikipedia.org/wiki/Bhattacharyya_distance
double BhattacharyyaDistanceVisualizer::bhattacharyya(const Eigen::Vector3f& dist,
										const Eigen::Matrix3f& cov1,
										const Eigen::Matrix3f& cov2)
{
  const Eigen::Matrix3f cov = (cov1+cov2)/2;
  const double d1 = mahalanobis(dist, cov)/8;
  const double d2 = log(cov.determinant()/sqrt(cov1.determinant()*cov2.determinant()))/2;
	return d1+d2;
}

// compute entropy and plane variance
float BhattacharyyaDistanceVisualizer::bhattacharyya(const Eigen::Vector3f &mean_1,
                                                     const Eigen::Vector3f &mean_2,
                                                     const Eigen::Matrix3f &cov_1,
                                                     const Eigen::Matrix3f &cov_2)
{
   Eigen::Vector3f means_diff = mean_1 - mean_2;
   return bhattacharyya(means_diff, cov_1, cov_2);
}


int main(int argc, char** argv)
{
	ros::init(argc, argv, "bhattacharyya_dist_visualization");

	BhattacharyyaDistanceVisualizer bhattacharyya_dist_vis;

	bhattacharyya_dist_vis.load();

	ros::spin();

	return 0;
}


