#ifndef __MULTI_OBJECT_TRACKING_H__
#define __MULTI_OBJECT_TRACKING_H__

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/PoseArray.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/Marker.h>
#include <std_srvs/Empty.h>
#include <tf/transform_listener.h>
#include <object_detection/ObjectDetections.h>
// #include "multi_object_tracking/TrackReset.h"
#include <multi_object_tracking/DebugTracking.h>

#include <vnl/vnl_vector_fixed.h>
#include <vnl/vnl_det.h>
#include <vnl/vnl_vector.h>
#include <limits>

#include "multiobjecttracker_algorithm.h"
#include <multihypothesistracker.h>
#include <RosTools.h>



namespace MultiObjectTracker {

	class Tracker {
		public:
			Tracker();
			~Tracker();


			//
			// void objectDetectionDataReceived( const nimbro_perception_msgs::ObjectDetectionDataConstPtr& objectDetectionData );
			// void odometryDataReceived( const nav_msgs::OdometryConstPtr& odometry );
			// void localizationDataReceived( const geometry_msgs::PoseConstPtr& pose ) { m_globalPositionData = *pose; }
			//
			// bool resetRequestReceived( std_srvs::EmptyRequest& req, std_srvs::EmptyResponse& resp);
			// bool selectSourcesRequestReceived( nimbro_perception_msgs::SelectSourcesRequest::Request& req, nimbro_perception_msgs::SelectSourcesRequest::Response& res );

			void update();

			// void detectionCallback(const geometry_msgs::PoseArray::ConstPtr& msg );
			void objectDetectionCallback(const object_detection::ObjectDetections::ConstPtr& msg);
			void laserDetectionCallback(const geometry_msgs::PoseArray::ConstPtr& msg);
			void object_picked_callback(const object_detection::ObjectDetections::ConstPtr& msg);


			void publishHypotheses();
			void publish_hypotheses_object_msg();
			void publish_hypotheses_future_object_msg();
			void publish_hypotheses_future();
			void publish_hypothesis_covariance();
			void publish_static_hypotheses();  //publishes a vertical line indicating which hypothesis are static (non-moveable)
			void publish_dynamic_hypotheses();
			void publish_picked_objects();     //publishes only the objects that were picked, both static and dynamic hypotheses
			void publish_debug();



			double prev_time;
			double time;

		private:
			std::vector<Measurement> m_picked_static_positions;
			int m_debug_counter;

			//Params
			double m_radius_around_picked;
			double m_merge_close_hypotheses_distance;
	    double m_max_mahalanobis_distance;
			std::string m_world_frame;
			double m_born_time_threshold;   //Time after which we start publishing the hypothesis. If the hypothesis is too young it may be unrealiable and therefore it will be removed by the isSpurious
			double m_future_time;
	    // double m_spurious_time;
	    // double m_time_start_velocity_decay;
	    // double m_time_finish_velocity_decay;


			ros::Publisher m_hypothesesPublisher;
			ros::Publisher m_measurementMarkerPublisher;
			ros::Publisher m_measurementCovPublisher;
			ros::Publisher m_hypothesisCovPublisher;
			ros::Publisher m_track_linePublisher;
			ros::Publisher m_static_objectsPublisher;
			ros::Publisher m_dynamic_objectsPublisher;
			ros::Publisher m_picked_objectsPublisher;
			ros::Publisher m_hypothesis_object_msg_publisher;
			ros::Publisher m_hypothesis_future_object_msg_publisher;
			ros::Publisher m_debug_publisher;
			ros::Publisher m_hypotheses_future_publisher;

			ros::Subscriber m_object_detection_1_subscriber;
			ros::Subscriber m_object_detection_2_subscriber;
			ros::Subscriber m_object_detection_3_subscriber;
			ros::Subscriber m_laser_detection_subscriber;
			ros::ServiceServer m_tracking_reset;

			ros::Subscriber m_picked_objectsSubcriber;
			tf::TransformListener* m_transformListener;

			MultiObjectTrackerAlgorithm* m_algorithm;

			visualization_msgs::Marker full_track;

      visualization_msgs::Marker createMarker(int x=0, int y=0, int z=0, float r=0.0, float g=1.0, float b=0.0);

      void generic_detection_handler(std::vector<Measurement>& measurements);
      bool transform_to_frame(std::vector<Measurement>&  measurements, std::string target_frame);
      void publish_mesurement_markers(std::vector<Measurement>  measurements);
			void publish_mesurement_covariance(std::vector<Measurement>  measurements);

			//block the measurements at positions corresponding to objects that were already picked. We may still get measurements in the laser since the object leave behind a ground socket once they are picked.
			void block_picked_static_measurements (std::vector<Measurement>& measurements);
			void block_hypothesis_near_measurement(Measurement measurement);
			bool tracking_reset(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);

			std::vector <Measurement> object_detections2measurements (const object_detection::ObjectDetections::ConstPtr& msg);
			std::vector <Measurement> laser_detections2measurements(const geometry_msgs::PoseArray::ConstPtr& msg);


			//HACK needed to do fake picks of objects
			double program_start_time;
	    bool did_fake_pick;


	};

}



#endif
