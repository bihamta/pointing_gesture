#pragma once

#include <boost/version.hpp>
#if ((BOOST_VERSION / 100) % 1000) >= 53
#include <boost/thread/lock_guard.hpp>
#endif

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PointStamped.h>
#include <image_geometry/pinhole_camera_model.h>
#include <depth_image_proc/depth_traits.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <stdlib.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <yolo2/ImageDetections.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl/filters/median_filter.h>
#include <tf/transform_datatypes.h>
#include <math.h>
#define PI 3.14159265

using namespace message_filters::sync_policies;
using namespace cv;
typedef sensor_msgs::PointCloud2 PointCloud;

class PointingGesture
{
	public:
		PointingGesture(ros::NodeHandle& _nh, ros::NodeHandle& _pnh);
		float points_median(std::vector<float> &v);
		void imageCb(const sensor_msgs::ImageConstPtr& depth_msg,
				const sensor_msgs::ImageConstPtr& rgb_msg,
				const sensor_msgs::CameraInfoConstPtr& info_msg,
				const yolo2::ImageDetectionsConstPtr &detection_msg);

		template<typename T>
		bool convert(const sensor_msgs::ImageConstPtr& depth_msg,
				const sensor_msgs::ImageConstPtr& rgb_msg,
				const PointCloud::Ptr& cloud_msg,
				const PointCloud::Ptr& cloud_1h,
				const PointCloud::Ptr& cloud_2h,
				const PointCloud::Ptr& cloud_f,
				const yolo2::ImageDetectionsConstPtr& detection_msg,
				int red_offset, int green_offset, int blue_offset, int color_step);


	protected:
		ros::NodeHandle nh;
		ros::NodeHandle pnh;

		ros::NodeHandlePtr rgb_nh_;
		boost::shared_ptr<image_transport::ImageTransport> rgb_it_, depth_it_;

		// Subs
		image_transport::SubscriberFilter sub_depth_, sub_rgb_;
		message_filters::Subscriber<sensor_msgs::CameraInfo> sub_info_;
		message_filters::Subscriber<yolo2::ImageDetections> sub_objects_;
		typedef ApproximateTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo, yolo2::ImageDetections> SyncPolicy;
		typedef ExactTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo, yolo2::ImageDetections> ExactSyncPolicy;
		typedef message_filters::Synchronizer<SyncPolicy> Synchronizer;
		typedef message_filters::Synchronizer<ExactSyncPolicy> ExactSynchronizer;
		boost::shared_ptr<Synchronizer> sync_;
		boost::shared_ptr<ExactSynchronizer> exact_sync_;

		// Pubs
		ros::Publisher pub_point_cloud_;
		ros::Publisher pub_point_cloud_right_hand;
		ros::Publisher pub_point_cloud_left_hand;
		ros::Publisher pub_point_cloud_face;
		ros::Publisher pub_pose_right_hand;
		ros::Publisher pub_pose_face;
		ros::Publisher pub_arrow_ave;
		ros::Publisher pub_arrow_med;
		ros::Publisher pub_arrow_furthest;
		ros::Publisher pub_arrow_closest;
		ros::Publisher pub_point_med;
		ros::Publisher pub_point_ave;

		image_geometry::PinholeCameraModel model_;
};
