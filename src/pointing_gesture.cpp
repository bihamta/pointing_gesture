#include <hands_3d/pointing_gesture.h>

#define MIN_POINTS 10
#define EPSILON 0.02
#define ROBOT_HEIGHT 1.2;

PointingGesture::PointingGesture(ros::NodeHandle& _nh, ros::NodeHandle& _pnh) : nh(_nh), pnh(_pnh)
{
	dbscan_face = new DBSCAN;
	dbscan_hand = new DBSCAN;

	imageReceived = false;
	rgb_nh_.reset(new ros::NodeHandle(nh, "rgb") );
	ros::NodeHandle depth_nh(nh, "depth_registered");
	ros::NodeHandle output_nh(nh, "3dr");

	rgb_it_.reset( new image_transport::ImageTransport(*rgb_nh_) );
	depth_it_.reset( new image_transport::ImageTransport(depth_nh) );

	// Read parameters
	int queue_size;
	pnh.param("queue_size", queue_size, 5);
	bool use_exact_sync = false;
	pnh.param("exact_sync", use_exact_sync, false);

	// Synchronize inputs. Topic subscriptions happen on demand in the connection callback.
	if (use_exact_sync)
	{
		exact_sync_.reset( new ExactSynchronizer(ExactSyncPolicy(queue_size), sub_depth_, sub_rgb_, sub_info_) );
		exact_sync_->registerCallback(boost::bind(&PointingGesture::imageOnlyCb, this, _1, _2, _3));
	}
	else
	{
		sync_.reset( new Synchronizer(SyncPolicy(queue_size), sub_depth_, sub_rgb_, sub_info_));
		sync_->registerCallback(boost::bind(&PointingGesture::imageOnlyCb, this, _1, _2, _3));
	}

	// parameter for depth_image_transport hint
	std::string depth_image_transport_param = "depth_image_transport";

	// depth image can use different transport.(e.g. compressedDepth)
	image_transport::TransportHints depth_hints("raw",ros::TransportHints(), pnh, depth_image_transport_param);

	sub_depth_.subscribe(*depth_it_, "image_rect", 1, depth_hints);

	// rgb uses normal ros transport hints.
	image_transport::TransportHints hints("raw", ros::TransportHints(), pnh);
	sub_rgb_.subscribe(*rgb_it_, "image_rect_color", 1, hints);
	sub_info_.subscribe(*rgb_nh_, "camera_info", 1);

	sub_objects_.subscribe(nh, "detections", 1);

        sub_objects_only_ = nh.subscribe<yolo2::ImageDetections> ("detections", 5, &PointingGesture::detectionCb, this);

	pub_point_cloud_ = output_nh.advertise<PointCloud>("points", 5);
	pub_point_cloud_left_hand = output_nh.advertise<PointCloud>("points_left_hand", 5);
	pub_point_cloud_right_hand = output_nh.advertise<PointCloud>("points_right_hand", 5);
	pub_point_cloud_face = output_nh.advertise<PointCloud>("points_face", 5);

	pub_face_ave_pose = output_nh.advertise<geometry_msgs::PointStamped>("face_ave_pose", 1);
	pub_hand_ave_pose = output_nh.advertise<geometry_msgs::PointStamped>("hand_ave_pose", 1);
	pub_end_point = output_nh.advertise<geometry_msgs::PointStamped>("end_point", 1);

	pub_face_med_pose = output_nh.advertise<geometry_msgs::PointStamped>("face_med_pose", 1);
	pub_hand_med_pose = output_nh.advertise<geometry_msgs::PointStamped>("hand_med_pose", 1);

	pub_face_cls_pose = output_nh.advertise<geometry_msgs::PointStamped>("face_cls_pose", 1);
	pub_hand_cls_pose = output_nh.advertise<geometry_msgs::PointStamped>("hand_cls_pose", 1);

	pub_face_ave_dbscan_pose = output_nh.advertise<geometry_msgs::PointStamped>("face_ave_dbscan_pose", 1);
	pub_hand_ave_dbscan_pose = output_nh.advertise<geometry_msgs::PointStamped>("hand_ave_dbscan_pose", 1);

  	pub_arrowMarker_ave = output_nh.advertise<visualization_msgs::Marker>("arrowMarker_ave",1);
}

void PointingGesture::imageCb(const sensor_msgs::ImageConstPtr& depth_msg,
		const sensor_msgs::ImageConstPtr& rgb_msg_in,
		const sensor_msgs::CameraInfoConstPtr& info_msg,
		const yolo2::ImageDetectionsConstPtr& detection_msg)
{
/*
	// Check for bad inputs
	if (depth_msg->header.frame_id != rgb_msg_in->header.frame_id)
	{
		ROS_ERROR_THROTTLE(5, "Depth image frame id [%s] doesn't match RGB image frame id [%s]",
				depth_msg->header.frame_id.c_str(), rgb_msg_in->header.frame_id.c_str());
		return;
	}
//	//No need to do the process if there is no subscriber
//	if(pub_point_cloud_.getNumSubscribers() == 0 )
//		return;

	// Update camera model
	model_.fromCameraInfo(info_msg);

	// Check if the input image has to be resized
	sensor_msgs::ImageConstPtr rgb_msg = rgb_msg_in;
	if (depth_msg->width != rgb_msg->width || depth_msg->height != rgb_msg->height)
	{
		sensor_msgs::CameraInfo info_msg_tmp = *info_msg;
		info_msg_tmp.width = depth_msg->width;
		info_msg_tmp.height = depth_msg->height;
		float ratio = float(depth_msg->width)/float(rgb_msg->width);
		info_msg_tmp.K[0] *= ratio;
		info_msg_tmp.K[2] *= ratio;
		info_msg_tmp.K[4] *= ratio;
		info_msg_tmp.K[5] *= ratio;
		info_msg_tmp.P[0] *= ratio;
		info_msg_tmp.P[2] *= ratio;
		info_msg_tmp.P[5] *= ratio;
		info_msg_tmp.P[6] *= ratio;
		model_.fromCameraInfo(info_msg_tmp);

		cv_bridge::CvImageConstPtr cv_ptr;
		try
		{
			cv_ptr = cv_bridge::toCvShare(rgb_msg, rgb_msg->encoding);
		}
		catch (cv_bridge::Exception& e)
		{
			ROS_ERROR("cv_bridge exception: %s", e.what());
			return;
		}
		cv_bridge::CvImage cv_rsz;
		cv_rsz.header = cv_ptr->header;
		cv_rsz.encoding = cv_ptr->encoding;
		cv::resize(cv_ptr->image.rowRange(0,depth_msg->height/ratio), cv_rsz.image, cv::Size(depth_msg->width, depth_msg->height));
		if ((rgb_msg->encoding == sensor_msgs::image_encodings::RGB8) || (rgb_msg->encoding == sensor_msgs::image_encodings::BGR8) || (rgb_msg->encoding == sensor_msgs::image_encodings::MONO8))
			rgb_msg = cv_rsz.toImageMsg();
		else
			rgb_msg = cv_bridge::toCvCopy(cv_rsz.toImageMsg(), sensor_msgs::image_encodings::RGB8)->toImageMsg();
	}
	else
		rgb_msg = rgb_msg_in;

	// Supported color encodings: RGB8, BGR8, MONO8
	int red_offset, green_offset, blue_offset, color_step;
	if (rgb_msg->encoding == sensor_msgs::image_encodings::RGB8)
	{
		red_offset   = 0;
		green_offset = 1;
		blue_offset  = 2;
		color_step   = 3;
	}
	else if (rgb_msg->encoding == sensor_msgs::image_encodings::BGR8)
	{
		red_offset   = 2;
		green_offset = 1;
		blue_offset  = 0;
		color_step   = 3;
	}
	else if (rgb_msg->encoding == sensor_msgs::image_encodings::MONO8)
	{
		red_offset   = 0;
		green_offset = 0;
		blue_offset  = 0;
		color_step   = 1;
	}
	else
	{
		try
		{
			rgb_msg = cv_bridge::toCvCopy(rgb_msg, sensor_msgs::image_encodings::RGB8)->toImageMsg();
		}
		catch (cv_bridge::Exception& e)
		{
			ROS_ERROR_THROTTLE(5, "Unsupported encoding [%s]: %s", rgb_msg->encoding.c_str(), e.what());
			return;
		}
		red_offset   = 0;
		green_offset = 1;
		blue_offset  = 2;
		color_step   = 3;
	}
	// Pose Messages
	geometry_msgs::PointStamped::Ptr face_ave_marker (new geometry_msgs::PointStamped);
	face_ave_marker->header = depth_msg->header;

	geometry_msgs::PointStamped::Ptr hand_ave_marker (new geometry_msgs::PointStamped);
	hand_ave_marker->header = depth_msg->header;

	geometry_msgs::PoseStamped::Ptr arrow_ave (new geometry_msgs::PoseStamped);
	arrow_ave->header = depth_msg->header;

	geometry_msgs::PoseStamped::Ptr arrow_med (new geometry_msgs::PoseStamped);
	arrow_med->header = depth_msg->header;

	geometry_msgs::PoseStamped::Ptr arrow_furthest (new geometry_msgs::PoseStamped);
	arrow_furthest->header = depth_msg->header;

	geometry_msgs::PoseStamped::Ptr arrow_closest (new geometry_msgs::PoseStamped);
	arrow_closest->header = depth_msg->header;

	// Allocate new point cloud message
	PointCloud::Ptr cloud_msg (new PointCloud);
	cloud_msg->header = depth_msg->header;
	cloud_msg->height = depth_msg->height;
	cloud_msg->width  = depth_msg->width;
	cloud_msg->is_dense = false;
	cloud_msg->is_bigendian = false;

	//PCL for different parts
	PointCloud::Ptr cloud_2h (new PointCloud);
	cloud_2h->header = depth_msg->header;
	cloud_2h->height = depth_msg->height;
	cloud_2h->width  = depth_msg->width;
	cloud_2h->is_dense = false;
	cloud_2h->is_bigendian = false;

	PointCloud::Ptr cloud_1h (new PointCloud);
	cloud_1h->header = depth_msg->header;
	cloud_1h->height = depth_msg->height;
	cloud_1h->width  = depth_msg->width;
	cloud_1h->is_dense = false;
	cloud_1h->is_bigendian = false;

	PointCloud::Ptr cloud_f (new PointCloud);
	cloud_f->header = depth_msg->header;
	cloud_f->height = depth_msg->height;
	cloud_f->width  = depth_msg->width;
	cloud_f->is_dense = false;
	cloud_f->is_bigendian = false;

	sensor_msgs::PointCloud2Modifier pcd_modifier_all(*cloud_msg);
	sensor_msgs::PointCloud2Modifier pcd_modifier_1h(*cloud_1h);
	sensor_msgs::PointCloud2Modifier pcd_modifier_2h(*cloud_2h);
	sensor_msgs::PointCloud2Modifier pcd_modifier_f(*cloud_f);

	pcd_modifier_all.setPointCloud2FieldsByString(2, "xyz", "rgb");
	pcd_modifier_1h.setPointCloud2FieldsByString(2, "xyz", "rgb");
	pcd_modifier_2h.setPointCloud2FieldsByString(2, "xyz", "rgb");
	pcd_modifier_f.setPointCloud2FieldsByString(2, "xyz", "rgb");

	bool success = false;
	if (depth_msg->encoding == sensor_msgs::image_encodings::TYPE_16UC1)
	{
		success = convert<uint16_t>(depth_msg, rgb_msg, cloud_msg, cloud_1h, cloud_2h, cloud_f, detection_msg, red_offset, green_offset, blue_offset, color_step);
	}
	else if (depth_msg->encoding == sensor_msgs::image_encodings::TYPE_32FC1)
	{
		success = convert<float>(depth_msg, rgb_msg, cloud_msg, cloud_1h, cloud_2h, cloud_f, detection_msg, red_offset, green_offset, blue_offset, color_step);
	}
	else
	{
		ROS_ERROR_THROTTLE(5, "Depth image has unsupported encoding [%s]", depth_msg->encoding.c_str());
		return;
	}

	pub_point_cloud_.publish( cloud_msg );
//	pub_point_cloud_left_hand.publish( cloud_1h );
//	pub_point_cloud_right_hand.publish( cloud_2h );
//	pub_point_cloud_face.publish( cloud_f );
//	pub_pose_face.publish( face_ave_marker );
//	pub_pose_right_hand.publish( hand_ave_marker );
//	pub_arrow_ave.publish(arrow_ave);
//	pub_arrow_med.publish(arrow_med);
//	pub_arrow_furthest.publish(arrow_furthest);
//	pub_arrow_closest.publish(arrow_closest);

	if (!success)
	{
		ROS_WARN_THROTTLE(1, "Reconstruction of the bounding box failed. Will not publish the object! (Bad depth?)");
	}
*/
}

void PointingGesture::imageOnlyCb(const sensor_msgs::ImageConstPtr& depth_msg,
		const sensor_msgs::ImageConstPtr& rgb_msg_in,
		const sensor_msgs::CameraInfoConstPtr& info_msg)
{
	this->depth_msg = depth_msg;
	this->rgb_msg_in = rgb_msg_in;
	this->info_msg = info_msg;
	this->imageReceived = true;
}
	
void PointingGesture::detectionCb(const yolo2::ImageDetectionsConstPtr& detection_msg)
{
	if (! imageReceived) {
		ROS_INFO("image has not been received yet");
		return;
	}

	// Check for bad inputs
	if (depth_msg->header.frame_id != rgb_msg_in->header.frame_id)
	{
		ROS_ERROR_THROTTLE(5, "Depth image frame id [%s] doesn't match RGB image frame id [%s]",
				depth_msg->header.frame_id.c_str(), rgb_msg_in->header.frame_id.c_str());
		return;
	}
	//No need to do the process if there is no subscriber
//	if(pub_point_cloud_.getNumSubscribers() == 0 )
//		return;

	// Update camera model
	model_.fromCameraInfo(info_msg);

	// Check if the input image has to be resized
	sensor_msgs::ImageConstPtr rgb_msg = rgb_msg_in;
	if (depth_msg->width != rgb_msg->width || depth_msg->height != rgb_msg->height)
	{
		sensor_msgs::CameraInfo info_msg_tmp = *info_msg;
		info_msg_tmp.width = depth_msg->width;
		info_msg_tmp.height = depth_msg->height;
		float ratio = float(depth_msg->width)/float(rgb_msg->width);
		info_msg_tmp.K[0] *= ratio;
		info_msg_tmp.K[2] *= ratio;
		info_msg_tmp.K[4] *= ratio;
		info_msg_tmp.K[5] *= ratio;
		info_msg_tmp.P[0] *= ratio;
		info_msg_tmp.P[2] *= ratio;
		info_msg_tmp.P[5] *= ratio;
		info_msg_tmp.P[6] *= ratio;
		model_.fromCameraInfo(info_msg_tmp);

		cv_bridge::CvImageConstPtr cv_ptr;
		try
		{
			cv_ptr = cv_bridge::toCvShare(rgb_msg, rgb_msg->encoding);
		}
		catch (cv_bridge::Exception& e)
		{
			ROS_ERROR("cv_bridge exception: %s", e.what());
			return;
		}
		cv_bridge::CvImage cv_rsz;
		cv_rsz.header = cv_ptr->header;
		cv_rsz.encoding = cv_ptr->encoding;
		cv::resize(cv_ptr->image.rowRange(0,depth_msg->height/ratio), cv_rsz.image, cv::Size(depth_msg->width, depth_msg->height));
		if ((rgb_msg->encoding == sensor_msgs::image_encodings::RGB8) || (rgb_msg->encoding == sensor_msgs::image_encodings::BGR8) || (rgb_msg->encoding == sensor_msgs::image_encodings::MONO8))
			rgb_msg = cv_rsz.toImageMsg();
		else
			rgb_msg = cv_bridge::toCvCopy(cv_rsz.toImageMsg(), sensor_msgs::image_encodings::RGB8)->toImageMsg();
	}
	else
		rgb_msg = rgb_msg_in;

	// Supported color encodings: RGB8, BGR8, MONO8
	int red_offset, green_offset, blue_offset, color_step;
	if (rgb_msg->encoding == sensor_msgs::image_encodings::RGB8)
	{
		red_offset   = 0;
		green_offset = 1;
		blue_offset  = 2;
		color_step   = 3;
	}
	else if (rgb_msg->encoding == sensor_msgs::image_encodings::BGR8)
	{
		red_offset   = 2;
		green_offset = 1;
		blue_offset  = 0;
		color_step   = 3;
	}
	else if (rgb_msg->encoding == sensor_msgs::image_encodings::MONO8)
	{
		red_offset   = 0;
		green_offset = 0;
		blue_offset  = 0;
		color_step   = 1;
	}
	else
	{
		try
		{
			rgb_msg = cv_bridge::toCvCopy(rgb_msg, sensor_msgs::image_encodings::RGB8)->toImageMsg();
		}
		catch (cv_bridge::Exception& e)
		{
			ROS_ERROR_THROTTLE(5, "Unsupported encoding [%s]: %s", rgb_msg->encoding.c_str(), e.what());
			return;
		}
		red_offset   = 0;
		green_offset = 1;
		blue_offset  = 2;
		color_step   = 3;
	}
	// Pose Messages
	geometry_msgs::PointStamped::Ptr face_ave_pose (new geometry_msgs::PointStamped);
	face_ave_pose->header = depth_msg->header;

	geometry_msgs::PointStamped::Ptr hand_ave_pose (new geometry_msgs::PointStamped);
	hand_ave_pose->header = depth_msg->header;

  	geometry_msgs::PointStamped::Ptr end_point (new geometry_msgs::PointStamped);
	end_point->header = depth_msg->header;

	geometry_msgs::PointStamped::Ptr face_med_pose (new geometry_msgs::PointStamped);
	face_med_pose->header = depth_msg->header;
	
	geometry_msgs::PointStamped::Ptr hand_med_pose (new geometry_msgs::PointStamped);
	hand_med_pose->header = depth_msg->header;
	
	geometry_msgs::PointStamped::Ptr face_cls_pose (new geometry_msgs::PointStamped);
	face_cls_pose->header = depth_msg->header;
	
	geometry_msgs::PointStamped::Ptr hand_cls_pose (new geometry_msgs::PointStamped);
	hand_cls_pose->header = depth_msg->header;

	geometry_msgs::PointStamped::Ptr face_ave_dbscan_pose (new geometry_msgs::PointStamped);
	face_ave_dbscan_pose->header = depth_msg->header;
	
	geometry_msgs::PointStamped::Ptr hand_ave_dbscan_pose (new geometry_msgs::PointStamped);
	hand_ave_dbscan_pose->header = depth_msg->header;
	
	// Allocate new point cloud message
	PointCloud::Ptr cloud_msg (new PointCloud);
	cloud_msg->header = depth_msg->header;
	cloud_msg->height = depth_msg->height;
	cloud_msg->width  = depth_msg->width;
	cloud_msg->is_dense = false;
	cloud_msg->is_bigendian = false;

	//PCL for different parts
	PointCloud::Ptr cloud_2h (new PointCloud);
	cloud_2h->header = depth_msg->header;
	cloud_2h->height = depth_msg->height;
	cloud_2h->width  = depth_msg->width;
	cloud_2h->is_dense = false;
	cloud_2h->is_bigendian = false;

	PointCloud::Ptr cloud_1h (new PointCloud);
	cloud_1h->header = depth_msg->header;
	cloud_1h->height = depth_msg->height;
	cloud_1h->width  = depth_msg->width;
	cloud_1h->is_dense = false;
	cloud_1h->is_bigendian = false;

	PointCloud::Ptr cloud_f (new PointCloud);
	cloud_f->header = depth_msg->header;
	cloud_f->height = depth_msg->height;
	cloud_f->width  = depth_msg->width;
	cloud_f->is_dense = false;
	cloud_f->is_bigendian = false;

	sensor_msgs::PointCloud2Modifier pcd_modifier_all(*cloud_msg);
	sensor_msgs::PointCloud2Modifier pcd_modifier_1h(*cloud_1h);
	sensor_msgs::PointCloud2Modifier pcd_modifier_2h(*cloud_2h);
	sensor_msgs::PointCloud2Modifier pcd_modifier_f(*cloud_f);

	pcd_modifier_all.setPointCloud2FieldsByString(2, "xyz", "rgb");
	pcd_modifier_1h.setPointCloud2FieldsByString(2, "xyz", "rgb");
	pcd_modifier_2h.setPointCloud2FieldsByString(2, "xyz", "rgb");
	pcd_modifier_f.setPointCloud2FieldsByString(2, "xyz", "rgb");
	
	bool success = false;
	if (depth_msg->encoding == sensor_msgs::image_encodings::TYPE_16UC1)
	{
		success = convert<uint16_t>(depth_msg, rgb_msg, cloud_msg, cloud_1h, cloud_2h, cloud_f, face_ave_pose, hand_ave_pose, end_point, face_med_pose, hand_med_pose, face_cls_pose, hand_cls_pose, face_ave_dbscan_pose, hand_ave_dbscan_pose, detection_msg, red_offset, green_offset, blue_offset, color_step);
	}
	else if (depth_msg->encoding == sensor_msgs::image_encodings::TYPE_32FC1)
	{
		success = convert<float>(depth_msg, rgb_msg, cloud_msg, cloud_1h, cloud_2h, cloud_f, face_ave_pose, hand_ave_pose, end_point, face_med_pose, hand_med_pose, face_cls_pose, hand_cls_pose, face_ave_dbscan_pose, hand_ave_dbscan_pose, detection_msg, red_offset, green_offset, blue_offset, color_step);
	}
	else
	{
		ROS_ERROR_THROTTLE(5, "Depth image has unsupported encoding [%s]", depth_msg->encoding.c_str());
		return;
	}

	if (!success)
	{
		//ROS_WARN_THROTTLE(1, "Reconstruction of the bounding box failed. Will not publish the object! (Bad depth?)");
		return;
	}

	pub_point_cloud_.publish( cloud_msg );
	pub_point_cloud_left_hand.publish( cloud_1h );
	pub_point_cloud_right_hand.publish( cloud_2h );
	pub_point_cloud_face.publish( cloud_f );

	pub_face_ave_pose.publish(face_ave_pose);
	pub_hand_ave_pose.publish(hand_ave_pose);
	pub_end_point.publish(end_point);

	pub_face_med_pose.publish(face_med_pose);
	pub_hand_med_pose.publish(hand_med_pose);

	pub_face_cls_pose.publish(face_cls_pose);
	pub_hand_cls_pose.publish(hand_cls_pose);

	pub_face_ave_dbscan_pose.publish(face_ave_dbscan_pose);
	pub_hand_ave_dbscan_pose.publish(hand_ave_dbscan_pose);
}


	template<typename T>
bool PointingGesture::convert(const sensor_msgs::ImageConstPtr& depth_msg,
		const sensor_msgs::ImageConstPtr& rgb_msg,
		const PointCloud::Ptr& cloud_msg,
		PointCloud::Ptr& cloud_1h,
		PointCloud::Ptr& cloud_2h,
		PointCloud::Ptr& cloud_f,
		const geometry_msgs::PointStamped::Ptr& face_ave_pose,
		const geometry_msgs::PointStamped::Ptr& hand_ave_pose,
		const geometry_msgs::PointStamped::Ptr& end_point, 
		const geometry_msgs::PointStamped::Ptr& face_med_pose,
		const geometry_msgs::PointStamped::Ptr& hand_med_pose,
		const geometry_msgs::PointStamped::Ptr& face_cls_pose,
		const geometry_msgs::PointStamped::Ptr& hand_cls_pose,
		const geometry_msgs::PointStamped::Ptr& face_ave_dbscan_pose,
		const geometry_msgs::PointStamped::Ptr& hand_ave_dbscan_pose,
		const yolo2::ImageDetectionsConstPtr& detection_msg,
		int red_offset, int green_offset, int blue_offset, int color_step)
{
	// Use correct principal point from calibration
	float center_x = model_.cx();
	float center_y = model_.cy();

	// Combine unit conversion (if necessary) with scaling by focal length for computing (X,Y)
	double unit_scaling = depth_image_proc::DepthTraits<T>::toMeters( T(1) );
	float constant_x = unit_scaling / model_.fx();
	float constant_y = unit_scaling / model_.fy();
	float bad_point = std::numeric_limits<float>::quiet_NaN ();

	const T* depth_row = reinterpret_cast<const T*>(&depth_msg->data[0]);
	int row_step = depth_msg->step / sizeof(T);
	const uint8_t* rgb = &rgb_msg->data[0];
	int rgb_skip = rgb_msg->step - rgb_msg->width * color_step;
	int _class_id = 0;

	double cloud_f_width = 0, cloud_f_height = 0;
	double cloud_1h_width = 0, cloud_1h_height = 0;
	double cloud_2h_width = 0, cloud_2h_height = 0;

	Point3* sum_f = new Point3();
	Point3* sum_1h = new Point3();
	Point3* sum_2h = new Point3();
	int num_face = 0, num_first_hand = 0, num_second_hand = 0;
	Point3* sum_f_all = new Point3();
	Point3* sum_1h_all = new Point3();
	int num_face_all = 0, num_hand_all = 0;	

	std::vector<Point3*> hand_all;
	std::vector<Point3*> face_all;

	std::vector<Point3*> first_hand_points;
	std::vector<Point3*> second_hand_points;
	std::vector<Point3*> face_points;

	sensor_msgs::PointCloud2Iterator<float> iter_x(*cloud_msg, "x");
	sensor_msgs::PointCloud2Iterator<float> iter_y(*cloud_msg, "y");
	sensor_msgs::PointCloud2Iterator<float> iter_z(*cloud_msg, "z");
	sensor_msgs::PointCloud2Iterator<uint8_t> iter_r(*cloud_msg, "r");
	sensor_msgs::PointCloud2Iterator<uint8_t> iter_g(*cloud_msg, "g");
	sensor_msgs::PointCloud2Iterator<uint8_t> iter_b(*cloud_msg, "b");
	sensor_msgs::PointCloud2Iterator<uint8_t> iter_a(*cloud_msg, "a");

	sensor_msgs::PointCloud2Iterator<float> iter_x_1h(*cloud_1h, "x");
	sensor_msgs::PointCloud2Iterator<float> iter_y_1h(*cloud_1h, "y");
	sensor_msgs::PointCloud2Iterator<float> iter_z_1h(*cloud_1h, "z");

	sensor_msgs::PointCloud2Iterator<float> iter_x_2h(*cloud_2h, "x");
	sensor_msgs::PointCloud2Iterator<float> iter_y_2h(*cloud_2h, "y");
	sensor_msgs::PointCloud2Iterator<float> iter_z_2h(*cloud_2h, "z");

	sensor_msgs::PointCloud2Iterator<float> iter_x_f(*cloud_f, "x");
	sensor_msgs::PointCloud2Iterator<float> iter_y_f(*cloud_f, "y");
	sensor_msgs::PointCloud2Iterator<float> iter_z_f(*cloud_f, "z");

	int all_points = 0;
	int _pointing_hand_ave_i = -1;

	for (int v = 0; v < int(cloud_msg->height); ++v, depth_row += row_step, rgb += rgb_skip)
	{
		for (int u = 0; u < int(cloud_msg->width); ++u, rgb += color_step, ++iter_x, ++iter_y, ++iter_z, ++iter_a, ++iter_r, ++iter_g, ++iter_b)
		{
			T depth = depth_row[u];
			bool in_bb = false;
			for( int i = 0; i < detection_msg.get()->detections.size(); i++)
				if ((u <= detection_msg.get()->detections[i].roi.x_offset+detection_msg.get()->detections[i].roi.width
							&& u >= detection_msg.get()->detections[i].roi.x_offset)
						&& (v <= detection_msg.get()->detections[i].roi.y_offset+detection_msg.get()->detections[i].roi.height
							&& v >= detection_msg.get()->detections[i].roi.y_offset)
				   )
				{
					in_bb = true;
					_class_id = detection_msg.get()->detections[i].class_id;

					if( _class_id == 0 && depth_image_proc::DepthTraits<T>::valid(depth)  )
					{
						if( _pointing_hand_ave_i == -1)
						{
							_pointing_hand_ave_i = i;
						}
						if(_pointing_hand_ave_i == i){
							*iter_x_1h = (u - center_x) * depth * constant_x;
							*iter_y_1h = (v - center_y) * depth * constant_y;
							*iter_z_1h = depth_image_proc::DepthTraits<T>::toMeters(depth);

							cloud_1h_width = detection_msg.get()->detections[i].roi.width;
							cloud_1h_height = detection_msg.get()->detections[i].roi.height;

							float center_x_1h = detection_msg.get()->detections[i].roi.x_offset + cloud_1h_height / 2;
							float center_y_1h = detection_msg.get()->detections[i].roi.y_offset + cloud_1h_width / 2;
							float radius_1h = (cloud_1h_width < cloud_1h_height) ? (cloud_1h_width / 2) * 0.75 : (cloud_1h_height / 2) * 0.75;
							sum_1h_all->x += *iter_x_1h;
							sum_1h_all->y += *iter_y_1h;
							sum_1h_all->z += *iter_z_1h;

							num_hand_all++;

							hand_all.push_back(new Point3(*iter_x_1h, *iter_y_1h, *iter_z_1h));

							if (fabs(u - center_x_1h) < radius_1h && fabs(v - center_y_1h) < radius_1h) {
								sum_1h->x += *iter_x_1h;
								sum_1h->y += *iter_y_1h;
								sum_1h->z += *iter_z_1h;

								num_first_hand++;

								first_hand_points.push_back(new Point3(*iter_x_1h, *iter_y_1h, *iter_z_1h));
							}

							++iter_z_1h;
							++iter_y_1h;
							++iter_x_1h;
						} else {
							*iter_x_2h = (u - center_x) * depth * constant_x;
							*iter_y_2h = (v - center_y) * depth * constant_y;
							*iter_z_2h = depth_image_proc::DepthTraits<T>::toMeters(depth);
							cloud_2h_width = detection_msg.get()->detections[i].roi.width;
							cloud_2h_height = detection_msg.get()->detections[i].roi.height;

							float center_x_2h = detection_msg.get()->detections[i].roi.x_offset + cloud_2h_height / 2;
							float center_y_2h = detection_msg.get()->detections[i].roi.y_offset + cloud_2h_width / 2;
							float radius_2h = (cloud_2h_width < cloud_2h_height) ? (cloud_2h_width / 2) * 0.75 : (cloud_2h_height / 2) * 0.75;
							if (fabs(u - center_x_2h) < radius_2h && fabs(v - center_y_2h) < radius_2h) {
								sum_2h->x += *iter_x_2h;
								sum_2h->y += *iter_y_2h;
								sum_2h->z += *iter_z_2h;

								num_second_hand++;

								second_hand_points.push_back(new Point3(*iter_x_2h, *iter_y_2h, *iter_z_2h));
							}


							++iter_z_2h;
							++iter_y_2h;
							++iter_x_2h;
						}
					} else if ( _class_id == 1 && depth_image_proc::DepthTraits<T>::valid(depth) )
					{
						*iter_x_f = (u - center_x) * depth * constant_x;
						*iter_y_f = (v - center_y) * depth * constant_y;
						*iter_z_f = depth_image_proc::DepthTraits<T>::toMeters(depth);

						cloud_f_width = detection_msg.get()->detections[i].roi.width;
						cloud_f_height = detection_msg.get()->detections[i].roi.height;

						float center_x_f = detection_msg.get()->detections[i].roi.x_offset + cloud_f_height / 2;
						float center_y_f = detection_msg.get()->detections[i].roi.y_offset + cloud_f_width / 2;
						float radius_f = (cloud_f_width < cloud_f_height) ? (cloud_f_width / 2) * 0.75 : (cloud_f_height / 2) * 0.75;
						sum_f_all->x += *iter_x_f;
						sum_f_all->y += *iter_y_f;
						sum_f_all->z += *iter_z_f;

						num_face_all++;

						face_all.push_back(new Point3(*iter_x_f, *iter_y_f, *iter_z_f));

						if (fabs(u - center_x_f) < radius_f && fabs(v - center_y_f) < radius_f) {
							sum_f->x += *iter_x_f;
							sum_f->y += *iter_y_f;
							sum_f->z += *iter_z_f;

							num_face++;

							face_points.push_back(new Point3(*iter_x_f, *iter_y_f, *iter_z_f));
						}

						++iter_z_f;
						++iter_y_f;
						++iter_x_f;
					}

					break;
				}
			if (in_bb) all_points++;
			if (!in_bb || !depth_image_proc::DepthTraits<T>::valid(depth))
			{
				*iter_x = *iter_y = *iter_z = bad_point;
			}
			else
			{
				// Fill in XYZ
				*iter_x = (u - center_x) * depth * constant_x;
				*iter_y = (v - center_y) * depth * constant_y;
				*iter_z = depth_image_proc::DepthTraits<T>::toMeters(depth);

				//Class_ID (Hands = 1, Face = 0)
				*iter_a = _class_id;
				// Fill in color
				*iter_r = rgb[red_offset];
				*iter_g = rgb[green_offset];
				*iter_b = rgb[blue_offset];
			}
		}
	}

	if(!all_points) return false;

	depth_row = reinterpret_cast<const T*>(&depth_msg->data[0]);

	double roll_closest, pitch_closest, yaw_closest;
	double roll_med, pitch_med, yaw_med;
	double roll_ave, pitch_ave, yaw_ave;

	//Mean Point
	Point3* pointing_hand_ave;

	//  Average point of FACE and HANDS 
	Point3* face_points_ave = new Point3( sum_f->x/num_face, sum_f->y/num_face, sum_f->z/num_face );
	Point3* first_hand_points_ave = new Point3( sum_1h->x/num_first_hand, sum_1h->y/num_first_hand, sum_1h->z/num_first_hand );
	Point3* second_hand_points_ave = new Point3( sum_2h->x/num_second_hand, sum_2h->y/num_second_hand, sum_2h->z/num_second_hand );
	pointing_hand_ave = first_hand_points_ave;

	// Filling poses
	face_ave_pose->point.x = face_points_ave->x;
	face_ave_pose->point.y = face_points_ave->y;
	face_ave_pose->point.z = face_points_ave->z;

	hand_ave_pose->point.x = pointing_hand_ave->x;
	hand_ave_pose->point.y = pointing_hand_ave->y;
	hand_ave_pose->point.z = pointing_hand_ave->z;

	finding_end_point( hand_ave_pose, face_ave_pose, end_point);
	/*
	////Finding Angle
	roll_ave = 0;
	pitch_ave = atan2((pointing_hand_ave->x - face_points_ave->x), (pointing_hand_ave->z - face_points_ave->z)) - PI/2;
	// yaw_ave = -atan2(sqrt(pow(pointing_hand_ave_Z - sumZ_f, 2) + pow(pointing_hand_ave_X - sumX_f, 2)), pointing_hand_ave_Y - sumY_f) + PI/2; 
	// -atan2(fabs(pointing_hand_ave_X - sumX_f), (pointing_hand_ave_Y - sumY_f)) + PI/2;
	int sign = (pointing_hand_ave->x > face_points_ave->x) ? -1 : 1;
	yaw_ave = sign * (atan2(sqrt(pow(pointing_hand_ave->z - face_points_ave->z, 2) + pow(pointing_hand_ave->x - face_points_ave->x, 2)), pointing_hand_ave->y - face_points_ave->y) - PI/2); 
	// -atan2(fabs(pointing_hand_ave_X - sumX_f), (pointing_hand_ave_Y - sumY_f)) + PI/2;

	tf::Quaternion arrow_angle_ave = tf::createQuaternionFromRPY(roll_ave, pitch_ave, yaw_ave);
	 */

	//Closest Point
	Point3* closest_point_hand = NULL;
	Point3* closest_point_face = NULL;

	//  Closest point to the camera in Pointing HAND
	if(first_hand_points.size() > 0 ) {
		closest_point_hand = first_hand_points[0];

		for (int i = 0; i < first_hand_points.size(); i++) {
			if(first_hand_points[i]->z < closest_point_hand->z) {
				closest_point_hand = first_hand_points[i];
			}
		}
	} else {
		return false;
	}

	//  Closest point to the camera in Pointing FACE
	if(face_points.size() > 0 ) {
		closest_point_face = face_points[0];

		for (int i = 0; i < face_points.size(); i++) {
			if(face_points[i]->z < closest_point_face->z) {
				closest_point_face = face_points[i];
			}
		}
	} else {
		return false;
	}
	///Filling Poses
	face_cls_pose->point.x = closest_point_face->x;
	face_cls_pose->point.y = closest_point_face->y;
	face_cls_pose->point.z = closest_point_face->z;

	hand_cls_pose->point.x = closest_point_hand->x;
	hand_cls_pose->point.y = closest_point_hand->y;
	hand_cls_pose->point.z = closest_point_hand->z;

	/*
	////Finding Angle
	if (first_hand_points.size() > 0){
	roll_closest = 0;
	pitch_closest = atan2((closest_point_hand->x - closest_point_face->x), ( closest_point_hand->z - closest_point_face->z)) - PI/2;
	yaw_closest = 0;

	tf::Quaternion arrow_angle_closest = tf::createQuaternionFromRPY(roll_closest , pitch_closest, yaw_closest);

	arrow_closest->pose.position.x = closest_point_face->x;
	arrow_closest->pose.position.y = closest_point_face->y;
	arrow_closest->pose.position.z = closest_point_face->z;
	arrow_closest->pose.orientation.x = arrow_angle_closest.getX();
	arrow_closest->pose.orientation.y = arrow_angle_closest.getY();
	arrow_closest->pose.orientation.z = arrow_angle_closest.getZ();
	arrow_closest->pose.orientation.w = arrow_angle_closest.getW();
	}
	 */

	//Median Point
	Point3* face_point_med = NULL;
	Point3* hand_point_med = NULL;
	if (first_hand_points.size() > 0) {
		hand_point_med = points_median(first_hand_points); 
	} else {
		return false;
	}
	if (face_points.size() > 0 ) {
		face_point_med = points_median(face_points);
	} else {
		return false;
	}
	//Filling Poses	
	face_med_pose->point.x = face_point_med->x;
	face_med_pose->point.y = face_point_med->y;
	face_med_pose->point.z = face_point_med->z;

	hand_med_pose->point.x = hand_point_med->x;
	hand_med_pose->point.y = hand_point_med->y;
	hand_med_pose->point.z = hand_point_med->z;
	/* 
	////Finding Angle
	roll_med = 0;
	pitch_med = atan2(hand_point_med->x - face_point_med->x, hand_point_med->z - face_point_med->z) - PI/2;
	yaw_med =0;

	tf::Quaternion arrow_angle_med = tf::createQuaternionFromRPY(roll_med, pitch_med, yaw_med);

	arrow_med->pose.position.x = face_point_med->x;
	arrow_med->pose.position.y = face_point_med->y;
	arrow_med->pose.position.z = face_point_med->z;
	arrow_med->pose.orientation.x = arrow_angle_med.getX();
	arrow_med->pose.orientation.y = arrow_angle_med.getY();
	arrow_med->pose.orientation.z = arrow_angle_med.getZ();
	arrow_med->pose.orientation.w = arrow_angle_med.getW();
*/	
	visualization_msgs::Marker::Ptr arrowMarker_ave (new visualization_msgs::Marker);
	arrowMarker_ave->header = depth_msg->header;
	arrowMarker_ave->type = visualization_msgs::Marker::ARROW;
	arrowMarker_ave->action = visualization_msgs::Marker::ADD;
	arrowMarker_ave->color.a = 1.0;
	arrowMarker_ave->scale.x = 0.01;
	arrowMarker_ave->scale.y = 0.1;
	arrowMarker_ave->scale.z = 0.1;

	arrowMarker_ave->points.resize(2);
	arrowMarker_ave->points[0].x = face_points_ave->x;
	arrowMarker_ave->points[0].y = face_points_ave->y;
	arrowMarker_ave->points[0].z = face_points_ave->z;

	arrowMarker_ave->points[1].x = pointing_hand_ave->x;
	arrowMarker_ave->points[1].y = pointing_hand_ave->y;
	arrowMarker_ave->points[1].z = pointing_hand_ave->z;

	arrowMarker_ave->colors.resize(2);
	arrowMarker_ave->colors[0].r = 255;
	arrowMarker_ave->colors[0].g = 0;
	arrowMarker_ave->colors[0].b = 0;
	arrowMarker_ave->colors[0].a = 1;

	arrowMarker_ave->colors[1].r = 1;
	arrowMarker_ave->colors[1].g = 0;
	arrowMarker_ave->colors[1].b = 0;
	arrowMarker_ave->colors[1].a = 1;

	pub_arrowMarker_ave.publish(arrowMarker_ave);
	std::cout << "here" << std::endl;
	// Clustering
	if( face_all.size() > 0 && hand_all.size() > 0){
		int cluster_max_face = 0, cluster_max_face_index = 0;
		int cluster_max_hand = 0, cluster_max_hand_index = 0;
		Point3* face_ave_dbscan = new Point3();
		Point3* hand_ave_dbscan = new Point3();
		std::vector<DBSCAN::Cluster> clusters_face = dbscan_face->cluster(face_all, EPSILON, MIN_POINTS);
		std::vector<DBSCAN::Cluster> clusters_hand = dbscan_hand->cluster(hand_all, EPSILON, MIN_POINTS);
		if (clusters_face.size() > 0 && clusters_hand.size() > 0 ){
			for (int i = 0; i < clusters_face.size(); i++) {
				if (clusters_face[i].points.size() > cluster_max_face) {
					cluster_max_face = clusters_face[i].points.size();
					cluster_max_face_index = i;
				}
			}
			for (int j = 0; j < cluster_max_face; j++) {
				face_ave_dbscan->x += clusters_face[cluster_max_face_index].points[j]->x;
				face_ave_dbscan->y += clusters_face[cluster_max_face_index].points[j]->y;
				face_ave_dbscan->z += clusters_face[cluster_max_face_index].points[j]->z;
			}
			for (int k = 0; k < clusters_hand.size(); k++) {
				if (clusters_hand[k].points.size() > cluster_max_hand) {
					cluster_max_hand = clusters_hand[k].points.size();
					cluster_max_hand_index = k;
				}
			}
			for (int l = 0; l < cluster_max_hand; l++) {
				hand_ave_dbscan->x += clusters_hand[cluster_max_hand_index].points[l]->x;
				hand_ave_dbscan->y += clusters_hand[cluster_max_hand_index].points[l]->y;
				hand_ave_dbscan->z += clusters_hand[cluster_max_hand_index].points[l]->z;
			}
			//Filling Poses	
			face_ave_dbscan_pose->point.x = face_ave_dbscan->x / cluster_max_face;
			face_ave_dbscan_pose->point.y = face_ave_dbscan->y / cluster_max_face;
			face_ave_dbscan_pose->point.z = face_ave_dbscan->z / cluster_max_face;

			hand_ave_dbscan_pose->point.x = hand_ave_dbscan->x / cluster_max_hand;
			hand_ave_dbscan_pose->point.y = hand_ave_dbscan->y / cluster_max_hand;
			hand_ave_dbscan_pose->point.z = hand_ave_dbscan->z / cluster_max_hand;
		}
	}
	return true;

}
Point3* PointingGesture::points_median(std::vector<Point3*> &v)
{
	size_t n = v.size() / 2;
	nth_element(v.begin(), v.begin()+n, v.end());
	return v[n];
}

bool PointingGesture::finding_end_point( const geometry_msgs::PointStamped::Ptr& hand_ave_pose, 
		const geometry_msgs::PointStamped::Ptr& face_ave_pose, 
		const geometry_msgs::PointStamped::Ptr& end_point) 
{
	/*
	   float pointing_yaw = 0;
	   float pointing_roll = 0;
	   float pointing_pitch = 0;
	   int x_sign = -1, z_sign = +1;

	   pointing_yaw = atan2(face_ave_pose->point.x - hand_ave_pose->point.x, face_ave_pose->point.z - hand_ave_pose->point.z);

	   pointing_pitch = PI/2 - ((atan2(sqrt(pow(face_ave_pose->point.x - hand_ave_pose->point.x, 2) + pow(face_ave_pose->point.z - hand_ave_pose->point.z, 2)), face_ave_pose->point.y - hand_ave_pose->point.y) - PI/2 ));

	   float Y = 0, X = 0, Z = 0; //Y is Height and Z is Depth ( assumption )

	   if (pointing_yaw < -(PI/2) && pointing_yaw > -PI)
	   {
	   x_sign = 1;
	   z_sign = 1;
	   }else if(pointing_yaw > PI/2 && pointing_yaw < PI){
	   x_sign = 1;
	   z_sign = 1;
	   }else if (pointing_yaw > 0){
	   x_sign = -1;
	   z_sign = -1;
	   } else {
	   x_sign = -1;
	   z_sign = -1;
	   }
	   Y = -face_ave_pose->point.y + ROBOT_HEIGHT;
	   Z = Y * tan(pointing_pitch);
	   X = x_sign * (Z * tan(pointing_yaw));
	 */
	double real_y_face = 0, real_y_hand = 0;
	
	real_y_face = -face_ave_pose->point.y + ROBOT_HEIGHT;
	real_y_hand= -hand_ave_pose->point.y + ROBOT_HEIGHT;
	double x_coeff = (face_ave_pose->point.x - hand_ave_pose->point.x);
	double y_coeff = (real_y_face - real_y_hand);
	double z_coeff = (face_ave_pose->point.z - hand_ave_pose->point.z);
	double t = face_ave_pose->point.y / y_coeff;
	end_point->point.x = face_ave_pose->point.x - x_coeff * t;
	end_point->point.y = 0;
	end_point->point.z = face_ave_pose->point.z - z_coeff * t;
	std::cout << "X: " << end_point->point.x << " Z: " << end_point->point.z << std::endl;

	return true;
	/*   tf::Quaternion q(quat.x, quat.y, quat.z, quat.w);
	     tf::Matrix3x3 m(q);
	     double roll, pitch, yaw;
	     m.getRPY(roll, pitch, yaw);
	     std::cout << "Roll: " << roll << ", Pitch: " << pitch << ", Yaw: " << yaw << std::endl;
	 */
}
