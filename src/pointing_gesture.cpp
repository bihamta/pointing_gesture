#include <hands_3d/pointing_gesture.h>

PointingGesture::PointingGesture(ros::NodeHandle& _nh, ros::NodeHandle& _pnh) : nh(_nh), pnh(_pnh)
{
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
		exact_sync_.reset( new ExactSynchronizer(ExactSyncPolicy(queue_size), sub_depth_, sub_rgb_, sub_info_, sub_objects_) );
		exact_sync_->registerCallback(boost::bind(&PointingGesture::imageCb, this, _1, _2, _3, _4));
	}
	else
	{
		sync_.reset( new Synchronizer(SyncPolicy(queue_size), sub_depth_, sub_rgb_, sub_info_, sub_objects_ ));
		sync_->registerCallback(boost::bind(&PointingGesture::imageCb, this, _1, _2, _3, _4));
	}


	// parameter for depth_image_transport hint
	std::string depth_image_transport_param = "depth_image_transport";

	// depth image can use different transport.(e.g. compressedDepth)
	image_transport::TransportHints depth_hints("raw",ros::TransportHints(), pnh, depth_image_transport_param);

	sub_depth_.subscribe(*depth_it_, "image_rect", 5, depth_hints);

	// rgb uses normal ros transport hints.
	image_transport::TransportHints hints("raw", ros::TransportHints(), pnh);
	sub_rgb_.subscribe(*rgb_it_, "image_rect_color", 5, hints);
	sub_info_.subscribe(*rgb_nh_, "camera_info", 5);

	sub_objects_.subscribe(nh, "detections", 5);

	pub_point_cloud_ = output_nh.advertise<PointCloud>("points", 5);
	pub_point_cloud_left_hand = output_nh.advertise<PointCloud>("points_left_hand", 5);
	pub_point_cloud_right_hand = output_nh.advertise<PointCloud>("points_right_hand", 5);
	pub_point_cloud_face = output_nh.advertise<PointCloud>("points_face", 5);

	pub_pose_face = output_nh.advertise<geometry_msgs::PointStamped>("face_pose", 1);
	pub_pose_right_hand = output_nh.advertise<geometry_msgs::PointStamped>("right_hand_pose", 1);

	pub_arrow_ave = output_nh.advertise<geometry_msgs::PoseStamped>("arrow_ave", 1);
	pub_arrow_med = output_nh.advertise<geometry_msgs::PoseStamped>("arrow_med", 1);
	pub_arrow_furthest = output_nh.advertise<geometry_msgs::PoseStamped>("arrow_furthest", 1);
	pub_arrow_closest = output_nh.advertise<geometry_msgs::PoseStamped>("arrow_closest", 1);
}

void PointingGesture::imageCb(const sensor_msgs::ImageConstPtr& depth_msg,
		const sensor_msgs::ImageConstPtr& rgb_msg_in,
		const sensor_msgs::CameraInfoConstPtr& info_msg,
		const yolo2::ImageDetectionsConstPtr& detection_msg)
{
	// Check for bad inputs
	if (depth_msg->header.frame_id != rgb_msg_in->header.frame_id)
	{
		ROS_ERROR_THROTTLE(5, "Depth image frame id [%s] doesn't match RGB image frame id [%s]",
				depth_msg->header.frame_id.c_str(), rgb_msg_in->header.frame_id.c_str());
		return;
	}
	//No need to do the process if there is no subscriber
	if(pub_point_cloud_.getNumSubscribers() == 0 )
		return;

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
	geometry_msgs::PointStamped::Ptr face_ave (new geometry_msgs::PointStamped);
	face_ave->header = depth_msg->header;

	geometry_msgs::PointStamped::Ptr right_hand_ave (new geometry_msgs::PointStamped);
	right_hand_ave->header = depth_msg->header;

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
	pub_point_cloud_left_hand.publish( cloud_1h );
	pub_point_cloud_right_hand.publish( cloud_2h );
	pub_point_cloud_face.publish( cloud_f );
//	pub_pose_face.publish( face_ave );
//	pub_pose_right_hand.publish( right_hand_ave );
//	pub_arrow_ave.publish(arrow_ave);
//	pub_arrow_med.publish(arrow_med);
//	pub_arrow_furthest.publish(arrow_furthest);
//	pub_arrow_closest.publish(arrow_closest);

	if (!success)
	{
		ROS_WARN_THROTTLE(1, "Reconstruction of the bounding box failed. Will not publish the object! (Bad depth?)");
	}
}

template<typename T>
bool PointingGesture::convert(const sensor_msgs::ImageConstPtr& depth_msg,
		const sensor_msgs::ImageConstPtr& rgb_msg,
		const PointCloud::Ptr& cloud_msg,
		const PointCloud::Ptr& cloud_1h,
		const PointCloud::Ptr& cloud_2h,
		const PointCloud::Ptr& cloud_f,
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
	int _pointing_hand_i = -1;
	
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
					//          ROS_INFO("U and V %d , %d, ---->", u, v, i);

					if( _class_id == 0 && depth_image_proc::DepthTraits<T>::valid(depth)  )
					{
						if( _pointing_hand_i == -1)
						{
							_pointing_hand_i = i;
						}
						if(_pointing_hand_i == i){
							*iter_x_2h = (u - center_x) * depth * constant_x;
							*iter_y_2h = (v - center_y) * depth * constant_y;
							*iter_z_2h = depth_image_proc::DepthTraits<T>::toMeters(depth);

							++iter_z_2h;
							++iter_y_2h;
							++iter_x_2h;
						} else {
							*iter_x_1h = (u - center_x) * depth * constant_x;
							*iter_y_1h = (v - center_y) * depth * constant_y;
							*iter_z_1h = depth_image_proc::DepthTraits<T>::toMeters(depth);

							++iter_z_1h;
							++iter_y_1h;
							++iter_x_1h;
						}
					} else if ( _class_id == 1 && depth_image_proc::DepthTraits<T>::valid(depth) )
					{
						*iter_x_f = (u - center_x) * depth * constant_x;
						*iter_y_f = (v - center_y) * depth * constant_y;
						*iter_z_f = depth_image_proc::DepthTraits<T>::toMeters(depth);

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

				//Class_ID (RightHand = 2, Face = 0, LeftHand = 1)
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

	return true;
}
