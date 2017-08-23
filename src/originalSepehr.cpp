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
#include <image_geometry/pinhole_camera_model.h>
#include <depth_image_proc/depth_traits.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <stdlib.h>
#include <pointignGes/ImageDetections.h>

using namespace message_filters::sync_policies;

class Reconstruct
{
protected:
  ros::NodeHandle nh;
  ros::NodeHandle private_nh;

  ros::NodeHandlePtr rgb_nh_;
  boost::shared_ptr<image_transport::ImageTransport> rgb_it_, depth_it_;

  // Subs
  image_transport::SubscriberFilter sub_depth_, sub_rgb_;
  message_filters::Subscriber<sensor_msgs::CameraInfo> sub_info_;
  message_filters::Subscriber<pointignGes::ImageDetections> sub_objects_;
  typedef ApproximateTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo, pointignGes::ImageDetections> SyncPolicy;
  typedef ExactTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo, pointignGes::ImageDetections> ExactSyncPolicy;
  typedef message_filters::Synchronizer<SyncPolicy> Synchronizer;
  typedef message_filters::Synchronizer<ExactSyncPolicy> ExactSynchronizer;
  boost::shared_ptr<Synchronizer> sync_;
  boost::shared_ptr<ExactSynchronizer> exact_sync_;

  // Pubs
  ros::Publisher pub_point_cloud_;

  typedef sensor_msgs::PointCloud2 PointCloud;
  image_geometry::PinholeCameraModel model_;

public:

  Reconstruct(ros::NodeHandle &_nh);

  void imageCb(const sensor_msgs::ImageConstPtr& depth_msg,
               const sensor_msgs::ImageConstPtr& rgb_msg,
               const sensor_msgs::CameraInfoConstPtr& info_msg,
               const pointignGes::ImageDetectionsConstPtr &detection_msg);

  template<typename T>
  bool convert(const sensor_msgs::ImageConstPtr& depth_msg,
               const sensor_msgs::ImageConstPtr& rgb_msg,
               const PointCloud::Ptr& cloud_msg,
               const pointignGes::ImageDetectionsConstPtr& detection_msg,
               int red_offset, int green_offset, int blue_offset, int color_step);

};

Reconstruct::Reconstruct(ros::NodeHandle& _nh)
    : nh(_nh),
      private_nh("~")
{
  rgb_nh_.reset( new ros::NodeHandle(nh, "rgb") );
  ros::NodeHandle depth_nh(nh, "depth_registered");
  ros::NodeHandle output_nh(nh, "3dr");

  rgb_it_.reset( new image_transport::ImageTransport(*rgb_nh_) );
  depth_it_.reset( new image_transport::ImageTransport(depth_nh) );

  // Read parameters
  int queue_size;
  private_nh.param("queue_size", queue_size, 5);
  bool use_exact_sync = false;
  private_nh.param("exact_sync", use_exact_sync, false);

  // Synchronize inputs. Topic subscriptions happen on demand in the connection callback.
  if (use_exact_sync)
  {
    exact_sync_.reset( new ExactSynchronizer(ExactSyncPolicy(queue_size), sub_depth_, sub_rgb_, sub_info_, sub_objects_) );
    exact_sync_->registerCallback(boost::bind(&Reconstruct::imageCb, this, _1, _2, _3, _4));
  }
  else
  {
    sync_.reset( new Synchronizer(SyncPolicy(queue_size), sub_depth_, sub_rgb_, sub_info_, sub_objects_ ));
    sync_->registerCallback(boost::bind(&Reconstruct::imageCb, this, _1, _2, _3, _4));
  }


  // parameter for depth_image_transport hint
  std::string depth_image_transport_param = "depth_image_transport";

  // depth image can use different transport.(e.g. compressedDepth)
  image_transport::TransportHints depth_hints("raw",ros::TransportHints(), private_nh, depth_image_transport_param);

  sub_depth_.subscribe(*depth_it_, "image_rect", 5, depth_hints);

  // rgb uses normal ros transport hints.
  image_transport::TransportHints hints("raw", ros::TransportHints(), private_nh);
  sub_rgb_.subscribe(*rgb_it_, "image_rect_color", 5, hints);
  sub_info_.subscribe(*rgb_nh_, "camera_info", 5);

  sub_objects_.subscribe(nh, "detections", 5);

  pub_point_cloud_ = output_nh.advertise<PointCloud>("points", 5);
}

void Reconstruct::imageCb(const sensor_msgs::ImageConstPtr& depth_msg,
                                      const sensor_msgs::ImageConstPtr& rgb_msg_in,
                                      const sensor_msgs::CameraInfoConstPtr& info_msg,
                                      const pointignGes::ImageDetectionsConstPtr& detection_msg)
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

  // Allocate new point cloud message
  PointCloud::Ptr cloud_msg (new PointCloud);
  cloud_msg->header = depth_msg->header;
  cloud_msg->height = depth_msg->height;
  cloud_msg->width  = depth_msg->width;
  cloud_msg->is_dense = false;
  cloud_msg->is_bigendian = false;

  sensor_msgs::PointCloud2Modifier pcd_modifier(*cloud_msg);
  pcd_modifier.setPointCloud2FieldsByString(2, "xyz", "rgb");

  bool success = false;
  if (depth_msg->encoding == sensor_msgs::image_encodings::TYPE_16UC1)
  {
    success = convert<uint16_t>(depth_msg, rgb_msg, cloud_msg, detection_msg, red_offset, green_offset, blue_offset, color_step);
  }
  else if (depth_msg->encoding == sensor_msgs::image_encodings::TYPE_32FC1)
  {
    success = convert<float>(depth_msg, rgb_msg, cloud_msg, detection_msg, red_offset, green_offset, blue_offset, color_step);
  }
  else
  {
    ROS_ERROR_THROTTLE(5, "Depth image has unsupported encoding [%s]", depth_msg->encoding.c_str());
    return;
  }

  pub_point_cloud_.publish( cloud_msg);
  if (!success)
  {
    ROS_WARN_THROTTLE(1, "Reconstruction of the bounding box failed. Will not publish the object! (Bad depth?)");
  }
}

template<typename T>
bool Reconstruct::convert(const sensor_msgs::ImageConstPtr& depth_msg,
                                      const sensor_msgs::ImageConstPtr& rgb_msg,
                                      const PointCloud::Ptr& cloud_msg,
                                      const pointignGes::ImageDetectionsConstPtr& detection_msg,
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

  sensor_msgs::PointCloud2Iterator<float> iter_x(*cloud_msg, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y(*cloud_msg, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z(*cloud_msg, "z");
  sensor_msgs::PointCloud2Iterator<uint8_t> iter_r(*cloud_msg, "r");
  sensor_msgs::PointCloud2Iterator<uint8_t> iter_g(*cloud_msg, "g");
  sensor_msgs::PointCloud2Iterator<uint8_t> iter_b(*cloud_msg, "b");
  sensor_msgs::PointCloud2Iterator<uint8_t> iter_a(*cloud_msg, "a");

  int all_points = 0;

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
                            && v >= detection_msg.get()->detections[i].roi.y_offset))
        {
	  ROS_INFO("hey");
          in_bb = true;
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
      }
      // Fill in color
      *iter_a = 255;
      *iter_r = rgb[red_offset];
      *iter_g = rgb[green_offset];
      *iter_b = rgb[blue_offset];
    }
  }

  // When bb.size = 0 or pc.size = 0 return
  if(!all_points) return false;

  sensor_msgs::PointCloud2Iterator<float> iter_x1(*cloud_msg, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y1(*cloud_msg, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z1(*cloud_msg, "z");
  depth_row = reinterpret_cast<const T*>(&depth_msg->data[0]);

  return true;
}

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "reconstruct");
    ROS_INFO_STREAM("Reconstructor Initaited");
    ros::NodeHandle nh;
    ROS_INFO("hello");
    Reconstruct tr(nh);
    ros::spin();
    return 0;
}

