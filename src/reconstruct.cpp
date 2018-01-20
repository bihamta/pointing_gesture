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
#include <visualization_msgs/Marker.h>
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
//#include "nabo/nabo.h"
#define PI 3.14159265

using namespace message_filters::sync_policies;
// using namespace cv;
//using namespace Nabo;
using namespace Eigen;

static const std::string OPENCV_WINDOW = "Image window";
typedef sensor_msgs::PointCloud2 PointCloud;
struct myPoint{
 double x;
 double y;
};
std::vector<float> hands_points_right_X;
std::vector<float> hands_points_right_Y;
std::vector<float> hands_points_right_Z;
std::vector<float> hands_points_left_X;
std::vector<float> hands_points_left_Y;
std::vector<float> hands_points_left_Z;
//std::vector<float> closest;
void ave_method(const PointCloud::Ptr &cloud_ave);
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
  ros::Publisher pub_arrowMarker_ave;

  image_geometry::PinholeCameraModel model_;

public:
  Reconstruct(ros::NodeHandle &_nh);

  float points_median(std::vector<float> &v);

  void imageCb(const sensor_msgs::ImageConstPtr& depth_msg,
               const sensor_msgs::ImageConstPtr& rgb_msg,
               const sensor_msgs::CameraInfoConstPtr& info_msg,
               const yolo2::ImageDetectionsConstPtr &detection_msg);

  template<typename T>
  bool convert(const sensor_msgs::ImageConstPtr& depth_msg,
               const sensor_msgs::ImageConstPtr& rgb_msg,
               const PointCloud::Ptr& cloud_msg,
               const PointCloud::Ptr& cloud_lh,
               const PointCloud::Ptr& cloud_rh,
               const PointCloud::Ptr& cloud_f,
               const geometry_msgs::PointStamped::Ptr& face_ave,
               const geometry_msgs::PointStamped::Ptr& right_hand_ave,
               const geometry_msgs::PoseStamped::Ptr& arrow_ave,
               const geometry_msgs::PoseStamped::Ptr& arrow_med,
               const geometry_msgs::PoseStamped::Ptr& arrow_furthest,
               const geometry_msgs::PoseStamped::Ptr& arrow_closest,
               //const geometry_msgs::PointStamped::Ptr& point_med,
               //const geometry_msgs::PointStamped::Ptr& point_ave,
               const visualization_msgs::Marker::Ptr& arrowMarker_ave,
               const yolo2::ImageDetectionsConstPtr& detection_msg,
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
  pub_point_cloud_left_hand = output_nh.advertise<PointCloud>("points_left_hand", 5);
  pub_point_cloud_right_hand = output_nh.advertise<PointCloud>("points_right_hand", 5);
  pub_point_cloud_face = output_nh.advertise<PointCloud>("points_face", 5);

  pub_pose_face = output_nh.advertise<geometry_msgs::PointStamped>("face_pose", 1);
  pub_pose_right_hand = output_nh.advertise<geometry_msgs::PointStamped>("right_hand_pose", 1);

  pub_arrow_ave = output_nh.advertise<geometry_msgs::PoseStamped>("arrow_ave", 1);
  pub_arrow_med = output_nh.advertise<geometry_msgs::PoseStamped>("arrow_med", 1);
  pub_arrow_furthest = output_nh.advertise<geometry_msgs::PoseStamped>("arrow_furthest", 1);
  pub_arrow_closest = output_nh.advertise<geometry_msgs::PoseStamped>("arrow_closest", 1);
  pub_arrowMarker_ave = output_nh.advertise<visualization_msgs::Marker>("arrowMarker_ave",1);

}

float Reconstruct::points_median(std::vector<float> &v)
{
    size_t n = v.size() / 2;
    nth_element(v.begin(), v.begin()+n, v.end());
//    ROS_INFO("v: %f", v[n]);
    return v[n];
}

void Reconstruct::imageCb(const sensor_msgs::ImageConstPtr& depth_msg,
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
    float ratio = float(depth_msg->width) / float(rgb_msg->width);
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

  visualization_msgs::Marker::Ptr arrowMarker_ave (new visualization_msgs::Marker);
  arrowMarker_ave->header = depth_msg->header;
  arrowMarker_ave->type = visualization_msgs::Marker::ARROW;
  arrowMarker_ave->action = visualization_msgs::Marker::ADD;
  arrowMarker_ave->color.a = 1.0;

  // Allocate new point cloud message
  PointCloud::Ptr cloud_msg (new PointCloud);
  cloud_msg->header = depth_msg->header;
  cloud_msg->height = depth_msg->height;
  cloud_msg->width  = depth_msg->width;
  cloud_msg->is_dense = false;
  cloud_msg->is_bigendian = false;

  //PCL for different parts
  PointCloud::Ptr cloud_rh (new PointCloud);
  cloud_rh->header = depth_msg->header;
  cloud_rh->height = depth_msg->height;
  cloud_rh->width  = depth_msg->width;
  cloud_rh->is_dense = false;
  cloud_rh->is_bigendian = false;

  PointCloud::Ptr cloud_lh (new PointCloud);
  cloud_lh->header = depth_msg->header;
  cloud_lh->height = depth_msg->height;
  cloud_lh->width  = depth_msg->width;
  cloud_lh->is_dense = false;
  cloud_lh->is_bigendian = false;

  PointCloud::Ptr cloud_f (new PointCloud);
  cloud_f->header = depth_msg->header;
  cloud_f->height = depth_msg->height;
  cloud_f->width  = depth_msg->width;
  cloud_f->is_dense = false;
  cloud_f->is_bigendian = false;

  sensor_msgs::PointCloud2Modifier pcd_modifier_all(*cloud_msg);
  sensor_msgs::PointCloud2Modifier pcd_modifier_lh(*cloud_lh);
  sensor_msgs::PointCloud2Modifier pcd_modifier_rh(*cloud_rh);
  sensor_msgs::PointCloud2Modifier pcd_modifier_f(*cloud_f);

  pcd_modifier_all.setPointCloud2FieldsByString(2, "xyz", "rgb");
  pcd_modifier_lh.setPointCloud2FieldsByString(2, "xyz", "rgb");
  pcd_modifier_rh.setPointCloud2FieldsByString(2, "xyz", "rgb");
  pcd_modifier_f.setPointCloud2FieldsByString(2, "xyz", "rgb");

  bool success = false;
  if (depth_msg->encoding == sensor_msgs::image_encodings::TYPE_16UC1)
  {
    success = convert<uint16_t>(depth_msg, rgb_msg, cloud_msg, cloud_lh, cloud_rh, cloud_f, face_ave, right_hand_ave, arrow_ave, arrow_med, arrow_furthest, arrow_closest, arrowMarker_ave, detection_msg, red_offset, green_offset, blue_offset, color_step);
  }
  else if (depth_msg->encoding == sensor_msgs::image_encodings::TYPE_32FC1)
  {
    success = convert<float>(depth_msg, rgb_msg, cloud_msg, cloud_lh, cloud_rh, cloud_f, face_ave, right_hand_ave, arrow_ave, arrow_med, arrow_furthest, arrow_closest, arrowMarker_ave, detection_msg, red_offset, green_offset, blue_offset, color_step);
  }
  else
  {
    ROS_ERROR_THROTTLE(5, "Depth image has unsupported encoding [%s]", depth_msg->encoding.c_str());
    return;
  }

  pub_point_cloud_.publish( cloud_msg );
  pub_point_cloud_left_hand.publish( cloud_lh );
  pub_point_cloud_right_hand.publish( cloud_rh );
  pub_point_cloud_face.publish( cloud_f );
  pub_pose_face.publish( face_ave );
  pub_pose_right_hand.publish( right_hand_ave );
  pub_arrow_ave.publish(arrow_ave);
  pub_arrow_med.publish(arrow_med);
  pub_arrow_furthest.publish(arrow_furthest);
  pub_arrow_closest.publish(arrow_closest);
  pub_arrowMarker_ave.publish(arrowMarker_ave);
  
  
  if (!success)
  {
    ROS_WARN_THROTTLE(1, "Reconstruction of the bounding box failed. Will not publish the object! (Bad depth?)");
  }
}

template<typename T>
bool Reconstruct::convert(const sensor_msgs::ImageConstPtr& depth_msg,
                                      const sensor_msgs::ImageConstPtr& rgb_msg,
                                      const PointCloud::Ptr& cloud_msg,
                                      const PointCloud::Ptr& cloud_lh,
                                      const PointCloud::Ptr& cloud_rh,
                                      const PointCloud::Ptr& cloud_f,
                                      const geometry_msgs::PointStamped::Ptr& face_ave,
                                      const geometry_msgs::PointStamped::Ptr& right_hand_ave,
                                      const geometry_msgs::PoseStamped::Ptr& arrow_ave,
                                      const geometry_msgs::PoseStamped::Ptr& arrow_med,
                                      const geometry_msgs::PoseStamped::Ptr& arrow_furthest,
                                      const geometry_msgs::PoseStamped::Ptr& arrow_closest,
                                      const visualization_msgs::Marker::Ptr& arrowMarker_ave,
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

  sensor_msgs::PointCloud2Iterator<float> iter_x_lh(*cloud_lh, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y_lh(*cloud_lh, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z_lh(*cloud_lh, "z");

  sensor_msgs::PointCloud2Iterator<float> iter_x_rh(*cloud_rh, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y_rh(*cloud_rh, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z_rh(*cloud_rh, "z");

  sensor_msgs::PointCloud2Iterator<float> iter_x_f(*cloud_f, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y_f(*cloud_f, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z_f(*cloud_f, "z");

  int all_points = 0;
  double sumX_f = 0, sumY_f = 0, sumZ_f = 0;
  double sumX_lh = 0, sumY_lh = 0, sumZ_lh = 0;
  double sumX_rh = 0, sumY_rh = 0, sumZ_rh = 0;
  int num_face = 0, num_left_hand = 0, num_right_hand = 0;
  float furthest_x, furthest_y, furthest_z;
  int depth_height = int(depth_msg->height);
  int depth_width = int(depth_msg->width);
  double roll_closest, pitch_closest, yaw_closest;
  double roll_med, pitch_med, yaw_med;
  double roll_ave, pitch_ave, yaw_ave;
  double pointing_hand_X, pointing_hand_Y, pointing_hand_Z;
  bool pointingFlag = true;
  hands_points_right_X.clear();
  hands_points_right_Y.clear();
  hands_points_right_Z.clear();
  hands_points_left_X.clear();
  hands_points_left_Y.clear();
  hands_points_left_Z.clear();
  int v_old, u_old;
  int _pointing_hand_i = -1;
  /*
  u_old = detection_msg.get()->detections[0].roi.x_offset;
  v_old = detection_msg.get()->detections[0].roi.y_offset;*/
  for (int v = 0; v < depth_height; ++v, depth_row += row_step, rgb += rgb_skip)
  {
    for (int u = 0; u < depth_width; ++u, rgb += color_step, ++iter_x, ++iter_y, ++iter_z, ++iter_a, ++iter_r, ++iter_g, ++iter_b)
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
                if( _pointing_hand_i == -1)//detecting hands in two different boxes
                {
                    _pointing_hand_i = i;
                }
                if(_pointing_hand_i == i){
                  // x y z of points of the pointing hand
                  *iter_x_rh = (u - center_x) * depth * constant_x;
                  *iter_y_rh = (v - center_y) * depth * constant_y;
                  *iter_z_rh = depth_image_proc::DepthTraits<T>::toMeters(depth);

                  if((u - (detection_msg.get()->detections[i].roi.x_offset+detection_msg.get()->detections[i].roi.width/2) < ((detection_msg.get()->detections[i].roi.width)*0.5)/2)
                          && (v - (detection_msg.get()->detections[i].roi.y_offset+detection_msg.get()->detections[i].roi.height/2) < ((detection_msg.get()->detections[i].roi.height)*0.5)/2)
                          ) //TODO
                  {
                    sumX_rh += *iter_x_rh;
                    sumY_rh += *iter_y_rh;
                    sumZ_rh += *iter_z_rh;

                    num_right_hand++;

                    hands_points_right_X.push_back(*iter_x_rh);
                    hands_points_right_Y.push_back(*iter_y_rh);
                    hands_points_right_Z.push_back(*iter_z_rh);

                  }

                  ++iter_z_rh;
                  ++iter_y_rh;
                  ++iter_x_rh;
              } else {
                    *iter_x_lh = (u - center_x) * depth * constant_x;
                    *iter_y_lh = (v - center_y) * depth * constant_y;
                    *iter_z_lh = depth_image_proc::DepthTraits<T>::toMeters(depth);

                    if((u - (detection_msg.get()->detections[i].roi.x_offset+detection_msg.get()->detections[i].roi.width/2) < ((detection_msg.get()->detections[i].roi.width)*0.5)/2)
                            && (v - (detection_msg.get()->detections[i].roi.y_offset+detection_msg.get()->detections[i].roi.height/2) < ((detection_msg.get()->detections[i].roi.height)*0.5)/2)
                            )
                    {
                      sumX_lh += *iter_x_lh;
                      sumY_lh += *iter_y_lh;
                      sumZ_lh += *iter_z_lh;

                      num_left_hand++;

                      hands_points_left_X.push_back(*iter_x_lh);
                      hands_points_left_Y.push_back(*iter_y_lh);
                      hands_points_left_Z.push_back(*iter_z_lh);
                    }
                    ++iter_z_lh;
                    ++iter_y_lh;
                    ++iter_x_lh;
              }
          } else if ( _class_id == 1 && depth_image_proc::DepthTraits<T>::valid(depth) )
          {
              *iter_x_f = (u - center_x) * depth * constant_x;
              *iter_y_f = (v - center_y) * depth * constant_y;
              *iter_z_f = depth_image_proc::DepthTraits<T>::toMeters(depth);

              if((u - (detection_msg.get()->detections[i].roi.x_offset+detection_msg.get()->detections[i].roi.width/2) < ((detection_msg.get()->detections[i].roi.width)*0.5)/2)
                      && (v - (detection_msg.get()->detections[i].roi.y_offset+detection_msg.get()->detections[i].roi.height/2) < ((detection_msg.get()->detections[i].roi.height)*0.5)/2)
                      )
              {
                sumX_f += *iter_x_f;
                sumY_f += *iter_y_f;
                sumZ_f += *iter_z_f;

                num_face++;
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

       //Class_ID (Hands = 0, Face = 1)
        *iter_a = _class_id;
       // Fill in color
        *iter_r = rgb[red_offset];
        *iter_g = rgb[green_offset];
        *iter_b = rgb[blue_offset];
      }
    }
  }
  float min = 0;
  int index_closest = -1;
  bool check_vec = false;
  float closest_point_X = 0;
  float closest_point_Y = 0;
  bool right_is_pointing = false;
  double dX = 0, dY = 0, dZ = 0;
  if(!all_points) return false;

  depth_row = reinterpret_cast<const T*>(&depth_msg->data[0]);

//Mean of the points
  sumX_f = sumX_f/num_face;
  sumY_f = sumY_f/num_face;
  sumZ_f = sumZ_f/num_face;

  sumX_rh = sumX_rh/num_right_hand;
  sumY_rh = sumY_rh/num_right_hand;
  sumZ_rh = sumZ_rh/num_right_hand;

  sumX_lh = sumX_lh/num_left_hand;
  sumY_lh = sumY_lh/num_left_hand;
  sumZ_lh = sumZ_lh/num_left_hand;


  if (sumY_lh < sumY_rh){
    pointing_hand_X = sumX_lh;
    pointing_hand_Y = sumY_lh;
    pointing_hand_Z = sumZ_lh;

    if(hands_points_left_Z.size()){
        check_vec = true;
        min = hands_points_left_Z.at(0);
        index_closest = 0;
      for ( int j = 0; j < hands_points_left_Z.size(); j++){
            if (hands_points_left_Z.at(j) < min){
                    min = hands_points_left_Z.at(j);
                    index_closest = j;
          }
      }
    }
    if(hands_points_left_X.size())
    closest_point_X = hands_points_left_X.at(index_closest);
//    closest_point_Y = hands_points_left_Y.at(index_closest);
  }
  else{
    pointing_hand_X = sumX_rh;
    pointing_hand_Y = sumY_rh;
    pointing_hand_Z = sumZ_rh;

    right_is_pointing = true;

    if(hands_points_right_Z.size()){
        check_vec = true;
        min = hands_points_right_Z.at(0);
        index_closest = 0;
        for ( int j = 0; j < hands_points_right_Z.size(); j++){
            if (hands_points_right_Z.at(j) < min){
                min = hands_points_right_Z.at(j);
                index_closest = j;
            }
        }
    }
    if(hands_points_right_X.size())
    closest_point_X = hands_points_right_X.at(index_closest);
//    closest_point_Y = hands_points_right_Y.at(index_closest);
  }

  face_ave->point.x = sumX_f;
  face_ave->point.y = sumY_f;
  face_ave->point.z = sumZ_f;

  right_hand_ave->point.x = pointing_hand_X;
  right_hand_ave->point.y = pointing_hand_Y;
  right_hand_ave->point.z = pointing_hand_Z;


//    right_hand_ave->point.x = points_median(hands_points_right_X);
//    right_hand_ave->point.y = points_median(hands_points_right_Y);
//    right_hand_ave->point.z = points_median(hands_points_right_Z);

//  face_ave->point.x = sumX_f;
//  face_ave->point.y = sumY_f;
//  face_ave->point.z = sumZ_f;

//  right_hand_ave->point.x = closest_point_X;
//  right_hand_ave->point.y = closest_point_Y;
//  right_hand_ave->point.z = min;
  arrowMarker_ave->scale.x = 0.01;
  arrowMarker_ave->scale.y = 0.1;
  arrowMarker_ave->scale.z = 0.1;

  arrowMarker_ave->points.resize(2);
  arrowMarker_ave->points[0].x = sumX_f;
  arrowMarker_ave->points[0].y = sumY_f;
  arrowMarker_ave->points[0].z = sumZ_f;

  arrowMarker_ave->points[1].x = pointing_hand_X;
  arrowMarker_ave->points[1].y = pointing_hand_Y;
  arrowMarker_ave->points[1].z = pointing_hand_Z;
  
  arrowMarker_ave->colors.resize(2);
  arrowMarker_ave->colors[0].r = 1;
  arrowMarker_ave->colors[0].g = 0;
  arrowMarker_ave->colors[0].b = 0;
  arrowMarker_ave->colors[0].a = 1;

  arrowMarker_ave->colors[1].r = 1;
  arrowMarker_ave->colors[1].g = 0;
  arrowMarker_ave->colors[1].b = 0;
  arrowMarker_ave->colors[1].a = 1;


  float dx = -pointing_hand_Z + sumZ_f;
  float dy = pointing_hand_X - sumX_f;
  float dz = -pointing_hand_Y + sumY_f;

  roll_ave = 0;
  yaw_ave = atan2(dy, dx);
  pitch_ave = atan2(sqrt(pow(dx, 2) + pow(dy, 2)), dz) - PI/2;
  // pitch_ave = 1.0472;

  tf::Matrix3x3 obs_mat;
  obs_mat.setEulerYPR(yaw_ave, pitch_ave, roll_ave);

  tf::Quaternion arrow_angle_ave;
  obs_mat.getRotation(arrow_angle_ave);

  ROS_INFO("roll %f, pitch %f, yaw %f", roll_ave * 180/PI, pitch_ave * 180/PI, yaw_ave * 180/PI);

//  roll_ave = 0;
//  pitch_ave = atan2((pointing_hand_X - sumX_f), (pointing_hand_Z - sumZ_f)) - PI/2;
//  // yaw_ave = -atan2(sqrt(pow(pointing_hand_Z - sumZ_f, 2) + pow(pointing_hand_X - sumX_f, 2)), pointing_hand_Y - sumY_f) + PI/2; // -atan2(fabs(pointing_hand_X - sumX_f), (pointing_hand_Y - sumY_f)) + PI/2;
//  int sign = (pointing_hand_X > sumX_f) ? -1 : 1;
//  yaw_ave = sign * (atan2(sqrt(pow(pointing_hand_Z - sumZ_f, 2) + pow(pointing_hand_X - sumX_f, 2)), pointing_hand_Y - sumY_f) - PI/2); // -atan2(fabs(pointing_hand_X - sumX_f), (pointing_hand_Y - sumY_f)) + PI/2;

  // tf::Quaternion arrow_angle_ave = tf::createQuaternionFromRPY(roll_ave , pitch_ave, yaw_ave); TODO: has been removed temporary

  arrow_ave->pose.position.x = sumX_f;
  arrow_ave->pose.position.y = sumY_f;
  arrow_ave->pose.position.z = sumZ_f;
  arrow_ave->pose.orientation.x = arrow_angle_ave.getX();
  arrow_ave->pose.orientation.y = arrow_angle_ave.getY();
  arrow_ave->pose.orientation.z = arrow_angle_ave.getZ();
  arrow_ave->pose.orientation.w = arrow_angle_ave.getW();

  if (check_vec){
      roll_closest = 0;
      pitch_closest = atan2((closest_point_X - sumX_f), ( min - sumZ_f)) - PI/2;
      yaw_closest = 0;

      tf::Quaternion arrow_angle_closest = tf::createQuaternionFromRPY(roll_closest , pitch_closest, yaw_closest);

      arrow_closest->pose.position.x = sumX_f;
      arrow_closest->pose.position.y = sumY_f;
      arrow_closest->pose.position.z = sumZ_f;
      arrow_closest->pose.orientation.x = arrow_angle_closest.getX();
      arrow_closest->pose.orientation.y = arrow_angle_closest.getY();
      arrow_closest->pose.orientation.z = arrow_angle_closest.getZ();
      arrow_closest->pose.orientation.w = arrow_angle_closest.getW();
  }
  if (right_is_pointing && hands_points_right_X.size() && hands_points_right_Y.size() && hands_points_right_Z.size()){
      dX = points_median(hands_points_right_X) - sumX_f ;
      dY = points_median(hands_points_right_Y) - sumY_f ;
      dZ = points_median(hands_points_right_Z) - sumZ_f ;

      face_ave->point.x = sumX_f;
      face_ave->point.y = sumY_f;
      face_ave->point.z = sumZ_f;

//      right_hand_ave->point.x = points_median(hands_points_right_X);
//      right_hand_ave->point.y = points_median(hands_points_right_Y);
//      right_hand_ave->point.z = points_median(hands_points_right_Z);
  }else if (!right_is_pointing && hands_points_left_X.size() && hands_points_left_Y.size() && hands_points_left_Z.size())
  {
      dX = points_median(hands_points_left_X) - sumX_f ;
      dY = points_median(hands_points_left_Y) - sumY_f ;
      dZ = points_median(hands_points_left_Z) - sumZ_f ;

      face_ave->point.x = sumX_f;
      face_ave->point.y = sumY_f;
      face_ave->point.z = sumZ_f;

//      right_hand_ave->point.x = points_median(hands_points_left_X);
//      right_hand_ave->point.y = points_median(hands_points_left_Y);
//      right_hand_ave->point.z = points_median(hands_points_left_Z);
  }
  if (dX != 0)
//    ROS_INFO ("dx %f, dy %f ,dz %f", dX, dY, dZ);
  roll_med = 0;
  pitch_med = atan2(dX, dZ) - PI/2;
  yaw_med =0;

  tf::Quaternion arrow_angle_med = tf::createQuaternionFromRPY(roll_med, pitch_med, yaw_med);

  arrow_med->pose.position.x = sumX_f;
  arrow_med->pose.position.y = sumY_f;
  arrow_med->pose.position.z = sumZ_f;
  arrow_med->pose.orientation.x = arrow_angle_med.getX();
  arrow_med->pose.orientation.y = arrow_angle_med.getY();
  arrow_med->pose.orientation.z = arrow_angle_med.getZ();
  arrow_med->pose.orientation.w = arrow_angle_med.getW();


  return true;
}


int main(int argc, char* argv[])
{
    ros::init(argc, argv, "reconstruct");
    ROS_INFO_STREAM("Reconstructor Initaited");
    ros::NodeHandle nh;
    Reconstruct tr(nh);

    ros::spin();
    return 0;
}
