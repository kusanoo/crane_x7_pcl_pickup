#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/common/common.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <visualization_msgs/MarkerArray.h>

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;
  
pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
pcl::PointIndices::Ptr inliers (new pcl::PointIndices);

class PointcloudNode
{
private:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  ros::Subscriber sub_points_;
  std::string target_frame_;
  tf::TransformListener tf_listener_;
  ros::Publisher pub_transformed_;
  PointCloud::Ptr cloud_tranformed_;

  // 範囲指定
  pcl::PassThrough<PointT> pass_;
  PointCloud::Ptr cloud_passthrough_;
  ros::Publisher pub_passthrough_;
  
  // ダウンサンプリング
  pcl::VoxelGrid<PointT> voxel_;
  PointCloud::Ptr cloud_voxel_;
  ros::Publisher pub_voxel_;

  // 平面を取り除く
  pcl::SACSegmentation<PointT> seg_;
  pcl::ExtractIndices<PointT> ex_;
  PointCloud::Ptr cloud_seg_;
  ros::Publisher pub_ex_;

  // クラスタリング
  pcl::search::KdTree<PointT>::Ptr tree_;
  pcl::EuclideanClusterExtraction<PointT> ec_;
  ros::Publisher pub_clusters_;

  void cbPoints(const PointCloud::ConstPtr &msg)
  {
    try
    {
      std::string frame_id = msg->header.frame_id;
      PointCloud::ConstPtr cloud_src = msg;
      if (target_frame_.empty() == false)
      {
        frame_id = target_frame_;
        if (pcl_ros::transformPointCloud(
                target_frame_, *msg, *cloud_tranformed_, tf_listener_) == false)
        {
          ROS_ERROR("Failed pcl_ros::transformPointCloud. target_frame = %s",
                    target_frame_.c_str());
          return;
        }
        pub_transformed_.publish(cloud_tranformed_);
        cloud_src = cloud_tranformed_;
      }

      pass_.setInputCloud(cloud_src);
      pass_.filter(*cloud_passthrough_);
      pub_passthrough_.publish(cloud_passthrough_);

      voxel_.setInputCloud(cloud_passthrough_);
      voxel_.filter(*cloud_voxel_);
      pub_voxel_.publish(cloud_voxel_);


      int i=0, nr_points = cloud_voxel_->points.size();
      while (cloud_voxel_->points.size() > 0.3 * nr_points)
      {
        seg_.setInputCloud(cloud_voxel_);
        seg_.segment(*inliers, *coefficients);
        
        ex_.setInputCloud(cloud_voxel_);
        ex_.setIndices(inliers);

        ex_.setNegative(true);
        ex_.filter(*cloud_voxel_);
        pub_ex_.publish(cloud_voxel_);
      }

      std::vector<pcl::PointIndices> cluster_indices;
      tree_->setInputCloud(cloud_voxel_);
      ec_.setInputCloud(cloud_voxel_);
      ec_.extract(cluster_indices);
      visualization_msgs::MarkerArray marker_array;

      int target_index = -1;
      int marker_id = 0;
      size_t ok = 0;

      for (std::vector<pcl::PointIndices>::const_iterator
             it = cluster_indices.begin(),
             it_end = cluster_indices.end();
         it != it_end; ++it, ++marker_id)
      {
        Eigen::Vector4f min_pt, max_pt;
        pcl::getMinMax3D(*cloud_voxel_, *it, min_pt, max_pt);
        Eigen::Vector4f cluster_size = max_pt - min_pt;
        if (cluster_size.x() > 0 && cluster_size.y() > 0 && cluster_size.z() > 0)
        {
          // 特定のクラスタの検出  
          ROS_INFO("cluster_x: %f, cluster_y: %f, cluster_z: %f", cluster_size.x(), cluster_size.y(), cluster_size.z());
          bool is_ok = true;
          if (cluster_size.x() < 0.04 || cluster_size.x() > 0.4)
          {
            is_ok = false;
          }
          else if (cluster_size.y() < 0.04 || cluster_size.y() > 0.6)
          {
            is_ok = false;
          }
          else if (cluster_size.z() < 0.02 || cluster_size.z() > 0.5)
          {
            is_ok = false;
          }
          visualization_msgs::Marker marker =
              makeMarker(
                  frame_id, "cluster", marker_id, min_pt, max_pt, 0.0f, 1.0f, 0.0f, 0.5f);
          
          // 特定のクラスタの中で最も近いクラスタの検出
          if (is_ok)  
          {
            marker.ns = "ok_cluster";
            marker.color.r = 1.0f;
            marker.color.g = 0.0f;
            marker.color.b = 0.0f;
            marker.color.a = 0.5f;
            ok++;

            if(target_index < 0){
              target_index = marker_array.markers.size();
            }
            else{
              float d1 = ::hypot(marker_array.markers[target_index].pose.position.x,
                                 marker_array.markers[target_index].pose.position.y);
              float d2 = ::hypot(marker.pose.position.x, marker.pose.position.y);
              if(d2 < d1){
                target_index = marker_array.markers.size();
              }
            }     
          }
          marker_array.markers.push_back(marker);
            
        }
      }
      if (marker_array.markers.empty() == false)
      {
        if(target_index >= 0){
          marker_array.markers[target_index].ns = "target_cluster";
          marker_array.markers[target_index].color.r = 1.0f;
          marker_array.markers[target_index].color.g = 0.0f;
          marker_array.markers[target_index].color.b = 1.0f;
          marker_array.markers[target_index].color.a = 0.5f;
        }        
        pub_clusters_.publish(marker_array);
      }
      ROS_INFO("points (src: %zu, paththrough: %zu, voxelgrid: %zu, cluster: %zu, ok_cluster: %zu)",
               msg->size(), cloud_passthrough_->size(), cloud_voxel_->size(),
               cluster_indices.size(),ok);
    }
    catch (std::exception &e)
    {
      ROS_ERROR("%s", e.what());
    }
  }

  visualization_msgs::Marker makeMarker(
      const std::string &frame_id, const std::string &marker_ns,
      int marker_id,
      const Eigen::Vector4f &min_pt, const Eigen::Vector4f &max_pt,
      float r, float g, float b, float a) const
  {
    visualization_msgs::Marker marker;
    marker.header.frame_id = frame_id;
    marker.header.stamp = ros::Time::now();
    marker.ns = marker_ns;
    marker.id = marker_id;
    marker.type = visualization_msgs::Marker::CUBE;
    marker.action = visualization_msgs::Marker::ADD;

    marker.pose.position.x = (min_pt.x() + max_pt.x()) / 2;
    marker.pose.position.y = (min_pt.y() + max_pt.y()) / 2;
    marker.pose.position.z = (min_pt.z() + max_pt.z()) / 2;

    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    marker.scale.x = max_pt.x() - min_pt.x();
    marker.scale.y = max_pt.y() - min_pt.y();
    marker.scale.z = max_pt.z() - min_pt.z();

    marker.color.r = r;
    marker.color.g = g;
    marker.color.b = b;
    marker.color.a = a;

    marker.lifetime = ros::Duration(0.3);
    return marker;
  }

public:
  PointcloudNode()
    : nh_()
    , pnh_("~")
  {
    std::string topic_name;
    pnh_.param("target_frame", target_frame_, std::string(""));
    pnh_.param("topic_name", topic_name, std::string("/camera/depth_registered/points"));
    ROS_INFO("target_frame = '%s'", target_frame_.c_str());
    ROS_INFO("topic_name = '%s'", topic_name.c_str());
    sub_points_ = nh_.subscribe(topic_name, 5, &PointcloudNode::cbPoints, this);
    pub_transformed_ = nh_.advertise<PointCloud>("cloud_transformed", 1);
    cloud_tranformed_.reset(new PointCloud());

    //PassThrough    
    pass_.setFilterFieldName("z"); // Z軸でフィルタをかける
    pass_.setFilterLimits(0.1, 1.0); // 0.1 ~ 1.0mの間にある点群の抽出 
    cloud_passthrough_.reset(new PointCloud());
    pub_passthrough_ = nh_.advertise<PointCloud>("passthrough", 1);

    //voxel_grid
    voxel_.setLeafSize(0.01f, 0.01f, 0.01f); // 0.01m間隔でダウンサンプリング
    cloud_voxel_.reset(new PointCloud());
    pub_voxel_ = nh_.advertise<PointCloud>("voxel", 1);

    //Seg_plane
    seg_.setOptimizeCoefficients(true);
    seg_.setModelType(pcl::SACMODEL_PLANE);  
    seg_.setMethodType(pcl::SAC_RANSAC);
    seg_.setMaxIterations(1000);
    seg_.setDistanceThreshold(0.01); // 閾値
    pub_ex_ = nh_.advertise<PointCloud>("seg_plane", 1);

    //euclideancluster
    tree_.reset(new pcl::search::KdTree<PointT>());
    ec_.setClusterTolerance(0.02); // 2cm以上離れていたら別クラスタ
    ec_.setMinClusterSize(20); // クラスタを構成する点群の数の最低
    ec_.setMaxClusterSize(200); // 最高
    ec_.setSearchMethod(tree_);
    pub_clusters_ = nh_.advertise<visualization_msgs::MarkerArray>("clusters", 1);
  }
};

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "pointcloud_node");

  PointcloudNode pointcloud_test;
  ROS_INFO("Point Cloud");
  ros::spin();
}

