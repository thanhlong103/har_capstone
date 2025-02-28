/*********************************************************************
 *
 *********************************************************************/
#ifndef SOCIAL_LAYER_HPP_
#define SOCIAL_LAYER_HPP_

#include "nav2_costmap_2d/costmap_layer.hpp"
#include "nav2_costmap_2d/layer.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"
#include "rclcpp/rclcpp.hpp"

#include "std_msgs/msg/string.hpp"
//#include "people_msg/msg/people.h"
//#include "ppl_interface/msg/AddressBook.hpp"

#include "people_msgs/msg/people.hpp"
#include "people_msgs/msg/my_person.hpp"
#include "people_msgs/msg/people_group_array.hpp"
#include "people_msgs/msg/people_group.hpp"

#include "nav2_costmap_2d/costmap_2d_publisher.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

#include <boost/thread.hpp>
#include <mutex>

namespace nav2_social_costmap_plugin {

class SocialLayer : public nav2_costmap_2d::CostmapLayer {
// Add to SocialLayer class declaration in social_layer.hpp
protected:
  // Interaction parameters
  double interaction_width_;
  unsigned char interaction_cost_;
  double interaction_amplitude_;
  double interaction_length_scale_;
  double interaction_width_scale_;
  double centroid_amplitude_;
  double centroid_scale_;
  double initial_activity_scale_;
  double walking_phone_scale_;
  double sitting_scale_;
  double sitwork_scale_;
  double talking_scale_;
  double wavehi_scale_;
  double drilling_scale_;

public:
  SocialLayer() { layered_costmap_ = NULL; }

  virtual void onInitialize();
  virtual void updateBounds(double origin_x, double origin_y, double origin_yaw,
                            double *min_x, double *min_y, double *max_x,
                            double *max_y);
  virtual void updateCosts(nav2_costmap_2d::Costmap2D &master_grid, int min_i,
                           int min_j, int max_i, int max_j);

  virtual void reset() { return; }

  bool isDiscretized() { return false; }

  // virtual void onFootprintChanged();

  virtual bool isClearable() { return false; }

    
  // Helper function
  double distanceToLineSegment(double x, double y, double x1, double y1, double x2, double y2);
  double distanceToMergedRegion(double wx, double wy, const std::vector<geometry_msgs::msg::Point>& hull);
  double getYawFromQuaternion(const geometry_msgs::msg::Quaternion& quat);
  std::vector<std::vector<people_msgs::msg::MyPerson>> clusterPeople(const std::list<people_msgs::msg::MyPerson>& people,double max_group_distance);
  std::vector<geometry_msgs::msg::Point> computeConvexHull(const std::vector<people_msgs::msg::MyPerson>& group);

private:
  void get_parameters();

  double gaussian(double x, double y, double x0, double y0, double A,
                  double varx, double vary, double skew);
  double get_radius(double cutoff, double A, double var);

  void peopleCallback(const people_msgs::msg::People::SharedPtr msg);
  void groupCallback(const people_msgs::msg::PeopleGroupArray::SharedPtr msg);

  void applyCentroidGaussian(const geometry_msgs::msg::Point& centroid, nav2_costmap_2d::Costmap2D* costmap);

  void connectToCentroid(const geometry_msgs::msg::Point& person_pos, const geometry_msgs::msg::Point& centroid, nav2_costmap_2d::Costmap2D* costmap, double& scale);

  std::mutex ppl_message_mutex_;
  rclcpp::Subscription<people_msgs::msg::People>::SharedPtr ppl_sub_;
  std::mutex group_message_mutex_;
  rclcpp::Subscription<people_msgs::msg::PeopleGroupArray>::SharedPtr group_sub_;

  // std::unique_ptr<nav2_costmap_2d::Costmap2DPublisher> costmap_pub_;
  // std::shared_ptr<
  std::shared_ptr<
      rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::OccupancyGrid>>
      grid_pub_;
  // rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::OccupancyGrid>
  // grid_pub_;
  // rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr grid_pub_;

  people_msgs::msg::People people_list_;
  people_msgs::msg::PeopleGroupArray groups_list_;
  std::list<people_msgs::msg::MyPerson> transformed_people_;
  std::list<people_msgs::msg::PeopleGroup> transformed_groups_;

  bool publish_occgrid_;
  bool use_vel_factor_;
  bool use_passing_;
  double sigma_when_still_;
  double tolerance_vel_still_;
  double cutoff_, amplitude_, speed_factor_; // sigma_
  double sigma_front_height_, sigma_front_width_, sigma_rear_height_,
      sigma_rear_width_, sigma_right_height_, sigma_right_width_;
  double mark_x_, mark_y_;
};

} // namespace nav2_social_costmap_plugin

#endif // SOCIAL_LAYER_HPP_
