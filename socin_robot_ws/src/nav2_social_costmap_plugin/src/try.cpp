#include "nav2_social_costmap_plugin/social_layer.hpp"

#include <algorithm>
#include <list>
#include <memory>
#include <string>
#include <vector>

// START INCLUDES from socialLayer
#include "std_msgs/msg/string.hpp"
#include <angles/angles.h>
#include <math.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
// END INCLUDES from socialLayer

#include "nav2_costmap_2d/costmap_math.hpp"
#include "nav2_costmap_2d/footprint.hpp"
#include "rclcpp/parameter_events_filter.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"

namespace nav2_social_costmap_plugin {

// This method is called at the end of plugin initialization.
// It contains ROS parameter(s) declaration and subscription to topics
void SocialLayer::onInitialize() {
  
  current_ = true;
  // START Subscription to topic
  {
    auto node_shared_ptr = node_;
    ppl_sub_ = node_shared_ptr->create_subscription<people_msgs::msg::People>(
        "/people", rclcpp::SensorDataQoS(),
        std::bind(&SocialLayer::peopleCallback, this, std::placeholders::_1));
  }

  rclcpp::Logger logger_ = node_->get_logger();

  RCLCPP_INFO(logger_,
              "SocialLayer: subscribed to "
              "topic %s",
              ppl_sub_->get_topic_name());

  // Whether to apply this plugin or not
  declareParameter("enabled", rclcpp::ParameterValue(true));
  // Smallest value to publish on costmap adjustments - not modify
  declareParameter("cutoff", rclcpp::ParameterValue(5.0));
  // Amplitude of adjustments at peak [0,254] or [0,100], we can keep it fixed
  declareParameter("amplitude", rclcpp::ParameterValue(255.0));
  // Covariance of adjustments [0, 1]
  declareParameter("covariance_front_height", rclcpp::ParameterValue(0.25));
  declareParameter("covariance_front_width", rclcpp::ParameterValue(0.25));
  declareParameter("covariance_rear_height", rclcpp::ParameterValue(0.25));
  declareParameter("covariance_rear_width", rclcpp::ParameterValue(0.25));
  declareParameter("covariance_right_height", rclcpp::ParameterValue(0.25));
  declareParameter("covariance_right_width", rclcpp::ParameterValue(0.25));
  declareParameter("covariance_when_still", rclcpp::ParameterValue(0.25));
  declareParameter("use_passing", rclcpp::ParameterValue(true));
  declareParameter("use_vel_factor", rclcpp::ParameterValue(true));
  // Factor with which to scale the velocity [1-10]
  declareParameter("speed_factor_multiplier", rclcpp::ParameterValue(5.0));
  declareParameter("publish_occgrid", rclcpp::ParameterValue(false));
  // In onInitialize() add new parameters
  declareParameter("max_interaction_distance", rclcpp::ParameterValue(10.0));
  declareParameter("interaction_width", rclcpp::ParameterValue(1.0));
  declareParameter("interaction_cost", rclcpp::ParameterValue(160));
  declareParameter("interaction_amplitude", rclcpp::ParameterValue(254.0));
  declareParameter("interaction_length_scale", rclcpp::ParameterValue(1.0));
  declareParameter("interaction_width_scale", rclcpp::ParameterValue(0.3));
  get_parameters();

  tolerance_vel_still_ = 0.1;

  if (publish_occgrid_) {
    auto node_shared_ptr = node_;
    grid_pub_ =
        node_shared_ptr->create_publisher<nav_msgs::msg::OccupancyGrid>("social_grid", 1);
    grid_pub_->on_activate();
  }
}

void SocialLayer::get_parameters() {
  auto node_shared_ptr = node_;
  node_shared_ptr->get_parameter(name_ + "." + "enabled", enabled_);
  node_shared_ptr->get_parameter(name_ + "." + "cutoff", cutoff_);
  node_shared_ptr->get_parameter(name_ + "." + "amplitude", amplitude_);
  node_shared_ptr->get_parameter(name_ + "." + "covariance_front_height",
                       sigma_front_height_);
  node_shared_ptr->get_parameter(name_ + "." + "covariance_front_width",
                       sigma_front_width_);
  node_shared_ptr->get_parameter(name_ + "." + "covariance_rear_height",
                       sigma_rear_height_);
  node_shared_ptr->get_parameter(name_ + "." + "covariance_rear_width",
                       sigma_rear_width_);
  node_shared_ptr->get_parameter(name_ + "." + "covariance_right_height",
                       sigma_right_height_);
  node_shared_ptr->get_parameter(name_ + "." + "covariance_right_width",
                       sigma_right_width_);
  node_shared_ptr->get_parameter(name_ + "." + "covariance_when_still",
                       sigma_when_still_);
  node_shared_ptr->get_parameter(name_ + "." + "use_passing", use_passing_);
  node_shared_ptr->get_parameter(name_ + "." + "use_vel_factor", use_vel_factor_);
  node_shared_ptr->get_parameter(name_ + "." + "speed_factor_multiplier", speed_factor_);
  node_shared_ptr->get_parameter(name_ + "." + "publish_occgrid", publish_occgrid_);
  // In get_parameters() add
  node_shared_ptr->get_parameter(name_ + "." + "max_interaction_distance", max_interaction_distance_);
  node_shared_ptr->get_parameter(name_ + "." + "interaction_width", interaction_width_);
  node_shared_ptr->get_parameter(name_ + "." + "interaction_cost", interaction_cost_);
  node_shared_ptr->get_parameter(name_ + "." + "interaction_amplitude", interaction_amplitude_);
  node_shared_ptr->get_parameter(name_ + "." + "interaction_length_scale", interaction_length_scale_);
  node_shared_ptr->get_parameter(name_ + "." + "interaction_width_scale", interaction_width_scale_);
}

void SocialLayer::peopleCallback(
    const people_msgs::msg::People::SharedPtr msg) {
  ppl_message_mutex_.lock();
  people_list_ = *msg;
  ppl_message_mutex_.unlock();
}

void SocialLayer::updateBounds(double origin_x, double origin_y,
                               double origin_z, double *min_x, double *min_y,
                               double *max_x, double *max_y) {

  std::string global_frame =
      layered_costmap_
          ->getGlobalFrameID(); // Returns the global frame of the costmap
  transformed_people_.clear();  // empty the array
  int cnt_j = 0;

  rclcpp::Logger logger_ = node_->get_logger();

  for (unsigned int i = 0; i < people_list_.people.size(); i++) {
    people_msgs::msg::Person &person = people_list_.people[i];
    people_msgs::msg::Person tpt;
    geometry_msgs::msg::PointStamped pt, opt;

    pt.point.x = person.position.x;
    pt.point.y = person.position.y;
    pt.point.z = person.position.z;
    pt.header.frame_id = people_list_.header.frame_id;
    pt.header.stamp = people_list_.header.stamp;

    if (!tf_->canTransform(pt.header.frame_id, global_frame,
                           tf2_ros::fromMsg(pt.header.stamp))) {
      RCLCPP_INFO(logger_,
                  "Social layer can't transform from %s to %s",
                  pt.header.frame_id.c_str(), global_frame.c_str());
      return;
    }

    // In general: tf_->transform(in_pose, out_pose, global_frame_,
    // transform_tolerance_);
    tf_->transform(pt, opt, global_frame);
    tpt.position.x = opt.point.x;
    tpt.position.y = opt.point.y;
    tpt.position.z = opt.point.z;

    pt.point.x += person.velocity.x;
    pt.point.y += person.velocity.y;
    pt.point.z += person.velocity.z;
    tf_->transform(pt, opt, global_frame);

    tpt.velocity.x = opt.point.x - tpt.position.x;
    tpt.velocity.y = opt.point.y - tpt.position.y;
    tpt.velocity.z = opt.point.z - tpt.position.z;

    cnt_j++;
    transformed_people_.push_back(
        tpt); // Adds a new element (tpt) at the end of the vector
  }

  std::list<people_msgs::msg::Person>::iterator p_it;

  for (p_it = transformed_people_.begin(); p_it != transformed_people_.end();
       ++p_it) {
    people_msgs::msg::Person person = *p_it;

    double mag = sqrt(pow(person.velocity.x, 2) + pow(person.velocity.y, 2));
    double greater = get_radius(cutoff_, amplitude_, sigma_when_still_);
    if (mag >= tolerance_vel_still_) {
      double front_height =
          get_radius(cutoff_, amplitude_, sigma_front_height_);
      if (use_vel_factor_) {
        double factor = 1.0 + mag * speed_factor_;
        front_height =
            get_radius(cutoff_, amplitude_, sigma_front_height_ * factor);
      }
      double rear_height = get_radius(cutoff_, amplitude_, sigma_rear_height_);

      double front_width = get_radius(cutoff_, amplitude_, sigma_front_width_);
      double rear_width = get_radius(cutoff_, amplitude_, sigma_rear_width_);
      double right_height = 0.0;
      if (use_passing_)
        right_height = get_radius(cutoff_, amplitude_, sigma_right_height_);

      greater = std::max(
          front_height,
          std::max(rear_height,
                   std::max(right_height, std::max(front_width, rear_width))));
    }

    *min_x = std::min(*min_x, person.position.x - greater);
    *min_y = std::min(*min_y, person.position.y - greater);
    *max_x = std::max(*max_x, person.position.x + greater);
    *max_y = std::max(*max_y, person.position.y + greater);
  }
}

// Function to cluster people into groups using distance threshold
std::vector<std::vector<people_msgs::msg::Person>> clusterPeople(
  const std::list<people_msgs::msg::Person>& people,
  double max_group_distance) {
  std::vector<std::vector<people_msgs::msg::Person>> groups;
  std::unordered_set<int> processed;

  int idx = 0;
  for (const auto& person : people) {
    if (processed.find(idx) == processed.end()) {
      std::vector<people_msgs::msg::Person> group;
      std::queue<int> to_process;
      to_process.push(idx);
      processed.insert(idx);

      while (!to_process.empty()) {
        int current = to_process.front();
        to_process.pop();
        group.push_back(*std::next(people.begin(), current));

        int inner_idx = 0;
        for (const auto& other : people) {
          if (processed.find(inner_idx) == processed.end()) {
            double dx = group.back().position.x - other.position.x;
            double dy = group.back().position.y - other.position.y;
            double dist = sqrt(dx * dx + dy * dy);
            if (dist <= max_group_distance) {
              to_process.push(inner_idx);
              processed.insert(inner_idx);
            }
          }
          inner_idx++;
        }
      }
      groups.push_back(group);
    }
    idx++;
  }
  return groups;
}

// Function to compute convex hull of a group
std::vector<geometry_msgs::msg::Point> computeConvexHull(
  const std::vector<people_msgs::msg::Person>& group) {
  std::vector<geometry_msgs::msg::Point> points;
  for (const auto& person : group) {
    geometry_msgs::msg::Point pt;
    pt.x = person.position.x;
    pt.y = person.position.y;
    points.push_back(pt);
  }
  // Implement convex hull algorithm here (e.g., Andrew's monotone chain)
  // Return the convex hull polygon points
  return points;
}

void SocialLayer::updateCosts(nav2_costmap_2d::Costmap2D &master_grid,
                              int min_i, int min_j, int max_i, int max_j) {
  if (!enabled_) {
    return;
  }
  if (people_list_.people.size() == 0)
    return;

  if (cutoff_ >= amplitude_)
    return;

  get_parameters();

  std::list<people_msgs::msg::Person>::iterator p_it;
  nav2_costmap_2d::Costmap2D *costmap = layered_costmap_->getCostmap();
  double res = costmap->getResolution();

  nav_msgs::msg::OccupancyGrid grid;
  grid.header.stamp = node_->get_clock()->now();
  grid.header.frame_id = layered_costmap_->getGlobalFrameID();
  // grid.info.map_load_time
  grid.info.height = costmap->getSizeInCellsY();
  grid.info.width = costmap->getSizeInCellsX();
  grid.info.resolution = res;
  grid.info.origin.position.x = costmap->getOriginX();
  grid.info.origin.position.y = costmap->getOriginY();
  grid.info.origin.orientation.w = 1.0;
  std::vector<int8_t> vect((grid.info.height * grid.info.width), 0);
  grid.data = vect;

  for (p_it = transformed_people_.begin(); p_it != transformed_people_.end();
       ++p_it) {
    people_msgs::msg::Person person = *p_it;
    double mag = sqrt(person.velocity.x * person.velocity.x +
                      person.velocity.y * person.velocity.y);
    double angle = atan2(person.velocity.y, person.velocity.x);
    double angle_right = angle - 1.57; // 1.51;
    double radius = get_radius(cutoff_, amplitude_, sigma_when_still_);
    double front_height = radius;
    double rear_height = radius;
    double greater_side = radius + radius;
    if (mag >= tolerance_vel_still_) {
      if (use_vel_factor_) {
        double factor = 1.0 + mag * speed_factor_;
        front_height =
            get_radius(cutoff_, amplitude_, sigma_front_height_ * factor);
      } else
        front_height = get_radius(cutoff_, amplitude_, sigma_front_height_);

      rear_height = get_radius(cutoff_, amplitude_, sigma_rear_height_);

      double front_width = get_radius(cutoff_, amplitude_, sigma_front_width_);
      double rear_width = get_radius(cutoff_, amplitude_, sigma_rear_width_);
      double right_height = 0.0;
      if (use_passing_)
        right_height = get_radius(cutoff_, amplitude_, sigma_right_height_);

      double height_diameter = std::max(front_height, rear_height) * 2.0;
      double width_diameter =
          (std::max(right_height, std::max(front_width, rear_width))) * 2.0;
      greater_side = std::max(height_diameter, width_diameter);
    }

    unsigned int width_cells =
        std::max(1, static_cast<int>(greater_side / res));
    unsigned int height_cells =
        std::max(1, static_cast<int>(greater_side / res));

    double cx = person.position.x, cy = person.position.y;

    double ox, oy;
    if (sin(angle) > 0)
      oy = cy - rear_height;
    else
      oy = cy + (front_height - rear_height) * sin(angle) - rear_height;

    if (cos(angle) >= 0)
      ox = cx - rear_height;
    else
      ox = cx + (front_height - rear_height) * cos(angle) - rear_height;

    int dx, dy;
    // Convert from world coordinates to map coordinates without checking for
    // legal bounds world x, world y, map x, map y
    costmap->worldToMapNoBounds(ox, oy, dx, dy);

    int start_x = 0, start_y = 0, end_x = width_cells, end_y = height_cells;
    if (dx < 0)
      start_x = -dx;
    else if (dx + width_cells > costmap->getSizeInCellsX())
      end_x = std::max(0, static_cast<int>(costmap->getSizeInCellsX()) - dx);

    if (static_cast<int>(start_x + dx) < min_i)
      start_x = min_i - dx;
    if (static_cast<int>(end_x + dx) > max_i)
      end_x = max_i - dx;

    if (dy < 0)
      start_y = -dy;
    else if (dy + height_cells > costmap->getSizeInCellsY())
      end_y = std::max(0, static_cast<int>(costmap->getSizeInCellsY()) - dy);

    if (static_cast<int>(start_y + dy) < min_j)
      start_y = min_j - dy;
    if (static_cast<int>(end_y + dy) > max_j)
      end_y = max_j - dy;

    double bx = ox + res / 2, by = oy + res / 2;
    for (int i = start_x; i < end_x; i++) {
      for (int j = start_y; j < end_y; j++) {
        unsigned char old_cost = costmap->getCost(i + dx, j + dy);
        if (old_cost == nav2_costmap_2d::NO_INFORMATION)
          continue;

        double a;
        double a_right = 0.0;
        double x = bx + i * res;
        double y = by + j * res;
        if (mag < tolerance_vel_still_) {
          // PERSON STANDS STILL
          a = gaussian(x, y, cx, cy, amplitude_, sigma_when_still_,
                       sigma_when_still_, 0);
        } else {

          double ma = atan2(y - cy, x - cx);
          double diff = angles::shortest_angular_distance(angle, ma);
          // RIGHT SIDE
          if (use_passing_) {
            double diff_right =
                angles::shortest_angular_distance(angle_right, ma);
            if (fabs(diff_right) < M_PI / 2) {
              a_right = gaussian(x, y, cx, cy, amplitude_, sigma_right_height_,
                                 sigma_right_width_, angle_right);
            }
          }
          // FRONT
          if (fabs(diff) < M_PI / 2) {
            if (use_vel_factor_) {
              double factor = 1.0 + mag * speed_factor_;
              a = gaussian(x, y, cx, cy, amplitude_,
                           sigma_front_height_ * factor, sigma_front_width_,
                           angle);
            } else
              a = gaussian(x, y, cx, cy, amplitude_, sigma_front_height_,
                           sigma_front_width_, angle);
          } else // REAR
            a = gaussian(x, y, cx, cy, amplitude_, sigma_rear_height_,
                         sigma_rear_width_,
                         angle); // 0

          a = std::max(a, a_right);
        }
        if (a < cutoff_)
          continue;

        unsigned char cvalue = (unsigned char)a;
        costmap->setCost(i + dx, j + dy, std::max(cvalue, old_cost));
        unsigned int index = costmap->getIndex(i + dx, j + dy);
        grid.data[index] = (unsigned int)a;
      }
    }
  }
  // In updateCosts():
  if (!transformed_people_.empty()) {
    // Cluster people into groups
    auto groups = clusterPeople(transformed_people_, max_interaction_distance_);

    for (const auto& group : groups) {
      if (group.size() < 2) continue; // Skip single-person groups

      // Compute merged region (convex hull)
      auto hull = computeConvexHull(group);

      // Bounding box of the merged region
      double min_x = std::numeric_limits<double>::max();
      double min_y = std::numeric_limits<double>::max();
      double max_x = std::numeric_limits<double>::lowest();
      double max_y = std::numeric_limits<double>::lowest();
      for (const auto& pt : hull) {
        min_x = std::min(min_x, pt.x);
        min_y = std::min(min_y, pt.y);
        max_x = std::max(max_x, pt.x);
        max_y = std::max(max_y, pt.y);
      }

      // Add padding for Gaussian drop-off
      double padding = interaction_width_ * 3.0;
      min_x -= padding;
      min_y -= padding;
      max_x += padding;
      max_y += padding;

      // Convert to map coordinates
      int min_i, min_j, max_i, max_j;
      costmap->worldToMapNoBounds(min_x, min_y, min_i, min_j);
      costmap->worldToMapNoBounds(max_x, max_y, max_i, max_j);

      // Clamp to costmap bounds
      min_i = std::max(min_i, 0);
      min_j = std::max(min_j, 0);
      max_i = std::min(max_i, static_cast<int>(costmap->getSizeInCellsX()) - 1);
      max_j = std::min(max_j, static_cast<int>(costmap->getSizeInCellsY()) - 1);

      // Iterate through cells in the bounding box
      for (int j = min_j; j <= max_j; ++j) {
        for (int i = min_i; i <= max_i; ++i) {
          double wx, wy;
          costmap->mapToWorld(i, j, wx, wy);

          // Calculate signed distance to merged region
          double dist = distanceToMergedRegion(wx, wy, hull);

          // Inside the merged region: Fixed high cost
          if (dist >= 0) {
            unsigned char old_cost = costmap->getCost(i, j);
            costmap->setCost(i, j, std::max(old_cost, static_cast<unsigned char>(254)));
          }
          // Outside the merged region: Gaussian drop-off
          else {
            double abs_dist = -dist;
            double gaussian_value = interaction_amplitude_ * exp(-(abs_dist * abs_dist) / (2.0 * interaction_width_scale_ * interaction_width_scale_));
            if (gaussian_value >= cutoff_) {
              unsigned char old_cost = costmap->getCost(i, j);
              unsigned char new_cost = std::max(old_cost, static_cast<unsigned char>(gaussian_value));
              costmap->setCost(i, j, new_cost);
            }
          }
        }
      }
    }
  }
    grid_pub_->publish(grid);
}

double SocialLayer::gaussian(double x, double y, double x0, double y0, double A,
                             double varx, double vary, double skew) {
  double dx = x - x0, dy = y - y0;
  double h = sqrt(dx * dx + dy * dy);
  double angle = atan2(dy, dx);
  double mx = cos(angle - skew) * h;
  double my = sin(angle - skew) * h;
  double f1 = pow(mx, 2.0) / (2.0 * varx);
  double f2 = pow(my, 2.0) / (2.0 * vary);
  return A * exp(-(f1 + f2));
}

double SocialLayer::get_radius(double cutoff, double A, double var) {
  return sqrt(-2 * var * log(cutoff / A));
}

// Add this helper function implementation
double SocialLayer::distanceToLineSegment(double x, double y, 
                                        double x1, double y1,
                                        double x2, double y2) {
  double dx = x2 - x1;
  double dy = y2 - y1;
  double sq_len = dx*dx + dy*dy;

  if (sq_len == 0.0) {  // Line segment is a point
    return sqrt((x-x1)*(x-x1) + (y-y1)*(y-y1));
  }

  // Calculate projection parameter
  double t = ((x - x1)*dx + (y - y1)*dy) / sq_len;
  t = std::max(0.0, std::min(1.0, t));

  // Projection point
  double proj_x = x1 + t*dx;
  double proj_y = y1 + t*dy;

  return sqrt((x - proj_x)*(x - proj_x) + (y - proj_y)*(y - proj_y));
}

// Function to compute signed distance to convex hull
double distanceToMergedRegion(double wx, double wy,
  const std::vector<geometry_msgs::msg::Point>& hull) {
  double min_dist = std::numeric_limits<double>::max();
  bool inside = true; // Assume point is inside (simplified for example)

  SocialLayer social_layer;

  // Simplified logic: Compute distance to the nearest edge of the hull
  // For actual implementation, use ray-casting for inside/outside check
  for (size_t i = 0; i < hull.size(); ++i) {
    size_t j = (i + 1) % hull.size();
    double x1 = hull[i].x, y1 = hull[i].y;
    double x2 = hull[j].x, y2 = hull[j].y;
    double dist = social_layer.distanceToLineSegment(wx, wy, x1, y1, x2, y2);
  if (dist < min_dist) min_dist = dist;
  }
  return inside ? min_dist : -min_dist;
}

} // namespace nav2_social_costmap_plugin

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_social_costmap_plugin::SocialLayer,
                       nav2_costmap_2d::Layer)