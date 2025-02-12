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
  // Subscribe to individual people topic
  auto node_shared_ptr = node_;
  ppl_sub_ = node_shared_ptr->create_subscription<people_msgs::msg::People>(
    "/people", rclcpp::SensorDataQoS(),
    std::bind(&SocialLayer::peopleCallback, this, std::placeholders::_1));

  // Subscribe to people groups topic
  group_sub_ = node_shared_ptr->create_subscription<people_msgs::msg::PeopleGroupArray>(
    "/people_groups", rclcpp::SensorDataQoS(),
    std::bind(&SocialLayer::groupCallback, this, std::placeholders::_1));

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
  // Group Interaction Cost Map Parameters
  declareParameter("max_interaction_distance", rclcpp::ParameterValue(10.0));
  declareParameter("interaction_width", rclcpp::ParameterValue(1.5));
  declareParameter("interaction_cost", rclcpp::ParameterValue(140));
  declareParameter("interaction_amplitude", rclcpp::ParameterValue(254.0));
  declareParameter("interaction_length_scale", rclcpp::ParameterValue(1.0));
  declareParameter("interaction_width_scale", rclcpp::ParameterValue(0.6));
  declareParameter("centroid_amplitude", rclcpp::ParameterValue(200.0));
  declareParameter("centroid_scale", rclcpp::ParameterValue(1.0));
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
  node_shared_ptr->get_parameter(name_ + "." + "centroid_amplitude", centroid_amplitude_);
  node_shared_ptr->get_parameter(name_ + "." + "centroid_scale", centroid_scale_);
}

void SocialLayer::peopleCallback(
    const people_msgs::msg::People::SharedPtr msg) {
  ppl_message_mutex_.lock();
  people_list_ = *msg;
  ppl_message_mutex_.unlock();
}

void SocialLayer::groupCallback(const people_msgs::msg::PeopleGroupArray::SharedPtr msg) {
  group_message_mutex_.lock();
  groups_list_ = *msg;
  group_message_mutex_.unlock();
}

void SocialLayer::updateBounds(double origin_x, double origin_y,
                               double origin_z, double *min_x, double *min_y,
                               double *max_x, double *max_y) {

  std::string global_frame =
      layered_costmap_
          ->getGlobalFrameID(); // Returns the global frame of the costmap
  transformed_people_.clear();  // empty the array
  transformed_groups_.clear();

  rclcpp::Logger logger_ = node_->get_logger();

  for (unsigned int i = 0; i < people_list_.people.size(); i++) {
    people_msgs::msg::Person &person = people_list_.people[i];
    people_msgs::msg::Person tpt;
    geometry_msgs::msg::PointStamped pt, opt;

    pt.point.x = person.pose.position.x;
    pt.point.y = person.pose.position.y;
    pt.point.z = person.pose.position.z;
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
    tpt.pose.position.x = opt.point.x;
    tpt.pose.position.y = opt.point.y;
    tpt.pose.position.z = opt.point.z;

    pt.point.x += person.velocity.x;
    pt.point.y += person.velocity.y;
    pt.point.z += person.velocity.z;
    tf_->transform(pt, opt, global_frame);

    tpt.velocity.x = opt.point.x - tpt.pose.position.x;
    tpt.velocity.y = opt.point.y - tpt.pose.position.y;
    tpt.velocity.z = opt.point.z - tpt.pose.position.z;

    // cnt_j++;
    transformed_people_.push_back(
        tpt); // Adds a new element (tpt) at the end of the vector
  }

  // std::list<people_msgs::msg::Person>::iterator p_it;

  for (auto p_it = transformed_people_.begin(); p_it != transformed_people_.end();
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

    *min_x = std::min(*min_x, person.pose.position.x - greater);
    *min_y = std::min(*min_y, person.pose.position.y - greater);
    *max_x = std::max(*max_x, person.pose.position.x + greater);
    *max_y = std::max(*max_y, person.pose.position.y + greater);
  }

  // Process groups
  for (const auto& group : groups_list_.groups) {
    people_msgs::msg::PeopleGroup transformed_group;
    // Transform centroid
    geometry_msgs::msg::PointStamped centroid_pt, transformed_centroid;
    centroid_pt.header = groups_list_.header;
    centroid_pt.point = group.centroid;

    try {
      tf_->transform(centroid_pt, transformed_centroid, global_frame);
      transformed_group.centroid = transformed_centroid.point;
    } catch (tf2::TransformException &ex) {
      RCLCPP_WARN(node_->get_logger(), "Centroid TF exception: %s", ex.what());
      continue;
    }

    for (const auto& original_person : group.people) {

      people_msgs::msg::Person transformed_person;
      geometry_msgs::msg::Point centroid;  
      geometry_msgs::msg::PointStamped gpt, gopt;

      gpt.point = original_person.pose.position;
      gpt.header = groups_list_.header;
      
      try {
        // Transform position
        tf_->transform(gpt, gopt, global_frame);
        transformed_person.pose.position = gopt.point;
        
        // Transform velocity
        geometry_msgs::msg::Vector3Stamped vel_in, vel_out;
        vel_in.vector.x = original_person.velocity.x;
        vel_in.vector.y = original_person.velocity.y;
        vel_in.vector.z = original_person.velocity.z;
        vel_in.header = groups_list_.header;
        tf_->transform(vel_in, vel_out, global_frame);
        transformed_person.velocity.x = vel_out.vector.x;
        transformed_person.velocity.y = vel_out.vector.y;
        transformed_person.velocity.z = vel_out.vector.z;
        
        transformed_group.people.push_back(transformed_person);
      } catch (tf2::TransformException &ex) {
        RCLCPP_WARN(node_->get_logger(), "TF exception: %s", ex.what());
      }
    }
    
    if (!transformed_group.people.empty()) {
      transformed_groups_.push_back(transformed_group);

      // Update bounds for centroid Gaussian
      double centroid_padding = centroid_scale_ * 3.0;  // 3σ coverage
      *min_x = std::min(*min_x, transformed_group.centroid.x - centroid_padding);
      *min_y = std::min(*min_y, transformed_group.centroid.y - centroid_padding);
      *max_x = std::max(*max_x, transformed_group.centroid.x + centroid_padding);
      *max_y = std::max(*max_y, transformed_group.centroid.y + centroid_padding);
    }
  }

  // Calculate bounds for groups
  for (const auto& group : transformed_groups_) {
    for (const auto& person : group.people) {
      double padding = interaction_width_ * 2.0;
      *min_x = std::min(*min_x, person.pose.position.x - padding);
      *min_y = std::min(*min_y, person.pose.position.y - padding);
      *max_x = std::max(*max_x, person.pose.position.x + padding);
      *max_y = std::max(*max_y, person.pose.position.y + padding);
    }
  }
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

    double cx = person.pose.position.x, cy = person.pose.position.y;

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
  // Process groups
  for (const auto& group : transformed_groups_) {
    
    if (group.people.size() < 2) continue;

    applyCentroidGaussian(group.centroid, costmap);

    // Connect each person to centroid
    for (const auto& person : group.people) {
      connectToCentroid(person.pose.position, group.centroid, costmap);
    }
    for (size_t i = 0; i < group.people.size(); ++i) {
      for (size_t j = i+1; j < group.people.size(); ++j) {
        const auto& person1 = group.people[i];
        const auto& person2 = group.people[j];

        // Calculate distance between people
        double dx = person1.pose.position.x - person2.pose.position.x;
        double dy = person1.pose.position.y - person2.pose.position.y;
        double distance = sqrt(dx * dx + dy * dy);

        // Skip if people are too far apart
        if (distance > max_interaction_distance_) continue;

        // Calculate line segment endpoints
        double x1 = person1.pose.position.x;
        double y1 = person1.pose.position.y;
        double x2 = person2.pose.position.x;
        double y2 = person2.pose.position.y;

        // Calculate bounding box for the interaction area
        double padding = interaction_width_ * 2.0; // Add padding for Gaussian drop-off
        double min_x = std::min(x1, x2) - padding;
        double max_x = std::max(x1, x2) + padding;
        double min_y = std::min(y1, y2) - padding;
        double max_y = std::max(y1, y2) + padding;

        // Convert to map coordinates
        int min_i, min_j, max_i, max_j;
        costmap->worldToMapNoBounds(min_x, min_y, min_i, min_j);
        costmap->worldToMapNoBounds(max_x, max_y, max_i, max_j);

        // Clamp to costmap bounds
        min_i = std::max(min_i, 0);
        min_j = std::max(min_j, 0);
        max_i = std::min(max_i, static_cast<int>(costmap->getSizeInCellsX()) - 1);
        max_j = std::min(max_j, static_cast<int>(costmap->getSizeInCellsY()) - 1);

        // Iterate through potential affected cells
        for (int j = min_j; j <= max_j; ++j) {
          for (int i = min_i; i <= max_i; ++i) {
            double wx, wy;
            costmap->mapToWorld(i, j, wx, wy);

            // Calculate perpendicular distance to the line segment
            double dist = distanceToLineSegment(wx, wy, x1, y1, x2, y2);

            // If the cell is on the line, set cost to 254
            if (dist <= interaction_width_ / 10.0) {
              unsigned char old_cost = costmap->getCost(i, j);
              unsigned char new_cost = 254; // Fixed high cost along the line
              costmap->setCost(i, j, std::max(old_cost, new_cost));

              if (publish_occgrid_) {
                unsigned int index = costmap->getIndex(i, j);
                grid.data[index] = new_cost;
              }
            }
            // If the cell is near the line, apply Gaussian drop-off
            else if (dist <= interaction_width_) {
              // Calculate Gaussian drop-off based on perpendicular distance
              double gaussian_value = interaction_amplitude_ * exp(-(dist * dist) / (2.0 * interaction_width_scale_ * interaction_width_scale_));

              // Apply cutoff
              if (gaussian_value < cutoff_) continue;

              // Get existing cost
              unsigned char old_cost = costmap->getCost(i, j);

              // Combine costs using maximum
              unsigned char new_cost = std::max(old_cost, static_cast<unsigned char>(gaussian_value));

              // Update costmap
              costmap->setCost(i, j, new_cost);

              // Update occupancy grid
              if (publish_occgrid_) {
                unsigned int index = costmap->getIndex(i, j);
                grid.data[index] = new_cost;
              }
            }
          }
        }
      }
    }
  }
  if (publish_occgrid_)
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

void SocialLayer::applyCentroidGaussian(const geometry_msgs::msg::Point& centroid,
                                      nav2_costmap_2d::Costmap2D* costmap) {
  double res = costmap->getResolution();
  int size = static_cast<int>((centroid_scale_ ) / res);  // 3σ radius
  
  int cx, cy;
  costmap->worldToMapNoBounds(centroid.x, centroid.y, cx, cy);

  for (int dy = -size; dy <= size; ++dy) {
    for (int dx = -size; dx <= size; ++dx) {
      int x = cx + dx;
      int y = cy + dy;
      // if (!costmap->inside(x, y)) continue;

      double wx, wy;
      costmap->mapToWorld(x, y, wx, wy);
      
      double dx_w = wx - centroid.x;
      double dy_w = wy - centroid.y;
      double dist = sqrt(dx_w*dx_w + dy_w*dy_w);
      
      double value = centroid_amplitude_ * exp(-(dist*dist)/(2*centroid_scale_*centroid_scale_));
      if (value < cutoff_) continue;

      unsigned char new_cost = static_cast<unsigned char>(value);
      unsigned char old_cost = costmap->getCost(x, y);
      costmap->setCost(x, y, std::max(old_cost, new_cost));
    }
  }
}

void SocialLayer::connectToCentroid(const geometry_msgs::msg::Point& person_pos,
                                  const geometry_msgs::msg::Point& centroid,
                                  nav2_costmap_2d::Costmap2D* costmap) {
  double res = costmap->getResolution();
  
  // Calculate line parameters
  double x1 = person_pos.x;
  double y1 = person_pos.y;
  double x2 = centroid.x;
  double y2 = centroid.y;
  
  // Calculate bounding box with padding
  double padding = interaction_width_ * 2.0;
  double min_x = std::min(x1, x2) - padding;
  double max_x = std::max(x1, x2) + padding;
  double min_y = std::min(y1, y2) - padding;
  double max_y = std::max(y1, y2) + padding;

  // Convert to map coordinates
  int min_i, min_j, max_i, max_j;
  costmap->worldToMapNoBounds(min_x, min_y, min_i, min_j);
  costmap->worldToMapNoBounds(max_x, max_y, max_i, max_j);

  // Clamp to costmap bounds
  min_i = std::max(min_i, 0);
  min_j = std::max(min_j, 0);
  max_i = std::min(max_i, static_cast<int>(costmap->getSizeInCellsX())-1);
  max_j = std::min(max_j, static_cast<int>(costmap->getSizeInCellsY())-1);

  // Process cells
  for (int j = min_j; j <= max_j; ++j) {
    for (int i = min_i; i <= max_i; ++i) {
      double wx, wy;
      costmap->mapToWorld(i, j, wx, wy);

      double dist = distanceToLineSegment(wx, wy, x1, y1, x2, y2);
      
      // Apply line cost with Gaussian falloff
      if (dist <= interaction_width_) {
        double gaussian_value = interaction_amplitude_ * 
          exp(-(dist*dist)/(2*interaction_width_scale_*interaction_width_scale_));
        
        if (gaussian_value >= cutoff_) {
          unsigned char new_cost = static_cast<unsigned char>(gaussian_value);
          unsigned char old_cost = costmap->getCost(i, j);
          costmap->setCost(i, j, std::max(old_cost, new_cost));
        }
      }
    }
  }
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

} // namespace nav2_social_costmap_plugin

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_social_costmap_plugin::SocialLayer,
                       nav2_costmap_2d::Layer)