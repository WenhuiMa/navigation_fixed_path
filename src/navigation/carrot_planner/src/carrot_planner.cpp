/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2008, Willow Garage, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of Willow Garage, Inc. nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*
* Authors: Eitan Marder-Eppstein, Sachin Chitta
*********************************************************************/
#include <carrot_planner/carrot_planner.h>
#include <pluginlib/class_list_macros.h>

//register this planner as a BaseGlobalPlanner plugin
PLUGINLIB_EXPORT_CLASS(carrot_planner::CarrotPlanner, nav_core::BaseGlobalPlanner)

namespace carrot_planner {

  CarrotPlanner::CarrotPlanner()
  : costmap_ros_(NULL), initialized_(false){}

  CarrotPlanner::CarrotPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros)
  : costmap_ros_(NULL), initialized_(false){
    initialize(name, costmap_ros);
  }
  
  void CarrotPlanner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros){
    if(!initialized_){
      costmap_ros_ = costmap_ros;
      costmap_ = costmap_ros_->getCostmap();

      ros::NodeHandle private_nh("~/" + name);
      private_nh.param("step_size", step_size_, costmap_->getResolution());
      private_nh.param("min_dist_from_robot", min_dist_from_robot_, 0.10);
      world_model_ = new base_local_planner::CostmapModel(*costmap_); 

      initialized_ = true;
    }
    else
      ROS_WARN("This planner has already been initialized... doing nothing");
  }

  //we need to take the footprint of the robot into account when we calculate cost to obstacles
  double CarrotPlanner::footprintCost(double x_i, double y_i, double theta_i){
    if(!initialized_){
      ROS_ERROR("The planner has not been initialized, please call initialize() to use the planner");
      return -1.0;
    }

    std::vector<geometry_msgs::Point> footprint = costmap_ros_->getRobotFootprint();
    //if we have no footprint... do nothing
    if(footprint.size() < 3)
      return -1.0;

    //check if the footprint is legal
    double footprint_cost = world_model_->footprintCost(x_i, y_i, theta_i, footprint);
    return footprint_cost;
  }

  bool CarrotPlanner::makePlan(const geometry_msgs::PoseStamped& start,
        const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan)
    {
      plan.clear();
      costmap_ = costmap_ros_->getCostmap();

      if(goal.header.frame_id != costmap_ros_->getGlobalFrameID()){
        ROS_ERROR("This planner as configured will only accept goals in the %s frame, but a goal was sent in the %s frame.",
            costmap_ros_->getGlobalFrameID().c_str(), goal.header.frame_id.c_str());
        return false;
      }

      if(footprintCost(goal.pose.position.x,goal.pose.position.y,goal.pose.position.z)<0)
      {
        //std::cout<<"Is goal point occupied?"<<std::endl;
        return false;
      }

      geometry_msgs::PoseStamped aux1;
      geometry_msgs::PoseStamped aux2;
      double diffx = goal.pose.position.x-start.pose.position.x;

      aux1.pose.position.x=start.pose.position.x+0.2*diffx;
      aux1.pose.position.y=goal.pose.position.y;

      aux2.pose.position.x=goal.pose.position.x-0.2*diffx;
      aux2.pose.position.y=start.pose.position.y;

      makeSegment(start,goal,aux1,aux2,plan);

      plan.push_back(goal);

      return true;
    }

    bool CarrotPlanner::makeSegment(const geometry_msgs::PoseStamped& start,
                                   const geometry_msgs::PoseStamped& goal,
                                   const geometry_msgs::PoseStamped& aux1,
                                   const geometry_msgs::PoseStamped& aux2,
                                   std::vector<geometry_msgs::PoseStamped>& plan)
    {
      geometry_msgs::PoseStamped path_point;
      Eigen::Matrix4f coefficient_matrix;
      coefficient_matrix <<1, 0, 0, 0,
                               -3, 3, 0, 0,
                               3,-6, 3, 0,
                               -1, 3,-3,1;
      Eigen::Vector2f start_point(0,0);
      Eigen::Vector2f end_point(0,0);
      Eigen::Vector2f aux_point1(0,0);
      Eigen::Vector2f aux_point2(0,0);

      start_point.x()=start.pose.position.x;
      start_point.y() = start.pose.position.y;
      end_point.x() = goal.pose.position.x;
      end_point.y() = goal.pose.position.y;
      aux_point1.x() = aux1.pose.position.x;
      aux_point1.y() = aux1.pose.position.y;
      aux_point2.x() = aux2.pose.position.x;
      aux_point2.y() = aux2.pose.position.y;

      Eigen::Vector2f result_point(0,0);

      float step = 0.1*costmap_ros_->getCostmap()->getResolution();
      Eigen::MatrixXf variance_matrix(1,4);
      variance_matrix = Eigen::MatrixXf::Zero(1,4);

      Eigen::MatrixXf temp_result(1,4);
      temp_result = Eigen::MatrixXf::Zero(1,4);

      for(float t =0; t<=1;t+=step)
      {
        variance_matrix<<1,t,t*t,t*t*t;
        temp_result = variance_matrix * coefficient_matrix;

        result_point = temp_result(0,0) * start_point +
            temp_result(0,1) * aux_point1  +
            temp_result(0,2) * aux_point2  +
            temp_result(0,3) * end_point;

        path_point.header.frame_id=start.header.frame_id;
        path_point.pose.orientation.w = 1;
        path_point.pose.position.x = result_point.x();
        path_point.pose.position.y = result_point.y();
        plan.push_back(path_point);
      }

    }

};
