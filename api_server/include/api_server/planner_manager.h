#ifndef _API_PLANNER_MANAGER_H_
#define _API_PLANNER_MANAGER_H_

#include <stdlib.h>
#include <rclcpp/rclcpp.hpp>

#include <bspline_opt/bspline_optimizer.h>
#include <bspline_opt/uniform_bspline.h>
#include <plan_env/grid_map.h>
#include <plan_env/obj_predictor.h>

#include <traj_utils/msg/data_disp.hpp>
#include <traj_utils/plan_container.hpp>

#include <traj_utils/planning_visualization.h>

namespace api_server
{
  // Fast Planner Manager
  // Key algorithms of mapping and planning are called

  class PlannerManager
  {
    // SECTION stable
  public:
    PlannerManager();
    ~PlannerManager();

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /* main planning interface */
    bool reboundReplan(Eigen::Vector3d start_pt, Eigen::Vector3d start_vel, Eigen::Vector3d start_acc,
                       Eigen::Vector3d end_pt, Eigen::Vector3d end_vel, bool flag_polyInit, bool flag_randomPolyTraj);
    bool EmergencyStop(Eigen::Vector3d stop_pos);
    bool planGlobalTraj(const Eigen::Vector3d &start_pos, const Eigen::Vector3d &start_vel, const Eigen::Vector3d &start_acc,
                        const Eigen::Vector3d &end_pos, const Eigen::Vector3d &end_vel, const Eigen::Vector3d &end_acc);
    bool planGlobalTrajWaypoints(const Eigen::Vector3d &start_pos, const Eigen::Vector3d &start_vel, const Eigen::Vector3d &start_acc,
                                 const std::vector<Eigen::Vector3d> &waypoints, const Eigen::Vector3d &end_vel, const Eigen::Vector3d &end_acc);

    void initPlanModules(rclcpp::Node::SharedPtr &node, ego_planner::PlanningVisualization::Ptr vis = NULL);

    void deliverTrajToOptimizer(void) { bspline_optimizer_->setSwarmTrajs(&swarm_trajs_buf_); };

    void setDroneIdtoOpt(void) { bspline_optimizer_->setDroneId(pp_.drone_id); }

    double getSwarmClearance(void) { return bspline_optimizer_->getSwarmClearance(); }

    bool checkCollision(int drone_id);
    

    ego_planner::PlanParameters pp_;
    ego_planner::LocalTrajData local_data_;
    ego_planner::GlobalTrajData global_data_;
    GridMap::Ptr grid_map_;
    fast_planner::ObjPredictor::Ptr obj_predictor_;    
    ego_planner::SwarmTrajData swarm_trajs_buf_;

  private:
    /* main planning algorithms & modules */
    ego_planner::PlanningVisualization::Ptr visualization_;

    // ros::Publisher obj_pub_; //zx-todo 

    ego_planner::BsplineOptimizer::Ptr bspline_optimizer_;

    int continous_failures_count_{0};

    void updateTrajInfo(const ego_planner::UniformBspline &position_traj, const rclcpp::Time time_now);

    void reparamBspline(ego_planner::UniformBspline &bspline, vector<Eigen::Vector3d> &start_end_derivative, double ratio, Eigen::MatrixXd &ctrl_pts, double &dt,
                        double &time_inc);

    bool refineTrajAlgo(ego_planner::UniformBspline &traj, vector<Eigen::Vector3d> &start_end_derivative, double ratio, double &ts, Eigen::MatrixXd &optimal_control_points);

    // !SECTION stable

    // SECTION developing

  public:
    typedef unique_ptr<PlannerManager> Ptr;

    // !SECTION
  };
} // namespace api_server

#endif