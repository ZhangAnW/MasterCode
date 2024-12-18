/**
* This file is part of Fast-Planner.
*
* Copyright 2019 Boyu Zhou, Aerial Robotics Group, Hong Kong University of Science and Technology, <uav.ust.hk>
* Developed by Boyu Zhou <bzhouai at connect dot ust dot hk>, <uv dot boyuzhou at gmail dot com>
* for more information see <https://github.com/HKUST-Aerial-Robotics/Fast-Planner>.
* If you use this code, please cite the respective publications as
* listed on the above website.
*
* Fast-Planner is free software: you can redistribute it and/or modify
* it under the terms of the GNU Lesser General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* Fast-Planner is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU Lesser General Public License
* along with Fast-Planner. If not, see <http://www.gnu.org/licenses/>.
*/



#include "bspline_opt/bspline_optimizer.h"
#include <nlopt.hpp>
// using namespace std;

namespace fast_planner {

const int BsplineOptimizer::SMOOTHNESS  = (1 << 0);
const int BsplineOptimizer::DISTANCE    = (1 << 1);
const int BsplineOptimizer::FEASIBILITY = (1 << 2);
const int BsplineOptimizer::ENDPOINT    = (1 << 3);
const int BsplineOptimizer::GUIDE       = (1 << 4);
const int BsplineOptimizer::WAYPOINTS   = (1 << 6);

const int BsplineOptimizer::GUIDE_PHASE = BsplineOptimizer::SMOOTHNESS | BsplineOptimizer::GUIDE;
const int BsplineOptimizer::NORMAL_PHASE =
    BsplineOptimizer::SMOOTHNESS | BsplineOptimizer::DISTANCE | BsplineOptimizer::FEASIBILITY;

void BsplineOptimizer::setParam(ros::NodeHandle& nh) {
  nh.param("optimization/lambda1", lambda1_, -1.0);
  nh.param("optimization/lambda2", lambda2_, -1.0);
  nh.param("optimization/lambda3", lambda3_, -1.0);
  nh.param("optimization/lambda4", lambda4_, -1.0);
  nh.param("optimization/lambda5", lambda5_, -1.0);
  nh.param("optimization/lambda6", lambda6_, -1.0);
  nh.param("optimization/lambda7", lambda7_, -1.0);
  nh.param("optimization/lambda8", lambda8_, -1.0);

  nh.param("optimization/dist0", dist0_, -1.0);
  nh.param("optimization/max_vel", max_vel_, -1.0);
  nh.param("optimization/max_acc", max_acc_, -1.0);
  nh.param("optimization/visib_min", visib_min_, -1.0);
  nh.param("optimization/dlmin", dlmin_, -1.0);
  nh.param("optimization/wnl", wnl_, -1.0);

  nh.param("optimization/max_iteration_num1", max_iteration_num_[0], -1);
  nh.param("optimization/max_iteration_num2", max_iteration_num_[1], -1);
  nh.param("optimization/max_iteration_num3", max_iteration_num_[2], -1);
  nh.param("optimization/max_iteration_num4", max_iteration_num_[3], -1);
  nh.param("optimization/max_iteration_time1", max_iteration_time_[0], -1.0);
  nh.param("optimization/max_iteration_time2", max_iteration_time_[1], -1.0);
  nh.param("optimization/max_iteration_time3", max_iteration_time_[2], -1.0);
  nh.param("optimization/max_iteration_time4", max_iteration_time_[3], -1.0);

  nh.param("optimization/algorithm1", algorithm1_, -1);
  nh.param("optimization/algorithm2", algorithm2_, -1);
  nh.param("optimization/order", order_, -1);
}

void BsplineOptimizer::setEnvironment(const EDTEnvironment::Ptr& env) {
  this->edt_environment_ = env;
}

void BsplineOptimizer::setControlPoints(const Eigen::MatrixXd& points) {
  control_points_ = points;
  dim_            = control_points_.cols();
}

void BsplineOptimizer::setBsplineInterval(const double& ts) { bspline_interval_ = ts; }

void BsplineOptimizer::setTerminateCond(const int& max_num_id, const int& max_time_id) {
  max_num_id_  = max_num_id;
  max_time_id_ = max_time_id;
}

void BsplineOptimizer::setCostFunction(const int& cost_code) {
  cost_function_ = cost_code;

  // print optimized cost function
  string cost_str;
  if (cost_function_ & SMOOTHNESS) cost_str += "smooth |";
  if (cost_function_ & DISTANCE) cost_str += " dist  |";
  if (cost_function_ & FEASIBILITY) cost_str += " feasi |";
  if (cost_function_ & ENDPOINT) cost_str += " endpt |";
  if (cost_function_ & GUIDE) cost_str += " guide |";
  if (cost_function_ & WAYPOINTS) cost_str += " waypt |";

  ROS_INFO_STREAM("cost func: " << cost_str);
}

void BsplineOptimizer::setGuidePath(const vector<Eigen::Vector3d>& guide_pt) { guide_pts_ = guide_pt; }

void BsplineOptimizer::setWaypoints(const vector<Eigen::Vector3d>& waypts,
                                    const vector<int>&             waypt_idx) {
  waypoints_ = waypts;
  waypt_idx_ = waypt_idx;
}

Eigen::MatrixXd BsplineOptimizer::BsplineOptimizeTraj(const Eigen::MatrixXd& points, const double& ts,
                                                      const int& cost_function, int max_num_id,
                                                      int max_time_id) {
  setControlPoints(points);
  setBsplineInterval(ts);
  // setCostFunction(cost_function);
  // setTerminateCond(max_num_id, max_time_id);

  // optimize();
  return this->control_points_;
}

void BsplineOptimizer::optimize() {
  /* initialize solver */
  iter_num_        = 0;
  min_cost_        = std::numeric_limits<double>::max();
  const int pt_num = control_points_.rows();
  g_q_.resize(pt_num);
  g_smoothness_.resize(pt_num);
  g_distance_.resize(pt_num);
  g_feasibility_.resize(pt_num);
  g_endpoint_.resize(pt_num);
  g_waypoints_.resize(pt_num);
  g_guide_.resize(pt_num);

  if (cost_function_ & ENDPOINT) {
    variable_num_ = dim_ * (pt_num - order_);
    // end position used for hard constraint
    end_pt_ = (1 / 6.0) *
        (control_points_.row(pt_num - 3) + 4 * control_points_.row(pt_num - 2) +
         control_points_.row(pt_num - 1));
  } else {
    variable_num_ = max(0, dim_ * (pt_num - 2 * order_)) ;
  }

  /* do optimization using NLopt slover */
  nlopt::opt opt(nlopt::algorithm(isQuadratic() ? algorithm1_ : algorithm2_), variable_num_);
  opt.set_min_objective(BsplineOptimizer::costFunction, this);
  opt.set_maxeval(max_iteration_num_[max_num_id_]);
  opt.set_maxtime(max_iteration_time_[max_time_id_]);
  opt.set_xtol_rel(1e-5);

  vector<double> q(variable_num_);
  for (int i = order_; i < pt_num; ++i) {
    if (!(cost_function_ & ENDPOINT) && i >= pt_num - order_) continue;
    for (int j = 0; j < dim_; j++) {
      q[dim_ * (i - order_) + j] = control_points_(i, j);
    }
  }

  if (dim_ != 1) {
    vector<double> lb(variable_num_), ub(variable_num_);
    const double   bound = 10.0;
    for (int i = 0; i < variable_num_; ++i) {
      lb[i] = q[i] - bound;
      ub[i] = q[i] + bound;
    }
    opt.set_lower_bounds(lb);
    opt.set_upper_bounds(ub);
  }

  try {
    // cout << fixed << setprecision(7);
    // vec_time_.clear();
    // vec_cost_.clear();
    // time_start_ = ros::Time::now();

    double        final_cost;
    nlopt::result result = opt.optimize(q, final_cost);

    /* retrieve the optimization result */
    // cout << "Min cost:" << min_cost_ << endl;
  } catch (std::exception& e) {
    ROS_WARN("[Optimization]: nlopt exception");
    cout << e.what() << endl;
  }

  for (int i = order_; i < control_points_.rows(); ++i) {
    if (!(cost_function_ & ENDPOINT) && i >= pt_num - order_) continue;
    for (int j = 0; j < dim_; j++) {
      control_points_(i, j) = best_variable_[dim_ * (i - order_) + j];
    }
  }

  if (!(cost_function_ & GUIDE)) ROS_INFO_STREAM("iter num: " << iter_num_);
}

void BsplineOptimizer::calcSmoothnessCost(const vector<Eigen::Vector3d>& q, double& cost,
                                          vector<Eigen::Vector3d>& gradient) {
  cost = 0.0;
  Eigen::Vector3d zero(0, 0, 0);
  std::fill(gradient.begin(), gradient.end(), zero);
  Eigen::Vector3d jerk, temp_j;

  for (int i = 0; i < q.size() - order_; i++) {
    /* evaluate jerk */
    jerk = q[i + 3] - 3 * q[i + 2] + 3 * q[i + 1] - q[i];
    cost += jerk.squaredNorm();
    temp_j = 2.0 * jerk;
    /* jerk gradient */
    gradient[i + 0] += -temp_j;
    gradient[i + 1] += 3.0 * temp_j;
    gradient[i + 2] += -3.0 * temp_j;
    gradient[i + 3] += temp_j;
  }
}

void BsplineOptimizer::calcDistanceCost(const vector<Eigen::Vector3d>& q, double& cost,
                                        vector<Eigen::Vector3d>& gradient) {
  cost = 0.0;
  Eigen::Vector3d zero(0, 0, 0);
  std::fill(gradient.begin(), gradient.end(), zero);

  double          dist;
  Eigen::Vector3d dist_grad, g_zero(0, 0, 0);

  int end_idx = (cost_function_ & ENDPOINT) ? q.size() : q.size() - order_;

  for (int i = order_; i < end_idx; i++) {
    edt_environment_->evaluateEDTWithGrad(q[i], -1.0, dist, dist_grad);
    if (dist_grad.norm() > 1e-4) dist_grad.normalize();

    if (dist < dist0_) {
      cost += pow(dist - dist0_, 2);
      gradient[i] += 2.0 * (dist - dist0_) * dist_grad;
    }
  }
}

// void BsplineOptimizer::calcFeasibilityCost(const vector<Eigen::Vector3d>& q, double& cost,
//                                            vector<Eigen::Vector3d>& gradient) {
//   cost = 0.0;
//   Eigen::Vector3d zero(0, 0, 0);
//   std::fill(gradient.begin(), gradient.end(), zero);

//   /* abbreviation */
//   double ts, vm2, am2, ts_inv2, ts_inv4;
//   vm2 = max_vel_ * max_vel_;
//   am2 = max_acc_ * max_acc_;

//   ts      = bspline_interval_;
//   ts_inv2 = 1 / ts / ts;
//   ts_inv4 = ts_inv2 * ts_inv2;

//   /* velocity feasibility */
//   for (int i = 0; i < q.size() - 1; i++) {
//     Eigen::Vector3d vi = q[i + 1] - q[i];

//     for (int j = 0; j < 3; j++) {
//       double vd = vi(j) * vi(j) * ts_inv2 - vm2;
//       if (vd > 0.0) {
//         cost += pow(vd, 2);

//         double temp_v = 4.0 * vd * ts_inv2;
//         gradient[i + 0](j) += -temp_v * vi(j);
//         gradient[i + 1](j) += temp_v * vi(j);
//       }
//     }
//   }

//   /* acceleration feasibility */
//   for (int i = 0; i < q.size() - 2; i++) {
//     Eigen::Vector3d ai = q[i + 2] - 2 * q[i + 1] + q[i];

//     for (int j = 0; j < 3; j++) {
//       double ad = ai(j) * ai(j) * ts_inv4 - am2;
//       if (ad > 0.0) {
//         cost += pow(ad, 2);

//         double temp_a = 4.0 * ad * ts_inv4;
//         gradient[i + 0](j) += temp_a * ai(j);
//         gradient[i + 1](j) += -2 * temp_a * ai(j);
//         gradient[i + 2](j) += temp_a * ai(j);
//       }
//     }
//   }
// }

void BsplineOptimizer::calcEndpointCost(const vector<Eigen::Vector3d>& q, double& cost,
                                        vector<Eigen::Vector3d>& gradient) {
  cost = 0.0;
  Eigen::Vector3d zero(0, 0, 0);
  std::fill(gradient.begin(), gradient.end(), zero);

  // zero cost and gradient in hard constraints
  Eigen::Vector3d q_3, q_2, q_1, dq;
  q_3 = q[q.size() - 3];
  q_2 = q[q.size() - 2];
  q_1 = q[q.size() - 1];

  dq = 1 / 6.0 * (q_3 + 4 * q_2 + q_1) - end_pt_;
  cost += dq.squaredNorm();

  gradient[q.size() - 3] += 2 * dq * (1 / 6.0);
  gradient[q.size() - 2] += 2 * dq * (4 / 6.0);
  gradient[q.size() - 1] += 2 * dq * (1 / 6.0);
}

void BsplineOptimizer::calcWaypointsCost(const vector<Eigen::Vector3d>& q, double& cost,
                                         vector<Eigen::Vector3d>& gradient) {
  cost = 0.0;
  Eigen::Vector3d zero(0, 0, 0);
  std::fill(gradient.begin(), gradient.end(), zero);

  Eigen::Vector3d q1, q2, q3, dq;

  // for (auto wp : waypoints_) {
  for (int i = 0; i < waypoints_.size(); ++i) {
    Eigen::Vector3d waypt = waypoints_[i];
    int             idx   = waypt_idx_[i];

    q1 = q[idx];
    q2 = q[idx + 1];
    q3 = q[idx + 2];

    dq = 1 / 6.0 * (q1 + 4 * q2 + q3) - waypt;
    cost += dq.squaredNorm();

    gradient[idx] += dq * (2.0 / 6.0);      // 2*dq*(1/6)
    gradient[idx + 1] += dq * (8.0 / 6.0);  // 2*dq*(4/6)
    gradient[idx + 2] += dq * (2.0 / 6.0);
  }
}

/* use the uniformly sampled points on a geomertic path to guide the
 * trajectory. For each control points to be optimized, it is assigned a
 * guiding point on the path and the distance between them is penalized */
void BsplineOptimizer::calcGuideCost(const vector<Eigen::Vector3d>& q, double& cost,
                                     vector<Eigen::Vector3d>& gradient) {
  cost = 0.0;
  Eigen::Vector3d zero(0, 0, 0);
  std::fill(gradient.begin(), gradient.end(), zero);

  int end_idx = q.size() - order_;

  for (int i = order_; i < end_idx; i++) {
    Eigen::Vector3d gpt = guide_pts_[i - order_];
    cost += (q[i] - gpt).squaredNorm();
    gradient[i] += 2 * (q[i] - gpt);
  }
}

void BsplineOptimizer::combineCost(const std::vector<double>& x, std::vector<double>& grad,
                                   double& f_combine) {
  // /* convert the NLopt format vector to control points. */

  // // This solver can support 1D-3D B-spline optimization, but we use Vector3d to store each control point
  // // For 1D case, the second and third elements are zero, and similar for the 2D case.
  // for (int i = 0; i < order_; i++) {
  //   for (int j = 0; j < dim_; ++j) {
  //     g_q_[i][j] = control_points_(i, j);
  //   }
  //   for (int j = dim_; j < 3; ++j) {
  //     g_q_[i][j] = 0.0;
  //   }
  // }

  // for (int i = 0; i < variable_num_ / dim_; i++) {
  //   for (int j = 0; j < dim_; ++j) {
  //     g_q_[i + order_][j] = x[dim_ * i + j];
  //   }
  //   for (int j = dim_; j < 3; ++j) {
  //     g_q_[i + order_][j] = 0.0;
  //   }
  // }

  // if (!(cost_function_ & ENDPOINT)) {
  //   for (int i = 0; i < order_; i++) {

  //     for (int j = 0; j < dim_; ++j) {
  //       g_q_[order_ + variable_num_ / dim_ + i][j] =
  //           control_points_(control_points_.rows() - order_ + i, j);
  //     }
  //     for (int j = dim_; j < 3; ++j) {
  //       g_q_[order_ + variable_num_ / dim_ + i][j] = 0.0;
  //     }
  //   }
  // }

  // f_combine = 0.0;
  // grad.resize(variable_num_);
  // fill(grad.begin(), grad.end(), 0.0);

  // /*  evaluate costs and their gradient  */
  // double f_smoothness, f_distance, f_feasibility, f_endpoint, f_guide, f_waypoints;
  // f_smoothness = f_distance = f_feasibility = f_endpoint = f_guide = f_waypoints = 0.0;

  // if (cost_function_ & SMOOTHNESS) {
  //   calcSmoothnessCost(g_q_, f_smoothness, g_smoothness_);
  //   f_combine += lambda1_ * f_smoothness;
  //   for (int i = 0; i < variable_num_ / dim_; i++)
  //     for (int j = 0; j < dim_; j++) grad[dim_ * i + j] += lambda1_ * g_smoothness_[i + order_](j);
  // }
  // if (cost_function_ & DISTANCE) {
  //   calcDistanceCost(g_q_, f_distance, g_distance_);
  //   f_combine += lambda2_ * f_distance;
  //   for (int i = 0; i < variable_num_ / dim_; i++)
  //     for (int j = 0; j < dim_; j++) grad[dim_ * i + j] += lambda2_ * g_distance_[i + order_](j);
  // }
  // if (cost_function_ & FEASIBILITY) {
  //   calcFeasibilityCost(g_q_, f_feasibility, g_feasibility_);
  //   f_combine += lambda3_ * f_feasibility;
  //   for (int i = 0; i < variable_num_ / dim_; i++)
  //     for (int j = 0; j < dim_; j++) grad[dim_ * i + j] += lambda3_ * g_feasibility_[i + order_](j);
  // }
  // if (cost_function_ & ENDPOINT) {
  //   calcEndpointCost(g_q_, f_endpoint, g_endpoint_);
  //   f_combine += lambda4_ * f_endpoint;
  //   for (int i = 0; i < variable_num_ / dim_; i++)
  //     for (int j = 0; j < dim_; j++) grad[dim_ * i + j] += lambda4_ * g_endpoint_[i + order_](j);
  // }
  // if (cost_function_ & GUIDE) {
  //   calcGuideCost(g_q_, f_guide, g_guide_);
  //   f_combine += lambda5_ * f_guide;
  //   for (int i = 0; i < variable_num_ / dim_; i++)
  //     for (int j = 0; j < dim_; j++) grad[dim_ * i + j] += lambda5_ * g_guide_[i + order_](j);
  // }
  // if (cost_function_ & WAYPOINTS) {
  //   calcWaypointsCost(g_q_, f_waypoints, g_waypoints_);
  //   f_combine += lambda7_ * f_waypoints;
  //   for (int i = 0; i < variable_num_ / dim_; i++)
  //     for (int j = 0; j < dim_; j++) grad[dim_ * i + j] += lambda7_ * g_waypoints_[i + order_](j);
  // }
  // /*  print cost  */
  // // if ((cost_function_ & WAYPOINTS) && iter_num_ % 10 == 0) {
  // //   cout << iter_num_ << ", total: " << f_combine << ", acc: " << lambda8_ * f_view
  // //        << ", waypt: " << lambda7_ * f_waypoints << endl;
  // // }

  // // if (optimization_phase_ == SECOND_PHASE) {
  // //  << ", smooth: " << lambda1_ * f_smoothness
  // //  << " , dist:" << lambda2_ * f_distance
  // //  << ", fea: " << lambda3_ * f_feasibility << endl;
  // // << ", end: " << lambda4_ * f_endpoint
  // // << ", guide: " << lambda5_ * f_guide
  // // }
}

double BsplineOptimizer::costFunction(const std::vector<double>& x, std::vector<double>& grad,
                                      void* func_data) {
  BsplineOptimizer* opt = reinterpret_cast<BsplineOptimizer*>(func_data);
  double            cost;
  opt->combineCost(x, grad, cost);
  opt->iter_num_++;

  /* save the min cost result */
  if (cost < opt->min_cost_) {
    opt->min_cost_      = cost;
    opt->best_variable_ = x;
  }
  return cost;

  // /* evaluation */
  // ros::Time te1 = ros::Time::now();
  // double time_now = (te1 - opt->time_start_).toSec();
  // opt->vec_time_.push_back(time_now);
  // if (opt->vec_cost_.size() == 0)
  // {
  //   opt->vec_cost_.push_back(f_combine);
  // }
  // else if (opt->vec_cost_.back() > f_combine)
  // {
  //   opt->vec_cost_.push_back(f_combine);
  // }
  // else
  // {
  //   opt->vec_cost_.push_back(opt->vec_cost_.back());
  // }
}

vector<Eigen::Vector3d> BsplineOptimizer::matrixToVectors(const Eigen::MatrixXd& ctrl_pts) {
  vector<Eigen::Vector3d> ctrl_q;
  for (int i = 0; i < ctrl_pts.rows(); ++i) {
    ctrl_q.push_back(ctrl_pts.row(i));
  }
  return ctrl_q;
}

Eigen::MatrixXd BsplineOptimizer::getControlPoints() { return this->control_points_; }

bool BsplineOptimizer::isQuadratic() {
  if (cost_function_ == GUIDE_PHASE) {
    return true;
  } else if (cost_function_ == (SMOOTHNESS | WAYPOINTS)) {
    return true;
  }
  return false;
}

//B样条优化轨迹，输入优化点，间隔
bool BsplineOptimizer::BsplineOptimizeTrajRebound(Eigen::MatrixXd &optimal_points, double ts)
{
  // //设置B样条曲线的间隔
  setBsplineInterval(ts);//bspline_interval_ = ts
  // 创建一个 3*n 的新矩阵来存储转换后的数据  
  Eigen::MatrixXd transposed_points = Eigen::MatrixXd::Zero(3, optimal_points.rows());  
    
  // 遍历 optimal_points 的每一行，并将其作为新矩阵的一列  
  for (int i = 0; i < optimal_points.rows(); ++i) {  
      transposed_points.col(i) = optimal_points.row(i);  
  }  
  cps_.points = transposed_points;
  cps_.size = cps_.points.cols();//点的个数
  //打印cps_.points的行和列数
  cout<<"cps_.points的行和列数"<<cps_.points.rows()<<" "<<cps_.points.cols()<<" "<<cps_.size<<endl;
  


  //优化成功后，将优化后的曲线控制点保存到给定的矩阵中
  bool flag_success =rebound_optimize(optimal_points);

  Eigen::MatrixXd transposed_opt_points = Eigen::MatrixXd::Zero(optimal_points.rows(),3 );  
    
  // 遍历 optimal_points 的每一行，并将其作为新矩阵的一列  
  for (int i = 0; i < 3; ++i) {  
      transposed_opt_points.col(i) = cps_.points.row(i);  
  } 
  
  optimal_points = transposed_opt_points;
  cout<<"optimal_points的行和列数"<<optimal_points.rows()<<" "<<optimal_points.cols()<<endl;

  return true;//flag_success;
}

  bool BsplineOptimizer::rebound_optimize(Eigen::MatrixXd &optimal_points)
  {
    iter_num_ = 0;
    int start_id = order_;
    int end_id = optimal_points.rows() - order_;//优化点个数-3
    variable_num_ = 3 * (end_id - start_id);//3*这么多待优化的点
    double final_cost;
  
    cout << "start_id=" << start_id << ",end_id=" << end_id << ",variable_num_=" << variable_num_ << endl;

    // 创建一个 3*n 的新矩阵来存储转换后的数据  
    Eigen::MatrixXd transposed_points = Eigen::MatrixXd::Zero(3, optimal_points.rows());  
      
    // 遍历 optimal_points 的每一行，并将其作为新矩阵的一列  
    for (int i = 0; i < optimal_points.rows(); ++i) {  
        transposed_points.col(i) = optimal_points.row(i);  
    }  

    ros::Time t0 = ros::Time::now(), t1, t2;
    int restart_nums = 0, rebound_times = 0;
    bool flag_force_return, flag_occ, success;
    new_lambda2_ = lambda2_;
    constexpr int MAX_RESART_NUMS_SET = 3;
    do
    {
      /* ---------- prepare ---------- */
      min_cost_ = std::numeric_limits<double>::max();
      iter_num_ = 0;
      flag_force_return = false;
      flag_occ = false;
      success = false;

      double q[variable_num_];
      
      

      memcpy(q, transposed_points.data() + 3 * start_id, variable_num_ * sizeof(q[0]));
      cout<<"待优化变量q\n";
      for(double q_var:q){
        cout<<q_var<<" ";
      }
      //设置参数
      lbfgs::lbfgs_parameter_t lbfgs_params;
      lbfgs::lbfgs_load_default_parameters(&lbfgs_params);
      lbfgs_params.mem_size = 16;
      lbfgs_params.max_iterations = 200;
      lbfgs_params.g_epsilon = 0.01;

      /* ---------- optimize ---------- */
      cout<<"启动优化"<< endl;
      t1 = ros::Time::now();
      // cout<<"before optimize cps_point is \n"<<cps_.points << endl;
      cout<<"final_cost before is "<<final_cost<<endl;

      int result = lbfgs::lbfgs_optimize(variable_num_, q, &final_cost, BsplineOptimizer::costFunctionRebound, NULL, BsplineOptimizer::earlyExit, this, &lbfgs_params);
      //点的数目，点的坐标，cost，优化函数，
      // cout<<"cps_point is \n"<<cps_.points << endl;
      //输出final_cost
      cout<<"final_cost is "<<final_cost<<endl;
      t2 = ros::Time::now();
      double time_ms = (t2 - t1).toSec() * 1000;
      double total_time_ms = (t2 - t0).toSec() * 1000;

    } while ((flag_occ && restart_nums < MAX_RESART_NUMS_SET) ||
             (flag_force_return && /*force_stop_type_ == STOP_FOR_REBOUND && */rebound_times <= 20));
    cout<<"优化成功"<< endl;

    return success;
  }

  //调用combineCostRebound(x,grad,cost,n) 返回cost代价
  double BsplineOptimizer::costFunctionRebound(void *func_data, const double *x, double *grad, const int n)
  {
    // 将传入的函数数据转换为BsplineOptimizer指针
    BsplineOptimizer *opt = reinterpret_cast<BsplineOptimizer *>(func_data);
    // 调用combineCostRebound函数计算优化目标函数的值和梯度
    double cost;
    opt->combineCostRebound(x, grad, cost, n);

    opt->iter_num_ += 1;
    return cost;
  }
  //求得轨迹的光滑项、碰撞项、动力学可行项的加权和f_combine以及梯度方向grad
  void BsplineOptimizer::combineCostRebound(const double *x, double *grad, double &f_combine, const int n)
  {
    // // 更新控制点位置!!!!
    memcpy(cps_.points.data() + 3 * order_, x, n * sizeof(x[0]));

    /* ---------- evaluate cost and gradient ---------- */
    double f_smoothness, f_distance, f_feasibility,f_swing;
    
    Eigen::MatrixXd g_smoothness = Eigen::MatrixXd::Zero(3, cps_.size);
    Eigen::MatrixXd g_distance = Eigen::MatrixXd::Zero(3, cps_.size);
    Eigen::MatrixXd g_feasibility = Eigen::MatrixXd::Zero(3, cps_.size);
    Eigen::MatrixXd g_swing = Eigen::MatrixXd::Zero(3, cps_.size);

    calcSmoothnessCost(cps_.points, f_smoothness, g_smoothness);// 计算平滑度代价和梯度
    calcDistanceCostRebound(cps_.points, f_distance, g_distance, iter_num_, f_smoothness);// 计算距离代价和梯度
    calcFeasibilityCost(cps_.points, f_feasibility, g_feasibility);// 计算可行性代价和梯度
    calcSwingCost(cps_.points, f_swing, g_swing);// 计算可行性代价和梯度
    // 组合成最终的代价函数值  
    lambda1_ = 1;
    new_lambda2_ = 1;
    lambda3_ = 1;
    lambda4_ = 1;

    f_combine = lambda1_ * f_smoothness + new_lambda2_ * f_distance + lambda3_ * f_feasibility + lambda4_ * f_swing;  
    printf("origin %f %f %f %f %f\n", f_smoothness, f_distance, f_feasibility, f_swing, f_combine);  
  
    // 组合成最终的梯度  
    Eigen::MatrixXd grad_3D = lambda1_ * g_smoothness + new_lambda2_ * g_distance + lambda3_ * g_feasibility + lambda4_ * g_swing;  
    memcpy(grad, grad_3D.data() + 3 * order_, n * sizeof(grad[0])); // 将梯度值拷贝到grad数组中  

  }
  //计算碰撞惩罚
  void BsplineOptimizer::calcDistanceCostRebound(const Eigen::MatrixXd &q, double &cost,
                                                 Eigen::MatrixXd &gradient, int iter_num, double smoothness_cost)
  {
        //打印所以输入的行数和列数
    // cout<<"calcDistanceCostRebound"<<endl;
    // cout<<"q "<<q.rows()<<"*"<<q.cols()<<endl;
    // cout<<"gradient "<<gradient.rows()<<"*"<<gradient.cols()<<endl;
    
    cost = 0.0;  
    gradient.setZero(); // 使用setZero()方法初始化gradient为全零矩阵  
  
    double dist;  
    Eigen::Vector3d dist_grad;  
  
    int end_idx = (cost_function_ & ENDPOINT) ? q.cols() : q.cols() - order_;  
  
    for (int i = order_; i < end_idx; ++i) {  
        // 提取q中的第i列作为当前点  
        Eigen::Vector3d current_point = q.col(i);  
          
        // 调用EDT环境的评估函数  
        edt_environment_->evaluateEDTWithGrad(current_point, -1.0, dist, dist_grad);  
          
        // 如果梯度的范数大于阈值，则归一化  
        if (dist_grad.norm() > 1e-4) {  
            dist_grad.normalize();  
        }  
          
        // 根据条件累加代价到总代价  
        if (dist < dist0_) {  
            cost += pow(dist - dist0_, 2);  
            // 将当前点的梯度累加到gradient的对应列  
            gradient.col(i) += 2.0 * (dist - dist0_) * dist_grad;  
        }  
    }  
}

//计算适应项代价
  void BsplineOptimizer::calcFitnessCost(const Eigen::MatrixXd &q, double &cost, Eigen::MatrixXd &gradient)
  {

  //   cost = 0.0;

  //   int end_idx = q.cols() - order_;//计算B-spline曲线的结束索引

  //   // def: f = |x*v|^2/a^2 + |x×v|^2/b^2
  //   double a2 = 25, b2 = 1;
  //   for (auto i = order_ - 1; i < end_idx + 1; ++i)//遍历B-spline曲线的每个点。
  //   {
  //     //计算当前点的坐标和速度
  //     Eigen::Vector3d x = (q.col(i - 1) + 4 * q.col(i) + q.col(i + 1)) / 6.0 - ref_pts_[i - 1];
  //     Eigen::Vector3d v = (ref_pts_[i] - ref_pts_[i - 2]).normalized();
  //     //计算x和v的点积
  //     double xdotv = x.dot(v);
  //     Eigen::Vector3d xcrossv = x.cross(v);//计算x和v的叉积。

  //     double f = pow((xdotv), 2) / a2 + pow(xcrossv.norm(), 2) / b2;
  //     cost += f;

  //     Eigen::Matrix3d m;
  //     m << 0, -v(2), v(1), v(2), 0, -v(0), -v(1), v(0), 0;
  //     Eigen::Vector3d df_dx = 2 * xdotv / a2 * v + 2 / b2 * m * xcrossv;

  //     gradient.col(i - 1) += df_dx / 6;
  //     gradient.col(i) += 4 * df_dx / 6;
  //     gradient.col(i + 1) += df_dx / 6;
  //   }
  }
//计算平滑代价，对应论文中Smoothness Penalty部分公式(4)
  void BsplineOptimizer::calcSmoothnessCost(const Eigen::MatrixXd &q, double &cost,
                                            Eigen::MatrixXd &gradient, bool falg_use_jerk /* = true*/)
  {

    cost = 0.0;//初始化cost = 0
    // ROS_INFO("calcSmoothnessCost");
    if (falg_use_jerk)//判断是否使用jerk优化
    {
      Eigen::Vector3d jerk, temp_j;
      // ROS_INFO("calcSmoothnessCost use ierk");

      for (int i = 0; i < q.cols() - 3; i++)//循环遍历控制点矩阵的每一列（从0开始）。
      {
        /* evaluate jerk */
        jerk = q.col(i + 3) - 3 * q.col(i + 2) + 3 * q.col(i + 1) - q.col(i);
        cost += jerk.squaredNorm();
        temp_j = 2.0 * jerk;
        /* jerk gradient */
        gradient.col(i + 0) += -temp_j;
        gradient.col(i + 1) += 3.0 * temp_j;
        gradient.col(i + 2) += -3.0 * temp_j;
        gradient.col(i + 3) += temp_j;
      }
    }
    else//不使用jerk优化
    {
      Eigen::Vector3d acc, temp_acc;
      // ROS_INFO("calcSmoothnessCost not use ierk");

      for (int i = 0; i < q.cols() - 2; i++)//循环遍历控制点矩阵的每一列（从0开始）。
      {
        /* evaluate acc */
        acc = q.col(i + 2) - 2 * q.col(i + 1) + q.col(i);
        cost += acc.squaredNorm();//二范数的平方
        temp_acc = 2.0 * acc;
        /* acc gradient *///central difference formula（中心差分法）来计算acceleration的梯度
        gradient.col(i + 0) += temp_acc;
        gradient.col(i + 1) += -2.0 * temp_acc;
        gradient.col(i + 2) += temp_acc;
      }
    }
  }
 //计算可行性代价，对应论文中Feasibility Penalty部分公式(8)(9)(10)
  void BsplineOptimizer::calcFeasibilityCost(const Eigen::MatrixXd &q, double &cost,
                                             Eigen::MatrixXd &gradient)
  {

    //#define SECOND_DERIVATIVE_CONTINOUS

#ifdef SECOND_DERIVATIVE_CONTINOUS

    cost = 0.0;
    double demarcation = 1.0; // 1m/s, 1m/s/s
    double ar = 3 * demarcation, br = -3 * pow(demarcation, 2), cr = pow(demarcation, 3);
    double al = ar, bl = -br, cl = cr;

    /* abbreviation */
    double ts, ts_inv2, ts_inv3;
    ts = bspline_interval_;
    ts_inv2 = 1 / ts / ts;
    ts_inv3 = 1 / ts / ts / ts;

    /* velocity feasibility */
    for (int i = 0; i < q.cols() - 1; i++)
    {
      Eigen::Vector3d vi = (q.col(i + 1) - q.col(i)) / ts;

      for (int j = 0; j < 3; j++)
      {
        if (vi(j) > max_vel_ + demarcation)
        {
          double diff = vi(j) - max_vel_;
          cost += (ar * diff * diff + br * diff + cr) * ts_inv3; // multiply ts_inv3 to make vel and acc has similar magnitude

          double grad = (2.0 * ar * diff + br) / ts * ts_inv3;
          gradient(j, i + 0) += -grad;
          gradient(j, i + 1) += grad;
        }
        else if (vi(j) > max_vel_)
        {
          double diff = vi(j) - max_vel_;
          cost += pow(diff, 3) * ts_inv3;
          ;

          double grad = 3 * diff * diff / ts * ts_inv3;
          ;
          gradient(j, i + 0) += -grad;
          gradient(j, i + 1) += grad;
        }
        else if (vi(j) < -(max_vel_ + demarcation))
        {
          double diff = vi(j) + max_vel_;
          cost += (al * diff * diff + bl * diff + cl) * ts_inv3;

          double grad = (2.0 * al * diff + bl) / ts * ts_inv3;
          gradient(j, i + 0) += -grad;
          gradient(j, i + 1) += grad;
        }
        else if (vi(j) < -max_vel_)
        {
          double diff = vi(j) + max_vel_;
          cost += -pow(diff, 3) * ts_inv3;

          double grad = -3 * diff * diff / ts * ts_inv3;
          gradient(j, i + 0) += -grad;
          gradient(j, i + 1) += grad;
        }
        else
        {
          /* nothing happened */
        }
      }
    }

    /* acceleration feasibility */
    for (int i = 0; i < q.cols() - 2; i++)
    {
      Eigen::Vector3d ai = (q.col(i + 2) - 2 * q.col(i + 1) + q.col(i)) * ts_inv2;

      for (int j = 0; j < 3; j++)
      {
        if (ai(j) > max_acc_ + demarcation)
        {
          double diff = ai(j) - max_acc_;
          cost += ar * diff * diff + br * diff + cr;

          double grad = (2.0 * ar * diff + br) * ts_inv2;
          gradient(j, i + 0) += grad;
          gradient(j, i + 1) += -2 * grad;
          gradient(j, i + 2) += grad;
        }
        else if (ai(j) > max_acc_)
        {
          double diff = ai(j) - max_acc_;
          cost += pow(diff, 3);

          double grad = 3 * diff * diff * ts_inv2;
          gradient(j, i + 0) += grad;
          gradient(j, i + 1) += -2 * grad;
          gradient(j, i + 2) += grad;
        }
        else if (ai(j) < -(max_acc_ + demarcation))
        {
          double diff = ai(j) + max_acc_;
          cost += al * diff * diff + bl * diff + cl;

          double grad = (2.0 * al * diff + bl) * ts_inv2;
          gradient(j, i + 0) += grad;
          gradient(j, i + 1) += -2 * grad;
          gradient(j, i + 2) += grad;
        }
        else if (ai(j) < -max_acc_)
        {
          double diff = ai(j) + max_acc_;
          cost += -pow(diff, 3);

          double grad = -3 * diff * diff * ts_inv2;
          gradient(j, i + 0) += grad;
          gradient(j, i + 1) += -2 * grad;
          gradient(j, i + 2) += grad;
        }
        else
        {
          /* nothing happened */
        }
      }
    }

#else

    cost = 0.0;
    /* abbreviation */
    double ts, /*vm2, am2, */ ts_inv2;
    // vm2 = max_vel_ * max_vel_;
    // am2 = max_acc_ * max_acc_;

    ts = bspline_interval_;
    ts_inv2 = 1 / ts / ts;

    /* velocity feasibility */
    for (int i = 0; i < q.cols() - 1; i++)//i=1到Nc,对应论文中Feasibility Penalty部分公式(8)
    {
      Eigen::Vector3d vi = (q.col(i + 1) - q.col(i)) / ts;//对应论文中公式(2)

      //cout << "temp_v * vi=" ;
      for (int j = 0; j < 3; j++)//xyz三轴上分别计算速度
      {
        if (vi(j) > max_vel_)//每个轴上的速度>最大速度
        {
          // cout << "fuck VEL" << endl;
          // cout << vi(j) << endl;
          cost += pow(vi(j) - max_vel_, 2) * ts_inv2; // multiply ts_inv3 to make vel and acc has similar magnitude

          gradient(j, i + 0) += -2 * (vi(j) - max_vel_) / ts * ts_inv2;
          gradient(j, i + 1) += 2 * (vi(j) - max_vel_) / ts * ts_inv2;
        }
        else if (vi(j) < -max_vel_)
        {
          cost += pow(vi(j) + max_vel_, 2) * ts_inv2;

          gradient(j, i + 0) += -2 * (vi(j) + max_vel_) / ts * ts_inv2;
          gradient(j, i + 1) += 2 * (vi(j) + max_vel_) / ts * ts_inv2;
        }
        else
        {
          /* code */
        }
      }
    }

    /* acceleration feasibility */
    for (int i = 0; i < q.cols() - 2; i++)//i=1到Nc-1,对应论文中Feasibility Penalty部分公式(8)
    {
      Eigen::Vector3d ai = (q.col(i + 2) - 2 * q.col(i + 1) + q.col(i)) * ts_inv2;

      //cout << "temp_a * ai=" ;
      for (int j = 0; j < 3; j++)//计算xyz每个轴上的加速度
      {
        if (ai(j) > max_acc_)//加速度>最大加速度
        {
          // cout << "fuck ACC" << endl;
          // cout << ai(j) << endl;
          cost += pow(ai(j) - max_acc_, 2);

          gradient(j, i + 0) += 2 * (ai(j) - max_acc_) * ts_inv2;
          gradient(j, i + 1) += -4 * (ai(j) - max_acc_) * ts_inv2;
          gradient(j, i + 2) += 2 * (ai(j) - max_acc_) * ts_inv2;
        }
        else if (ai(j) < -max_acc_)
        {
          cost += pow(ai(j) + max_acc_, 2);

          gradient(j, i + 0) += 2 * (ai(j) + max_acc_) * ts_inv2;
          gradient(j, i + 1) += -4 * (ai(j) + max_acc_) * ts_inv2;
          gradient(j, i + 2) += 2 * (ai(j) + max_acc_) * ts_inv2;
        }
        else
        {
          /* code */
        }
      }
      //cout << endl;
    }

#endif
  }
void BsplineOptimizer::calcSwingCost(const Eigen::MatrixXd &q, double &cost, Eigen::MatrixXd &gradient) {
    // // 检查输入参数 q 的有效性
    // if (q.cols() < 4) {
    //     std::cerr << "Input matrix q should have at least 4 columns." << std::endl;
    //     cost = 0.0;
    //     gradient.setZero();
    //     return;
    // }
    // cost = 0.0;
    // // 1. 由 q 计算各点的速度和加速度
    // int n = q.cols();
    // Eigen::MatrixXd v(3, n);
    // Eigen::MatrixXd a(3, n);
    // // 第一个速度值通过外推得到
    // v.col(0) = 2 * (q.col(1) - q.col(0)) / bspline_interval_ - v.col(1);
    
    // for (int i = 1; i < n - 1; i++) {
    //     v.col(i) = (q.col(i + 1) - q.col(i - 1)) / (2 * bspline_interval_);
    // }
    // // 最后一个速度值通过外推得到
    // v.col(n - 1) = 2 * (q.col(n - 1) - q.col(n - 2)) / bspline_interval_ - v.col(n - 2);
    
    // for (int i = 1; i < n - 1; i++) {
    //     a.col(i) = (q.col(i + 1) - 2 * q.col(i) + q.col(i - 1)) / (bspline_interval_ * bspline_interval_);
    // }
    
    // // 第一个和最后一个加速度值通过外推得到
    // a.col(0) = 2 * (a.col(1) - a.col(2)) / bspline_interval_;
    // a.col(n - 1) = 2 * (a.col(n - 2) - a.col(n - 3)) / bspline_interval_;
    // // 2. 计算摆角
    // double g = 9.81; // 重力加速度
    // // 计算摆角
    // Eigen::MatrixXd ge_3(3, 1);
    // ge_3 << 0, 0, g; // 重力向量
    // Eigen::MatrixXd  angle(n, 1);
    // Eigen::MatrixXd angle_Vec(3, n);

    // for (int i = 0; i < n; i++) {
    //     Eigen::VectorXd acc_with_g = a.col(i) + ge_3;
    //     double norm = acc_with_g.norm(); // 计算模长
    //     if (norm > 0) { // 避免除以0
    //         //double rad = atan2(acc_with_g(2), acc_with_g.head<2>().norm()); // 计算摆角（弧度）
    //         double rad = atan2(acc_with_g(2), norm); // 计算偏离z轴的角度（弧度）
    //         angle(i, 0) = rad * (180.0 / M_PI); // 将弧度转换为角度
    //         angle_Vec.col(i) = acc_with_g/norm;

    //     } else {
    //         //norm为0，则angle_Vec是0向量
    //         angle_Vec.col(i)  = Eigen::Vector3d::Zero();
    //         angle(i, 0) = 0; // 如果模长为0，摆角设为0
    //     }
    // }
    // std::cout << "q: " << q << std::endl;
    // std::cout << "v: " << v << std::endl;
    // std::cout << "a: " << a << std::endl;
    // std::cout << "angle: " << angle << std::endl;
    // // 3. 定义摆角阈值
    // double swing_angle_threshold = 10.0; // 10度
    // // 4. 计算摆角代价和梯度信息
    // calculateSwingCostAndGradient(q,v , a, angle, swing_angle_threshold, cost, gradient);

    // // for (int i = 0; i < n-3; i++) {
    // //   //角度的绝对值大于阈值，则加上差值的平方
    // //   if (fabs(angle(i, 0)) > swing_angle_threshold) {
    // //     double err = fabs(angle(i, 0)) - swing_angle_threshold;
    // //     cost += err * err;
    // //     double temp_j = 2.0 * err;
    // //     /* jerk gradient */

    // //     gradient.col(i + 0) += -temp_j;
    // //     gradient.col(i + 1) += 3.0 * temp_j;
    // //     gradient.col(i + 2) += -3.0 * temp_j;
    // //     gradient.col(i + 3) += temp_j;
    // //     }
    // // }

    // // 处理可能的数值不稳定情况
    // if (!std::isfinite(cost)) {
    //     std::cerr << "Calculated cost is not finite. Resetting to 0." << std::endl;
    //     cost = 0.0;
    // }

    // 检查输入参数 q 的有效性
    if (q.cols() < 4) {
        std::cerr << "Input matrix q should have at least 4 columns." << std::endl;
        cost = 0.0;
        gradient.setZero();
        return;
    }
    cost = 0.0;
    // 1. 由 q 计算各点的速度和加速度
    int n = q.cols();
    Eigen::MatrixXd v(3, n);
    Eigen::MatrixXd a(3, n);

    for (int i = 1; i < n-1; i++) {
        v.col(i) = (q.col(i+1) - q.col(i)) / (bspline_interval_);
    }
    // 第一个速度值为局部规划的值
    v.col(0) = 2 * (q.col(1) - q.col(0)) / bspline_interval_ - v.col(1);
    // 最后一个速度值通过外推得到
    v.col(n - 1) = 2 * (q.col(n - 1) - q.col(n - 2)) / bspline_interval_ - v.col(n - 2);
    
    for (int i = 1; i < n - 1; i++) {
        a.col(i) = (v.col(i + 1) - v.col(i))/bspline_interval_;
    }
    // 第一个和最后一个加速度值通过外推得到
    a.col(0) = 2 * (a.col(1) - a.col(2)) / bspline_interval_;
    a.col(n - 1) = 2 * (a.col(n - 2) - a.col(n - 3)) / bspline_interval_;
    
    // 2. 计算摆角
    double g = 9.81; // 重力加速度
    // 计算摆角
    Eigen::MatrixXd ge_3(3, 1);
    ge_3 << 0, 0, g; // 重力向量
    Eigen::MatrixXd  angle(n, 1);
    Eigen::MatrixXd angle_Vec(3, n);

    for (int i = 0; i < n; i++) {
        Eigen::VectorXd acc_with_g = a.col(i) + ge_3;
        double norm = acc_with_g.norm(); // 计算模长
        if (norm > 0) { // 避免除以0
            //double rad = atan2(acc_with_g(2), acc_with_g.head<2>().norm()); // 计算摆角（弧度）
            double rad = atan2(acc_with_g(2), norm); // 计算偏离z轴的角度（弧度）
            angle(i, 0) = rad * (180.0 / M_PI); // 将弧度转换为角度
            angle_Vec.col(i) = acc_with_g/norm;

        } else {
            //norm为0，则angle_Vec是0向量
            angle_Vec.col(i)  = Eigen::Vector3d::Zero();
            angle(i, 0) = 0; // 如果模长为0，摆角设为0
        }
    }
    std::cout << "q: " << q << std::endl;
    std::cout << "v: " << v << std::endl;
    std::cout << "a: " << a << std::endl;
    std::cout << "angle: " << angle << std::endl;
    // 3. 定义摆角阈值
    double swing_angle_threshold = 10.0; // 10度
    // 4. 计算摆角代价和梯度信息
    calculateSwingCostAndGradient(q,v , a, angle, swing_angle_threshold, cost, gradient);

    // for (int i = 0; i < n-3; i++) {
    //   //角度的绝对值大于阈值，则加上差值的平方
    //   if (fabs(angle(i, 0)) > swing_angle_threshold) {
    //     double err = fabs(angle(i, 0)) - swing_angle_threshold;
    //     cost += err * err;
    //     double temp_j = 2.0 * err;
    //     /* jerk gradient */

    //     gradient.col(i + 0) += -temp_j;
    //     gradient.col(i + 1) += 3.0 * temp_j;
    //     gradient.col(i + 2) += -3.0 * temp_j;
    //     gradient.col(i + 3) += temp_j;
    //     }
    // }

    // 处理可能的数值不稳定情况
    if (!std::isfinite(cost)) {
        std::cerr << "Calculated cost is not finite. Resetting to 0." << std::endl;
        cost = 0.0;
    }
}


// 计算摆角代价和梯度信息的函数
void BsplineOptimizer::calculateSwingCostAndGradient(const Eigen::MatrixXd &q, const Eigen::MatrixXd &v,const Eigen::MatrixXd &a, const Eigen::MatrixXd &angle,
                                    const double swing_angle_threshold, double &cost, Eigen::MatrixXd &gradient) {
    cost = 0.0;
    gradient.setZero(q.rows(), q.cols());
    int n = q.cols();
    // 计算代价函数对摆角的导数以及根据链式法则计算梯度
    for (int i = 0; i < angle.cols(); i++) {
        // 计算代价函数对摆角的导数
        double dC_dtheta_i;
        if (angle(i, 0) >= 0) {
            dC_dtheta_i = 2 * (angle(i, 0) - swing_angle_threshold);
        } else {
            dC_dtheta_i = 2 * (-angle(i, 0) - swing_angle_threshold);
        }

        // 计算摆角对加速度各元素的导数
        Eigen::MatrixXd dtheta_i_da(3, n);
        calculate_dtheta_i_da(v, a, angle, i, dtheta_i_da);

        // 计算加速度各元素对q各元素的导数
        Eigen::MatrixXd da_jk_dq(q.rows(), q.cols());
        calculate_da_jk_dq(q,v, a, da_jk_dq);

        // 根据链式法则更新梯度
        for (int j = 0; j < q.rows(); j++) {
            for (int k = 0; k < q.cols(); k++) {
                gradient(j, k) += dC_dtheta_i * dtheta_i_da(j, k) * da_jk_dq(j, k);
            }
        }

        // 如果摆角的绝对值大于阈值，则加上差值的平方到代价中
        if (fabs(angle(i, 0)) > swing_angle_threshold) {
            double err = fabs(angle(i, 0)) - swing_angle_threshold;
            cost += err * err;
        }
    }
}

// 计算摆角对加速度各元素的导数的函数
void BsplineOptimizer::calculate_dtheta_i_da(const Eigen::MatrixXd &v,const Eigen::MatrixXd &a, const Eigen::MatrixXd &angle, int i, Eigen::MatrixXd &dtheta_i_da) {
    int n = a.cols();
    double g = 9.81; // 重力加速度
    // 计算摆角
    Eigen::MatrixXd ge_3(3, 1);
    ge_3 << 0, 0, g; // 重力向量
    Eigen::VectorXd acc_with_g = a.col(i) + ge_3;
    double norm = acc_with_g.norm();

    if (norm > 0) {
        // 根据摆角的具体计算方式求导
        // 这里是基于atan2函数计算摆角的情况，求导过程相对复杂
        // 以下是一个简化的示例推导，实际可能需要更精确的计算
        double z_component = acc_with_g(2);
        double x_y_norm = acc_with_g.head<2>().norm();

        // 先求atan2对其参数的导数
        double d_atan2_dz = x_y_norm / (z_component * z_component + x_y_norm * x_y_norm);
        double d_atan2_dxy = -z_component / (z_component * z_component + x_y_norm * x_y_norm);

        // 然后根据链式法则求摆角对加速度各元素的导数
        for (int j = 0; j < 3; j++) {
            if (j == 2) {
                dtheta_i_da(j, i) = d_atan2_dz;
            } else {
                dtheta_i_da(j, i) = d_atan2_dxy * (acc_with_g(j) / x_y_norm);
            }
        }
    } else {
        dtheta_i_da.setZero();
    }
}

// 计算加速度各元素对q各元素的导数的函数
void BsplineOptimizer::calculate_da_jk_dq(const Eigen::MatrixXd &q, const Eigen::MatrixXd &v, const Eigen::MatrixXd &a, Eigen::MatrixXd &da_jk_dq) {
    int n = q.cols();
    // 计算加速度各元素对q各元素的导数
    for (int j = 0; j < 3; j++) {
        for (int k = 0; k < n; k++) {
            if (k == 0) {
                // 第一个加速度值通过外推得到
                da_jk_dq(j, k) = 2 * (v(j, 1) - v(j, 2)) / bspline_interval_;
            } else if (k == n - 1) {
                // 最后一个加速度值通过外推得到
                da_jk_dq(j, k) = 2 * (v(j, n - 2) - v(j, n - 3)) / bspline_interval_;
            } else {
                da_jk_dq(j, k) = (q(j, k + 1) - 2 * q(j, k) + q(j, k - 1)) / (bspline_interval_ * bspline_interval_);
            }
        }
    }
}


//  //计算摆角代价，对应
//   void BsplineOptimizer::calcSwingCost(const Eigen::MatrixXd &q, double &cost, Eigen::MatrixXd &gradient){// 计算可行性代价和梯度
//     cost = 0.0;//初始化cost = 0
//     std::cout << "calcSwingCost" << std::endl;
//     //1. 由q计算各点的速度和加速度
//     int point_num = q.cols();
//     Eigen::MatrixXd v(3, point_num);//速度矩阵
//     Eigen::MatrixXd a(3, point_num);//加速度矩阵
//     // 计算速度矩阵  
//     for (int i = 0; i < point_num; ++i) {  
//         if (i == 0 ||i == point_num - 1){ // 起点和终点  
//             v.col(i).setZero();
//         } else {  
//             // 假设internal是时间间隔，需要作为类的成员变量传入或定义  
//             double time_interval = bspline_interval_; // 替换为实际的时间间隔值  
//             v.col(i) = (q.col(i + 1) - q.col(i)) / time_interval;  
//         }  
//     }  
      
//     // 计算加速度矩阵  
//     for (int i = 0; i < point_num; ++i) {  
//         // 使用已计算的速度来计算加速度
//         if (i == 0 ||i == point_num - 1){ // 起点和终点  
//             a.col(i).setZero();
//         } else {  
//             a.col(i) = (v.col(i + 1) - v.col(i)) / bspline_interval_;  
//         }  
//         a.col(i) = (v.col(i + 1) - v.col(i)) / bspline_interval_; // 假设internal是时间间隔  
//     }  
//     //分别打印每个点的位置速度和加速度
//     cout<<"pos"<<endl;
//     for(int i=0;i<q.cols();i++){
//       cout << "position "<<i<<": " << q.col(i).transpose()<< endl;
//     }
//     cout<<"vel"<<endl;
//     for(int i=0;i<q.cols();i++){
//       cout << "vel "<<i<<": " << v.col(i).transpose()<< endl;
//     }
//     cout<<"acc"<<endl;
//     for(int i=0;i<q.cols();i++){
//       cout << "acc "<<i<<": " << a.col(i).transpose()<< endl;
//     }


//     //2. 计算摆角
//     Eigen::MatrixXd swing_angle(3, q.cols());//摆角矩阵
//     Eigen::Vector3d gravity_vector(0, 0, 9.8); 

//     for (int i = 0; i < point_num; i++) {  
//         Eigen::Vector3d direction = -a.col(i);  //报错
//         direction += gravity_vector;
//         swing_angle.col(i) = direction.normalized();  
//     }
//     cout<<"acc"<<endl;
//     for(int i=0;i<q.cols();i++){
//       cout << "q "<<i<<": " << a.col(i).transpose()<< endl;
//     }

//     //3.定义摆角阈值
//     Eigen::Vector3d swing_angle_threshold(0.1,0.1,0.1);//摆角阈值
    
//     //4.计算摆角代价和梯度信息
//     Eigen::Vector3d q_error, temp_j;

//     for (int i = 0; i < swing_angle.cols() - 3; i++)//循环遍历控制点矩阵的每一列（从0开始）。
//     {
//       /* evaluate jerk */
//       q_error = swing_angle.col(i)-swing_angle_threshold;
//       cost += q_error.squaredNorm();
//       temp_j = 2.0 * q_error;
//       /* jerk gradient */
//       gradient.col(i + 0) += -temp_j;
//       gradient.col(i + 1) += 3.0 * temp_j;
//       gradient.col(i + 2) += -3.0 * temp_j;
//       gradient.col(i + 3) += temp_j;
//     }
//   }
  //如果force_stop_type_不为DONT_STOP就返回true，否则返回false。
  int BsplineOptimizer::earlyExit(void *func_data, const double *x, const double *g, const double fx, const double xnorm, const double gnorm, const double step, int n, int k, int ls)
  {
    BsplineOptimizer *opt = reinterpret_cast<BsplineOptimizer *>(func_data);
    // cout << "k=" << k << endl;
    // cout << "opt->flag_continue_to_optimize_=" << opt->flag_continue_to_optimize_ << endl;
    return (opt->force_stop_type_ == STOP_FOR_ERROR || opt->force_stop_type_ == STOP_FOR_REBOUND);
  }
  
}  // namespace fast_planner