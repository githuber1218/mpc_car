///////////////////////////////////
//mpc_tracker  2022.7.26
//Xie Yunhang
//////////////////////////////////

#ifndef _MPC_TRACKER_
#define _MPC_TRACKER_

#include "Eigen/Eigen"
#include <eigen3/unsupported/Eigen/KroneckerProduct>
#include <iostream>
#include "OsqpEigen/OsqpEigen.h"
#include "ros/ros.h"
#include "math.h"
#include <nav_msgs/Path.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_broadcaster.h>
#include <tf/tf.h>


class mpc
{
  private:
    int Nx = 3;
    int Nu = 2;
    int Np = 20;
    int Nc = 10;
    int row = 10;
    double dt = 0.02;
    Eigen::MatrixXd Q = Eigen::MatrixXd::Identity(Np*Nx, Np*Nx);
    Eigen::MatrixXd R = Eigen::MatrixXd::Identity(Nu*Nc, Nu*Nc);

    Eigen::VectorXd state_min, state_max, u_min, u_max, delta_umin, delta_umax;

    Eigen::VectorXd U;
    //error flag vector
    Eigen::VectorXd error;

    ros::Publisher ref_state_pub;
    ros::Publisher pre_state_pub;

  public:
    mpc(std::string name = "mpc_controller"){
      U.resize(2);
      U << 0, 0;
      state_min.resize(3);
      state_max.resize(3);
      state_min << -1000,-1000,-1000;
      state_max << 1000,1000,1000;
      u_min.resize(2);
      u_max.resize(2);
      u_min << -1000 , -1000;
      u_max << 1000, 1000;
      delta_umin.resize(2);
      delta_umax.resize(2);
      delta_umin << -1000, -1000;
      delta_umax << 1000, 1000;
      error.resize(1);
      error << -1000;

      /////private node for Visualization/////
      ros::NodeHandle private_nh("~/" + name);
      ref_state_pub = private_nh.advertise<nav_msgs::Path>("mpc_ref_traj", 1);
      pre_state_pub = private_nh.advertise<nav_msgs::Path>("mpc_pre_traj", 1);
    }
    ~mpc(){}
    
    // set coefficient matrix Q and R. 
    // set dt,Nx,Nu,Np,Nc,row
    void SetConfig(Eigen::MatrixXd q, Eigen::MatrixXd r,double T = 0.02, int Nx0 = 3,int Nu0 = 2,int Np0 = 20,int Nc0 = 10,int row0 = 10)
    {
      Nx = Nx0;
      Nu = Nu0;
      Np = Np0;
      Nc = Nc0;
      row = row0;
      dt = T;
      Q = Eigen::kroneckerProduct(Eigen::MatrixXd::Identity(Np,Np), q);
      R = Eigen::kroneckerProduct(Eigen::MatrixXd::Identity(Nc,Nc), r);
    }

    //set constraints of state, u, delta_u
    void SetConstraints(Eigen::VectorXd state_min0, Eigen::VectorXd state_max0, Eigen::VectorXd u_min0, Eigen::VectorXd u_max0, Eigen::VectorXd delta_umin0, Eigen::VectorXd delta_umax0)
    {
      state_min = state_min0;
      state_max = state_max0;
      u_min = u_min0;
      u_max = u_max0;
      delta_umin = delta_umin0;
      delta_umax = delta_umax0;
    }

    //get the idx of the nearest refstate point 
    int calc_target_index(const Eigen::VectorXd &state_now, const Eigen::VectorXd* refstate, const int &number)
    {
      int i,idx;
      double dist=0;
      double dist_min=10000;
      for(i=0;i<number;i++){
        dist = pow(refstate[i][0] - state_now[0], 2) + pow(refstate[i][1] - state_now[1], 2);
        if(dist<dist_min){
          dist_min = dist;
          idx = i;
          // std::cout << "dist_min:" << std::endl << dist_min << std::endl;
          // std::cout << "idx:" << std::endl << idx << std::endl;
        }
      }
      return idx; 
    }

    Eigen::VectorXd OSQPsolver(const Eigen::MatrixXd &H, Eigen::VectorXd &g, const Eigen::MatrixXd &A_cons, Eigen::VectorXd &lb, Eigen::VectorXd &ub)
    {
      // instantiate the solver
      OsqpEigen::Solver solver;

      // settings
      //solver.settings()->setVerbosity(false);
      solver.settings()->setWarmStart(true);

      //!!!must to convert H and A_cons to SparseMatrix!!!
      Eigen::SparseMatrix<double> hessian(H.rows(), H.cols());
      Eigen::SparseMatrix<double> linearMatrix(A_cons.rows(), A_cons.cols());
      for(int i=0;i<H.rows();i++)
        for(int j=0;j<H.cols();j++){
          hessian.insert(i,j) = H(i,j);
        }

      for(int i=0;i<A_cons.rows();i++)
        for(int j=0;j<A_cons.cols();j++){
          linearMatrix.insert(i,j) = A_cons(i,j);
        }

      // set the initial data of the QP solver
      //矩阵A为m*n矩阵
      solver.data()->setNumberOfVariables(A_cons.cols()); //设置A矩阵的列数，即n
      solver.data()->setNumberOfConstraints(A_cons.rows()); //设置A矩阵的行数，即m
      if(!solver.data()->setHessianMatrix(hessian)) return error;//设置P矩阵  SparseMatrix
      if(!solver.data()->setGradient(g)) return error; //设置q or f矩阵。当没有时设置为全0向量  Vector
      if(!solver.data()->setLinearConstraintsMatrix(linearMatrix)) return error;//设置线性约束的A矩阵 SparseMatrix
      if(!solver.data()->setLowerBound(lb)) return error;//设置下边界  Vector
      if(!solver.data()->setUpperBound(ub)) return error;//设置上边界  Vector

      // instantiate the solver
      if(!solver.initSolver()) return error;

      Eigen::VectorXd QPSolution;

      // solve the QP problem
      if(!solver.solve()) return error;

      // get the controller input
      QPSolution = solver.getSolution();

      std::cout << "QPSolution:" << std::endl << QPSolution << std::endl;    
      return QPSolution;
    }

    void Visualization(const Eigen::VectorXd &Y_ref, const Eigen::VectorXd &Y_pre)
    {
      nav_msgs::Path mpc_ref_traj, mpc_pre_traj;
      geometry_msgs::PoseStamped pose_stamped;
      ros::Time current_time = ros::Time::now();
      int state_size = Y_ref.size()/Nx;

      mpc_ref_traj.header.stamp = current_time;
      mpc_ref_traj.header.frame_id = "map";

      for(int i=0;i<state_size;i++){
        pose_stamped.pose.position.x = Y_ref[i*Nx];
        pose_stamped.pose.position.y = Y_ref[i*Nx+1];
        pose_stamped.pose.position.z = 0;

        geometry_msgs::Quaternion goal_quat = tf::createQuaternionMsgFromYaw(Y_ref[i*Nx+2]);
        pose_stamped.pose.orientation.x = goal_quat.x;
        pose_stamped.pose.orientation.y = goal_quat.y;
        pose_stamped.pose.orientation.z = goal_quat.z;
        pose_stamped.pose.orientation.w = goal_quat.w;

        pose_stamped.header.stamp = current_time;
        pose_stamped.header.frame_id = "map";
        mpc_ref_traj.poses.push_back(pose_stamped);
      }

      current_time = ros::Time::now();
      mpc_pre_traj.header.stamp = current_time;
      mpc_pre_traj.header.frame_id = "map";

      for(int i=0;i<state_size;i++){
        pose_stamped.pose.position.x = Y_pre[i*Nx];
        pose_stamped.pose.position.y = Y_pre[i*Nx+1];
        pose_stamped.pose.position.z = 0;

        geometry_msgs::Quaternion goal_quat = tf::createQuaternionMsgFromYaw(Y_pre[i*Nx+2]);
        pose_stamped.pose.orientation.x = goal_quat.x;
        pose_stamped.pose.orientation.y = goal_quat.y;
        pose_stamped.pose.orientation.z = goal_quat.z;
        pose_stamped.pose.orientation.w = goal_quat.w;

        pose_stamped.header.stamp = current_time;
        pose_stamped.header.frame_id = "map";
        mpc_pre_traj.poses.push_back(pose_stamped);
      }

      ref_state_pub.publish(mpc_ref_traj);
      pre_state_pub.publish(mpc_pre_traj);
    }

    // Eigen::VectorXd Vector_intersection(const Eigen::VectorXd &Vector1, const Eigen::VectorXd &Vector2, const int sign)
    // {
    //   //for sign : 0 means larger, 1 means smaller.
    //   Eigen::VectorXd result;
    //   int size1 = Vector1.size();
    //   int size2 = Vector2.size();
    //   int size;

    //   if(size1<=size2){
    //     size = size1;
    //     result.resize(size2);
    //     for(int i=size;i<size2;i++)
    //       result(i) = Vector2(i);
    //   }else{
    //     size = size2;
    //     result.resize(size1);
    //     for(int i=size;i<size1;i++)
    //       result(i) = Vector1(i);
    //   }

    //   if(sign == 0){
    //     for(int i=0;i<size;i++){
    //       if(Vector1(i)>=Vector2(i)){
    //         result(i) = Vector1(i);
    //       }else{
    //         result(i) = Vector2(i);
    //       }
    //     }
    //   }else if(sign == 1){
    //     for(int i=0;i<size;i++){
    //       if(Vector1(i)<=Vector2(i)){
    //         result(i) = Vector1(i);
    //       }else{
    //         result(i) = Vector2(i);
    //       }
    //     }
    //   }
    //   return result;
    // }

    Eigen::VectorXd getresult(const Eigen::VectorXd &state_now, int &idx, const Eigen::VectorXd* refstate, const Eigen::VectorXd* refinput, const int &number)
    {
      idx = calc_target_index(state_now, refstate, number);
      /////not reach the end point/////
      if(idx!=number-1){
        Eigen::Matrix3d a;
        Eigen::Matrix<double, 3, 2> b;
        //refstate 
        double refv = refinput[idx][0];
        double refw = refinput[idx][1];
        double refx = refstate[idx][0];
        double refy = refstate[idx][1];
        double reftheta = refstate[idx][2];

        // std::cout << "refv:" << std::endl << refv << std::endl;
        // std::cout << "refw:" << std::endl << refw << std::endl;
        // std::cout << "refx:" << std::endl << refx << std::endl;
        // std::cout << "refy:" << std::endl << refy << std::endl;
        // std::cout << "reftheta:" << std::endl << reftheta << std::endl;

        ////kinematics model////
        a << 0,0,-refv * sin(reftheta), 0, 0, refv * cos(reftheta), 0, 0, 0;
        b << cos(reftheta), 0, sin(reftheta), 0, 0, 1;

        // std::cout << "a0:" << std::endl << a << std::endl;
        // std::cout << "b0:" << std::endl << b << std::endl;

        ////discretization////
        a = a*dt + Eigen::MatrixXd::Identity(3,3);
        b = b*dt;

        // std::cout << "a:" << std::endl << a << std::endl;
        // std::cout << "b:" << std::endl << b << std::endl;

        Eigen::VectorXd kesi;
        kesi.resize(Nx+Nu);
        kesi.head(Nx) = state_now-refstate[idx];
        kesi.tail(Nu) = U;
        // std::cout << "kesi:" << std::endl << kesi << std::endl;

        Eigen::MatrixXd A;
        A.resize(Nx+Nu,Nx+Nu);
        A.topLeftCorner(Nx,Nx) = a;
        A.topRightCorner(Nx,Nu) = b;
        A.bottomLeftCorner(Nu,Nx) = Eigen::MatrixXd::Zero(Nu,Nx);
        A.bottomRightCorner(Nu,Nu) = Eigen::MatrixXd::Identity(Nu,Nu);
        // std::cout << "A:" << std::endl << A << std::endl;

        Eigen::MatrixXd B;
        B.resize(Nx+Nu,Nu);
        B << b, Eigen::MatrixXd::Identity(Nu,Nu);
        // std::cout << "B:" << std::endl << B << std::endl;

        Eigen::MatrixXd C(Nx,Nx+Nu);
        C << Eigen::MatrixXd::Identity(Nx,Nx), Eigen::MatrixXd::Zero(Nx,Nu);
        // std::cout << "C:" << std::endl << C << std::endl;
        // std::cout << "C*A:" << std::endl << C*A << std::endl;
        // std::cout << "A*B:" << std::endl << A*B << std::endl;
        // std::cout << "C*A*B:" << std::endl << C*A*B << std::endl;

        Eigen::MatrixXd PHI(Nx*Np, Nx+Nu);
        Eigen::MatrixXd A_product = A;
        for(int i=0;i<Np;i++){
          PHI.block(i*Nx,0,Nx,Nx+Nu) = C*A_product;
          A_product *= A;
        }
        // std::cout << "PHI:" << std::endl << PHI << std::endl;

        Eigen::MatrixXd THETA(Nx*Np, Nu*Nc);
        Eigen::MatrixXd A_product1 = Eigen::MatrixXd::Identity(Nx+Nu,Nx+Nu);
        for(int i=0;i<Np;i++){
          for(int j=0;j<Nc;j++){
            if(j<=i){
              for(int m=0;m<i-j;m++){
                A_product1 *= A;
              }
              THETA.block(i*Nx,j*Nu,Nx,Nu) = C*A_product1*B;
            }
            else{
              THETA.block(i*Nx,j*Nu,Nx,Nu) = Eigen::MatrixXd::Zero(Nx,Nu);
            }
            A_product1 = Eigen::MatrixXd::Identity(Nx+Nu,Nx+Nu);
          }
        }
        // std::cout << "THETA:" << std::endl << THETA << std::endl;

        Eigen::MatrixXd H(Nu*Nc+1, Nu*Nc+1);
        H.block(0,0,Nu*Nc,Nu*Nc) = THETA.transpose()*Q*THETA + R;
        H.block(0,Nu*Nc,Nu*Nc,1) = Eigen::MatrixXd::Zero(Nu*Nc,1);
        H.block(Nu*Nc,0,1,Nu*Nc) = Eigen::MatrixXd::Zero(1,Nu*Nc);
        H(Nu*Nc,Nu*Nc) = row;
        // std::cout << "H:" << std::endl << H << std::endl;

        Eigen::MatrixXd E;
        E = PHI*kesi;
        // std::cout << "E:" << std::endl << E << std::endl;

        Eigen::VectorXd g(Nu*Nc+1);
        g.block(0,0,Nu*Nc,1) = THETA.transpose()*Q.transpose()*E;
        g(Nu*Nc) = 0;
        // std::cout << "g:" << std::endl << g << std::endl;

        Eigen::MatrixXd At(Nc,Nc);
        for(int i=0;i<Nc;i++)
          for(int j=0;j<Nc;j++){
            if(j<=i)
              At(i,j) = 1;
            else
              At(i,j) = 0;
          }
        // std::cout << "At:" << std::endl << At << std::endl;

        Eigen::MatrixXd A_I(Nu*Nc, Nu*Nc);
        A_I = Eigen::kroneckerProduct(At,Eigen::MatrixXd::Identity(Nu,Nu));
        // std::cout << "A_I:" << std::endl << A_I << std::endl;
        // std::cout << "A_I's determinant:" << std::endl << A_I.determinant() << std::endl;
        // std::cout << "A_I's inverse:" << std::endl << A_I.inverse() << std::endl;

        ////constraints////
        Eigen::MatrixXd Ut(Nu*Nc, 1), Umin(Nu*Nc, 1), Umax(Nu*Nc, 1), delta_Umin(Nu*Nc, 1), delta_Umax(Nu*Nc, 1);
        Ut = Eigen::kroneckerProduct(Eigen::MatrixXd::Ones(Nc,1), U);
        Umin = Eigen::kroneckerProduct(Eigen::MatrixXd::Ones(Nc,1), u_min);
        Umax = Eigen::kroneckerProduct(Eigen::MatrixXd::Ones(Nc,1), u_max);
        delta_Umin = Eigen::kroneckerProduct(Eigen::MatrixXd::Ones(Nc,1), delta_umin);
        delta_Umax = Eigen::kroneckerProduct(Eigen::MatrixXd::Ones(Nc,1), delta_umax);

        Eigen::MatrixXd Y_min(Nx*Np, 1), Y_max(Nx*Np, 1);
        Y_min = Eigen::kroneckerProduct(Eigen::MatrixXd::Ones(Np,1), state_min);
        Y_max = Eigen::kroneckerProduct(Eigen::MatrixXd::Ones(Np,1), state_max);

        // lb <= A*X <= ub
        Eigen::MatrixXd A_cons(Nx*Np+2*Nu*Nc+1, Nu*Nc+1);
        A_cons.block(0,0,Nx*Np,Nu*Nc) = THETA;
        A_cons.block(Nx*Np,0,Nu*Nc,Nu*Nc) = A_I;
        A_cons.block(Nx*Np+Nu*Nc,0,Nu*Nc,Nu*Nc) = Eigen::MatrixXd::Identity(Nu*Nc,Nu*Nc);
        A_cons.block(0,Nu*Nc,Nx*Np+2*Nu*Nc,1) = Eigen::MatrixXd::Zero(Nx*Np+2*Nu*Nc,1);
        A_cons.block(Nx*Np+2*Nu*Nc,0,1,Nu*Nc) = Eigen::MatrixXd::Zero(1,Nu*Nc);
        A_cons(Nx*Np+2*Nu*Nc,Nu*Nc) = 1;
        // std::cout << "A_cons:" << std::endl << A_cons << std::endl;

        Eigen::VectorXd lb1(Nx*Np),lb2(Nu*Nc),lb3(Nu*Nc),lb(Nx*Np+2*Nu*Nc+1);
        lb1 << Y_min-PHI*kesi;
        lb2 << Umin-Ut;
        lb3 << delta_Umin;
        lb << lb1, lb2, lb3, 0;
        std::cout << "lb1:" << std::endl << lb1 << std::endl;
        std::cout << "lb2:" << std::endl << lb2 << std::endl;
        std::cout << "lb3:" << std::endl << lb3 << std::endl;
        std::cout << "lb:" << std::endl << lb << std::endl;
        // std::cout << "lb0:" << std::endl << lb0 << std::endl;
        // // lb1 << A_I*delta_Umin;
        // std::cout << "lb1:" << std::endl << lb1 << std::endl;
        // // lb_ = Vector_intersection(lb0,lb1,0);
        // std::cout << "lb_:" << std::endl << lb_ << std::endl;

        Eigen::VectorXd ub1(Nx*Np),ub2(Nu*Nc), ub3(Nu*Nc), ub(Nx*Np+2*Nu*Nc+1);
        ub1 << Y_max-PHI*kesi;
        ub2 << Umax-Ut;
        // std::cout << "ub0:" << std::endl << ub0 << std::endl;
        ub3 << delta_Umax;
        ub << ub1, ub2, ub3, 1;
        std::cout << "ub1:" << std::endl << ub1 << std::endl;
        std::cout << "ub2:" << std::endl << ub2 << std::endl;
        std::cout << "ub3:" << std::endl << ub3 << std::endl;
        std::cout << "ub:" << std::endl << ub << std::endl;
        // std::cout << "ub1:" << std::endl << ub1 << std::endl;
        // ub_ = Vector_intersection(ub0,ub1,1);
        // std::cout << "ub_:" << std::endl << ub_ << std::endl;

        // Eigen::VectorXd lb(Nx*Np+1), lb00(Nx*Np), lb11(Nx*Np);
        // lb00 << Y_min-PHI*kesi;
        // std::cout << "lb00:" << std::endl << lb00 << std::endl;
        // lb11 << THETA*A_I.inverse()*lb_;
        // std::cout << "A_I.inverse():" << std::endl << A_I.inverse() << std::endl;
        // std::cout << "lb_:" << std::endl << lb_ << std::endl;
        // std::cout << "A_I.inverse()*lb_:" << std::endl << A_I.inverse()*lb_ << std::endl;
        // std::cout << "THETA:" << std::endl << THETA << std::endl;
        // std::cout << "lb11:" << std::endl << lb11 << std::endl;
        // lb.head(Nx*Np) = Vector_intersection(lb00,lb11,0);
        // lb(Nx*Np) = 0;
        // std::cout << "lb:" << std::endl << lb << std::endl;

        // Eigen::VectorXd ub(Nx*Np+1), ub00(Nx*Np), ub11(Nx*Np);
        // ub00 << Y_max-PHI*kesi;
        // std::cout << "ub00:" << std::endl << ub00 << std::endl;
        // ub11 << THETA*A_I.inverse()*ub_;
        // std::cout << "ub11:" << std::endl << ub11 << std::endl;
        // ub.head(Nx*Np) = Vector_intersection(ub00,ub11,1);
        // ub(Nx*Np) = 1;
        // std::cout << "ub:" << std::endl << ub << std::endl;

        Eigen::VectorXd QPSolution;
        
        ////get the controller input////
        QPSolution = OSQPsolver(H,g,A_cons,lb,ub);  
        //if OSQP error, output the lastest U;
        if(QPSolution(0) == error(0)){
          std::cout << "!!!!!!!!!!OSQP ERROR!!!!!!!!!" << std::endl;
          QPSolution(0) = 0;
          QPSolution(1) = 0;
        }

        double delta_v = QPSolution(0);
        double delta_w = QPSolution(1);

        U(0) = kesi(Nx) + delta_v;
        U(1) = kesi(Nx+1) + delta_w;

        Eigen::VectorXd U_real(Nu);
        U_real(0) = U(0) + refv;
        U_real(1) = U(1) + refw;
        std::cout << "U_real:" << std::endl << U_real << std::endl;   

        ////Visualization in Rviz////
        Eigen::VectorXd K(Nu*Nc);
        K.tail(Nu*Nc) = QPSolution.tail(Nu*Nc);
        Eigen::VectorXd Y(Nx*Np);
        Y = PHI*kesi + THETA*K;

        Eigen::VectorXd Y_ref;
        Eigen::VectorXd Y_pre;
        if(idx+Np<=number-1){
          Y_ref.resize(Np*Nx);
        }else{
          Y_ref.resize(Nx*(number-1-idx));
        }
        int size_Y_ref = Y_ref.size();
        int state_size_Y_ref = size_Y_ref/Nx;
        for(int i=0;i<state_size_Y_ref;i++){
          Y_ref.segment(i*Nx,Nx) = refstate[idx+i+1];
        }
        Y_pre.resize(size_Y_ref);
        Y_pre = Y_ref + Y.tail(size_Y_ref);

        Visualization(Y_ref,Y_pre);

        return U_real;
      /////reach the end point/////
      }else if(idx == number-1){
        Eigen::VectorXd U_real(Nu);
        U_real = Eigen::MatrixXd::Zero(Nu,1);

        std::cout << "reach the end point!" << std::endl;
        return U_real;
      }
    }
};


#endif