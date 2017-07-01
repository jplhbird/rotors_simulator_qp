/*
 * Copyright 2015 Fadri Furrer, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Michael Burri, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Mina Kamel, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Janosch Nikolic, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Markus Achtelik, ASL, ETH Zurich, Switzerland
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0

 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "rotors_control/lee_position_controller.h"

#include <qpOASES.hpp>

namespace rotors_control {

LeePositionController::LeePositionController()
    : initialized_params_(false),
      controller_active_(false) ,pnh_("~/clf_cbf_para"){
  InitializeParameters();


	//config_motion = asctec_mav_motion_planning::motion_planning_paraConfig::__getDefault__();
	  // bring up dynamic reconfigure
//	motionconf_srv_ = new ReconfigureServer(pnh_);
//	ReconfigureServer::CallbackType f = boost::bind(&LeePositionController::cbmotionConfig, this, _1, _2);
//	motionconf_srv_->setCallback(f);

  //test beginning

  USING_NAMESPACE_QPOASES;
	/* Setup data of first QP. */
	real_t H[2*2] = { 1.0, 0.0, 0.0, 0.5 };
	real_t A[1*2] = { 1.0, 1.0 };
	real_t g[2] = { 1.5, 1.0 };
	real_t lb[2] = { 0.5, -2.0 };
	real_t ub[2] = { 5.0, 2.0 };
	real_t lbA[1] = { -1.0 };
	real_t ubA[1] = { 2.0 };

	/* Setup data of second QP. */
	real_t g_new[2] = { 1.0, 1.5 };
	real_t lb_new[2] = { 0.0, -1.0 };
	real_t ub_new[2] = { 5.0, -0.5 };
	real_t lbA_new[1] = { -2.0 };
	real_t ubA_new[1] = { 1.0 };


	/* Setting up QProblem object. */
	QProblem example( 2,1 );

	Options options;
	example.setOptions( options );

	/* Solve first QP. */
	int nWSR = 10;
	example.init( H,g,A,lb,ub,lbA,ubA, nWSR );

	/* Get and print solution of first QP. */
	real_t xOpt[2];
	real_t yOpt[2+1];
	example.getPrimalSolution( xOpt );
	example.getDualSolution( yOpt );
	printf( "\nxOpt = [ %e, %e ];  yOpt = [ %e, %e, %e ];  objVal = %e\n\n",
			xOpt[0],xOpt[1],yOpt[0],yOpt[1],yOpt[2],example.getObjVal() );

	//test finish

}

//void LeePositionController::cbmotionConfig(rotors_control::cbf_clfConfig & config, uint32_t level){
//
//
//}

LeePositionController::~LeePositionController() {}

void LeePositionController::InitializeParameters() {
  calculateAllocationMatrix(vehicle_parameters_.rotor_configuration_, &(controller_parameters_.allocation_matrix_));
  // To make the tuning independent of the inertia matrix we divide here.
  normalized_attitude_gain_ = controller_parameters_.attitude_gain_.transpose()
      * vehicle_parameters_.inertia_.inverse();
  // To make the tuning independent of the inertia matrix we divide here.
  normalized_angular_rate_gain_ = controller_parameters_.angular_rate_gain_.transpose()
      * vehicle_parameters_.inertia_.inverse();

  Eigen::Matrix4d I;
  I.setZero();
  I.block<3, 3>(0, 0) = vehicle_parameters_.inertia_;
  I(3, 3) = 1;
  angular_acc_to_rotor_velocities_.resize(vehicle_parameters_.rotor_configuration_.rotors.size(), 4);
  // Calculate the pseude-inverse A^{ \dagger} and then multiply by the inertia matrix I.
  // A^{ \dagger} = A^T*(A*A^T)^{-1}
  angular_acc_to_rotor_velocities_ = controller_parameters_.allocation_matrix_.transpose()
      * (controller_parameters_.allocation_matrix_
      * controller_parameters_.allocation_matrix_.transpose()).inverse() * I;

  //modified no Feb. 4th, the inertia matrix is moved to the attitude control:
  angular_acc_to_rotor_velocities_ = controller_parameters_.allocation_matrix_.transpose()
      * (controller_parameters_.allocation_matrix_
      * controller_parameters_.allocation_matrix_.transpose()).inverse();
  initialized_params_ = true;
}

void LeePositionController::CalculateRotorVelocities(Eigen::VectorXd* rotor_velocities) const {
  assert(rotor_velocities);
  assert(initialized_params_);

  rotor_velocities->resize(vehicle_parameters_.rotor_configuration_.rotors.size());
  // Return 0 velocities on all rotors, until the first command is received.
  if (!controller_active_) {
    *rotor_velocities = Eigen::VectorXd::Zero(rotor_velocities->rows());
    return;
  }

  Eigen::Vector3d acceleration;
  ComputeDesiredAcceleration(&acceleration);

  Eigen::Vector3d angular_acceleration;
  double deltaf = 0.0; //cbf based qp output
  ComputeDesiredAngularAcc(acceleration, &angular_acceleration, &deltaf);

  // Project thrust onto body z axis.
  //original
  double thrust = -vehicle_parameters_.mass_ * acceleration.dot(odometry_.orientation.toRotationMatrix().col(2));
  //modified on Feb., 4th, 2017, CLF-CBF:
  thrust = thrust + deltaf * vehicle_parameters_.mass_;

  Eigen::Vector4d angular_acceleration_thrust;
  angular_acceleration_thrust.block<3, 1>(0, 0) = angular_acceleration;
  angular_acceleration_thrust(3) = thrust;

  *rotor_velocities = angular_acc_to_rotor_velocities_ * angular_acceleration_thrust;
  *rotor_velocities = rotor_velocities->cwiseMax(Eigen::VectorXd::Zero(rotor_velocities->rows()));
  *rotor_velocities = rotor_velocities->cwiseSqrt();
}

void LeePositionController::SetOdometry(const EigenOdometry& odometry) {
  odometry_ = odometry;
}

void LeePositionController::SetTrajectoryPoint(
    const mav_msgs::EigenTrajectoryPoint& command_trajectory) {
  command_trajectory_ = command_trajectory;
  controller_active_ = true;
}

void LeePositionController::ComputeDesiredAcceleration(Eigen::Vector3d* acceleration) const {
  assert(acceleration);

  Eigen::Vector3d position_error;
  position_error = odometry_.position - command_trajectory_.position_W;

  // Transform velocity to world frame.
  const Eigen::Matrix3d R_W_I = odometry_.orientation.toRotationMatrix();
  Eigen::Vector3d velocity_W =  R_W_I * odometry_.velocity;
  Eigen::Vector3d velocity_error;
  velocity_error = velocity_W - command_trajectory_.velocity_W;

  Eigen::Vector3d e_3(Eigen::Vector3d::UnitZ());

  *acceleration = (position_error.cwiseProduct(controller_parameters_.position_gain_)
      + velocity_error.cwiseProduct(controller_parameters_.velocity_gain_)) / vehicle_parameters_.mass_
      - vehicle_parameters_.gravity_ * e_3 - command_trajectory_.acceleration_W;


  //clf position control:
  //Yushu Yu, Feb., 2017


//  % first level of QP : compute the virtual force
//  eta1 = 2.5;
//  epsilon1 = 2;
//  c1 = 8;
//
//  eta1 = 2.5;
//  epsilon1 = 2;
//  c1 = 8;
//
//  % scale the mass properties to
//  V1 =  (ev' * ev)/2 + epsilon1 * (ex' * ev) + c1 * (ex' * ex)/2;
//  LgV1 = ( ev + epsilon1 * ex)';
//  LfV1  =  epsilon1 * (ev' * ev) + c1 * (ev' * ex) - LgV1 * trajd.acc;
//
//  A1 = LgV1;
//  b1 = -LfV1 - eta1 * V1;
//  H1 = diag([5 5 1]);
//  f1 = zeros(3, 1);
//  optoption_1 = optimset('Display', 'off', 'TolFun', 1e-6); % the first level has more accurate
//  accd = quadprog(H1, f1, A1, b1, [], [], [], [], [], optoption_1);
//  fd = (accd + g * e3);

  //begin CLF qp:
  //ENU frame:

	Eigen::Vector3d accd;
	accd(0) = command_trajectory_.acceleration_W(0);
	accd(1) = command_trajectory_.acceleration_W(1);
	accd(2) = command_trajectory_.acceleration_W(2);

	double eta1, epsilon1, c1;
	eta1 = 2.5;
	epsilon1 = 2;
	c1 = 8;

	eta1 = clf_cbfpara.eta1;
	epsilon1 = clf_cbfpara.epsilon1;
	c1 = clf_cbfpara.c1;

	double V1;
	Eigen::Vector3d LgV1;
	double LfV1;
	V1 =  velocity_error.dot(velocity_error)/2.0 + epsilon1 * (position_error.dot(velocity_error)) +
		c1 * (position_error.dot(position_error))/2.0;
	LgV1 = ( velocity_error + epsilon1 * position_error);
	LgV1 = LgV1.transpose();

	LfV1  =  epsilon1 * (velocity_error.dot(velocity_error)) + c1 * (velocity_error.dot(position_error)) - LgV1.dot(accd);

	Eigen::Vector3d A1;
	A1 = LgV1;
	double b1;
	b1 = -LfV1 - eta1 * V1;
	Eigen::Matrix3d H1;
	H1.setZero();
	H1(0, 0) = 5;
	H1(1, 1) = 5;
	H1(2, 2) = 5;
	Eigen::Vector3d f1;
	f1.setZero();

	USING_NAMESPACE_QPOASES
	SQProblem positionqp( 3,1 );

	real_t H1_[3*3];
	real_t f1_[3] = {0.0, 0.0};
	real_t A1_[1*3];
	real_t b1_[1] = {b1};

	for (int iqp = 0; iqp < 3; iqp ++ ){
	for (int cqp = 0; cqp < 3; cqp ++ ){
		H1_[iqp*3+cqp] = H1(iqp, cqp);
	}
	A1_[iqp] = A1(iqp);
	}

	real_t  *ptrnll = NULL;

	/* Solve first QP. */
	int nWSR1 = 10;

	//example.hotstart( H_new,g_new,A_new,lb_new,ub_new,lbA_new,ubA_new, nWSR,0);
	//A1_*x<=b1_
	positionqp.init(H1_,f1_,A1_,ptrnll,ptrnll,ptrnll,b1_, nWSR1, 0 );

	/* Get and print solution of second QP. */
	real_t accOpt[3];
	positionqp.getPrimalSolution( accOpt );
	printf( "\n force = [ %e, %e, %e ];  objVal = %e\n\n", accOpt[0], accOpt[1], accOpt[2], positionqp.getObjVal() );
	printf("xe =  [ %e, %e, %e ];  ve = [ %e, %e, %e ]\n\n ", position_error(0), position_error(1), position_error(2),
			velocity_error(0), velocity_error(1), velocity_error(2));

	printf("b1_ =  [ %e ];  A1_ = [ %e, %e, %e ]\n\n ", b1_[0], A1_[0], A1_[1], A1_[2]);

	//expressed in ENU, notice, very important
	Eigen::Vector3d acc_nogravity;
	acc_nogravity(0) = accOpt[0];
	acc_nogravity(1) = accOpt[1];
	acc_nogravity(2) = accOpt[2];

	//contains the gravity, notice
	acc_nogravity = acc_nogravity + vehicle_parameters_.gravity_ * e_3;

	if (clf_cbfpara.flag_clf == 1)
	{
		//ENU frame:
		//notice the sign of acc here:
		//*acceleration = -acc_nogravity;
	}

	//printf(" acc = [ %e, %e, %e ]\n\n ", acceleration(0), acceleration(1), acceleration(2));

}

// Implementation from the T. Lee et al. paper
// Control of complex maneuvers for a quadrotor UAV using geometric methods on SE(3)
void LeePositionController::ComputeDesiredAngularAcc(const Eigen::Vector3d& acceleration,
                                                     Eigen::Vector3d* angular_acceleration, double* deltaf) const {
  assert(angular_acceleration);

  Eigen::Matrix3d R = odometry_.orientation.toRotationMatrix();

  // Get the desired rotation matrix.
  Eigen::Vector3d b1_des;
  double yaw = command_trajectory_.getYaw();
  b1_des << cos(yaw), sin(yaw), 0;

  Eigen::Vector3d b3_des;
  b3_des = -acceleration / acceleration.norm();

  Eigen::Vector3d b2_des;
  b2_des = b3_des.cross(b1_des);
  b2_des.normalize();

  Eigen::Matrix3d R_des;
  R_des.col(0) = b2_des.cross(b3_des);
  R_des.col(1) = b2_des;
  R_des.col(2) = b3_des;

  // Angle error according to lee et al.
  Eigen::Matrix3d angle_error_matrix = 0.5 * (R_des.transpose() * R - R.transpose() * R_des);
  Eigen::Vector3d angle_error;
  vectorFromSkewMatrix(angle_error_matrix, &angle_error);

  // TODO(burrimi) include angular rate references at some point.
  Eigen::Vector3d angular_rate_des(Eigen::Vector3d::Zero());
  angular_rate_des[2] = command_trajectory_.getYawRate();

  Eigen::Vector3d angular_rate_error = odometry_.angular_velocity - R_des.transpose() * R * angular_rate_des;

//  *angular_acceleration = -1 * angle_error.cwiseProduct(normalized_attitude_gain_)
//                           - angular_rate_error.cwiseProduct(normalized_angular_rate_gain_)
//                           + odometry_.angular_velocity.cross(odometry_.angular_velocity); // we don't need the inertia matrix here
//
//
//  // we  need the inertia matrix here, modified on Feb. 4th, 2017
  *angular_acceleration = vehicle_parameters_.inertia_*
		  (-1 * angle_error.cwiseProduct(normalized_attitude_gain_)
                             - angular_rate_error.cwiseProduct(normalized_angular_rate_gain_))
                             + odometry_.angular_velocity.cross(vehicle_parameters_.inertia_*odometry_.angular_velocity);

  //clf-cbf-based obstacle avoiding control

//  % compute the orientation error
//  eR = vee(Rc' * R - R' * Rc)/2;
//  eOmega = y(16:18) - R' * Rc * trajd.Omega;
//  dR = R * Omega_hat;
//  dRc = Rc * hat(trajd.Omega);
//  deR = (vee(dRc' * R - R' * dRc) + vee(Rc' * dR - dR' * Rc))/2;

  Eigen::Vector3d angular_rate_dot_des(Eigen::Vector3d::Zero());
  Eigen::Vector3d omega = odometry_.angular_velocity; //angular velocity
  Eigen::Matrix3d omega_hat;
  skewMatrixFromVector(omega, &omega_hat);
  Eigen::Matrix3d dR = R*omega_hat;
  Eigen::Matrix3d omegac_hat;
  skewMatrixFromVector(angular_rate_des, &omegac_hat);
  Eigen::Matrix3d dRc = R_des * omegac_hat;
  Eigen::Vector3d deR; //the dot of eR
  Eigen::Vector3d deR1, deR2;
  Eigen::Matrix3d deR1hat = dRc.transpose() * R - R.transpose() * dRc;
  Eigen::Matrix3d deR2hat = R_des.transpose() * dR - dR.transpose() * R_des;
  vectorFromSkewMatrix(deR1hat, &deR1);
  vectorFromSkewMatrix(deR2hat, &deR2);
  deR = (deR1 + deR2) / 2.0;

//  % CLF construction
//  eta2 = 150;
//  J_scale = J/min(eig(J));
//  epsilon2 = 4;
//  c2 = 20;
//
//  V2 = eOmega' * J_scale * eOmega/2 + epsilon2 * (eR' * eOmega)...
//       + c2 * (eR' * eR)/2;
//  LgV2 = (eOmega' * J_scale + epsilon2 * eR');
//  LfV2 = (epsilon2 * eOmega' + c2 * eR') * deR...
//          -  LgV2 * ( dR' * Rc * trajd.Omega + R' * Rc * trajd.dOmega);
  double eta2 = 10.0;
  double epsilon2 = 2.0;
  double c2 = 15.0;

  eta2 = clf_cbfpara.eta2;
  epsilon2 = clf_cbfpara.epsilon2;
  c2 = clf_cbfpara.c2;


  Eigen::Matrix3d J_scale = vehicle_parameters_.inertia_;
  double minJ = vehicle_parameters_.inertia_(0,0);
  if ((vehicle_parameters_.inertia_(0,0) < vehicle_parameters_.inertia_(1,1)) &
		  (vehicle_parameters_.inertia_(0,0) < vehicle_parameters_.inertia_(2,2)))
		  minJ = vehicle_parameters_.inertia_(0,0);
  else if ((vehicle_parameters_.inertia_(1,1) < vehicle_parameters_.inertia_(0,0)) &
		  (vehicle_parameters_.inertia_(1,1) < vehicle_parameters_.inertia_(2,2)))
		  minJ = vehicle_parameters_.inertia_(1,1);
  else if ((vehicle_parameters_.inertia_(2,2) < vehicle_parameters_.inertia_(1,1)) &
		  (vehicle_parameters_.inertia_(2,2) < vehicle_parameters_.inertia_(0,0)))
		  minJ = vehicle_parameters_.inertia_(2,2);
  J_scale = J_scale / minJ;

  double V2;
  V2 = angular_rate_error.dot(J_scale * angular_rate_error) / 2.0 + epsilon2 * (angle_error.dot(angular_rate_error)) +
		  c2 * (angle_error.dot(angle_error)) / 2.0;

  Eigen::Vector3d LgV2;
  LgV2 =  J_scale.transpose() * angular_rate_error + epsilon2 * angle_error;

  double LfV2;
  LfV2 = deR.dot(epsilon2 * angular_rate_error + c2 * angle_error)
   -  LgV2.dot(dR.transpose() * R_des * angular_rate_des + R.transpose() * R_des * angular_rate_dot_des);

//  % CBF construction
//  A21 = zeros(cbf_prms.cbfNum, 5);
//  b21 = zeros(cbf_prms.cbfNum, 1);
//  gamma = 5;
//
//  for j = 1 : cbf_prms.cbfNum
//      exo = y(1:3) - cbf_prms.xc(:, j); %r
//      evo = y(4:6) - cbf_prms.dxc(:, j); %dot r
//      d2xo = cbf_prms.d2xc(:, j); %
//      db = cbf_prms.db(j);
//      d2b = cbf_prms.d2b(j);
//      s = exo' * R * e3;
//      p = exo' * dR * e3 + evo' * R * e3;
//      [sigma, dsigma, d2sigma] = angle_barrier(s, cbf_prms.alpha(j), cbf_prms.beta(j));
//
//      % compute the g hat and h hat
//      g_hat = exo' * exo - cbf_prms.b(j) - sigma;
//      dg_hat = 2 * evo' * exo - db - dsigma * p;
//      h_hat = gamma * g_hat + dg_hat;
//      xi = (2 * exo - dsigma * R(:, 3))' ;
//      Lgh_hat = [ xi * R(:, 3), dsigma * exo' * R * e3_hat];
//      Lfh_hat = gamma * dg_hat + 2 * (evo' * evo) - d2b -d2sigma * p^2 ...
//                   - dsigma  * (2 * evo' * dR * e3 + exo' * R * Omega_hat^2 * e3)...
//                   - xi * (d2xo + g * e3) + Lgh_hat(1) * Fd;
//      A21(j, 1:4)  = -Lgh_hat;
//      b21(j) = Lfh_hat + cbf_prms.eta3 * h_hat;
//  end

  //CBF construction

  double Fd = -acceleration.dot(odometry_.orientation.toRotationMatrix().col(2));

  cbf_prms_stru cbf_prms;


  //update the obstacles
  cbf_prms.cbfNum = 1;  // number of obstacles
  double pos_obstacles[3][20];
  pos_obstacles[0][0] = 0;
  pos_obstacles[1][0] = 0;
  pos_obstacles[2][0] = 4;

  pos_obstacles[0][0] = 10;
  pos_obstacles[1][0] = 1;
  pos_obstacles[2][0] = 3;


  pos_obstacles[0][0] = 1011111;
  pos_obstacles[1][0] = 1;
  pos_obstacles[2][0] = 3;


  Eigen::MatrixXd A21 = Eigen::MatrixXd::Zero(cbf_prms.cbfNum, 5);
  Eigen::VectorXd b21 = Eigen::MatrixXd::Zero(cbf_prms.cbfNum, 1);
  double gamma = 10.0;

  for (int i = 0; i < cbf_prms.cbfNum; i ++ ){
	  cbf_x_b cbf_x_b_i;
	  cbf_x_b_i.b = 1.0;
	  cbf_x_b_i.db = 0.0;
	  cbf_x_b_i.d2b = 0.0;
	  cbf_x_b_i.xc  << pos_obstacles[0][i], pos_obstacles[1][i], pos_obstacles[2][i];
	  cbf_x_b_i.dxc << 0.0, 0.0, 0.0;
	  cbf_x_b_i.d2xc << 0.0, 0.0, 0.0;
	  double radius = 1.0;
	  cbf_prms.beta = 1.0/(1.2 * radius * radius);
	  cbf_prms.alpha = 0.25*radius*radius;
	  cbf_prms.eta3 = 8.0;

	  Eigen::Vector3d exo = odometry_.position - cbf_x_b_i.xc;
	  Eigen::Vector3d evo = odometry_.velocity - cbf_x_b_i.dxc;
	  Eigen::Vector3d d2xo = cbf_x_b_i.d2xc;
	  double db = cbf_x_b_i.db;
	  double d2b = cbf_x_b_i.d2b;
	  Eigen::Vector3d e_3(Eigen::Vector3d::UnitZ());
	  Eigen::Matrix3d e_3_hat;
	  skewMatrixFromVector(e_3, &e_3_hat);
	  double s = exo.dot(R * e_3);
	  double p = exo.dot(dR * e_3) + evo.dot(R * e_3);

	  double sigma;
	  double dsigma;
	  double d2sigma;
	  angle_barrier(s, cbf_prms.alpha, cbf_prms.beta, &sigma, &dsigma, &d2sigma);

	  double g_hat = exo.dot(exo) - cbf_x_b_i.b - sigma;
	  double dg_hat = 2 * evo.dot(exo) - db -dsigma * p;
	  double h_hat = gamma * g_hat + dg_hat;

	  printf( "h_hat = %e \n", h_hat );


	  Eigen::Vector3d xi = 2 * exo - dsigma * R * e_3;
	  Eigen::Vector4d Lgh_hat;
 	  Lgh_hat(0) = xi.dot(R * e_3);
	  Eigen::Vector3d Lgh_haddd = dsigma * e_3_hat.transpose() * R.transpose() * exo;
	  Lgh_hat.tail(3) = Lgh_haddd;


	  double Lfh_hat = gamma * dg_hat + 2 * (evo.dot(evo)) - d2b -d2sigma * p *p
              - dsigma  * (2 * evo.dot(dR * e_3) + exo.dot(R * omega_hat * omega_hat * e_3))
              - xi.dot (d2xo + vehicle_parameters_.gravity_ * e_3) + Lgh_hat(0) * Fd;
	  A21.block(i, 0, 1, 4) << -Lgh_hat(0), -Lgh_hat(1), -Lgh_hat(2), -Lgh_hat(3);
	  b21(i) = Lfh_hat + cbf_prms.eta3 * h_hat;
   }

//  % set up the quadratic programming for the orientation control
//  H2 = diag([25 1 1 1 200]);
//  f2 = zeros(5, 1);
//  I = eye(3);
//  O = zeros(3, 1);
//  A2 = [0, LgV2, -1;
//            A21;
//           1, 0, 0, 0, 0;
//           O, I, O;
//           O, -I, O];
//  b2 = [-LfV2 - eta2 * V2;
//             b21;
//             sys_prms.Fmax - Fd;
//             sys_prms.Mmax;
//             sys_prms.Mmax];
//
//  optoption_2 = optimset('Display', 'off', 'TolFun', 1e-5);
//  x= quadprog(H2, f2, A2, b2, [], [], [], [], [], optoption_2);

  //qp
	Eigen::MatrixXd H2 = Eigen::MatrixXd::Zero(5,5);
	H2(0, 0) = 200;

	H2(0, 0) = 25;
	H2(1, 1) = 1;
	H2(2, 2) = 1;
	H2(3, 3) = 1;
	H2(4, 4) = 741.28;
	H2(4, 4) = 741.5;
	H2(4, 4) = 750;


	H2(4, 4) = 200;

	Eigen::VectorXd f2 = Eigen::VectorXd::Zero(5);
	Eigen::Matrix3d I;
	I.setZero();
	I(0, 0) = 1;
	I(1, 1) = 1;
	I(2, 2) = 1;
	Eigen::Vector3d O(0,0,0);
	Eigen::MatrixXd A2 = Eigen::MatrixXd::Zero(cbf_prms.cbfNum + 8, 5);
	Eigen::MatrixXd b2 = Eigen::MatrixXd::Zero(cbf_prms.cbfNum + 8, 1);

	//value of A2 and b2:
	A2.block(0, 1, 1, 3) << LgV2(0), LgV2(1), LgV2(2);
	A2(0, 4) = -1.0;
	A2.block(1, 0, cbf_prms.cbfNum, 5) = A21;
	A2(cbf_prms.cbfNum + 1, 0) = 1.0;
	A2.block(cbf_prms.cbfNum + 2, 1, 3, 3) = I;
	A2.block(cbf_prms.cbfNum + 5, 1, 3, 3) = -I;

 	b2(0) = -LfV2 - eta2 * V2;
 	for (int j = 0; j < cbf_prms.cbfNum; j ++)
 	{
 		b2(j + 1) = b21(j);
 	}
 	b2.block(cbf_prms.cbfNum + 1, 0, 7, 1) << (80 - Fd), 100, 100, 100, 100, 100, 100; //bounded of force and moment

	USING_NAMESPACE_QPOASES
	SQProblem attitudeqp( 5,1 );
	real_t H2_[5 * 5];
	real_t f2_[5 * 1] = {0.0, 0.0, 0.0, 0.0, 0.0};
	real_t A2_[(cbf_prms.cbfNum + 8)* 5];
	real_t b2_[cbf_prms.cbfNum + 8];

	for (int iqp = 0; iqp < 5; iqp ++ ){
		for (int cqp = 0; cqp < 5; cqp ++ ){
			H2_[iqp*5 + cqp] = H2(iqp, cqp);
		}
		f2_[iqp] = f2(iqp);
	}

	for (int iqp = 0; iqp < (cbf_prms.cbfNum + 8); iqp ++ ){
		for (int cqp = 0; cqp < 5; cqp ++ ){
			A2_[iqp*5 + cqp] = A2(iqp, cqp);
		}
		b2_[iqp] = b2(iqp);
	}


	real_t  *ptrnll = NULL;
 	int nWSR1 = 100;
    //example.hotstart( H_new,g_new,A_new,lb_new,ub_new,lbA_new,ubA_new, nWSR,0);
	//A2_*x<=b2_
    attitudeqp.init(H2_,f2_,A2_,ptrnll,ptrnll,ptrnll,b2_, nWSR1, 0 );

    attitudeqp.hotstart(H2_,f2_,A2_,ptrnll,ptrnll,ptrnll,b2_, nWSR1, 0 );

    real_t attitudeOpt[5];
    attitudeqp.getPrimalSolution( attitudeOpt );




	// return the control input
	if (clf_cbfpara.flag_clf == 1){

		printf( "\n delta force, moment, and delta V = [ %e, %e, %e, %e, %e];  objVal = %e\n\n", attitudeOpt[0], attitudeOpt[1], attitudeOpt[2],  attitudeOpt[3],  attitudeOpt[4],
				attitudeqp.getObjVal() );

		*deltaf = attitudeOpt[0];
		Eigen::Vector3d dOmega;
		dOmega(0) = attitudeOpt[1],
		dOmega(1) = attitudeOpt[2],
		dOmega(2) = attitudeOpt[3];
		//*angular_acceleration = vehicle_parameters_.inertia_* dOmega + omega_hat * vehicle_parameters_.inertia_ * omega;
	}

//	*angular_acceleration = vehicle_parameters_.inertia_*
//			  (-1 * angle_error.cwiseProduct(normalized_attitude_gain_)
//	                             - angular_rate_error.cwiseProduct(normalized_angular_rate_gain_))
//	                             + odometry_.angular_velocity.cross(vehicle_parameters_.inertia_*odometry_.angular_velocity);

	ROS_INFO_STREAM("lee_position_controller_.clf_cbfpara.flag_clf: "<<clf_cbfpara.flag_clf);

	ROS_INFO_STREAM("b2"<<b2);

}



//void LeePositionController::Computehat( Eigen::Matrix3d* hatx,
//		  const Eigen::Vector3d& x) const{
//
//
//
//}
//
//void LeePositionController::Computevee( const Eigen::Matrix3d& hatx,
//		   Eigen::Vector3d* x) const{
//
//}
void LeePositionController::angle_barrier( const double& y, const double&  alpha,  const double&  beta,
		  double* sigma, double* dsigma, double* d2sigma) const{

//	func_type = 2;
//	offset = 0.0;
//	x = y + offset;
//	    switch(func_type)
//	        % sigmoidal function
//	        case 1
//	            sigma = -(alpha*(exp(beta*x) - 1))/(exp(beta*x) + 1);
//	            dsigma = (alpha*beta*exp(beta*x)*(exp(beta*x) - 1))/(exp(beta*x) + 1)^2 ...
//	                            - (alpha*beta*exp(beta*x))/(exp(beta*x) + 1);
//	            d2sigma = (2*alpha*beta^2*exp(2*beta*x))/(exp(beta*x) + 1)^2 ...
//	                              - (alpha*beta^2*exp(beta*x))/(exp(beta*x) + 1) ...
//	                              +(alpha*beta^2*exp(beta*x)*(exp(beta*x) - 1))/(exp(beta*x) + 1)^2 ...
//	                              - (2*alpha*beta^2*exp(2*beta*x)*(exp(beta*x) - 1))/(exp(beta*x) + 1)^3;
//	        % arctan function
//	        case 2
//	            sigma = -alpha * atan(x * beta);
//	            dsigma =  -(alpha*beta)/(beta^2*x^2 + 1);
//	            d2sigma =  (2*alpha*beta^3*x)/(beta^2*x^2 + 1)^2;
//	    end
	int func_type = 2;
	double offset = 0.0;
	double x = y + offset;
	switch(func_type)
	{
		case 1:
			*sigma  = -(alpha*(exp(beta*x) - 1))/(exp(beta*x) + 1);
			*dsigma = (alpha*beta*exp(beta*x)*(exp(beta*x) - 1))/((exp(beta*x) + 1) *(exp(beta*x) + 1))
											- (alpha*beta*exp(beta*x))/(exp(beta*x) + 1);
			*d2sigma = (2*alpha*beta*beta*exp(2*beta*x))/((exp(beta*x) + 1)*(exp(beta*x) + 1))
			  - (alpha*beta*beta*exp(beta*x))/(exp(beta*x) + 1)
			  +(alpha*beta*beta*exp(beta*x)*(exp(beta*x) - 1))/((exp(beta*x) + 1) *(exp(beta*x) + 1))
			  - (2*alpha*beta*beta*exp(2*beta*x)*(exp(beta*x) - 1))/((exp(beta*x) + 1)*(exp(beta*x) + 1)*(exp(beta*x) + 1));
			break;
		case 2:
			*sigma = -alpha * atan(x * beta);
			*dsigma =  -(alpha*beta)/(beta*beta*x*x + 1);
			*d2sigma =  (2*alpha*beta*beta*beta*x)/(beta*beta*x*x + 1)*(beta*beta*x*x + 1);
			break;
	}

}

void LeePositionController::getCbf(int i){
	// i-th obstacle
//	  struct cbf_x_b
//	  {
//		  Eigen::Vector3d xc;
//		  Eigen::Vector3d dxc;
//		  Eigen::Vector3d d2xc;
//		  double db;
//		  double d2b;
//		  double b;
//	  };
//	  cbf_x_b.b = 0.0;
//	  cbf_x_b.db = 0.0;
//	  cbf_x_b.d2b = 0.0;
//	  cbf_x_b.xc  << pos_obstable[0][i], pos_obstable[1][i], pos_obstable[2][i];
//	  cbf_x_b.dxc << 0.0, 0.0, 0.0;
//	  cbf_x_b.d2xc << 0.0, 0.0, 0.0;

}



}
