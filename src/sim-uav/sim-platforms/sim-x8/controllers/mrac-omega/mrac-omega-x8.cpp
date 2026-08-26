/***********************************************************************************************************************
 * Copyright (c) 2025 Giri M. Kumar, Mattia Gramuglia, Andrea L'Afflitto. All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without modification, are permitted provided that the
 * following conditions are met:
 * 
 * 1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following
 *    disclaimer.
 * 
 * 2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the
 *    following disclaimer in the documentation and/or other materials provided with the distribution.
 * 
 * 3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote
 *    products derived from this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS “AS IS” AND ANY EXPRESS OR IMPLIED WARRANTIES,
 * INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 **********************************************************************************************************************/

 /***********************************************************************************************************************
 * File:        mrac-omega-x8.cpp
 * Author:      Xavier Casanova
 * Date:        August 04, 2026
 * For info:    Andrea L'Afflitto 
 *              a.lafflitto@vt.edu
 * 
 * Description: MRAC with angular velocities for the X8. Inherts the class controller_base for the basic 
 *              functionality that is to be used for all control algorithms.
 * 
 * GitHub:    https://github.com/xavier1cas/acsl-chrono-simulator.git
 **********************************************************************************************************************/

#include "mrac-omega-x8.hpp"

namespace _acsl_
{

namespace _x8_
{

namespace _mrac_omega_
{

// -------------------------------------------------------------------------
// Constructor Implementation:
//   - Calls the base (controller_base) constructor and passes 
//     both logger and trajectory.
// -------------------------------------------------------------------------
mrac_omega::mrac_omega(_acsl_::_logger_::simlog& logger, ::_acsl_::_trajectory_::trajectorybase& trajectory)
                    : ::_acsl_::_control_::controller_base(logger, trajectory)
{
    // Initial Conditions
    init();   
}

// -------------------------------------------------------------------------
// read_params Implementation:
// - Takes the hardcoded path for the gains and parameters and reads it in
// -------------------------------------------------------------------------
void mrac_omega::read_params(const std::string& jsonFile)
{
	// Implementation here
	std::ifstream file(jsonFile);
	nlohmann::json j;
    file >> j;

	// Translational Parameters
	cip.Kp_refmod_tran = ::_shared_::_deserialize_::jsonToScaledMatrixXd(j["TRANSLATIONAL"]["KP_refmod_translational"], 3, 3);
	cip.Ki_refmod_tran = ::_shared_::_deserialize_::jsonToScaledMatrixXd(j["TRANSLATIONAL"]["KI_refmod_translational"], 3, 3);
	cip.Kd_refmod_tran = ::_shared_::_deserialize_::jsonToScaledMatrixXd(j["TRANSLATIONAL"]["KD_refmod_translational"], 3, 3);
	cip.Kp_tran = ::_shared_::_deserialize_::jsonToScaledMatrixXd(j["TRANSLATIONAL"]["KP_translational"], 3, 3);
	cip.Ki_tran = ::_shared_::_deserialize_::jsonToScaledMatrixXd(j["TRANSLATIONAL"]["KI_translational"], 3, 3);
	cip.Kd_tran = ::_shared_::_deserialize_::jsonToScaledMatrixXd(j["TRANSLATIONAL"]["KD_translational"], 3, 3);
	cip.Gamma_x_tran = ::_shared_::_deserialize_::jsonToScaledMatrixXd(j["TRANSLATIONAL"]["Gamma_x_translational"], 6, 6);
	cip.Gamma_r_tran = ::_shared_::_deserialize_::jsonToScaledMatrixXd(j["TRANSLATIONAL"]["Gamma_r_translational"], 3, 3);
	cip.Gamma_Theta_tran = ::_shared_::_deserialize_::jsonToScaledMatrixXd(j["TRANSLATIONAL"]["Gamma_Theta_translational"], 6, 6);
	cip.Q_tran = ::_shared_::_deserialize_::jsonToScaledMatrixXd(j["TRANSLATIONAL"]["Q_translational"], 6, 6);

	// Rotational Parameters
	cip.Kp_omega_ref = ::_shared_::_deserialize_::jsonToScaledMatrixXd(j["ROTATIONAL"]["KP_omega_ref"], 3, 3);
	cip.Ki_omega_ref = ::_shared_::_deserialize_::jsonToScaledMatrixXd(j["ROTATIONAL"]["KI_omega_ref"], 3, 3);
	cip.Kp_rot = ::_shared_::_deserialize_::jsonToScaledMatrixXd(j["ROTATIONAL"]["KP_rotational"], 3, 3);
	cip.Ki_rot = ::_shared_::_deserialize_::jsonToScaledMatrixXd(j["ROTATIONAL"]["KI_rotational"], 3, 3);
	cip.Kd_rot = ::_shared_::_deserialize_::jsonToScaledMatrixXd(j["ROTATIONAL"]["KD_rotational"], 3, 3);
	cip.Gamma_x_rot = ::_shared_::_deserialize_::jsonToScaledMatrixXd(j["ROTATIONAL"]["Gamma_x_rotational"], 3, 3);
	cip.Gamma_r_rot = ::_shared_::_deserialize_::jsonToScaledMatrixXd(j["ROTATIONAL"]["Gamma_r_rotational"], 3, 3);
	cip.Gamma_Theta_rot = ::_shared_::_deserialize_::jsonToScaledMatrixXd(j["ROTATIONAL"]["Gamma_Theta_rotational"], 6, 6);
	cip.Q_rot = ::_shared_::_deserialize_::jsonToScaledMatrixXd(j["ROTATIONAL"]["Q_rotational"], 3, 3);

	// Robustification Parameters
	cip.use_projection_operator = j["ROBUSTIFICATION"]["use_projection_operator"];
	cip.dead_zone_delta_translational = j["ROBUSTIFICATION"]["dead_zone_delta_translational"];
	cip.dead_zone_e0_translational = j["ROBUSTIFICATION"]["dead_zone_e0_translational"];
	cip.dead_zone_delta_rotational = j["ROBUSTIFICATION"]["dead_zone_delta_rotational"];
	cip.dead_zone_e0_rotational = j["ROBUSTIFICATION"]["dead_zone_e0_rotational"];

	cip.sigma_x_translational = j["ROBUSTIFICATION"]["sigma_x_translational"];
	cip.sigma_r_translational = j["ROBUSTIFICATION"]["sigma_r_translational"];
	cip.sigma_Theta_translational = j["ROBUSTIFICATION"]["sigma_Theta_translational"];
	cip.sigma_x_rotational = j["ROBUSTIFICATION"]["sigma_x_rotational"];
	cip.sigma_r_rotational = j["ROBUSTIFICATION"]["sigma_r_rotational"];
	cip.sigma_Theta_rotational = j["ROBUSTIFICATION"]["sigma_Theta_rotational"];

	cip.projection_x_max_x_translational = j["ROBUSTIFICATION"]["projection_x_max_x_translational"];
	cip.projection_epsilon_x_translational = j["ROBUSTIFICATION"]["projection_epsilon_x_translational"];

	cip.projection_x_max_r_translational = j["ROBUSTIFICATION"]["projection_x_max_r_translational"];
	cip.projection_epsilon_r_translational = j["ROBUSTIFICATION"]["projection_epsilon_r_translational"];

	cip.projection_x_max_Theta_translational = j["ROBUSTIFICATION"]["projection_x_max_Theta_translational"];
	cip.projection_epsilon_Theta_translational = j["ROBUSTIFICATION"]["projection_epsilon_Theta_translational"];

	cip.projection_x_max_x_rotational = j["ROBUSTIFICATION"]["projection_x_max_x_rotational"]; 
	cip.projection_epsilon_x_rotational = j["ROBUSTIFICATION"]["projection_epsilon_x_rotational"];

	cip.projection_x_max_r_rotational = j["ROBUSTIFICATION"]["projection_x_max_r_rotational"];
	cip.projection_epsilon_r_rotational = j["ROBUSTIFICATION"]["projection_epsilon_r_rotational"];

	cip.projection_x_max_Theta_rotational = j["ROBUSTIFICATION"]["projection_x_max_Theta_rotational"];
	cip.projection_epsilon_Theta_rotational = j["ROBUSTIFICATION"]["projection_epsilon_Theta_rotational"];
}

// Implementing virtual functios from controller_base
void mrac_omega::init(){
	// Reading in the parameters
	read_params("../chrono-assets/parameters/x8/MRAC_OMEGA/gains_MRAC_OMEGA.json");

	// Set the inital conditions
    y.fill(0.0);
    dy.fill(0.0);

	// Setup the logging 
	InitiateLogging();
	ConfigureHeaders();

	/// Translational - Initialize matrices
	// Initialize to zero the 6x6 matrix and set the top-right 3x3 block as follows
	::_shared_::_initiate_::initMat(cip.A_tran);
	cip.A_tran.block<3, 3>(0, 3) = Eigen::Matrix3d::Identity();

	// Initialize to zero the 6x3 matrix and set the bottom 3x3 block as follows
	::_shared_::_initiate_::initMat(cip.B_tran);
	cip.B_tran.block<3, 3>(3, 0) = Eigen::Matrix3d::Identity();

	// Initialize to zero the 6x6 matrix
	::_shared_::_initiate_::initMat(cip.A_ref_tran);
	// Set the top-right 3x3 block as follows
	cip.A_ref_tran.block<3, 3>(0, 3) = Eigen::Matrix3d::Identity();
	// Set the bottom-left 3x3 block as follows
	cip.A_ref_tran.block<3, 3>(3, 0) = -cip.Kp_refmod_tran;
	// Set the bottom-right 3x3 block as follows
	cip.A_ref_tran.block<3, 3>(3, 3) = -cip.Kd_refmod_tran;

	// Initialize to zero the 6x3 matrix and set the bottom 3x3 block as follows
	::_shared_::_initiate_::initMat(cip.B_ref_tran);
	cip.B_ref_tran.block<3, 3>(3, 0) = (1.0 / MASS) * Eigen::Matrix3d::Identity();

	// Solve the continuous Lyapunov equation to compute P_translational
	cip.P_tran = ::_lyapunov_solver_::RealContinuousLyapunovEquation(cip.A_ref_tran, cip.Q_tran);

	/// Rotational - Initialize matrices
	// Intialize to zero the 3x3 matrix
	::_shared_::_initiate_::initMat(cip.A_rot);

	// Initlize to identity the 3x3 matrix 
	cip.B_rot = Eigen::Matrix3d::Identity();
	
	// Set the 3x3 matrix as follows
	cip.A_ref_rot << -cip.Kp_omega_ref;

	// Set the 3x3 matrix as follows
	cip.B_ref_rot = Eigen::Matrix3d::Identity();

	// Solve the continuous Lyapunov eequation to compute P_rotational
	cip.P_rot = ::_lyapunov_solver_::RealContinuousLyapunovEquation(cip.A_ref_rot, cip.Q_rot);

}

// Update function for the controller
void mrac_omega::update(double time, 
                        double x,
                        double y,
                        double z,
                        double vx,
                        double vy,
                        double vz,
                        double q0,
                        double q1,
                        double q2,
                        double q3,
                        double roll,
                        double pitch,
                        double yaw,
                        double w_x,
                        double w_y,
                        double w_z)     
{
    // 1. Assign all the states ----------------------------------------------------
    cim.t = time;

    // Assign the Translational States
    cim.x_tran_pos << x,y,z;
    cim.x_tran_vel << vx, vy, vz;
    cim.x_tran << cim.x_tran_pos, cim.x_tran_vel;

    // Assign the Rotational States
    cim.eta_rot << roll, pitch, yaw;
    cim.omega_rot << w_x, w_y, w_z;

    // Assign the quaternions
    cim.q.w() = q0;
    cim.q.z() = q1;
    cim.q.y() = q2;
    cim.q.z() = q3;

    // 2. Get the reference trajectory ---------------------------------------------
    m_traj.UpdateModule(time);
    cim.r_user = m_traj.GetPosition();
    cim.r_dot_user = m_traj.GetVelocity();
    cim.r_ddot_user = m_traj.GetAcceleration();   

    // Set the desired yaw and yaw_rate; we don't need the acceleration as 
    // we are going to compute it from the filter.
    cim.eta_rot_d(2) = m_traj.GetEulerAngle()(2);
    cim.eta_rot_rate_d(2) = m_traj.GetEulerRate()(2);

    // 3. Capture the time before the execution of the controller ------------------
    cim.alg_start_time = std::chrono::high_resolution_clock::now();

    // 4. Compute the rotation matrices --------------------------------------------
    cim.R_I_J = _shared_::_transformations_::rotationMatrix321GlobalToLocal(cim.eta_rot(0), cim.eta_rot(1), cim.eta_rot(2));
    cim.R_J_I = _shared_::_transformations_::rotationMatrix321LocalToGlobal(cim.eta_rot(0), cim.eta_rot(1), cim.eta_rot(2));

    // 5. Compute the body velocities in the quadcopter frame from the inertial velocities
    cim.x_tran_vel_J = cim.R_I_J * cim.x_tran_vel;

    // 6. Assign the values from the integrator ------------------------------------
    assign_from_rk4();
}

// Function to assign elements from the rk4 integrator
void mrac_omega::assign_from_rk4()
{
    //------------ assign after integration  ------------//
    int index = 0;
	::_shared_::_deserialize_::assignElementsToMembers(csm.state_roll_d_filter, y, index);
	::_shared_::_deserialize_::assignElementsToMembers(csm.state_pitch_d_filter, y, index);
	::_shared_::_deserialize_::assignElementsToMembers(csm.state_omega_x_d_filter, y, index);
	::_shared_::_deserialize_::assignElementsToMembers(csm.state_omega_y_d_filter, y, index);
	::_shared_::_deserialize_::assignElementsToMembers(csm.state_omega_z_d_filter, y, index);
	::_shared_::_deserialize_::assignElementsToMembers(csm.e_tran_pos_I, y, index);
	::_shared_::_deserialize_::assignElementsToMembers(csm.e_tran_pos_ref_I, y, index);
	::_shared_::_deserialize_::assignElementsToMembers(csm.x_tran_ref, y, index);
	::_shared_::_deserialize_::assignElementsToMembers(csm.K_hat_x_tran, y, index);
	::_shared_::_deserialize_::assignElementsToMembers(csm.K_hat_r_tran, y, index);
	::_shared_::_deserialize_::assignElementsToMembers(csm.Theta_hat_tran, y, index);
	::_shared_::_deserialize_::assignElementsToMembers(csm.e_rot_omega_ref_I, y, index);
	::_shared_::_deserialize_::assignElementsToMembers(csm.e_rot_eta_I, y, index);
	::_shared_::_deserialize_::assignElementsToMembers(csm.omega_rot_ref, y, index);
	::_shared_::_deserialize_::assignElementsToMembers(csm.K_hat_x_rot, y, index);
	::_shared_::_deserialize_::assignElementsToMembers(csm.K_hat_r_rot, y, index);
	::_shared_::_deserialize_::assignElementsToMembers(csm.Theta_hat_rot, y, index);
}

// Model function for integration
void mrac_omega::model(const _control_::rk4_array<double, NSI> &y, _control_::rk4_array<double, NSI> &dy, double t)
{
    //------------ Fill up the dy for integration  ------------//
    int index = 0;
    ::_shared_::_serialize_::assignElementsToDxdt(cim.internal_state_roll_d_filter, dy, index);
	::_shared_::_serialize_::assignElementsToDxdt(cim.internal_state_pitch_d_filter, dy, index);
	::_shared_::_serialize_::assignElementsToDxdt(cim.internal_state_omega_x_d_filter, dy, index);
	::_shared_::_serialize_::assignElementsToDxdt(cim.internal_state_omega_y_d_filter, dy, index);
	::_shared_::_serialize_::assignElementsToDxdt(cim.internal_state_omega_z_d_filter, dy, index);
	::_shared_::_serialize_::assignElementsToDxdt(cim.e_tran_pos, dy, index);
	::_shared_::_serialize_::assignElementsToDxdt(cim.e_tran_pos_ref, dy, index);
	::_shared_::_serialize_::assignElementsToDxdt(cim.x_tran_ref_dot, dy, index);
	::_shared_::_serialize_::assignElementsToDxdt(cim.K_hat_x_tran_dot, dy, index);
	::_shared_::_serialize_::assignElementsToDxdt(cim.K_hat_r_tran_dot, dy, index);
	::_shared_::_serialize_::assignElementsToDxdt(cim.Theta_hat_tran_dot, dy, index);
	::_shared_::_serialize_::assignElementsToDxdt(cim.e_rot_omega_ref, dy, index);
	::_shared_::_serialize_::assignElementsToDxdt(cim.e_rot_eta, dy, index);
	::_shared_::_serialize_::assignElementsToDxdt(cim.omega_rot_ref_dot, dy, index);
	::_shared_::_serialize_::assignElementsToDxdt(cim.K_hat_x_rot_dot, dy, index);
	::_shared_::_serialize_::assignElementsToDxdt(cim.K_hat_r_rot_dot, dy, index);
	::_shared_::_serialize_::assignElementsToDxdt(cim.Theta_hat_rot_dot, dy, index);
}

// Function to compute the outerloop regressor vector
void mrac_omega::compute_outer_loop_regressor()
{
    // (aerodynamic drag term)
    // Compute velocity in body frame J: v_J = R_ij * v_I
    Eigen::Matrix<double, 3, 1> v_J = cim.R_I_J * cim.x_tran_vel;
    double v_J_norm = v_J.norm();
    cim.outer_loop_regressor << -0.5 * v_J * v_J_norm;
}

// Function to compute the inner loop regressor vector
void mrac_omega::compute_inner_loop_regressor()
{
    cim.inner_loop_regressor << cim.omega_rot(1) * cim.omega_rot(2),
                                cim.omega_rot(0) * cim.omega_rot(2),
                                cim.omega_rot(0) * cim.omega_rot(1);
}

// Function to compute the outerloop control in I
void mrac_omega::compute_translational_control_in_I()
{
	// Compute the error in the states
	cim.e_tran << cim.x_tran - csm.x_tran_ref;
	cim.e_tran_pos << cim.e_tran.head<3>();
	cim.e_tran_vel << cim.e_tran.tail<3>();

	// Compute the translational position error between the reference model and the user defined trajectory
	cim.e_tran_pos_ref << csm.x_tran_ref.head<3>() - cim.r_user;

	// Compute the reference command input [reference model - user_defined_trajectory]
	cim.r_cmd_tran << MASS * (- cip.Ki_refmod_tran * csm.e_tran_pos_ref_I		// Integral term
							  + cip.Kp_refmod_tran * cim.r_user					// Proportional term
							  + cip.Kd_refmod_tran * cim.r_dot_user				// Derivative term
							  + cim.r_ddot_user);								// Feedforward term

	// Reference model
	cim.x_tran_ref_dot << cip.A_ref_tran * csm.x_tran_ref
						+ cip.B_ref_tran * cim.r_cmd_tran;

	// Compute the baseline control input
	cim.mu_tran_baseline << MASS * (- cip.Kp_tran * cim.e_tran_pos				// Proportional term
									- cip.Kd_tran * cim.e_tran_vel				// Derivative term
									- cip.Ki_tran * csm.e_tran_pos_I			// Integral term
									+ cim.x_tran_ref_dot.tail<3>()				// Feedforward term
									- G * e3_basis);							// Weight inversion term
									// - cim.aero.dyn.outer_loop_dynamic_inv;		// Aerodynamic inverstion term

	// Compute the augemented regressor vector
	cim.augmented_outer_loop_regressor << cim.mu_tran_baseline,
										  cim.outer_loop_regressor;

	// Cache the transpose of the tracking error * P * B
	Eigen::Matrix<double, 1, 3> e_transpose_p_b = cim.e_tran.transpose() * cip.P_tran * cip.B_tran;

	// Computing the scalar value output from the dead-zone modification modulation function
	cim.dead_zone_value_translational = ::_shared_::_deadzone_operator_::deadZoneModulationFunction(cim.e_tran.transpose(),
																   						 cip.dead_zone_delta_translational,
																   							cip.dead_zone_e0_translational);

	// Adaptive laws
	cim.K_hat_x_tran_dot = ::_shared_::_adaptive_laws_::AdaptiveLawDeadZoneEMod(-cip.Gamma_x_tran,
																				cim.dead_zone_value_translational,
																				cim.x_tran,
																				e_transpose_p_b,
																				cip.sigma_x_translational,
																				csm.K_hat_x_tran);

	cim.K_hat_r_tran_dot = ::_shared_::_adaptive_laws_::AdaptiveLawDeadZoneEMod(-cip.Gamma_r_tran,
																				cim.dead_zone_value_translational,
																				cim.r_cmd_tran,
																				e_transpose_p_b,
																				cip.sigma_r_translational,
																				csm.K_hat_r_tran);

	cim.Theta_hat_tran_dot = ::_shared_::_adaptive_laws_::AdaptiveLawDeadZoneEMod(cip.Gamma_Theta_tran,
																				  cim.dead_zone_value_translational,
																				  cim.augmented_outer_loop_regressor,
																				  e_transpose_p_b,
																				  cip.sigma_Theta_translational,
																				  csm.Theta_hat_tran);

	// Projection operator - Ball
	if (cip.use_projection_operator)
	{
		// Projection operator K_hat_x
		::_shared_::_projection_operator_::MatrixProjectionOutput<decltype(csm.K_hat_x_tran)> proj_op_output_K_hat_x_translational = 
			::_shared_::_projection_operator_::_ball_::projectionMatrix(csm.K_hat_x_tran, 
																		cim.K_hat_x_tran_dot,
																		cip.projection_x_max_x_translational,
																		cip.projection_epsilon_x_translational);

		cim.K_hat_x_tran_dot = proj_op_output_K_hat_x_translational.projected_matrix;
		cim.proj_op_activated_K_hat_x_translational = proj_op_output_K_hat_x_translational.projection_operator_activated;
		
		// Projection operator K_hat_r
		::_shared_::_projection_operator_::MatrixProjectionOutput<decltype(csm.K_hat_r_tran)> proj_op_output_K_hat_r_translational = 
			::_shared_::_projection_operator_::_ball_::projectionMatrix(csm.K_hat_r_tran,
																		cim.K_hat_r_tran_dot,
																		cip.projection_x_max_r_translational,
																		cip.projection_epsilon_r_translational);

		cim.K_hat_r_tran_dot = proj_op_output_K_hat_r_translational.projected_matrix;
		cim.proj_op_activated_K_hat_r_translational = proj_op_output_K_hat_r_translational.projection_operator_activated;															 

		// Projection operator Theta_hat
		::_shared_::_projection_operator_::MatrixProjectionOutput<decltype(csm.Theta_hat_tran)> proj_op_output_Theta_hat_translational =
			::_shared_::_projection_operator_::_ball_::projectionMatrix(csm.Theta_hat_tran,
																		cim.Theta_hat_tran_dot,
																		cip.projection_x_max_Theta_translational,
																		cip.projection_epsilon_Theta_translational);

		cim.Theta_hat_tran_dot = proj_op_output_Theta_hat_translational.projected_matrix;
		cim.proj_op_activated_Theta_hat_translational = proj_op_output_Theta_hat_translational.projection_operator_activated;
	}

	// Adaptive control law
	cim.mu_tran_adaptive << csm.K_hat_x_tran.transpose() * cim.x_tran
												+ csm.K_hat_r_tran.transpose() * cim.r_cmd_tran
												- csm.Theta_hat_tran.transpose() * cim.augmented_outer_loop_regressor;

	// Compute the total baseline + adaptive control
	cim.mu_tran_I << cim.mu_tran_baseline + cim.mu_tran_adaptive;

}

// Compute the orientation commands and the desired total thrust
void mrac_omega::compute_u1_eta_d()
{
	// Compute the virtual forces in J
	cim.mu_tran_J << cim.R_I_J * cim.mu_tran_I;

	// Compute u1 - Total Thrust
	cim.u(0) = sqrt(  pow(cim.mu_tran_J(0), 2)
					+ pow(cim.mu_tran_J(1), 2)
					+ pow(cim.mu_tran_J(2), 2));

	// Compute the desired roll
	double r1 = cim.mu_tran_J(1) / cim.u(0);
	cim.eta_rot_d(0) = atan2(r1, sqrt(1 - pow(r1,2)));

	// Compute the desired pitch
	cim.eta_rot_d(1) = atan2( -cim.mu_tran_J(0), -cim.mu_tran_J(2) );

	// Compute the internal state for angular rates
	cim.internal_state_roll_d_filter << A_filter_roll_ref * csm.state_roll_d_filter
                                      + B_filter_roll_ref * cim.eta_rot_d(0);

	cim.internal_state_pitch_d_filter << A_filter_pitch_ref * csm.state_pitch_d_filter
                                       + B_filter_pitch_ref * cim.eta_rot_d(1);


	// Compute the desired roll and pitch rates
	cim.eta_rot_rate_d(0) = C_filter_roll_ref * csm.state_roll_d_filter;    
	cim.eta_rot_rate_d(1) = C_filter_pitch_ref * csm.state_pitch_d_filter;	

	// Compute the Jacobian with the current orientation
	cim.Jacobian = ::_shared_::_transformations_::jacobianMatrix(cim.eta_rot(0), cim.eta_rot(1));

	// Compute the desired angular velocities
	cim.omega_rot_d << cim.Jacobian * cim.eta_rot_rate_d;
	
	// Compute the internal state for angular accelration
	cim.internal_state_omega_x_d_filter << A_filter_roll_dot_ref * csm.state_omega_x_d_filter
																				+ B_filter_roll_dot_ref * cim.omega_rot_d(0);

	cim.internal_state_omega_y_d_filter << A_filter_pitch_dot_ref * csm.state_omega_y_d_filter
																				+ B_filter_pitch_dot_ref * cim.omega_rot_d(1);

	cim.internal_state_omega_z_d_filter << A_filter_yaw_dot_ref * csm.state_omega_z_d_filter
																				+ B_filter_yaw_dot_ref * cim.omega_rot_d(2);

	// Compute the desired angular acceleration
	cim.alpha_rot_d(0) = C_filter_roll_dot_ref * csm.state_omega_x_d_filter;
	cim.alpha_rot_d(1) = C_filter_pitch_dot_ref * csm.state_omega_y_d_filter;
	cim.alpha_rot_d(2) = C_filter_yaw_dot_ref * csm.state_omega_z_d_filter;	
}

// Compute the rotational control
void mrac_omega::compute_rotational_control()
{
	// Compute the error in the orientation [actual - desired]
	cim.e_rot_eta(0) = cim.eta_rot(0) - cim.eta_rot_d(0);
	cim.e_rot_eta(1) = cim.eta_rot(1) - cim.eta_rot_d(1);
	cim.e_rot_eta(2) = ::_shared_::_compute_::makeYawAngularErrorContinuous(cim.eta_rot(2), cim.eta_rot_d(2)); 

	// Compute the error in the angular velocity
	cim.e_rot_omega << cim.omega_rot - csm.omega_rot_ref;

	// Compute the error between the angular velocity reference model and the desired angular velocity
	cim.e_rot_omega_ref << csm.omega_rot_ref - cim.omega_rot_d;

	// Compute the command angular velocity
	cim.omega_cmd << cip.Kp_omega_ref * cim.omega_rot_d - cip.Ki_omega_ref * csm.e_rot_omega_ref_I + cim.alpha_rot_d;

	// Compute the derivative of the reference angular velocity
	cim.omega_rot_ref_dot << cip.A_ref_rot * csm.omega_rot_ref + cip.B_ref_rot * cim.omega_cmd;

	// Baseline PID controller
	cim.tau_rot_baseline << inertia_matrix_q * ( -cip.Kp_rot * cim.e_rot_eta               					 // Proportional term
													-cip.Kd_rot * cim.e_rot_omega          					 // Derivative term
												 	-cip.Ki_rot * csm.e_rot_eta_I          					 // Integral term
												 	+ cim.omega_rot_ref_dot)                  				 // Feedforward term
													+ cim.omega_rot.cross(inertia_matrix_q * cim.omega_rot); // Dynamic inversion term
													// - cim.aero.dyn.inner_loop_dynamic_inv;                // Aero inversion term

	// Compute the augmented regressor vector
	cim.augmented_inner_loop_regressor << cim.tau_rot_baseline, cim.inner_loop_regressor;

	// Cache the tranpose of the tracking error * P * B
	Eigen::Matrix<double, 1, 3> e_transpose_p_b = cim.e_rot_omega.transpose() * cip.P_rot * cip.B_rot;

	// Computing the scalar value output from the dead-zone modification modulation function
	cim.dead_zone_value_rotational = ::_shared_::_deadzone_operator_::deadZoneModulationFunction(cim.e_rot_omega.transpose(),
																								 cip.dead_zone_delta_rotational,
																								 cip.dead_zone_e0_rotational);

	// Adaptive laws
	cim.K_hat_x_rot_dot = ::_shared_::_adaptive_laws_::AdaptiveLawDeadZoneEMod(-cip.Gamma_x_rot,
																				cim.dead_zone_value_rotational,
																				cim.omega_rot,
																				e_transpose_p_b,
																				cip.sigma_x_rotational,
																				csm.K_hat_x_rot);

	cim.K_hat_r_rot_dot = ::_shared_::_adaptive_laws_::AdaptiveLawDeadZoneEMod(-cip.Gamma_r_rot,
																				cim.dead_zone_value_rotational,
																				cim.omega_cmd,
																				e_transpose_p_b,
																				cip.sigma_r_rotational,
																				csm.K_hat_r_rot);

	cim.Theta_hat_rot_dot = ::_shared_::_adaptive_laws_::AdaptiveLawDeadZoneEMod(cip.Gamma_Theta_rot,
																				  cim.dead_zone_value_rotational,
																				  cim.augmented_inner_loop_regressor,
																				  e_transpose_p_b,
																				  cip.sigma_Theta_rotational,
																				  csm.Theta_hat_rot);	

	// Projection operator - Ball
	if (cip.use_projection_operator)
	{
		// Projection operator K_hat_x
		::_shared_::_projection_operator_::MatrixProjectionOutput<decltype(csm.K_hat_x_rot)> proj_op_output_K_hat_x_rotational = 
			::_shared_::_projection_operator_::_ball_::projectionMatrix(csm.K_hat_x_rot,
															 			cim.K_hat_x_rot_dot,
																		cip.projection_x_max_x_rotational,
																		cip.projection_epsilon_x_rotational);

		cim.K_hat_x_rot_dot = proj_op_output_K_hat_x_rotational.projected_matrix;
		cim.proj_op_activated_K_hat_x_rotational = proj_op_output_K_hat_x_rotational.projection_operator_activated;

		// Projection operator K_hat_r
		::_shared_::_projection_operator_::MatrixProjectionOutput<decltype(csm.K_hat_r_rot)> proj_op_output_K_hat_r_rotational = 
			::_shared_::_projection_operator_::_ball_::projectionMatrix(csm.K_hat_r_rot,
																		cim.K_hat_r_rot_dot,
																		cip.projection_x_max_r_rotational,
																		cip.projection_epsilon_r_rotational);
									
		cim.K_hat_r_rot_dot = proj_op_output_K_hat_r_rotational.projected_matrix;
		cim.proj_op_activated_K_hat_r_rotational = proj_op_output_K_hat_r_rotational.projection_operator_activated;

		// Projection operator Theta_hat
		::_shared_::_projection_operator_::MatrixProjectionOutput<decltype(csm.Theta_hat_rot)> proj_op_output_Theta_hat_rotational = 
			::_shared_::_projection_operator_::_ball_::projectionMatrix(csm.Theta_hat_rot,
																		cim.Theta_hat_rot_dot,
																		cip.projection_x_max_Theta_rotational,
																		cip.projection_epsilon_Theta_rotational);

		cim.Theta_hat_rot_dot = proj_op_output_Theta_hat_rotational.projected_matrix;
		cim.proj_op_activated_Theta_hat_rotational = proj_op_output_Theta_hat_rotational.projection_operator_activated;
	}													

	// Adaptive control law
	cim.tau_rot_adaptive << csm.K_hat_x_rot.transpose() * cim.omega_rot
												+ csm.K_hat_r_rot.transpose() * cim.omega_cmd
												- csm.Theta_hat_rot.transpose() * cim.augmented_inner_loop_regressor;			

	// Total rotational control input
	cim.tau_rot << cim.tau_rot_baseline + cim.tau_rot_adaptive;							

	// Assign the control inputs
	cim.u(1) = cim.tau_rot(0);
	cim.u(2) = cim.tau_rot(1);
	cim.u(3) = cim.tau_rot(2);												
}


// Function to compute the normalized thrusts
void mrac_omega::compute_normalized_thrusts()
{
    // Compute the individual thrusts in Newtons
    cim.Thrust << mixer_matrix_x8 * cim.u;

    // Saturate each element of the Thrust vector between MIN_THRUST and MAX_THRUST
    cim.Sat_Thrust = (cim.Thrust.cwiseMin(MAX_THRUST).cwiseMax(MIN_THRUST));

    // Compute the final control inputs
    control_input(0) = ::_shared_::_compute_::evaluatePolynomial(thrust_polynomial_coeff_x8, cim.Sat_Thrust(0));
    control_input(1) = ::_shared_::_compute_::evaluatePolynomial(thrust_polynomial_coeff_x8, cim.Sat_Thrust(1));
    control_input(2) = ::_shared_::_compute_::evaluatePolynomial(thrust_polynomial_coeff_x8, cim.Sat_Thrust(2));
    control_input(3) = ::_shared_::_compute_::evaluatePolynomial(thrust_polynomial_coeff_x8, cim.Sat_Thrust(3));
    control_input(4) = ::_shared_::_compute_::evaluatePolynomial(thrust_polynomial_coeff_x8, cim.Sat_Thrust(4));
    control_input(5) = ::_shared_::_compute_::evaluatePolynomial(thrust_polynomial_coeff_x8, cim.Sat_Thrust(5));
    control_input(6) = ::_shared_::_compute_::evaluatePolynomial(thrust_polynomial_coeff_x8, cim.Sat_Thrust(6));
    control_input(7) = ::_shared_::_compute_::evaluatePolynomial(thrust_polynomial_coeff_x8, cim.Sat_Thrust(7));

}

// Function that is called in sim-bridge.cpp
void mrac_omega::run(const double time_step_rk4_) {

    // Process the dynamics --------------------------------------------------------
    // 1. Compute the aerodynamics

	// 2. Compute the outerloop regressor vector
	compute_outer_loop_regressor();

    // 3. Compute the translational control input
    compute_translational_control_in_I();

    // 4. Compute the thrust needed and the desired angles
    compute_u1_eta_d();

	// 5. Compute the innerlooop regressor vector
	compute_inner_loop_regressor();

    // 6. Compute the rotational control input
    compute_rotational_control();

    // 7. Compute the normalized thrust - Final Step
    compute_normalized_thrusts();

    // 8. Do the integration
    rk4.do_step(boost::bind(&mrac_omega::model, this, bph::_1, bph::_2, bph::_3),
                y, cim.t, time_step_rk4_);
    
    // Capture the time after the execution of the controller
    cim.alg_end_time = std::chrono::high_resolution_clock::now();
    
    // Calculate the duration of the execution 
    cim.alg_duration = std::chrono::duration_cast<std::chrono::microseconds>(
                            cim.alg_end_time - cim.alg_start_time).count();

    // 9. Log the Data after all the calculataions
    this->LogData();
    
}

// Function that is called during the constructor. 
bool mrac_omega::InitiateLogging()
{
    auto status = _logger_::_filesystem_::setupControllerLogging(this->m_logger, "x8" ,"MRAC_OMEGA");
    return status;
}

// Funciton that sets up the headers for the log file
void mrac_omega::ConfigureHeaders()
{

    // Create the oss object
    std::ostringstream oss;

    oss << ", "
	    << "Controller Time [s], "
		<< "Algorithm Execution Time [us], "
		<< "Coefficient of Lift Upper wing [-], "
        << "Coefficient of Lift Lower wing [-], "
        << "Coefficient of Lift Left stabilizer [-], "
        << "Coefficient of Lift Right stabilizer [-], "
        << "Coefficient of Drag Upper wing [-], "
        << "Coefficient of Drag Lower wing [-], "
        << "Coefficient of Drag Left stabilizer [-], "
        << "Coefficient of Drag Right stabilizer [-], "
        << "Coefficient of Aero Moment Upper wing [-], "
        << "Coefficient of Aero Moment Lower wing [-], "
        << "Coefficient of Aero Moment Left stabilizer [-], "
        << "Coefficient of Aero Moment Right stabilizer [-], "
		<< "Squared Norm of Velocity [-], "
        << "Angle of attack COG [rad], "
        << "Angle of attack Upper wing [rad], "
        << "Angle of attack Lower wing [rad], "
        << "Sideslip angle COG [rad], "
        << "Sideslip angle Left stabilizer [rad], "
        << "Sideslip angle Right stabilizer [rad], "
		<< "Aero F_x [N], "
        << "Aero F_y [N], "
        << "Aero F_z [N], "
        << "Aero M_x [Nm], "
        << "Aero M_y [Nm], "
        << "Aero M_z [Nm], "
		<< "outer_loop_dyn_inv F_x [N], "
		<< "outer_loop_dyn_inv F_y [N], "
		<< "outer_loop_dyn_inv F_z [N], "
        << "inner_loop_dyn_inv M_x [Nm], "
		<< "inner_loop_dyn_inv M_y [Nm], "
		<< "inner_loop_dyn_inv M_z [Nm], "
		<< "Position x [m], "
        << "Position y [m], "
        << "Position z [m], "
        << "Velocity x [m/s], "
        << "Velocity y [m/s], "
        << "Velocity z [m/s], "
		<< "Reference Position x [m], "
        << "Reference Position y [m], "
        << "Reference Position z [m], "
        << "Reference Velocity x [m/s], "
        << "Reference Velocity y [m/s], "
        << "Reference Velocity z [m/s], "
		<< "Reference Acceleration x [m/s^2], "
        << "Reference Acceleration y [m/s^2], "
        << "Reference Acceleration z [m/s^2], "
        << "User Position x [m], "
        << "User Position y [m], "
        << "User Position z [m], "
        << "User Velocity x [m/s], "
        << "User Velocity y [m/s], "
        << "User Velocity z [m/s], "
        << "User Acceleration x [m/s^2], "
        << "User Acceleration y [m/s^2], "
        << "User Acceleration z [m/s^2], "
		<< "Error in x [m], "
        << "Error in y [m], "
        << "Error in z [m], "
        << "Error in vx [m/s], "
        << "Error in vy [m/s], "
        << "Error in vz [m/s], "
        << "Integral Error in x [m], "
        << "Integral Error in y [m], "
        << "Integral Error in z [m], "
        << "Reference Error in x [m], "
        << "Reference Error in y [m], "
        << "Reference Error in z [m], "
        << "Integral Reference Error in x [m], "
        << "Integral Reference Error in y [m], "
        << "Integral Reference Error in z [m], "
        << "Reference Command in x [-], "
        << "Reference Command in y [-], "
        << "Reference Command in z [-], "
        << "mu_baseline x [N], " 
        << "mu_baseline y [N], " 
        << "mu_baseline z [N], " 
        << "mu_adaptive x [N], "
        << "mu_adaptive y [N], "
        << "mu_adaptive z [N], "
        << "mu in Body x [N], "
        << "mu in Body y [N], "
        << "mu in Body z [N], "
        << "Desired Phi [rad], "
        << "Desired Theta [rad], "
        << "Desired Psi [rad], "
        << "Desired w_x [rad/s], "
        << "Desired w_y [rad/s], "
        << "Desired w_z [rad/s], "
        << "Desired w_x_dot [rad/s^2], "
        << "Desired w_y_dot [rad/s^2], "
        << "Desired w_z_dot [rad/s^2], "
        << "Phi [rad], "
        << "Theta [rad], "
        << "Psi [rad], "
        << "w_x [rad/s], "
        << "w_y [rad/s], "
        << "w_z [rad/s], "
        << "Command w_x [rad/s], "
        << "Command w_y [rad/s], "
        << "Command w_z [rad/s], "
        << "Reference w_x [rad/s], "
        << "Reference w_y [rad/s], "
        << "Reference w_z [rad/s], "
        << "Reference alpha_x [rad/s^2], "
        << "Reference alpha_y [rad/s^2], "
        << "Reference alpha_z [rad/s^2], "
        << "Error in phi [rad], "
        << "Error in theta [rad], "
        << "Error in psi [rad], "
        << "Integral Error in phi [rad], "
        << "Integral Error in theta [rad], "
        << "Integral Error in psi [rad], "
        << "Reference error in w_x [rad/s], "
        << "Reference error in w_y [rad/s], "
        << "Reference error in w_z [rad/s], "
        << "Error in w_x [rad/s], "
        << "Error in w_y [rad/s], "
        << "Error in w_z [rad/s], "
        << "Integral reference error in w_x [rad/s], "
        << "Integral reference error in w_y [rad/s], "
        << "Integral reference error in w_z [rad/s], "
        << "Tau x baseline [Nm], "
        << "Tau y baseline [Nm], "
        << "Tau z baseline [Nm], "
        << "Tau x adaptive [Nm], "
        << "Tau y adaptive [Nm], "
        << "Tau z adaptive [Nm], "
        << "Control input 1 [N], "
        << "Control input 2 [N], "
        << "Control input 3 [N], "
        << "Control input 4 [N], "
        << "Thrust Motor 1 [N], "
        << "Thrust Motor 2 [N], "
        << "Thrust Motor 3 [N], "
        << "Thrust Motor 4 [N], "
        << "Thrust Motor 5 [N], "
        << "Thrust Motor 6 [N], "
        << "Thrust Motor 7 [N], "
        << "Thrust Motor 8 [N], "
        << "Normalized Thrust 1 [-], "
        << "Normalized Thrust 2 [-], "
        << "Normalized Thrust 3 [-], "
        << "Normalized Thrust 4 [-], "
        << "Normalized Thrust 5 [-], "
        << "Normalized Thrust 6 [-], "
        << "Normalized Thrust 7 [-], "
        << "Normalized Thrust 8 [-], "
        << "dead_zone_value_translational [-], "
        << "dead_zone_value_rotational [-], "
        << "proj_op_activated_K_hat_x_translational [-], "
        << "proj_op_activated_K_hat_r_translational [-], "
        << "proj_op_activated_Theta_hat_translational [-], "
        << "proj_op_activated_K_hat_x_rotational [-], "
        << "proj_op_activated_K_hat_r_rotational [-], "
        << "proj_op_activated_Theta_hat_rotational [-], "
		;

        // Use the deader function to create the header for the matrix data
        ::_shared_::_serialize_::generateMatrixHeaders(oss, "K_hat_x_translational", csm.K_hat_x_tran, "[-]");
        ::_shared_::_serialize_::generateMatrixHeaders(oss, "K_hat_r_translational", csm.K_hat_r_tran, "[-]");
        ::_shared_::_serialize_::generateMatrixHeaders(oss, "Theta_hat_translational", csm.Theta_hat_tran, "[-]");
        ::_shared_::_serialize_::generateMatrixHeaders(oss, "K_hat_x_rotational", csm.K_hat_x_rot, "[-]");
        ::_shared_::_serialize_::generateMatrixHeaders(oss, "K_hat_r_rotational", csm.K_hat_r_rot, "[-]");
        ::_shared_::_serialize_::generateMatrixHeaders(oss, "Theta_hat_rotational", csm.Theta_hat_rot, "[-]");    

    try {
        BOOST_LOG_SCOPED_THREAD_TAG("Tag", "ControllerTag");

        BOOST_LOG(m_logger.GetControlLogger()) << oss.str();

        _message_::SIMULATOR_INFO("[SIMCTL]: WROTE MRAC OMEGA LOG HEADER DATA");
    }
    catch (const std::exception& e) {
        _message_::SIMULATOR_ERROR("[SIMCTL]: FAILED TO WRITE MRAC OMEGA LOG HEADER DATA", e.what());
    }

}

void mrac_omega::LogData()
{
    // Log the data
    std::ostringstream oss;

    oss << ", "
        << cim.t << ", "
        << cim.alg_duration << ", "
        // << cim.aero.coeff.Cl_up << ", "
        // << cim.aero.coeff.Cl_lw << ", "
        // << cim.aero.coeff.Cl_lt << ", "
        // << cim.aero.coeff.Cl_rt << ", "
        // << cim.aero.coeff.Cd_up << ", "
        // << cim.aero.coeff.Cd_lw << ", "
        // << cim.aero.coeff.Cd_lt << ", "
        // << cim.aero.coeff.Cd_rt << ", "
        // << cim.aero.coeff.Cm_up << ", "
        // << cim.aero.coeff.Cm_lw << ", "
        // << cim.aero.coeff.Cm_lt << ", "
        // << cim.aero.coeff.Cm_rt << ", "
        // << cim.aero.states.v_norm_sq << ", "
        // << cim.aero.states.alpha << ", "
        // << cim.aero.states.alpha_up << ", "
        // << cim.aero.states.alpha_lw << ", "
        // << cim.aero.states.beta << ", "
        // << cim.aero.states.beta_lt << ", "
        // << cim.aero.states.beta_rt << ", "
        // << cim.aero.dyn.F_aero(0) << ", "
        // << cim.aero.dyn.F_aero(1) << ", "
        // << cim.aero.dyn.F_aero(2) << ", "
        // << cim.aero.dyn.M_aero(0) << ", "
        // << cim.aero.dyn.M_aero(1) << ", "
        // << cim.aero.dyn.M_aero(2) << ", "
        // << cim.aero.dyn.outer_loop_dynamic_inv(0) << ", "
        // << cim.aero.dyn.outer_loop_dynamic_inv(1) << ", "
        // << cim.aero.dyn.outer_loop_dynamic_inv(2) << ", "
        // << cim.aero.dyn.inner_loop_dynamic_inv(0) << ", "
        // << cim.aero.dyn.inner_loop_dynamic_inv(1) << ", "
        // << cim.aero.dyn.inner_loop_dynamic_inv(2) << ", "
		<< 0 << ", " << 0 << ", " << 0 << ", " << 0 << ", " // 4
		<< 0 << ", " << 0 << ", " << 0 << ", " << 0 << ", " // 8
		<< 0 << ", " << 0 << ", " << 0 << ", " << 0 << ", " // 12
		<< 0 << ", " << 0 << ", " << 0 << ", " << 0 << ", " // 16
		<< 0 << ", " << 0 << ", " << 0 << ", " << 0 << ", " // 20
		<< 0 << ", " << 0 << ", " << 0 << ", " << 0 << ", " // 24
		<< 0 << ", " << 0 << ", " << 0 << ", " << 0 << ", " // 28
		<< 0 << ", " << 0 << ", " << 0 << ", " 				// 31
        << cim.x_tran(0) << ", "
        << cim.x_tran(1) << ", "
        << cim.x_tran(2) << ", "
        << cim.x_tran(3) << ", "
        << cim.x_tran(4) << ", "
        << cim.x_tran(5) << ", "
        << csm.x_tran_ref(0) << ", "
        << csm.x_tran_ref(1) << ", "
        << csm.x_tran_ref(2) << ", "
        << csm.x_tran_ref(3) << ", "
        << csm.x_tran_ref(4) << ", "
        << csm.x_tran_ref(5) << ", "
        << cim.x_tran_ref_dot(3) << ", "
        << cim.x_tran_ref_dot(4) << ", "
        << cim.x_tran_ref_dot(5) << ", "
        << cim.r_user(0) << ", "
        << cim.r_user(1) << ", "
        << cim.r_user(2) << ", "
        << cim.r_dot_user(0) << ", "
        << cim.r_dot_user(1) << ", "
        << cim.r_dot_user(2) << ", "
        << cim.r_ddot_user(0) << ", "
        << cim.r_ddot_user(1) << ", "
        << cim.r_ddot_user(2) << ", "
        << cim.e_tran(0) << ", "
        << cim.e_tran(1) << ", "
        << cim.e_tran(2) << ", "
        << cim.e_tran(3) << ", "
        << cim.e_tran(4) << ", "
        << cim.e_tran(5) << ", "
        << csm.e_tran_pos_I(0) << ", "
        << csm.e_tran_pos_I(1) << ", "
        << csm.e_tran_pos_I(2) << ", "
        << cim.e_tran_pos_ref(0) << ", "
        << cim.e_tran_pos_ref(1) << ", "
        << cim.e_tran_pos_ref(2) << ", "
        << csm.e_tran_pos_ref_I(0) << ", "
        << csm.e_tran_pos_ref_I(1) << ", "
        << csm.e_tran_pos_ref_I(2) << ", "
        << cim.r_cmd_tran(0) << ", "
        << cim.r_cmd_tran(1) << ", "
        << cim.r_cmd_tran(2) << ", "
        << cim.mu_tran_baseline(0) << ", "
        << cim.mu_tran_baseline(1) << ", "
        << cim.mu_tran_baseline(2) << ", "
        << cim.mu_tran_adaptive(0) << ", "
        << cim.mu_tran_adaptive(1) << ", "
        << cim.mu_tran_adaptive(2) << ", "
        << cim.mu_tran_J(0) << ", "
        << cim.mu_tran_J(1) << ", "
        << cim.mu_tran_J(2) << ", "
        << cim.eta_rot_d(0) << ", "
        << cim.eta_rot_d(1) << ", "
        << cim.eta_rot_d(2) << ", "
        << cim.omega_rot_d(0) << ", "
        << cim.omega_rot_d(1) << ", "
        << cim.omega_rot_d(2) << ", "
        << cim.alpha_rot_d(0) << ", "
        << cim.alpha_rot_d(1) << ", "
        << cim.alpha_rot_d(2) << ", "
        << cim.eta_rot(0) << ", "
        << cim.eta_rot(1) << ", "
        << cim.eta_rot(2) << ", "
        << cim.omega_rot(0) << ", "
        << cim.omega_rot(1) << ", "
        << cim.omega_rot(2) << ", "
        << cim.omega_cmd(0) << ", "
        << cim.omega_cmd(1) << ", "
        << cim.omega_cmd(2) << ", "
        << csm.omega_rot_ref(0) << ", "
        << csm.omega_rot_ref(1) << ", "
        << csm.omega_rot_ref(2) << ", "
        << cim.omega_rot_ref_dot(0) << ", "
        << cim.omega_rot_ref_dot(1) << ", "
        << cim.omega_rot_ref_dot(2) << ", "
        << cim.e_rot_eta(0) << ", "
        << cim.e_rot_eta(1) << ", "
        << cim.e_rot_eta(2) << ", "
        << csm.e_rot_eta_I(0) << ", "
        << csm.e_rot_eta_I(1) << ", "
        << csm.e_rot_eta_I(2) << ", "
        << cim.e_rot_omega_ref(0) << ", "
        << cim.e_rot_omega_ref(1) << ", "
        << cim.e_rot_omega_ref(2) << ", "
        << cim.e_rot_omega(0) << ", "
        << cim.e_rot_omega(1) << ", "
        << cim.e_rot_omega(2) << ", "
        << csm.e_rot_omega_ref_I(0) << ", "
        << csm.e_rot_omega_ref_I(1) << ", "
        << csm.e_rot_omega_ref_I(2) << ", "
        << cim.tau_rot_baseline(0) << ", "
        << cim.tau_rot_baseline(1) << ", "
        << cim.tau_rot_baseline(2) << ", "
        << cim.tau_rot_adaptive(0) << ", "
        << cim.tau_rot_adaptive(1) << ", "
        << cim.tau_rot_adaptive(2) << ", "
        << cim.u(0) << ", "
        << cim.u(1) << ", "
        << cim.u(2) << ", "
        << cim.u(3) << ", "
        << cim.Thrust(0) << ", "
        << cim.Thrust(1) << ", "
        << cim.Thrust(2) << ", "
        << cim.Thrust(3) << ", "
        << cim.Thrust(4) << ", "
        << cim.Thrust(5) << ", "
        << cim.Thrust(6) << ", "
        << cim.Thrust(7) << ", "
        << control_input(0) << ", "
        << control_input(1) << ", "
        << control_input(2) << ", "
        << control_input(3) << ", "
        << control_input(4) << ", "
        << control_input(5) << ", "
        << control_input(6) << ", "
        << control_input(7) << ", "
        << cim.dead_zone_value_translational << ", "
        << cim.dead_zone_value_rotational << ", "
        << cim.proj_op_activated_K_hat_x_translational << ", "
        << cim.proj_op_activated_K_hat_r_translational << ", "
        << cim.proj_op_activated_Theta_hat_translational << ", "
        << cim.proj_op_activated_K_hat_x_rotational << ", "
        << cim.proj_op_activated_K_hat_r_rotational << ", "
        << cim.proj_op_activated_Theta_hat_rotational << ", "
        ;

        // Use helper function to output the matrix data
        ::_shared_::_serialize_::appendEigenData(oss, csm.K_hat_x_tran);
        ::_shared_::_serialize_::appendEigenData(oss, csm.K_hat_r_tran);
        ::_shared_::_serialize_::appendEigenData(oss, csm.Theta_hat_tran);
        ::_shared_::_serialize_::appendEigenData(oss, csm.K_hat_x_rot);
        ::_shared_::_serialize_::appendEigenData(oss, csm.K_hat_r_rot);
        ::_shared_::_serialize_::appendEigenData(oss, csm.Theta_hat_rot);

    try {
        BOOST_LOG_SCOPED_THREAD_TAG("Tag", "ControllerTag");

        BOOST_LOG(m_logger.GetControlLogger()) << oss.str();
    }
    catch (const std::exception& e) {
        _message_::SIMULATOR_ERROR("[SIMCTL]: FAILED TO WRITE MRAC OMEGA LOG HEADER DATA", e.what());
    }
}


} // namespace _mrac_omega_

} // namespace _x8_

} // namespace _acsl_
