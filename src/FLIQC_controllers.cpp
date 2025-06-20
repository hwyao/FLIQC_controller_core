/**
 * @file FLIQC_controllers.cpp
 * @brief Implementation of the FLIQC controller classes.
 *  Copyright (C) 2025 Haowen Yao

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */
#include "FLIQC_controller_core/FLIQC_controllers.hpp"
#include <stdexcept>
#include <iostream>
#include <fstream>
#include <filesystem>

namespace FLIQC_controller_core {

    FLIQC_controller_joint_velocity_basic::FLIQC_controller_joint_velocity_basic(int dim_q)
        : nJoint(dim_q), lcqp_solver() {
        // nothing to do here
    }

    Eigen::VectorXd FLIQC_controller_joint_velocity_basic::runController(const FLIQC_state_input& state_input, 
                                                                         const std::vector<FLIQC_distance_input> &dist_inputs,
                                                                               FLIQC_control_output& control_output) {
        // filter out the active distance inputs: if the distance is less than the active threshold, then it is active
        std::vector<FLIQC_distance_input> active_dist_inputs;
        for (auto dist_input_i : dist_inputs) {
            if (dist_input_i.distance <= active_threshold) {
                active_dist_inputs.push_back(dist_input_i);
            }
        }

        if (active_dist_inputs.size() == 0) {
            // no active distance inputs, return the guide velocity
            return state_input.q_dot_guide;
        }
        int nContacts = active_dist_inputs.size();
        int nVariables = nJoint + nContacts;

        // construct the input of the LCQProblem: [0] The cost input
        Eigen::MatrixXd quad_cost = Eigen::MatrixXd::Zero(nJoint, nJoint);
        Eigen::VectorXd lin_cost = Eigen::VectorXd::Zero(nJoint);
        if (quad_cost_type == FLIQC_QUAD_COST_IDENTITY) {
            quad_cost += Eigen::MatrixXd::Identity(nJoint, nJoint);
        } else if (quad_cost_type == FLIQC_QUAD_COST_JOINT_VELOCITY_ERROR){
            quad_cost += Eigen::MatrixXd::Identity(nJoint, nJoint);
            lin_cost  += -2 * state_input.q_dot_guide;
        } else if (quad_cost_type == FLIQC_QUAD_COST_MASS_MATRIX) {
            quad_cost += state_input.M;
        } else if (quad_cost_type == FLIQC_QUAD_COST_MASS_MATRIX_VELOCITY_ERROR) {
            Eigen::MatrixXd W = weight_on_mass_matrix.asDiagonal();
            quad_cost += W * state_input.M;
            lin_cost  += -2 * W * state_input.M * state_input.q_dot_guide;
        } else {
            throw std::invalid_argument("The type of the cost function" + std::to_string(quad_cost_type) + " is not supported.");
        }

        if (linear_cost_type == FLIQC_LINEAR_COST_NONE) {
            //lin_cost += Eigen::VectorXd::Zero(nJoint);              // do nothing
        } else {
            throw std::invalid_argument("The type of the linear cost function " + std::to_string(linear_cost_type) + " is not supported.");
        }

        lcqp_input.Q = Eigen::MatrixXd::Zero(nVariables, nVariables);
        lcqp_input.Q << quad_cost,                                 Eigen::MatrixXd::Zero(nJoint, nContacts),
                        Eigen::MatrixXd::Zero(nContacts, nJoint),  Eigen::MatrixXd::Identity(nContacts, nContacts) * lambda_cost_penalty;

        lcqp_input.g = Eigen::VectorXd::Zero(nVariables);
        lcqp_input.g << lin_cost, Eigen::VectorXd::Zero(nContacts);
        
        // construct the input of the LCQProblem: [1] all the close-constant input.
        lcqp_input.L = Eigen::MatrixXd::Zero(nContacts, nVariables);
        lcqp_input.L.block(0, nJoint, nContacts, nContacts) = Eigen::MatrixXd::Identity(nContacts, nContacts);
        lcqp_input.lbL = Eigen::VectorXd::Zero(nContacts);
        if (enable_lambda_constraint_in_L){
            lcqp_input.ubL = Eigen::VectorXd::Constant(nContacts, lambda_max);
        }
        
        lcqp_input.R = Eigen::MatrixXd::Zero(nContacts, nVariables);
        lcqp_input.lbR = Eigen::VectorXd::Zero(nContacts);
        if (enable_esc_vel_constraint){
            lcqp_input.ubR = Eigen::VectorXd::Constant(nContacts, esc_vel_max);
        }

        lcqp_input.A = Eigen::MatrixXd::Zero(nJoint, nVariables);
        lcqp_input.A.block(0, 0, nJoint, nJoint) = Eigen::MatrixXd::Identity(nJoint, nJoint);
        lcqp_input.lbA = state_input.q_dot_guide;
        lcqp_input.ubA = state_input.q_dot_guide;

        lcqp_input.lb = Eigen::VectorXd::Zero(nVariables);
        lcqp_input.ub = Eigen::VectorXd::Constant(nVariables, std::numeric_limits<double>::max());
        if (enable_lambda_constraint_in_x){
            lcqp_input.ub.tail(nContacts) = Eigen::VectorXd::Constant(nContacts, lambda_max);
        } 
        
        if (q_dot_max.size() != 0) {
            lcqp_input.lb.head(nJoint) = -q_dot_max;
            lcqp_input.ub.head(nJoint) = q_dot_max;
        }
        else{
            lcqp_input.lb.head(nJoint) = Eigen::VectorXd::Constant(nJoint, -std::numeric_limits<double>::max());
            //lcqp_input.ub.head(nJoint) = Eigen::VectorXd::Constant(nJoint, std::numeric_limits<double>::max()); // already set
        }

        // construct the input of the LCQProblem: [2] all the distance-dependent input.
        Eigen::MatrixXd nullspace_projector;
        if (enable_nullspace_projector_in_A) {
            Eigen::MatrixXd Jpos = state_input.J.block(0, 0, 3, nJoint);
            nullspace_projector = Eigen::MatrixXd::Identity(nJoint, nJoint) - Jpos.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(Jpos);
        } else {
            nullspace_projector = Eigen::MatrixXd::Identity(nJoint, nJoint);
        }
        for (int i = 0; i < active_dist_inputs.size(); i++) {
            // lbR
            lcqp_input.lbR(i) = -active_dist_inputs[i].distance + eps;
            // R
            lcqp_input.R.block(i, 0, 1, nJoint) = dt * active_dist_inputs[i].projector_control_to_dist;
            // A
            if(active_dist_inputs[i].projector_dist_to_control.size() != 0){
                lcqp_input.A.block(0, nJoint + i, nJoint, 1) = - nullspace_projector 
                    * active_dist_inputs[i].projector_dist_to_control;
            }
            else{
                // This is then the pseudo-inverse of the projector_control_to_dist vector
                lcqp_input.A.block(0, nJoint + i, nJoint, 1) = - nullspace_projector 
                    * (1.0 / active_dist_inputs[i].projector_control_to_dist.norm() / active_dist_inputs[i].projector_control_to_dist.norm()) 
                    * active_dist_inputs[i].projector_control_to_dist.transpose();
            }
        }

        // construct the input of the LCQProblem: [3] initial guess
        lcqp_input.x0.resize(0);
        lcqp_input.y0.resize(0);
        
        lcqp_solver.nVariables = nVariables;
        lcqp_solver.nConstraints = nContacts;
        lcqp_solver.nComplementarity = nContacts;
        
        lcqp_solver.runSolver(lcqp_input, lcqp_output);
        control_output.x = lcqp_output.x;
        control_output.y = lcqp_output.y;
        
        Eigen::VectorXd result = lcqp_output.x.head(nJoint);
        return result;
    }

}