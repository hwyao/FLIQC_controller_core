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

    Eigen::VectorXd FLIQC_controller_joint_velocity_basic::runController(const Eigen::VectorXd& vel_guide, 
                                                                         const FLIQC_cost_input& cost_input, 
                                                                         const std::vector<FLIQC_distance_input> &dist_inputs) {
        // filter out the active distance inputs: if the distance is less than the active threshold, then it is active
        std::vector<FLIQC_distance_input> active_dist_inputs;
        for (auto dist_input_i : dist_inputs) {
            if (dist_input_i.distance <= active_threshold) {
                active_dist_inputs.push_back(dist_input_i);
            }
        }

        if (active_dist_inputs.size() == 0) {
            // no active distance inputs, return the guide velocity
            return vel_guide;
        }

        int nContacts = active_dist_inputs.size();
        int nVariables = nJoint + nContacts;

        // construct the input of the LCQProblem: [0] The cost input
        lcqp_input.Q = Eigen::MatrixXd::Zero(nVariables, nVariables);
        lcqp_input.Q << cost_input.Q,                              Eigen::MatrixXd::Zero(nJoint, nContacts),
                        Eigen::MatrixXd::Zero(nContacts, nJoint),  Eigen::MatrixXd::Identity(nContacts, nContacts);

        lcqp_input.g = Eigen::VectorXd::Zero(nVariables);
        lcqp_input.g << cost_input.g, Eigen::VectorXd::Zero(nContacts);
        
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
        lcqp_input.lbA = vel_guide;
        lcqp_input.ubA = vel_guide;

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
        for (int i = 0; i < active_dist_inputs.size(); i++) {
            // lbR
            lcqp_input.lbR(i) = -active_dist_inputs[i].distance + eps;
            // R
            lcqp_input.R.block(i, 0, 1, nJoint) = dt * active_dist_inputs[i].projector_control_to_dist;
            // A
            if(active_dist_inputs[i].projector_dist_to_control.size() != 0){
                lcqp_input.A.block(0, nJoint + i, nJoint, 1) = active_dist_inputs[i].projector_dist_to_control;
            }
            else{
                // This is then the pseudo-inverse of the projector_control_to_dist vector
                lcqp_input.A.block(0, nJoint + i, nJoint, 1) = 
                - active_dist_inputs[i].projector_control_to_dist.transpose() * 
                ((active_dist_inputs[i].projector_control_to_dist * active_dist_inputs[i].projector_control_to_dist.transpose()).inverse());
            }
        }

        // construct the input of the LCQProblem: [3] initial guess
        if (buffer_history) {
            
        }
        else{
            lcqp_input.x0.resize(0);
            lcqp_input.y0.resize(0);
        }

        #ifdef CONTROLLER_DEBUG
            // print the input of the LCQP
            std::cout << "Q: " << std::endl << lcqp_input.Q << std::endl;
            std::cout << "g: " << std::endl << lcqp_input.g << std::endl;
            std::cout << "L: " << std::endl << lcqp_input.L << std::endl;
            std::cout << "lbL: " << std::endl << lcqp_input.lbL << std::endl;
            std::cout << "ubL: " << std::endl << lcqp_input.ubL << std::endl;
            std::cout << "R: " << std::endl << lcqp_input.R << std::endl;
            std::cout << "lbR: " << std::endl << lcqp_input.lbR << std::endl;
            std::cout << "ubR: " << std::endl << lcqp_input.ubR << std::endl;
            std::cout << "A: " << std::endl << lcqp_input.A << std::endl;
            std::cout << "lbA: " << std::endl << lcqp_input.lbA << std::endl;
            std::cout << "ubA: " << std::endl << lcqp_input.ubA << std::endl;
            std::cout << "lb: " << std::endl << lcqp_input.lb << std::endl;
            std::cout << "ub: " << std::endl << lcqp_input.ub << std::endl;
            std::cout << "x0: " << std::endl << lcqp_input.x0 << std::endl;
            std::cout << "y0: " << std::endl << lcqp_input.y0 << std::endl;
        #endif // CONTROLLER_DEBUG
        
        lcqp_solver.nVariables = nVariables;
        lcqp_solver.nConstraints = nContacts;
        lcqp_solver.nComplementarity = nContacts;
        bool isSuccess = lcqp_solver.runSolver(lcqp_input, lcqp_output);

        if (!isSuccess) {
            // display an error, throw an exception
            std::cout << "[ERROR] LCQP solver failed to find a solution." << std::endl;
            WriteInputToFile(lcqp_input, "instance_1");
            throw std::runtime_error("LCQP solver failed to find a solution.");
        }

        if (buffer_history) {
            
        }
        
        Eigen::VectorXd result = lcqp_output.x.head(nJoint);
        return result;
    }

}