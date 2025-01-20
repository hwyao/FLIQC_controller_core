#ifndef FLIQC_CONTROLLERS_HPP_
#define FLIQC_CONTROLLERS_HPP_

#include <Eigen/Dense>
#include <vector>
#include <unordered_map>
#include <limits>
#include "FLIQC_controller_core/LCQPow_bridge.hpp"

namespace FLIQC_controller_core {
    struct FLIQC_cost_input{
        Eigen::MatrixXd Q;      ///< The cost matrix for the optimization problem
        Eigen::VectorXd g;      ///< The cost vector for the optimization problem
    };
    
    /**
     * @brief The distance sensory signal input for the controller
     * 
     */
    struct FLIQC_distance_input {
        int id;                                         ///< The id of the sensory signal, this is used to identify different obstacles
        double distance;                                ///< The distance to the obstacle
        Eigen::RowVectorXd projector_control_to_dist;   ///< The projector from control space velocity to the distance velocity
        Eigen::VectorXd projector_dist_to_control;      ///< The projector from distance to control space, leave empty to use pseudo-inverse of projector_control_to_dist
    };

    /**
     * @brief The basic controller for FLIQC
     * 
     * This controller could have very different implementations, the each controller follows the same semantic pattern:
     * 
     * 1. Construct the controller that optimizes against same control variable q_dot (joint velocity)
     * 2. Parameters:
     *    a forward step [dt] for the prediction of the distance in next moment
     *    a safety margin [eps] for the prediction of the distance in next moment to avoid
     * 3. Input:
     *    a velocity [q_dot_guide] that the controller would like to follow (but might not be too safe) 
     *    a series of distances [phi_i] that would like to be positive, the projector from joint velocity to the distance velocity [P_i], and its reverse [Pinv_i].
     * 4. The controller will output the control variable q_dot according to the optimization problem:
     * 
     *                        minimize    1/2 * q_dot' * Q * q_dot + q_dot' * g
     *                            s.t.    q_dot   =   q_dot_guide + SUM( Pinv_i * lambda_i)     (velocity guide constraint)
     *                            0 <=  lambda_i  ⊥  phi_i + dt * P_i * q_dot  >= eps          (complementarity constraint)
     * 
     *    Which will be converted to LQCPow formulation:
     *                       x = [q_dot; lambda_1; lambda_2; ...; lambda_n] = [q_dot]
     *                                                                        [lambda]
     *                       minimize    1/2 * x' * [{Q} 0] * x + x' * [g]  
     *                                              [0 {I}]            [0]
     *                           s.t.        0 = x' * L' * R * x
     *                        0       <=          [{0}, {I}] x           <=   lambda_max              (L)
     *     [-phi_1;...;-phi_i] + eps  <=    [{dt*P_1;...;dt*P_i}; {0}] x   <=  escape_velocity_max      (R) 
     *                   q_dot_guide  <=  [{I}, -Pinv_1, ..., -Pinv_i] x <=   q_dot_guide             (A)
     *               [-q_dot_min; 0]  <=               x                 <= [q_dot_max; lambda_max]  
     * 
     *  There are several options for the controller:
     *  1. buffer_history: buffer the last solution as initial guess for the next run
     *  2. enable_lambda_constraint_in_L: enable the lambda constraint in left complementarity constraint (L matrix)
     *  3. enable_lambda_constraint_in_x: enable the lambda constraint in the optimization variable x
     *  4. enable_esc_vel_constraint: enable the escape velocity constraint in right complementarity constraint (R matrix)
     * 
     *  There are several parameters for the controller:
     *  1. dt: the forward step for the optimization prediction in right complementarity constraint (dt in R matrix)
     *  2. eps: the safety margin for the forward step prediction in right complementarity constraint (epsilon in lower bound of R matrix)
     *  3. active_threshold: the active tolerance for considering the distance as active. This filters with distances 
     *  4. lambda_max: the maximum lambda value, for the lambda constraint.
     *  5. esc_vel_max: the maximum escape velocity, for the upper bound of the R matrix.
     *  6. q_dot_max: the maximum joint velocity, for the lower and upper bound of first nJoint variables in x.                   
     */
    class FLIQC_controller_joint_velocity_basic{
    public:
        /**
         * @brief Construct a new fliqc controller basic object
         * 
         * @param dim_q The dimension of the control variable q_dot
         */
        FLIQC_controller_joint_velocity_basic(int dim_q);
        
        /**
         * @brief Destroy the FLIQC_controller_joint_velocity_basic object
         * 
         */
        ~FLIQC_controller_joint_velocity_basic() = default;

        /**
         * @brief Run the controller
         * 
         * @param vel_guide The joint velocity guide for the controller, this is the velocity that the controller would like to follow
         * @param cost_input The cost input for the optimization problem
         * @param dist_inputs the input of the controller, which is the sensory signals to obstacles
         * @return Eigen::VectorXd The result control variable
         */
        Eigen::VectorXd runController(const Eigen::VectorXd& vel_guide, const FLIQC_cost_input& cost_input, const std::vector<FLIQC_distance_input> &dist_inputs);

        bool buffer_history = false;                ///< Buffer the last solution as initial guess for the next run
        bool enable_lambda_constraint_in_L = false; ///< Enable the lambda constraint in left complementarity constraint
        bool enable_lambda_constraint_in_x = true;  ///< Enable the lambda constraint in the optimization variable x
        bool enable_esc_vel_constraint = false;     ///< Enable the escape velocity constraint in right complementarity constraint

        double dt = 0.02;                     ///< The forward step for the optimization prediction in right complementarity constraint
        double eps = 0.01;                    ///< The safety margin for the forward step prediction in right complementarity constraint
        double active_threshold = 0.03;       ///< The active tolerance for considering the distance as active
        double lambda_max = std::numeric_limits<double>::infinity();     ///< The maximum lambda value
        double esc_vel_max = std::numeric_limits<double>::infinity();    ///< The maximum escape velocity calculated in R
        Eigen::VectorXd q_dot_max;            ///< The maximum joint velocity

        LCQPow_bridge lcqp_solver;            ///< The solver for the optimization problem

    protected:
        int nJoint;                                          ///< The dimension of the control variables (number of joints)

        LCQProblemInput lcqp_input;                          ///< The input of the LCQProblem
        LCQProblemOutput lcqp_output;                        ///< The output of the LCQProblem

        std::unordered_map<int, double> history_solution_x;  ///< The history of the primary solution x
        std::unordered_map<int, double> history_solution_y;  ///< The history of the dual solution y
    };
}

#endif // FLIQC_CONTROLLERS_HPP_