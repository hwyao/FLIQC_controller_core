/**
 * @file FLIQC_controllers.hpp
 * @brief The header file for the FLIQC controller
 * 
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

#ifndef FLIQC_CONTROLLERS_HPP_
#define FLIQC_CONTROLLERS_HPP_

#include <Eigen/Dense>
#include <vector>
#include <unordered_map>
#include <limits>
#include "FLIQC_controller_core/LCQPow_bridge.hpp"

namespace FLIQC_controller_core {
    /**
     * @brief The configuration for the FLIQC controller quadratic cost
     *
     */
    enum FLIQC_quad_cost_type{
        FLIQC_QUAD_COST_IDENTITY,                       ///< The cost is the identity matrix  \f$\frac{1}{2} \boldsymbol{\dot{q}}^\top \boldsymbol{\dot{q}} \f$
        FLIQC_QUAD_COST_JOINT_VELOCITY_ERROR,           ///< The cost is the joint velocity error \f$\frac{1}{2} (\boldsymbol{\dot{q}} - \boldsymbol{\dot{q}}_{\text{guide}})^\top (\boldsymbol{\dot{q}} - \boldsymbol{\dot{q}}_{\text{guide}})\f$
        FLIQC_QUAD_COST_MASS_MATRIX,                    ///< The cost is the mass matrix  \f$\frac{1}{2} \boldsymbol{\dot{q}}^\top \boldsymbol{M} \boldsymbol{\dot{q}}\f$
        FLIQC_QUAD_COST_MASS_MATRIX_VELOCITY_ERROR      ///< The cost is the error weighted in the mass matrix \f$\frac{1}{2} (\boldsymbol{\dot{q}} - \boldsymbol{\dot{q}}_{\text{guide}})^\top \boldsymbol{M} (\boldsymbol{\dot{q}} - \boldsymbol{\dot{q}}_{\text{guide}})\f$
    };

    /**
     * @brief The configuration for the FLIQC controller linear cost
     * 
     */
    enum FLIQC_linear_cost_type{
        FLIQC_LINEAR_COST_NONE            ///< No linear cost
    };
    
    /**
     * @brief The distance and kinematic signal input for the controller
     * 
     */
    struct FLIQC_distance_input {
        int id;                                         ///< The id of the sensory signal, this is used to identify different obstacles
        double distance;                                ///< The distance to the obstacle \f$ \psi_i \f$
        Eigen::RowVectorXd projector_control_to_dist;   ///< The projector from control space velocity to the distance velocity \f$ \boldsymbol{P}_i \f$
        Eigen::VectorXd projector_dist_to_control;      ///< The projector from distance to control space \f$ \boldsymbol{P}_i^{\mathrm{inv}} \f$, leave empty to use pseudo-inverse of projector_control_to_dist
    };

    /**
     * @brief The input of the FLIQC controller
     *  
     */
    struct FLIQC_state_input{
        Eigen::VectorXd q_dot_guide; ///< The joint velocity guide for the controller \f$ \boldsymbol{\dot{q}}_{\text{guide}} \f$
        Eigen::MatrixXd M;           ///< The mass matrix for the current optimization problem
        Eigen::MatrixXd J;           ///< The Jacobian matrix for the current optimization problem
    };

    /**
     * @brief The output of the FLIQC controller
     * 
     *
     */
    struct FLIQC_control_output{
        Eigen::VectorXd x;           ///< The x output of the optimization problem
        Eigen::VectorXd y;           ///< The y output of the optimization problem
    };

    /**
     * @brief The basic controller for FLIQC
     * 
     * This controller is activated by running runController() function.
     * 
     * When running the controller, we need input:
     * 1. a velocity \f$ \boldsymbol{\dot{q}}_{\text{guide}} \f$ that the controller would like to follow, the mass matrix \f$ \boldsymbol{M} \f$ and the Jacobian matrix \f$ \boldsymbol{J} \f$ for the current end-effector configuration.
     * 2. a series of distances \f$ \psi_i \f$ that would like to be positive, along with the projector from joint velocity to the distance velocity \f$ \boldsymbol{P}_i \f$, and its reverse \f$ \boldsymbol{P}_i^{\mathrm{inv}} \f$.
     * 
     * The controller will run according to the optimization problem:
     * 
     *    \f[
     *    \begin{aligned}
     *    \min_{\boldsymbol{\dot{q}}} \ &\frac{1}{2} \boldsymbol{\dot{q}}^\top \boldsymbol{Q} \boldsymbol{\dot{q}} + \boldsymbol{\dot{q}}^\top \boldsymbol{g} \\
     *    \text{s.t.} \ &\boldsymbol{\dot{q}} = \boldsymbol{\dot{q}}_{\text{guide}} + \boldsymbol{P}^{\mathrm{null}} \sum_{i\in n_{\mathrm{act}}} \boldsymbol{P}^{\mathrm{inv}}_i \lambda_i \\
     *                  &0 \leq \lambda_i \perp \psi_i + \Delta t\, \boldsymbol{P}_i \boldsymbol{\dot{q}} \geq \epsilon
     *    \end{aligned}
     *    \f]
     * 
     *    In background, we use LCQPow_bridge(), which connects to the <a href="https://github.com/nosnoc/LCQPow">LCQPow library</a> to solve the optimization problem.
     *    To formulate this as the LCQPow standard form, we define the optimization variable as:
     * 
     *    \f[
     *    \boldsymbol{x} = \left[ \boldsymbol{\dot{q}}^\top,\ \lambda_1,\ \lambda_2,\ \ldots,\ \lambda_n \right]^\top
     *    \f]
     * 
     *    The result will be converted to LCQP formulation:
     * 
     *    \f[
     *    \min_{\boldsymbol{x}} \quad \frac{1}{2} \boldsymbol{x}^\top
     *    \begin{bmatrix} \boldsymbol{Q} & 0 \\ 0 & k_\lambda \boldsymbol{I} \end{bmatrix}
     *    \boldsymbol{x} +  \boldsymbol{x}^\top \begin{bmatrix} \boldsymbol{g} \\ 0 \end{bmatrix}
     *    \f]
     *    \f[
            \begin{array}{rrrcl}
                \mathrm{s}.\mathrm{t}.&		\boldsymbol{0}\le&		\left[ \begin{matrix}
                0&		\boldsymbol{I}\\
            \end{matrix} \right] \boldsymbol{x}\le&		\lambda _{\max}&		\left( \text{inequality constraint } \boldsymbol{L} \right)\\
                &		\left[ \begin{array}{c}
                -\psi _1\\
                \vdots\\
                -\psi _n\\
            \end{array} \right] +\epsilon \le&		\left[ \begin{array}{c}
                \Delta t\,\boldsymbol{P}_1\\
                \vdots\\
                \Delta t\,\boldsymbol{P}_n\\
                0\\
            \end{array} \right] \boldsymbol{x}\le&		v_{\mathrm{esc},\max}&		\left( \text{left compementarity constraint }\boldsymbol{L} \right)\\
                &		\dot{\boldsymbol{q}}_{\mathrm{guide}}\le&		\left[ \begin{matrix}
                \boldsymbol{I}&		-\boldsymbol{P}^{\mathrm{null}} \boldsymbol{P}_{1}^{\mathrm{inv}}&		\cdots&		-\boldsymbol{P}^{\mathrm{null}} \boldsymbol{P}_{n}^{\mathrm{inv}}\\
            \end{matrix} \right] \boldsymbol{x}\le&		\dot{\boldsymbol{q}}_{\mathrm{guide}}&		\left( \text{right compementarity constraint } \boldsymbol{R} \right)\\
                &		\left[ \begin{array}{c}
                -\dot{\boldsymbol{q}}_{\min}\\
                0\\
            \end{array} \right] \le&		\boldsymbol{x}\le&		\left[ \begin{array}{c}
                \dot{\boldsymbol{q}}_{\max}\\
                \lambda _{\max}\\
            \end{array} \right]&		\left( \text{variable constraint } \right)\\
            \end{array}
     *    \f]
     * 
     *  Before running the controller, there are many public attributes that can be configured to change the behavior of the controller.
     *
     *  Options for the cost function:
     *  1. #quad_cost_type: the type of the quadratic cost function. 
     *  2. #linear_cost_type: [Currently unused] the type of the linear cost function.
     *  3. #lambda_cost_penalty: the penalty for the lambda constraint in the optimization problem.
     * 
     *  Options for the controller constraint formulation:
     *  1. buffer_history: [Unimplemented] buffer the last solution as initial guess for the next run 
     *  2. #enable_lambda_constraint_in_L: enable the lambda constraint in left complementarity constraint
     *  3. #enable_lambda_constraint_in_x: enable the lambda constraint in the optimization variable
     *  4. #enable_esc_vel_constraint: enable the escape velocity constraint in right complementarity constraint
     *  5. #enable_nullspace_projector_in_A: enable the nullspace projector in the A matrix
     * 
     *  Parameters for the controller:
     *  1. #dt: the forward step for the optimization prediction in right complementarity constraint \f$ \Delta t \f$
     *  2. #eps: the safety margin for the forward step prediction in right complementarity constraint \f$ \epsilon \f$
     *  3. #active_threshold: The active tolerance for considering the distance as active
     *  4. #lambda_max: The maximum lambda value \f$ \lambda_{\max} \f$
     *  5. #esc_vel_max: The maximum escape velocity calculated in R \f$ v_{\mathrm{esc},\max} \f$
     *  6. #q_dot_max: The maximum joint velocity \f$ \dot{\boldsymbol{q}}_{\max} \f$
     *  7. #weight_on_mass_matrix: [Currently don't suggest to use] the weight on the mass matrix, this is used to scale the mass matrix to the same scale as the other cost function.
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
         * @param[in]  state_input    The cost input for the optimization problem
         * @param[in]  dist_inputs    The input of the controller, which is the sensory signals to obstacles
         * @param[out] control_output The output of the controller, which is the full solution of the optimization problem
         * @return Eigen::VectorXd    The result control variable
         * @exception LCQPowException If the LCQPow solver fails to solve the optimization problem, it will throw an exception.
         */
        Eigen::VectorXd runController(const FLIQC_state_input& state_input, 
                                      const std::vector<FLIQC_distance_input> &dist_inputs,
                                            FLIQC_control_output& control_output);

        // cost configurations
        FLIQC_quad_cost_type quad_cost_type = FLIQC_QUAD_COST_IDENTITY;   ///< The type of the cost function, either identity or mass matrix
        FLIQC_linear_cost_type linear_cost_type = FLIQC_LINEAR_COST_NONE; ///< The type of the linear cost function, currently not used
        double lambda_cost_penalty = 1.0;                                 ///< The penalty for the lambda constraint in the optimization problem \f$ k_\lambda \f$

        // constraint formulation configurations
        bool enable_lambda_constraint_in_L = false;    ///< Enable the lambda constraint in left complementarity constraint. If true, the #lambda_max will be used in upper bound of \f$ \boldsymbol{L} \f$. If false, it will be left as unconstrained.
        bool enable_lambda_constraint_in_x = true;     ///< Enable the lambda constraint in the optimization variable x. If true, the #lambda_max will be used in upper bound of \f$ \boldsymbol{x} \f$. If false, the upper bound is maximum of numeric limits of double.
        bool enable_esc_vel_constraint = false;        ///< Enable the escape velocity constraint in right complementarity constraint. If true, the #esc_vel_max will be used in upper bound of \f$ \boldsymbol{R} \f$. If false, it will be left as unconstrained.
        bool enable_nullspace_projector_in_A = true;   ///< Enable the nullspace projector in the A matrix. If true, \f$ \boldsymbol{P}^{\mathrm{null}} \f$ will be \f$ \boldsymbol{I} - \boldsymbol{J}^\dagger_{\mathrm{pos}} \boldsymbol{J}_{\mathrm{pos}} \f$. If false, it will be left as identity matrix. 

        // parameters configurations
        double dt = 0.02;                      ///< The forward step for the optimization prediction in right complementarity constraint \f$ \Delta t \f$
        double eps = 0.02;                     ///< The safety margin for the forward step prediction in right complementarity constraint \f$ \epsilon \f$
        double active_threshold = 0.05;        ///< The active tolerance for considering the distance as active
        double lambda_max = std::numeric_limits<double>::max();     ///< The maximum lambda value \f$ \lambda_{\max} \f$
        double esc_vel_max = std::numeric_limits<double>::max();    ///< The maximum escape velocity calculated in R \f$ v_{\mathrm{esc},\max} \f$
        Eigen::VectorXd q_dot_max;             ///< The maximum joint velocity \f$ \dot{\boldsymbol{q}}_{\max} \f$
        Eigen::VectorXd weight_on_mass_matrix; ///< The weight on the mass matrix, this is used to scale the mass matrix to the same scale as the other cost function

        LCQPow_bridge lcqp_solver;             ///< The solver for the optimization problem

    protected:
        int nJoint;                                          ///< The dimension of the control variables (number of joints)

        LCQProblemInput lcqp_input;                          ///< The input of the LCQProblem
        LCQProblemOutput lcqp_output;                        ///< The output of the LCQProblem
    };
}

#endif // FLIQC_CONTROLLERS_HPP_