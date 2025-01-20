#ifndef LCQPOW_BRIDGE_HPP_
#define LCQPOW_BRIDGE_HPP_

#include <memory>
#include <Eigen/Dense>

namespace FLIQC_controller_core {
    /**
     * @brief The input of the LCQProblem
     * 
     * Copied from LCQPow description:
     *  LCQPow is intended for solving quadratic programs with
     *  linear complementarity constraints of the form
     * 
     *          minimize   1/2*x'Qx + x'g
     *             s.t.    0  = x'*L'*R*x
     *                 lbL <= L*x <= ubL
     *                 lbR <= R*x <= ubR
     *                 lbA <=  Ax  <= ubA     {optional}
     *                  lb <=   x  <= ub      {optional}
     * 
     */
    struct LCQProblemInput {
        Eigen::Matrix<double, -1, -1, Eigen::RowMajor> Q;    //< Quadratic cost matrix
        Eigen::VectorXd g;                                   //< Linear cost vector
        Eigen::Matrix<double, -1, -1, Eigen::RowMajor> L;    //< Left constraint matrix
        Eigen::VectorXd lbL;                                 //< Lower bound for left constraint
        Eigen::VectorXd ubL;                                 //< Upper bound for left constraint
        Eigen::Matrix<double, -1, -1, Eigen::RowMajor> R;    //< Right constraint matrix
        Eigen::VectorXd lbR;                                 //< Lower bound for right constraint
        Eigen::VectorXd ubR;                                 //< Upper bound for right constraint
        Eigen::Matrix<double, -1, -1,Eigen:: RowMajor> A;    //< Equality constraint matrix
        Eigen::VectorXd lbA;                                 //< Lower bound for equality constraint
        Eigen::VectorXd ubA;                                 //< Upper bound for equality constraint
        Eigen::VectorXd lb;                                  //< Lower bound for the optimization
        Eigen::VectorXd ub;                                  //< Upper bound for the optimization
        Eigen::VectorXd x0;                                  //< Initial guess for the optimization
        Eigen::VectorXd y0;                                  //< Initial guess for the dual solution vector
    };

    /**
     * @brief The output of the LCQProblem
     * 
     * Which x and y are the primary and dual solution vectors. See LQCPow paper for more details.
     */
    struct LCQProblemOutput {
        Eigen::VectorXd x;                                  //< Primary solution vector
        Eigen::VectorXd y;                                  //< Dual solution vector
    };

    /**
     * @brief The debug statistics of the LCQProblem
     * 
     * Which could be used for debugging and profiling the problem.
     */
    struct LCQProblemDebug {

    };

    class LCQPow_bridge {
    protected:
        /**
         * @brief forward implementation on external LCQPow
         * 
         */
        class LCQPow_impl;

        /**
         * @brief The deleter for the pimplData
         * 
         */
        struct LCQPow_impl_deleter {               
            void operator()(LCQPow_impl* p);
        };  

        /**
         * @brief Pointer to the implementation.
         * 
         */
        std::unique_ptr<LCQPow_impl, LCQPow_impl_deleter> pimpl;
    
    public:
        /**
         * @brief Construct a new LCQPow_bridge object
         */
        LCQPow_bridge();
        
        /**
         * @brief Destroy the LCQPow_bridge object
         * 
         */
        ~LCQPow_bridge() = default;

        /**
         * @brief update the options for the solver. Call this when you change the option attributes.
         * 
         * Options that could be initialized one time and used in each run (which needs updateOptions): 
         * stationarityTolerance, complementarityTolerance, initialPenaltyParameter, penaltyUpdateFactor
         * 
         * Options that will be initialized in each run (Which doesn't need updateOptions):
         * nVariables, nConstraints, nComplementarity
         */
        void updateOptions(void);

        double stationarityTolerance = 1.0e-3;       //< Stationarity tolerance, tolerance for optimization
        double complementarityTolerance = 1.0e-3;            //< Complementarity tolerance, tolerance for complementarity vertical constraint
        double initialPenaltyParameter = 0.01;      //< Initial penalty parameter, initial penalty parameter for complementarity
        double penaltyUpdateFactor = 2.0;           //< Penalty update factor, factor for updating penaltised complementarity term
        int nVariables;               //< Number of variables
        int nConstraints;             //< Number of constraints
        int nComplementarity;         //< Number of complementarity variables

        /**
         * @brief run the solver and get the result
         * 
         * @param input the input of the problem
         * @param output the output of the problem
         * @return true if the problem is solved successfully
         */
        bool runSolver(const LCQProblemInput &input, LCQProblemOutput &output);

        /**
         * @brief getDebugStatistics
         * 
         * @return LCQProblemDebug The statistics of the problem
         */
        LCQProblemDebug getDebugStatistics(void);
    };
}

#endif // LCQPOW_BRIDGE_HPP_