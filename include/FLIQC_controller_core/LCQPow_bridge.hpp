#ifndef LCQPOW_BRIDGE_HPP_
#define LCQPOW_BRIDGE_HPP_

#include <memory>
#include <Eigen/Dense>

namespace LCQPow_bridge {
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
        Eigen::VectorXd x0;                                  //< Initial guess for the optimization
        Eigen::VectorXd y0;                                  //< Initial guess for the dual solution vector
    };

    struct LCQProblemOutput {
        Eigen::VectorXd x;                                  //< Primary solution vector
        Eigen::VectorXd y;                                  //< Dual solution vector
    };

    struct LCQProblemDebug {

    };

    class LCQPow_raw {
    public:
        /**
         * @brief Construct a new LCQPow_raw object
         */
        LCQPow_raw();
        
        /**
         * @brief Destroy the LCQPow_raw object
         * 
         */
        ~LCQPow_raw() = default;

        /**
         * @brief update the options for the solver.
         * 
         * Options that could be initialized one time and used in each run: 
         * stationarityTolerance, complementarityTolerance, initialPenaltyParameter, penaltyUpdateFactor
         * Options that will be initialized in each run:
         * nVariables, nConstraints, nComplementarity
         */
        void updateOptions(void);

        double stationarityTolerance;       //< Stationarity tolerance, tolerance for optimization
        double complementarityTolerance;    //< Complementarity tolerance, tolerance for complementarity vertical constraint
        double initialPenaltyParameter;     //< Initial penalty parameter, initial penalty parameter for complementarity
        double penaltyUpdateFactor;         //< Penalty update factor, factor for updating penaltised complementarity term
        const int nVariables;               //< Number of variables
        const int nConstraints;             //< Number of constraints
        const int nComplementarity;        //< Number of complementarity variables

        /**
         * @brief set the size for the LCQProblemInput
         * 
         * @param input the input to be initialized
         * The result will be initialized with given dimensions.
         */
        void setEmptyLCQProblemInput(LCQProblemInput &input);

        /**
         * @brief run the solver and get the result
         * 
         * @param input the input of the problem
         * @param output the output of the problem
         */
        bool runSolver(const LCQProblemInput &input, LCQProblemOutput &output);

        /**
         * @brief getDebugStatistics
         * 
         * @return LCQProblemDebug 
         */
        LCQProblemDebug getDebugStatistics(void);

    protected:
        /**
         * @brief implementation on external LCQPow
         * 
         */
        struct LCQPow_impl;

        /**
         * @brief Pointer to the implementation.
         * 
         */
        std::unique_ptr<LCQPow_impl> pimpl;
    };
}

#endif // LCQPOW_BRIDGE_HPP_