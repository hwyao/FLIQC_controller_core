/**
 * @file LCQPow_bridge.hpp
 * @brief The header file for bridge to LCQPow library
 * For detail implementation, please refer to the <a href="https://github.com/nosnoc/LCQPow">LCQPow library</a>.
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
#ifndef LCQPOW_BRIDGE_HPP_
#define LCQPOW_BRIDGE_HPP_

#include <memory>
#include <string>
#include <Eigen/Dense>
#include <stdexcept>
#include <fstream>
#include <iostream>
#include <type_traits>

namespace FLIQC_controller_core {
    /**
     * @brief The input of the LCQProblem
     * 
     * Copied from LCQPow description:
     *  LCQPow is intended for solving quadratic programs with
     *  linear complementarity constraints of the form
     * 
     *    \f[
     *    \begin{aligned}
     *    &\min_{\boldsymbol{x}} \quad \frac{1}{2} \boldsymbol{x}^\top \boldsymbol{Q} \boldsymbol{x} + \boldsymbol{x}^\top \boldsymbol{g} \\
     *    &\text{s.t.} \quad 0 = \boldsymbol{x}^\top \boldsymbol{L}^\top \boldsymbol{R} \boldsymbol{x} \\
     *    &\qquad \boldsymbol{lb}_L \leq \boldsymbol{L} \boldsymbol{x} \leq \boldsymbol{ub}_L \\
     *    &\qquad \boldsymbol{lb}_R \leq \boldsymbol{R} \boldsymbol{x} \leq \boldsymbol{ub}_R \\
     *    &\qquad \boldsymbol{lb}_A \leq \boldsymbol{A} \boldsymbol{x} \leq \boldsymbol{ub}_A \quad \text{(optional)} \\
     *    &\qquad \boldsymbol{lb} \leq \boldsymbol{x} \leq \boldsymbol{ub} \qquad \quad \text{(optional)}
     *    \end{aligned}
     *    \f]
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
     * For the exact meaning of each variable, please refer to the LCQPow documentation.
     */
    struct LCQProblemDebug {
        struct Options {
            double complementarityTolerance;
            double stationarityTolerance;
            double initialPenaltyParameter;
            double penaltyUpdateFactor;
            bool solveZeroPenaltyFirst;
            bool perturbStep;
            int maxIterations;
            double maxPenaltyParameter;
            int nDynamicPenalty;
            double etaDynamicPenalty;
            bool storeSteps;
        } options;

        struct OutputStatistics {
            int iterTotal;
            int iterOuter;
            int subproblemIter;
            double rhoOpt;
            int status;
            int qpSolver_exit_flag;
            std::vector<std::vector<double>> xSteps;
            std::vector<int> innerIters;
            std::vector<int> subproblemIters;
            std::vector<int> accuSubproblemIters;
            std::vector<double> stepLength;
            std::vector<double> stepSize;
            std::vector<double> statVals;
            std::vector<double> objVals;
            std::vector<double> phiVals;
            std::vector<double> meritVals;
        } outputStatistics;
    };

    /**
     * @brief Exception class for LCQPow solver failures.
     * 
     * This exception is thrown when the LCQPow solver fails to find a solution.
     * It contains detailed information about the problem input, output, and debug statistics
     * to assist in diagnosing the issue.
     */
    class LCQPowException : public std::exception {
    public:
        LCQProblemInput input;       //< The input of the LCQProblem
        LCQProblemOutput output;     //< The output of the LCQProblem
        LCQProblemDebug debug;       //< The debug statistics of the LCQProblem
        size_t maxMatrixSize = 12;   //< Maximum size for matrix dimensions to display full content

        /**
         * @brief Construct a new LCQPowException object.
         * 
         * @param[in] input The input of the LCQProblem.
         * @param[in] output The output of the LCQProblem.
         * @param[in] debug The debug statistics of the LCQProblem.
         */
        LCQPowException(const LCQProblemInput& input, const LCQProblemOutput& output, const LCQProblemDebug& debug);

        /**
         * @brief Get the exception message.
         * This will format the input, output, and debug statistics into a human-readable string. If
         * the matrix or vector sizes exceed the maximum size, it will truncate the output.
         * @return const char* The exception message.
         */
        const char* what() const noexcept override;

    protected:
        std::string message; //< The formatted exception message.

        /**
         * @brief Format a matrix for output.
         * 
         * @tparam MatrixType The type of the matrix.
         * @param[in] matrix The matrix to format.
         * @return std::string The formatted matrix as a string.
         */
        template <typename MatrixType>
        std::string formatMatrix(const MatrixType& matrix) const;

        /**
         * @brief Format a vector for output.
         * 
         * @tparam T The type of the vector elements.
         * @param[in] vec The vector to format.
         * @return std::string The formatted vector as a string.
         */
        template <typename T>
        std::string formatVector(const std::vector<T>& vec) const;

        /**
         * @brief Declaration of specialization of formatVector for vectors of vectors of doubles.
         * 
         * @param[in] vec The vector of vectors of doubles to format.
         * @return std::string The formatted vector of vectors as a string.
         */
        std::string formatDoubleVector(const std::vector<std::vector<double>>& vec) const;
    };

    /**
     * @brief Write a variable to a CSV file.
     * 
     * @tparam T The type of the variable.
     * @param[in] base_path The base path for the CSV file.
     * @param[in] variable The variable to write.
     * @param[in] variable_name The name of the variable (used as the filename).
     */
    template <typename T>
    void writeVariableAsCSV(const std::string& base_path, const T& variable, const std::string& variable_name) {
        std::cout << "Starting to write variable: " << variable_name << " to CSV." << std::endl;

        std::ofstream file(base_path + "/" + variable_name + ".csv");
        if (!file.is_open()) {
            throw std::runtime_error("Failed to open file: " + base_path + "/" + variable_name + ".csv");
        }

        if constexpr (std::is_arithmetic_v<T>) {
            file << variable;
        } else if constexpr (std::is_base_of_v<Eigen::MatrixBase<T>, T>) {
            for (int i = 0; i < variable.rows(); ++i) {
                for (int j = 0; j < variable.cols(); ++j) {
                    file << variable(i, j);
                    if (j < variable.cols() - 1) {
                        file << ",";
                    }
                }
                file << "\n";
            }
        } else if constexpr (std::is_same_v<T, std::vector<double>>) {
            for (const auto& val : variable) {
                file << val << "\n";
            }
        } else if constexpr (std::is_same_v<T, std::vector<int>>) {
            for (const auto& val : variable) {
                file << val << "\n";
            }
        } else if constexpr (std::is_same_v<T, std::vector<std::vector<double>>>) {
            for (const auto& row : variable) {
                for (size_t i = 0; i < row.size(); ++i) {
                    file << row[i];
                    if (i < row.size() - 1) {
                        file << ",";
                    }
                }
                file << "\n";
            }
        } else {
            throw std::runtime_error("Unsupported variable type for CSV export: " + variable_name + " as " + typeid(T).name());
        }

        file.close();
    }

    /**
     * @brief Log the LCQPowException to a file.
     * 
     * @param[in] e The LCQPowException to log.
     * @param[in] base_path The base path for the log file.
     */
    void logLCQPowExceptionAsFile(const LCQPowException& e, const std::string& base_path);

    /**
     * @brief A bridge class to the LCQPow library.
     * 
     * This class provides an interface to the LCQPow library for solving
     * linear complementarity quadratic programming (LCQP) problems.
     * 
     * This class encapsulates the LCQPow implementation and provides methods to
     * update options, run the solver, and retrieve debug statistics.
     * 
     * After initializing the LCQPow_bridge object, you can set several options
     * and need to call updateOptions() before running the solver if you change
     * some of the options. 
     * 
     * Then you can call runSolver() with input to retrieve the output. 
     */
    class LCQPow_bridge {
    protected:
        /**
         * @brief forward implementation on external LCQPow
         * 
         */
        class LCQPow_impl;

        /**
         * @brief The deleter for LCQPow_impl
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
         * #stationarityTolerance, #complementarityTolerance, #initialPenaltyParameter, #penaltyUpdateFactor
         * 
         * Options that will be initialized in each run (Which doesn't need updateOptions):
         * #nVariables, #nConstraints, #nComplementarity
         */
        void updateOptions(void);

        double stationarityTolerance = 1.0e-3;       ///< Stationarity tolerance, tolerance for optimization
        double complementarityTolerance = 1.0e-3;    ///< Complementarity tolerance, tolerance for complementarity vertical constraint
        double initialPenaltyParameter = 0.01;       ///< Initial penalty parameter, initial penalty parameter for complementarity
        double penaltyUpdateFactor = 2.0;            ///< Penalty update factor, factor for updating penaltised complementarity term
        int nVariables;               ///< Number of variables
        int nConstraints;             ///< Number of constraints
        int nComplementarity;         ///< Number of complementarity variables

        /**
         * @brief run the solver and get the result
         * 
         * @param[in] input The input of the problem.
         * @param[out] output The output of the problem.
         * @throws LCQPowException If the solver fails to solve the problem, it will throw an exception.
         */
        void runSolver(const LCQProblemInput &input, LCQProblemOutput &output);

        /**
         * @brief getDebugStatistics
         * 
         * @return LCQProblemDebug The statistics of the problem
         */
        LCQProblemDebug getDebugStatistics(void);
    };
}

#endif // LCQPOW_BRIDGE_HPP_