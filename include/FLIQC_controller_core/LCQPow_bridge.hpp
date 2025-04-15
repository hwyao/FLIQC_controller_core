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
         * @param input The input of the LCQProblem.
         * @param output The output of the LCQProblem.
         * @param debug The debug statistics of the LCQProblem.
         */
        LCQPowException(const LCQProblemInput& input, const LCQProblemOutput& output, const LCQProblemDebug& debug);

        /**
         * @brief Get the exception message.
         * 
         * @return const char* The exception message.
         */
        const char* what() const noexcept override;

    private:
        std::string message; //< The formatted exception message.

        /**
         * @brief Format a matrix for output.
         * 
         * @tparam MatrixType The type of the matrix.
         * @param matrix The matrix to format.
         * @return std::string The formatted matrix as a string.
         */
        template <typename MatrixType>
        std::string formatMatrix(const MatrixType& matrix) const;

        /**
         * @brief Format a vector for output.
         * 
         * @tparam T The type of the vector elements.
         * @param vec The vector to format.
         * @return std::string The formatted vector as a string.
         */
        template <typename T>
        std::string formatVector(const std::vector<T>& vec) const;

        /**
         * @brief Declaration of specialization of formatVector for vectors of vectors of doubles.
         * 
         * @param vec The vector of vectors of doubles to format.
         * @return std::string The formatted vector of vectors as a string.
         */
        std::string formatDoubleVector(const std::vector<std::vector<double>>& vec) const;
    };

    /**
     * @brief Write a variable to a CSV file.
     * 
     * @tparam T The type of the variable.
     * @param base_path The base path for the CSV file.
     * @param variable The variable to write.
     * @param variable_name The name of the variable (used as the filename).
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
     * @param e The LCQPowException to log.
     * @param base_path The base path for the log file.
     */
    void logLCQPowExceptionAsFile(const LCQPowException& e, const std::string& base_path);

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
         * @throws std::runtime_error if loading the LCQP problem fails
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