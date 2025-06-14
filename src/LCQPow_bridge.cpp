/**
 * @file LCQPow_bridge.cpp
 * @brief Implementation of the LCQPow_bridge class.
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
#include "FLIQC_controller_core/LCQPow_bridge.hpp"
#include <memory>
#include <LCQProblem.hpp>
#include <filesystem>

namespace FLIQC_controller_core{
    LCQPowException::LCQPowException(const LCQProblemInput& input, const LCQProblemOutput& output, const LCQProblemDebug& debug)
        : input(input), output(output), debug(debug) {
        std::ostringstream oss;
        oss << "LCQPowException occurred. Check the input, output and debug statistics for more details.\n";

        // Input
        oss << "LCQProblemInput:\n";
        oss << "    Q: " << formatMatrix(input.Q) << "\n";
        oss << "    g: " << formatMatrix(input.g) << "\n";
        oss << "    L: " << formatMatrix(input.L) << "\n";
        oss << "    lbL: " << formatMatrix(input.lbL) << "\n";
        oss << "    ubL: " << formatMatrix(input.ubL) << "\n";
        oss << "    R: " << formatMatrix(input.R) << "\n";
        oss << "    lbR: " << formatMatrix(input.lbR) << "\n";
        oss << "    ubR: " << formatMatrix(input.ubR) << "\n";
        oss << "    A: " << formatMatrix(input.A) << "\n";
        oss << "    lbA: " << formatMatrix(input.lbA) << "\n";
        oss << "    ubA: " << formatMatrix(input.ubA) << "\n";
        oss << "    lb: " << formatMatrix(input.lb) << "\n";
        oss << "    ub: " << formatMatrix(input.ub) << "\n";
        oss << "    x0: " << formatMatrix(input.x0) << "\n";
        oss << "    y0: " << formatMatrix(input.y0) << "\n";

        // Output
        oss << "LCQProblemOutput:\n";
        oss << "    x: " << formatMatrix(output.x) << "\n";
        oss << "    y: " << formatMatrix(output.y) << "\n";

        // Debug
        oss << "LCQProblemDebug:\n";
        oss << "    Options:\n";
        oss << "        complementarityTolerance: " << debug.options.complementarityTolerance << "\n";
        oss << "        stationarityTolerance: " << debug.options.stationarityTolerance << "\n";
        oss << "        initialPenaltyParameter: " << debug.options.initialPenaltyParameter << "\n";
        oss << "        penaltyUpdateFactor: " << debug.options.penaltyUpdateFactor << "\n";
        oss << "        solveZeroPenaltyFirst: " << debug.options.solveZeroPenaltyFirst << "\n";
        oss << "        perturbStep: " << debug.options.perturbStep << "\n";
        oss << "        maxIterations: " << debug.options.maxIterations << "\n";
        oss << "        maxPenaltyParameter: " << debug.options.maxPenaltyParameter << "\n";
        oss << "        nDynamicPenalty: " << debug.options.nDynamicPenalty << "\n";
        oss << "        etaDynamicPenalty: " << debug.options.etaDynamicPenalty << "\n";
        oss << "        storeSteps: " << debug.options.storeSteps << "\n";
        oss << "    OutputStatistics:\n";
        oss << "        iterTotal: " << debug.outputStatistics.iterTotal << "\n";
        oss << "        iterOuter: " << debug.outputStatistics.iterOuter << "\n";
        oss << "        subproblemIter: " << debug.outputStatistics.subproblemIter << "\n";
        oss << "        rhoOpt: " << debug.outputStatistics.rhoOpt << "\n";
        oss << "        status: " << debug.outputStatistics.status << "\n";
        oss << "        qpSolver_exit_flag: " << debug.outputStatistics.qpSolver_exit_flag << "\n";
        oss << "        xSteps: " << formatDoubleVector(debug.outputStatistics.xSteps) << "\n";
        oss << "        innerIters: " << formatVector(debug.outputStatistics.innerIters) << "\n";
        oss << "        subproblemIters: " << formatVector(debug.outputStatistics.subproblemIters) << "\n";
        oss << "        accuSubproblemIters: " << formatVector(debug.outputStatistics.accuSubproblemIters) << "\n";
        oss << "        stepLength: " << formatVector(debug.outputStatistics.stepLength) << "\n";
        oss << "        stepSize: " << formatVector(debug.outputStatistics.stepSize) << "\n";
        oss << "        statVals: " << formatVector(debug.outputStatistics.statVals) << "\n";
        oss << "        objVals: " << formatVector(debug.outputStatistics.objVals) << "\n";
        oss << "        phiVals: " << formatVector(debug.outputStatistics.phiVals) << "\n";
        oss << "        meritVals: " << formatVector(debug.outputStatistics.meritVals) << "\n";

        message = oss.str();
    }

    const char* LCQPowException::what() const noexcept {
        return message.c_str();
    }

    template <typename MatrixType>
    std::string LCQPowException::formatMatrix(const MatrixType& matrix) const {
        if (matrix.size() == 0) {
            return "[empty]";
        } else if (matrix.rows() > maxMatrixSize || matrix.cols() > maxMatrixSize) {
            return "[" + std::to_string(matrix.rows()) + "x" + std::to_string(matrix.cols()) + " Matrix]";
        } else {
            std::ostringstream oss;
            oss << "\n" << matrix;
            return oss.str();
        }
    }

    template <typename T>
    std::string LCQPowException::formatVector(const std::vector<T>& vec) const {
        if (vec.empty()) {
            return "[empty]";
        } else if (vec.size() > maxMatrixSize) {
            return "[" + std::to_string(vec.size()) + " elements]";
        } else {
            std::ostringstream oss;
            oss << "[";
            for (size_t i = 0; i < vec.size(); ++i) {
                oss << vec[i];
                if (i < vec.size() - 1) oss << ", ";
            }
            oss << "]";
            return oss.str();
        }
    }

    std::string LCQPowException::formatDoubleVector(const std::vector<std::vector<double>>& vec) const {
        if (vec.empty()) {
            return "[empty]";
        } else if (vec.size() > maxMatrixSize) {
            return "[" + std::to_string(vec.size()) + " rows]";
        } else {
            std::ostringstream oss;
            oss << "[\n";
            for (size_t i = 0; i < vec.size(); ++i) {
                if (vec[i].empty()) {
                    oss << "  [empty]";
                } else if (vec[i].size() > maxMatrixSize) {
                    oss << "  [" << vec[i].size() << " elements]";
                } else {
                    oss << "  [";
                    for (size_t j = 0; j < vec[i].size(); ++j) {
                        oss << vec[i][j];
                        if (j < vec[i].size() - 1) oss << ", ";
                    }
                    oss << "]";
                }
                if (i < vec.size() - 1) oss << ",";
                oss << "\n";
            }
            oss << "]";
            return oss.str();
        }
    }

    void logLCQPowExceptionAsFile(const LCQPowException& e, const std::string& base_path) {
        // Create directories for logging
        std::cout<< "Start logging the data." << std::endl;
        std::string input_dir = base_path + "/solver_input";
        std::string output_dir = base_path + "/solver_output";
        std::string options_dir = base_path + "/solver_options";
        std::string statistics_dir = base_path + "/solver_statistics";
        std::filesystem::create_directories(input_dir);
        std::filesystem::create_directories(output_dir);
        std::filesystem::create_directories(options_dir);
        std::filesystem::create_directories(statistics_dir);

        // Log input data
        writeVariableAsCSV(input_dir, e.input.Q, "Q");
        writeVariableAsCSV(input_dir, e.input.g, "g");
        writeVariableAsCSV(input_dir, e.input.L, "L");
        writeVariableAsCSV(input_dir, e.input.lbL, "lbL");
        writeVariableAsCSV(input_dir, e.input.ubL, "ubL");
        writeVariableAsCSV(input_dir, e.input.R, "R");
        writeVariableAsCSV(input_dir, e.input.lbR, "lbR");
        writeVariableAsCSV(input_dir, e.input.ubR, "ubR");
        writeVariableAsCSV(input_dir, e.input.A, "A");
        writeVariableAsCSV(input_dir, e.input.lbA, "lbA");
        writeVariableAsCSV(input_dir, e.input.ubA, "ubA");
        writeVariableAsCSV(input_dir, e.input.lb, "lb");
        writeVariableAsCSV(input_dir, e.input.ub, "ub");
        writeVariableAsCSV(input_dir, e.input.x0, "x0");
        writeVariableAsCSV(input_dir, e.input.y0, "y0");

        // Log output data
        writeVariableAsCSV(output_dir, e.output.x, "x");
        writeVariableAsCSV(output_dir, e.output.y, "y");

        // Log options (each variable in a separate file)
        writeVariableAsCSV(options_dir, e.debug.options.complementarityTolerance, "complementarityTolerance");
        writeVariableAsCSV(options_dir, e.debug.options.stationarityTolerance, "stationarityTolerance");
        writeVariableAsCSV(options_dir, e.debug.options.initialPenaltyParameter, "initialPenaltyParameter");
        writeVariableAsCSV(options_dir, e.debug.options.penaltyUpdateFactor, "penaltyUpdateFactor");
        writeVariableAsCSV(options_dir, e.debug.options.solveZeroPenaltyFirst, "solveZeroPenaltyFirst");
        writeVariableAsCSV(options_dir, e.debug.options.perturbStep, "perturbStep");
        writeVariableAsCSV(options_dir, e.debug.options.maxIterations, "maxIterations");
        writeVariableAsCSV(options_dir, e.debug.options.maxPenaltyParameter, "maxPenaltyParameter");
        writeVariableAsCSV(options_dir, e.debug.options.nDynamicPenalty, "nDynamicPenalty");
        writeVariableAsCSV(options_dir, e.debug.options.etaDynamicPenalty, "etaDynamicPenalty");
        writeVariableAsCSV(options_dir, e.debug.options.storeSteps, "storeSteps");

        // Log statistics
        writeVariableAsCSV(statistics_dir, e.debug.outputStatistics.xSteps, "xSteps");
        writeVariableAsCSV(statistics_dir, e.debug.outputStatistics.innerIters, "innerIters");
        writeVariableAsCSV(statistics_dir, e.debug.outputStatistics.subproblemIters, "subproblemIters");
        writeVariableAsCSV(statistics_dir, e.debug.outputStatistics.accuSubproblemIters, "accuSubproblemIters");
        writeVariableAsCSV(statistics_dir, e.debug.outputStatistics.stepLength, "stepLength");
        writeVariableAsCSV(statistics_dir, e.debug.outputStatistics.stepSize, "stepSize");
        writeVariableAsCSV(statistics_dir, e.debug.outputStatistics.statVals, "statVals");
        writeVariableAsCSV(statistics_dir, e.debug.outputStatistics.objVals, "objVals");
        writeVariableAsCSV(statistics_dir, e.debug.outputStatistics.phiVals, "phiVals");
        writeVariableAsCSV(statistics_dir, e.debug.outputStatistics.meritVals, "meritVals");

        // Log readable exception message
        std::ofstream exception_file(base_path + "/exception_message.txt");
        if (exception_file.is_open()) {
            exception_file << e.what();
            exception_file.close();
        }
    }

    /**
     * @brief Implementation of the LCQPow_bridge class.
     * 
     * This class uses PIMPL idiom to hide the implementation details of the LCQPow solver.
     * It contains a unique pointer LCQPow solver and options.
     */
    class LCQPow_bridge::LCQPow_impl{
    public:
        std::unique_ptr<LCQPow::LCQProblem> lcqp;
        std::unique_ptr<LCQPow::Options> options;
    };

    void LCQPow_bridge::LCQPow_impl_deleter::operator()(LCQPow_bridge::LCQPow_bridge::LCQPow_impl* p) {
        delete p;
    }

    LCQPow_bridge::LCQPow_bridge(): pimpl(new LCQPow_bridge::LCQPow_impl()){
        this->pimpl->options = std::make_unique<LCQPow::Options>();
        this->pimpl->options->setPrintLevel(LCQPow::PrintLevel::NONE);
        this->updateOptions();
    }

    void LCQPow_bridge::updateOptions(void){
        this->pimpl->options->setStationarityTolerance(this->stationarityTolerance);
        this->pimpl->options->setComplementarityTolerance(this->complementarityTolerance);
        this->pimpl->options->setInitialPenaltyParameter(this->initialPenaltyParameter);
        this->pimpl->options->setPenaltyUpdateFactor(this->penaltyUpdateFactor);
    }

    void LCQPow_bridge::runSolver(const LCQProblemInput &input, LCQProblemOutput &output){
        #ifdef FLIQC_DEBUG
        // check the size of the input matches what is needed
        #endif

        this->pimpl->lcqp.reset(new LCQPow::LCQProblem(this->nVariables, this->nConstraints, this->nComplementarity));
        this->pimpl->lcqp->setOptions(*(this->pimpl->options));

        LCQPow::ReturnValue retVal = this->pimpl->lcqp->loadLCQP(
            input.Q.data(), input.g.data(), 
            input.L.data(), input.R.data(), input.lbL.data(), input.ubL.data(), input.lbR.data(), input.ubR.data(),
            input.A.data(), input.lbA.data(), input.ubA.data(), 
            input.lb.data(), input.ub.data(),
            input.x0.data(), input.y0.data()
        );
        if (retVal != LCQPow::SUCCESSFUL_RETURN){
            throw std::runtime_error("Failed to load LCQP problem.");
        }

        retVal = this->pimpl->lcqp->runSolver();
        if (retVal != LCQPow::SUCCESSFUL_RETURN){
            LCQProblemDebug debug = this->getDebugStatistics();
            throw LCQPowException(input, output, debug);
        }

        double xOut[this->nVariables];
        this->pimpl->lcqp->getPrimalSolution(xOut);
        double yOut[this->nVariables + this->nConstraints + 2*this->nComplementarity];
        this->pimpl->lcqp->getDualSolution(yOut);
        output.x = Eigen::Map<Eigen::VectorXd>(xOut, this->nVariables);
        output.y = Eigen::Map<Eigen::VectorXd>(yOut, this->nVariables + this->nConstraints + 2*this->nComplementarity);
    }

    LCQProblemDebug LCQPow_bridge::getDebugStatistics(void) {
        LCQProblemDebug debug;

        // Populate options
        debug.options.complementarityTolerance = this->pimpl->options->getComplementarityTolerance();
        debug.options.stationarityTolerance    = this->pimpl->options->getStationarityTolerance();
        debug.options.initialPenaltyParameter  = this->pimpl->options->getInitialPenaltyParameter();
        debug.options.penaltyUpdateFactor      = this->pimpl->options->getPenaltyUpdateFactor();
        debug.options.solveZeroPenaltyFirst    = this->pimpl->options->getSolveZeroPenaltyFirst();
        debug.options.perturbStep              = this->pimpl->options->getPerturbStep();
        debug.options.maxIterations            = this->pimpl->options->getMaxIterations();
        debug.options.maxPenaltyParameter      = this->pimpl->options->getMaxPenaltyParameter();
        debug.options.nDynamicPenalty          = this->pimpl->options->getNDynamicPenalty();
        debug.options.etaDynamicPenalty        = this->pimpl->options->getEtaDynamicPenalty();
        debug.options.storeSteps               = this->pimpl->options->getStoreSteps();

        // Populate output statistics
        LCQPow::OutputStatistics stats; // Declare a proper OutputStatistics object
        this->pimpl->lcqp->getOutputStatistics(stats); // Pass it by reference
        debug.outputStatistics.iterTotal            = stats.getIterTotal();
        debug.outputStatistics.iterOuter            = stats.getIterOuter();
        debug.outputStatistics.subproblemIter       = stats.getSubproblemIter();
        debug.outputStatistics.rhoOpt               = stats.getRhoOpt();
        debug.outputStatistics.status               = stats.getSolutionStatus();
        debug.outputStatistics.qpSolver_exit_flag   = stats.getQPSolverExitFlag();
        debug.outputStatistics.xSteps               = stats.getxStepsStdVec();
        debug.outputStatistics.innerIters           = stats.getInnerItersStdVec();
        debug.outputStatistics.subproblemIters      = stats.getSubproblemItersStdVec();
        debug.outputStatistics.accuSubproblemIters  = stats.getAccuSubproblemItersStdVec();
        debug.outputStatistics.stepLength           = stats.getStepLengthStdVec();
        debug.outputStatistics.stepSize             = stats.getStepSizeStdVec();
        debug.outputStatistics.statVals             = stats.getStatValsStdVec();
        debug.outputStatistics.objVals              = stats.getObjValsStdVec();
        debug.outputStatistics.phiVals              = stats.getPhiValsStdVec();
        debug.outputStatistics.meritVals            = stats.getMeritValsStdVec();

        return debug;
    }
}