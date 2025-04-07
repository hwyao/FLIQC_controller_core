#include "FLIQC_controller_core/LCQPow_bridge.hpp"
#include <memory>
#include <LCQProblem.hpp>
#include <filesystem>
#include <fstream>
#include <iostream>

namespace FLIQC_controller_core{

    void WriteInputToFile(const LCQProblemInput &input, const std::string &dirname) {

        // create directory 'failed_instances/<dirname>' if not exist
        std::string output_dir = "/home/geriatronics/failed_lcqp_instances/" + dirname;
        std::filesystem::create_directories(output_dir);

        if (!std::filesystem::is_directory(output_dir)) {
            std::cout << "Failed to create directory " << output_dir << std::endl;
            return;
        } else {
            std::cout << "Directory " << output_dir << " created." << std::endl;
        }

        // write each component to an individual file in the directory
        std::ofstream file;

        // Q
        file.open(output_dir + "/Q.txt");
        file << input.Q;
        file.close();

        // g
        file.open(output_dir + "/g.txt");
        file << input.g;
        file.close();

        // L
        file.open(output_dir + "/L.txt");
        file << input.L;
        file.close();

        // lbL
        file.open(output_dir + "/lbL.txt");
        file << input.lbL;
        file.close();

        // ubL
        file.open(output_dir + "/ubL.txt");
        file << input.ubL;
        file.close();

        // R
        file.open(output_dir + "/R.txt");
        file << input.R;
        file.close();

        // lbR
        file.open(output_dir + "/lbR.txt");
        file << input.lbR;
        file.close();

        // ubR
        file.open(output_dir + "/ubR.txt");
        file << input.ubR;
        file.close();

        // A
        file.open(output_dir + "/A.txt");
        file << input.A;
        file.close();

        // lbA
        file.open(output_dir + "/lbA.txt");
        file << input.lbA;
        file.close();

        // ubA
        file.open(output_dir + "/ubA.txt");
        file << input.ubA;
        file.close();

        // lb
        file.open(output_dir + "/lb.txt");
        file << input.lb;
        file.close();

        // ub
        file.open(output_dir + "/ub.txt");
        file << input.ub;
        file.close();
        
    }


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

    bool LCQPow_bridge::runSolver(const LCQProblemInput &input, LCQProblemOutput &output){
        #ifdef FLIQC_DEBUG
        // check the size of the input matches what is needed
        #endif

        bool success = true;

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
            success = false;
        }

        retVal = this->pimpl->lcqp->runSolver();
        if (retVal != LCQPow::SUCCESSFUL_RETURN){
            success = false;
        }

        if(success){
            double xOut[this->nVariables];
            this->pimpl->lcqp->getPrimalSolution(xOut);
            double yOut[this->nVariables + this->nConstraints + 2*this->nComplementarity];
            this->pimpl->lcqp->getDualSolution(yOut);
            output.x = Eigen::Map<Eigen::VectorXd>(xOut, this->nVariables);
            output.y = Eigen::Map<Eigen::VectorXd>(yOut, this->nVariables + this->nConstraints + 2*this->nComplementarity);
        }

        return success;
    }

    LCQProblemDebug LCQPow_bridge::getDebugStatistics(void){
        return LCQProblemDebug();
    }
}