#include "FLIQC_controller_core/LCQPow_bridge.hpp"
#include <memory>
#include <LCQProblem.hpp>

namespace FLIQC_controller_core{
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