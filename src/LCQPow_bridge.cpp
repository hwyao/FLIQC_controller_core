#include "FLIQC_controller_core/LCQPow_bridge.hpp"
#include <LCQProblem.hpp>

namespace LCQPow_bridge {
    LCQPow_raw::LCQPow_raw() {
        this->pimpl = std::make_unique<LCQPow_impl>();
    }

    void LCQPow_raw::updateOptions(void) {
        
    }

    void LCQPow_raw::setEmptyLCQProblemInput(LCQProblemInput &input) {

    }

    bool LCQPow_raw::runSolver(const LCQProblemInput &input, LCQProblemOutput &output) {
        return false;
    }

    LCQProblemDebug LCQPow_raw::getDebugStatistics(void) {
        return LCQProblemDebug();
    }

    struct LCQPow_raw::LCQPow_impl {
        LCQPow::Options options;             //< Options for the solver
        LCQPow::LCQProblem lcqp;             //< LCQPow problem
    };
}