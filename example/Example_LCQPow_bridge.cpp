/**
 * @example Example_LCQPow_bridge.cpp
 * @brief Example of using the LCQPow_bridge to solve a linear complementarity problem.
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
#include<FLIQC_controller_core/LCQPow_bridge.hpp>
#include<cmath>
#include<iostream>

void runLQCPow(const FLIQC_controller_core::LCQProblemInput &input, FLIQC_controller_core::LCQProblemOutput &output){
    FLIQC_controller_core::LCQPow_bridge lcqp;
    lcqp.stationarityTolerance = 1e-3;
    lcqp.complementarityTolerance = 1e-3;
    lcqp.initialPenaltyParameter = 0.01;
    lcqp.penaltyUpdateFactor = 2.0;

    lcqp.nVariables = input.g.size();
    lcqp.nConstraints = input.lbA.size();
    lcqp.nComplementarity = input.L.rows();

    lcqp.updateOptions();
    lcqp.runSolver(input, output);
}

int main(void){
    FLIQC_controller_core::LCQProblemInput input;
    FLIQC_controller_core::LCQProblemOutput output;

    int N = 100;
    int nV = 2 + 2*N;
    int nC = N+1;
    int nComp = N;

    // Reference
    Eigen::Vector2d x_ref(0.5, -0.6);
    // Allocate vectors
    input.Q = Eigen::MatrixXd::Zero(nV, nV);
    input.g = Eigen::VectorXd::Zero(nV);
    input.L = Eigen::MatrixXd::Zero(nComp, nV);
    input.R = Eigen::MatrixXd::Zero(nComp, nV);
    input.A = Eigen::MatrixXd::Zero(nC, nV);
    input.lbA = Eigen::VectorXd::Zero(nC);
    input.ubA = Eigen::VectorXd::Zero(nC);
    input.x0 = Eigen::VectorXd::Zero(nV);
    
    // Assign problem data
    input.x0(0) = x_ref(0);
    input.x0(1) = x_ref(1);
    input.Q(0, 0) = 17; 
    input.Q(1, 1) = 17;
    input.Q(0, 1) = -15;
    input.Q(1, 0) = -15;

    // Regularization on Q
    for (int i = 2; i < nV; i++)
        input.Q(i, i) = 5e-12;

    // Objective linear term
    Eigen::Matrix2d Qx;
    Qx << 17, -15, -15, 17;
    input.g.head(2) = Qx*x_ref;
    input.g = -input.g;

    // Constraints
    for (int i = 0; i < N; i++) {
        // Equality constraint ([cos sin]'*x + lambda = 1)
        input.A(i, 0) = cos((2*M_PI*i)/N);
        input.A(i, 1) = sin((2*M_PI*i)/N);
        input.A(i, 2 + 2*i) = 1;

        // Convex combination constraint (sum theta = 1)
        input.A(N, 3 + 2*i) = 1;

        // Complementarity constraints (lambda*theta = 0)
        input.L(i, 2 + 2*i) = 1;
        input.R(i, 3 + 2*i) = 1;

        // constraint bounds
        input.lbA(i) = 1;
        input.ubA(i) = 1;

        input.x0(2*i + 2) = 1;
        input.x0(2*i + 3) = 1;
    }

    // Constraint bound for last constraint
    input.lbA(N) = 1;
    input.ubA(N) = 1;

    runLQCPow(input, output);

	printf( "\nxOpt = [ %g, %g ]; \n\n",
			output.x(0), output.x(1));

    // Print a reference to the global and local solutions
    printf("For reference: Global solution is at:  [ %g, %g ]\n", 0.1811, -0.9835);
    printf("               Another local solution: [ %g, %g ]\n",  0.9764, -0.2183);
}