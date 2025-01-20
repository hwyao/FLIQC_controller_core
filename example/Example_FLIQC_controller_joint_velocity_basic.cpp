#include <FLIQC_controller_core/FLIQC_controllers.hpp>
#include <iostream>
#include <Eigen/Dense>
#include <vector>

void geometry_calculation(const Eigen::Vector2d& pos, double &distance, Eigen::Vector2d &normal){
    distance = pos.norm() - 1.0;
    normal = pos.normalized();
}

int main(void){
    // prepare controller and its input.
    FLIQC_controller_core::FLIQC_controller_joint_velocity_basic controller(2);

    Eigen::Vector2d velocity_guide(0.0, -0.2);
    FLIQC_controller_core::FLIQC_cost_input cost_input;
    cost_input.Q = Eigen::MatrixXd::Identity(2, 2);
    cost_input.g = Eigen::VectorXd::Zero(2);
    std::vector<FLIQC_controller_core::FLIQC_distance_input> dist_inputs;

    // initial states
    Eigen::Vector2d pos(1.414/2, 1.414/2 + 0.03);

    // recording states
    std::vector<double> x_history;
    std::vector<double> y_history;

    int step = 0;
    int max_step = 5000;
    double freq = 500;
    while(step <= max_step){
        // calculate the distance and normal
        double distance;
        Eigen::Vector2d normal;
        geometry_calculation(pos, distance, normal);
        dist_inputs.clear();
        dist_inputs.push_back({0, distance, normal, normal.transpose()/normal.norm()});

        Eigen::VectorXd output = controller.runController(velocity_guide, cost_input, dist_inputs);
        pos = pos + output * 0.005;
        
        x_history.push_back(pos(0));
        y_history.push_back(pos(1));

        step++;
    }

    // plot the result
    
}