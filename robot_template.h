
#ifndef ROBOT_TEMPLATE_H
#define ROBOT_TEMPLATE_H

#include <math.h>
#include <vector>
#include <Eigen/Dense>
#include <iostream>

class RobotTemplate {
public:
    virtual std::vector<std::vector<double>>
    forward_kinematics(std::vector<double> q) = 0;

    virtual std::vector<std::vector<std::vector<double>>>
    differential_kinematics(std::vector<double> q) = 0;

    std::vector<std::vector<double>>
    get_jacobian(std::vector<double> q, bool position_only = false);

    std::vector<double>
    inverse_differential_kinematics(
        std::vector<std::vector<double>> jacobian,
        std::vector<double> delta_position);

};

std::vector<std::vector<double>>
RobotTemplate::get_jacobian(std::vector<double> q, bool position_only) {
    auto FK = forward_kinematics(q);
    auto DK = differential_kinematics(q);

    const auto& R33 = FK[2][2];
    const auto& R32 = FK[2][1];
    const auto& R31 = FK[2][0];
    const auto& R21 = FK[1][0];
    const auto& R11 = FK[0][0];

    std::vector<std::vector<double>> retval;
    for (auto Dq : DK) {
        const auto& dX = Dq[0][3];
        const auto& dY = Dq[1][3];
        const auto& dZ = Dq[2][3];

        if (!position_only) {
            const auto& dR33 = Dq[2][2];
            const auto& dR32 = Dq[2][1];
            const auto& dR31 = Dq[2][0];
            const auto& dR21 = Dq[1][0];
            const auto& dR11 = Dq[0][0];

            // Converting from rotation matrix to euler angles
            // theta_x = atan2(R32,R33);
            double d_theta_x = dR32*R33/(R32*R32 + R33*R33) +
                               dR33*R32/(R32*R32 + R33*R33);

            // theta_y = atan2(-R31, sqrt(R32*R32 + R33*R33));
            double d_theta_y = -dR31*sqrt(R32*R32 + R33*R33)/(R31*R31 + R32*R32 + R33*R33) +
                               dR32*(R31*R32)/(sqrt(R32*R32 + R33*R33) + (R31*R31 + R32*R32 + R33*R33)) +
                               dR33*(R31*R32)/(sqrt(R32*R32 + R33*R33) + (R31*R31 + R32*R32 + R33*R33));

            // theta_z = atan2(R21,R11);
            double d_theta_z = dR21*R11/(R21*R21 + R11*R11) +
                               dR11*R21/(R21*R21 + R11*R11);

            retval.push_back( {d_theta_x, d_theta_y, d_theta_z, dX, dY, dZ} );
        } else {
            retval.push_back( {dX, dY, dZ} );
        }
    }

    return retval;
}

std::vector<double>
RobotTemplate::inverse_differential_kinematics(std::vector<std::vector<double>> jacobian,
                                               std::vector<double> delta_position) {
    int n_rows = jacobian.size();
    int n_cols = jacobian[0].size();
    Eigen::MatrixXd J (n_rows, n_cols);
    Eigen::MatrixXd J_inv;

    for (int dq_row = 0; dq_row < n_rows; dq_row++) {
        for (int dq_col = 0; dq_col < n_cols; dq_col++) {
            J(dq_row, dq_col) = jacobian[dq_row][dq_col];
        }
    }

    // check jacobian matrix rank to ensure full controllability
    Eigen::ColPivHouseholderQR<Eigen::MatrixXd> J_decomposition (J);
    if (J_decomposition.rank() < delta_position.size()) {
        throw std::out_of_range("Inverse Differential Kinematics failed!\n"
                           "Robot geometry is not controllable on the space defined by delta_position!\n");
    }

    // check if the true jacobian inverse can be used,
    if (n_rows == n_cols && abs(J.determinant()) > 1e-6 ) {
        // use true inverse
        J_inv = J.inverse();
    } else {
        // use pseudoinverse
        J_inv = (J.transpose()*J).inverse() * J;
    }

    Eigen::VectorXd delta_position_vector(delta_position.size());
    for (int c = 0; c < n_cols; c++) {
        delta_position_vector[c] = delta_position[c];
    }

    Eigen::VectorXd delta_q_vector = J_inv*delta_position_vector;

    std::vector<double> delta_q(n_cols);
    for (int c = 0; c < n_cols; c++) {
        delta_q[c] = delta_q_vector[c];
    }
    return delta_q;
}

#endif // ROBOT_TEMPLATE_H
