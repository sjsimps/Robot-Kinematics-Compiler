
#include "robot_template.h"
#include <iostream>
#include <string>
#include <vector>
#include <math.h>
#include <time.h>

class Robot : public RobotTemplate {
public:
    Robot();
    ~Robot();

    std::vector<std::vector<double>> forward_kinematics(std::vector<double> q);
    std::vector<std::vector<std::vector<double>>> differential_kinematics(std::vector<double> q);

};
Robot::Robot(){}
Robot::~Robot(){}


std::vector<std::vector<double>>
Robot::forward_kinematics(std::vector<double> q) {
    const double& q1 = q[0];
    const double& q2 = q[1];
    const double& q3 = q[2];

    double c_q1 = cos(q1);
    double s_q1 = sin(q1);
    double c_q2 = cos(q2);
    double s_q2 = sin(q2);
    double c_q3 = cos(q3);
    double s_q3 = sin(q3);
    double c_q1_q3 = cos(q1+q3);
    double s_q1_q3 = sin(q1+q3);
    double R11 = +c_q1*c_q2*c_q3-1.06939e-26*s_q1*c_q2*s_q3+s_q1*s_q3+1.03412e-13*s_q2*s_q1_q3;
    double R12 = +1.03412e-13*c_q1*c_q2*s_q3+1.10588e-39*s_q1*c_q2*c_q3-1.03412e-13*s_q1*c_q3+c_q1*s_q2-1.03412e-13*s_q1*c_q2-1.03412e-13*s_q1-1.06939e-26*s_q2*c_q1_q3;
    double R13 = +c_q1*c_q2*s_q3+1.06939e-26*s_q1*c_q2*c_q3-s_q1*c_q3-1.03412e-13*c_q1*s_q2+1.06939e-26*s_q1*c_q2+1.06939e-26*s_q1-1.03412e-13*s_q2*c_q1_q3;
    double X   = +c_q1*s_q2-1.03412e-13*s_q1*c_q2+1*s_q1;
    double R21 = +s_q1*c_q2*c_q3+1.06939e-26*c_q1*c_q2*s_q3-c_q1*s_q3-1.03412e-13*s_q2*c_q1_q3;
    double R22 = +1.03412e-13*s_q1*c_q2*s_q3-1.10588e-39*c_q1*c_q2*c_q3+1.03412e-13*c_q1*c_q3+s_q1*s_q2+1.03412e-13*c_q1*c_q2+1.03412e-13*c_q1-1.06939e-26*s_q2*s_q1_q3;
    double R23 = +s_q1*c_q2*s_q3-1.06939e-26*c_q1*c_q2*c_q3+c_q1*c_q3-1.03412e-13*s_q1*s_q2-1.06939e-26*c_q1*c_q2-1.06939e-26*c_q1-1.03412e-13*s_q2*s_q1_q3;
    double Y   = +s_q1*s_q2+1.03412e-13*c_q1*c_q2-1*c_q1;
    double R31 = +s_q2*c_q3-1.03412e-13*c_q2*s_q3-1.03412e-13*s_q3;
    double R32 = +1.03412e-13*s_q2*s_q3+1.06939e-26*c_q2*c_q3+1.06939e-26*c_q3-c_q2+1.06939e-26;
    double R33 = +s_q2*s_q3+1.03412e-13*c_q2*c_q3+1.03412e-13*c_q3+1.03412e-13*c_q2-1.10588e-39;
    double Z   = -c_q2+1;
    std::vector<std::vector<double>> kinematics
        { {R11, R12, R13, X},
          {R21, R22, R23, Y},
          {R31, R32, R33, Z},
          {0, 0, 0, 1} };
    return kinematics;
}
std::vector<std::vector<double>> differential_kinematics_dq1(std::vector<double> q) {
    const double& q1 = q[0];
    const double& q2 = q[1];
    const double& q3 = q[2];

    double c_q1 = cos(q1);
    double s_q1 = sin(q1);
    double c_q2 = cos(q2);
    double s_q2 = sin(q2);
    double c_q3 = cos(q3);
    double s_q3 = sin(q3);
    double c_q1_q3 = cos(q1+q3);
    double s_q1_q3 = sin(q1+q3);
    double R11 = -s_q1*c_q2*c_q3-1.06939e-26*c_q1*c_q2*s_q3+c_q1*s_q3+1.03412e-13*s_q2*c_q1_q3;
    double R12 = -1.03412e-13*s_q1*c_q2*s_q3+1.10588e-39*c_q1*c_q2*c_q3-1.03412e-13*c_q1*c_q3-s_q1*s_q2-1.03412e-13*c_q1*c_q2-1.03412e-13*c_q1+1.06939e-26*s_q2*s_q1_q3;
    double R13 = -s_q1*c_q2*s_q3+1.06939e-26*c_q1*c_q2*c_q3-c_q1*c_q3+1.03412e-13*s_q1*s_q2+1.06939e-26*c_q1*c_q2+1.06939e-26*c_q1+1.03412e-13*s_q2*s_q1_q3;
    double X   = -s_q1*s_q2-1.03412e-13*c_q1*c_q2+1*c_q1;
    double R21 = +c_q1*c_q2*c_q3-1.06939e-26*s_q1*c_q2*s_q3+s_q1*s_q3+1.03412e-13*s_q2*s_q1_q3;
    double R22 = +1.03412e-13*c_q1*c_q2*s_q3+1.10588e-39*s_q1*c_q2*c_q3-1.03412e-13*s_q1*c_q3+c_q1*s_q2-1.03412e-13*s_q1*c_q2-1.03412e-13*s_q1-1.06939e-26*s_q2*c_q1_q3;
    double R23 = +c_q1*c_q2*s_q3+1.06939e-26*s_q1*c_q2*c_q3-s_q1*c_q3-1.03412e-13*c_q1*s_q2+1.06939e-26*s_q1*c_q2+1.06939e-26*s_q1-1.03412e-13*s_q2*c_q1_q3;
    double Y   = +c_q1*s_q2-1.03412e-13*s_q1*c_q2+1*s_q1;
    double R31 = +0;
    double R32 = +0;
    double R33 = +0;
    double Z   = +0;
    std::vector<std::vector<double>> kinematics
        { {R11, R12, R13, X},
          {R21, R22, R23, Y},
          {R31, R32, R33, Z},
          {0, 0, 0, 1} };
    return kinematics;
}
std::vector<std::vector<double>> differential_kinematics_dq2(std::vector<double> q) {
    const double& q1 = q[0];
    const double& q2 = q[1];
    const double& q3 = q[2];

    double c_q1 = cos(q1);
    double s_q1 = sin(q1);
    double c_q2 = cos(q2);
    double s_q2 = sin(q2);
    double c_q3 = cos(q3);
    double s_q3 = sin(q3);
    double c_q1_q3 = cos(q1+q3);
    double s_q1_q3 = sin(q1+q3);
    double R11 = -c_q1*s_q2*c_q3+1.06939e-26*s_q1*s_q2*s_q3+1.03412e-13*c_q2*s_q1_q3;
    double R12 = -1.03412e-13*c_q1*s_q2*s_q3-1.10588e-39*s_q1*s_q2*c_q3+c_q1*c_q2+1.03412e-13*s_q1*s_q2-1.06939e-26*c_q2*c_q1_q3;
    double R13 = -c_q1*s_q2*s_q3-1.06939e-26*s_q1*s_q2*c_q3-1.03412e-13*c_q1*c_q2-1.06939e-26*s_q1*s_q2-1.03412e-13*c_q2*c_q1_q3;
    double X   = +c_q1*c_q2+1.03412e-13*s_q1*s_q2;
    double R21 = -s_q1*s_q2*c_q3-1.06939e-26*c_q1*s_q2*s_q3-1.03412e-13*c_q2*c_q1_q3;
    double R22 = -1.03412e-13*s_q1*s_q2*s_q3+1.10588e-39*c_q1*s_q2*c_q3+s_q1*c_q2-1.03412e-13*c_q1*s_q2-1.06939e-26*c_q2*s_q1_q3;
    double R23 = -s_q1*s_q2*s_q3+1.06939e-26*c_q1*s_q2*c_q3-1.03412e-13*s_q1*c_q2+1.06939e-26*c_q1*s_q2-1.03412e-13*c_q2*s_q1_q3;
    double Y   = +s_q1*c_q2-1.03412e-13*c_q1*s_q2;
    double R31 = +c_q2*c_q3+1.03412e-13*s_q2*s_q3;
    double R32 = +1.03412e-13*c_q2*s_q3-1.06939e-26*s_q2*c_q3+s_q2;
    double R33 = +c_q2*s_q3-1.03412e-13*s_q2*c_q3-1.03412e-13*s_q2;
    double Z   = +s_q2;
    std::vector<std::vector<double>> kinematics
        { {R11, R12, R13, X},
          {R21, R22, R23, Y},
          {R31, R32, R33, Z},
          {0, 0, 0, 1} };
    return kinematics;
}
std::vector<std::vector<double>> differential_kinematics_dq3(std::vector<double> q) {
    const double& q1 = q[0];
    const double& q2 = q[1];
    const double& q3 = q[2];

    double c_q1 = cos(q1);
    double s_q1 = sin(q1);
    double c_q2 = cos(q2);
    double s_q2 = sin(q2);
    double c_q3 = cos(q3);
    double s_q3 = sin(q3);
    double c_q1_q3 = cos(q1+q3);
    double s_q1_q3 = sin(q1+q3);
    double R11 = -c_q1*c_q2*s_q3-1.06939e-26*s_q1*c_q2*c_q3+s_q1*c_q3+1.03412e-13*s_q2*c_q1_q3;
    double R12 = +1.03412e-13*c_q1*c_q2*c_q3-1.10588e-39*s_q1*c_q2*s_q3+1.03412e-13*s_q1*s_q3+1.06939e-26*s_q2*s_q1_q3;
    double R13 = +c_q1*c_q2*c_q3-1.06939e-26*s_q1*c_q2*s_q3+s_q1*s_q3+1.03412e-13*s_q2*s_q1_q3;
    double X   = +0;
    double R21 = -s_q1*c_q2*s_q3+1.06939e-26*c_q1*c_q2*c_q3-c_q1*c_q3+1.03412e-13*s_q2*s_q1_q3;
    double R22 = +1.03412e-13*s_q1*c_q2*c_q3+1.10588e-39*c_q1*c_q2*s_q3-1.03412e-13*c_q1*s_q3-1.06939e-26*s_q2*c_q1_q3;
    double R23 = +s_q1*c_q2*c_q3+1.06939e-26*c_q1*c_q2*s_q3-c_q1*s_q3-1.03412e-13*s_q2*c_q1_q3;
    double Y   = +0;
    double R31 = -s_q2*s_q3-1.03412e-13*c_q2*c_q3-1.03412e-13*c_q3;
    double R32 = +1.03412e-13*s_q2*c_q3-1.06939e-26*c_q2*s_q3-1.06939e-26*s_q3;
    double R33 = +s_q2*c_q3-1.03412e-13*c_q2*s_q3-1.03412e-13*s_q3;
    double Z   = +0;
    std::vector<std::vector<double>> kinematics
        { {R11, R12, R13, X},
          {R21, R22, R23, Y},
          {R31, R32, R33, Z},
          {0, 0, 0, 1} };
    return kinematics;
}

std::vector<std::vector<std::vector<double>>>
Robot::differential_kinematics(std::vector<double> q) {
    std::vector<std::vector<std::vector<double>>> retval;
    retval.push_back(differential_kinematics_dq1(q));
    retval.push_back(differential_kinematics_dq2(q));
    retval.push_back(differential_kinematics_dq3(q));
    return retval;
}

static std::ostream &operator<<(std::ostream &os, std::vector<std::vector<double>> const &matrix) {
    os << "[\n";
    for (auto x : matrix) {
        os << "	[";
        for (auto y : x) {
            os << " " << y;
        }
        os << " ]\n";
    }
    os << "]\n";
}

int main (int argc, char* argv[]) {
    Robot* robot = new Robot();
    std::vector<double> q {1.0, 1.0, 1.0};

    clock_t timer = clock();
    auto result = robot->forward_kinematics(q);
    timer = clock() - timer;
    std::cout << result;

    for (int x = 0; x < 10; x++){
        double X = result[0][3];
        double Y = result[1][3];
        double Z = result[2][3];
        std::cout << "X:"<<X << ", Y:"<<Y <<", Z:"<<Z << "\n";

        auto jacobian = robot->get_jacobian(q, true);
        std::cout << "J: " << jacobian;
        auto D_IK = robot->inverse_differential_kinematics(jacobian, {0, 0, -0.001});
        std::cout << "DQ:"<<D_IK[0] << ", "<<D_IK[1] <<", "<<D_IK[2] << "\n";
        q = {q[0] + D_IK[0], q[1] + D_IK[1], q[2] + D_IK[2]};
        result = robot->forward_kinematics(q);
    }

    std::cout << "Forward Kinematics Computation Time : " << ((float)timer)/CLOCKS_PER_SEC << " seconds\n";
}
