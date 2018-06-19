
#include "RoboticsTools/arm.h"
#include "RoboticsTools/renderer.h"
#define PI 3.14159265359

int main (int argc, char* argv[]) {
    // Arm specification using Denavit-Hartenberg parameters:
    Transform T1(0,1,0,PI/2,REVOLUTE,1);
    Transform T2(0,1,0,PI/2,REVOLUTE,2);
    Transform T3(0,1,0,PI/2,REVOLUTE,3);
    std::vector<Transform> transforms = {T1, T2, T3};

    // Arm instantiation, rendering, and expression compilation
    Arm arm(transforms);
    render_arm(&arm);
    arm.export_expressions("example_out.cpp");
    return 0;
}

