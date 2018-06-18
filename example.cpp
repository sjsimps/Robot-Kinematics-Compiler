
#include "RoboticsTools/arm.h"
#include "RoboticsTools/expressiontree.h"
#include "RoboticsTools/renderer.h"
#include "RoboticsTools/transform.h"

#define PI 3.14159265359

int main (int argc, char* argv[]) {

    Transform ROT1(0,0,0,PI/2,REVOLUTE,0);
    Transform ROT2(0,0,0,PI/2,REVOLUTE,0);

    Transform T1(0,1,0,PI/2,REVOLUTE,1);
    Transform T2(0,1,0,PI/2,REVOLUTE,2);
    Transform T3(0,1,0,PI/2,REVOLUTE,3);
    Transform T4(1,1,1,1,REVOLUTE,4);
    Transform T5(1,1,1,1,REVOLUTE,5);
    Transform T6(1,1,1,1,REVOLUTE,6);
    std::vector<Transform> transforms = {ROT1, ROT2, T1, T2, T3};

    Arm arm(transforms);

    render_arm(&arm);

    arm.export_expressions();
    return 0;
}






