
#include <iostream>
#include <string>
#include <vector>

#include "symbolicc++.h"

#define PRISMATIC 1
#define REVOLUTE 2
#define STATIC 3

class Transform {
public:
    Symbolic m_theta, m_d, m_a, m_alpha, m_transform;
    int m_joint_type;
    std::string m_joint_id;

    Transform(double theta, double d, double a, double alpha,
                     int joint_type, int joint_id=STATIC);
    ~Transform();

    Symbolic get_actuated_joint();
};

Transform::Transform(double theta, double d, double a, double alpha,
                     int joint_type, int joint_id){
    std::string id = std::to_string(joint_id);
    m_theta = Symbolic("theta"+id);
    m_d = Symbolic("d"+id);
    m_a = Symbolic("a"+id);
    m_alpha = Symbolic("alpha"+id);
    m_joint_type = joint_type;
    m_joint_id = joint_id;

    Symbolic zero("0");
    Symbolic one("1");

    switch(joint_type) {
        case REVOLUTE:
            m_theta = Symbolic("q"+id);
            break;
        case PRISMATIC:
            m_d = Symbolic("q"+id);
            break;
        case STATIC:
            break;
        default:
            // TODO : throw
            break;
    }

    m_transform = (
        (cos(m_theta), -sin(m_theta)*cos(m_alpha), sin(m_theta)*sin(m_alpha), m_a*cos(m_theta)),
        (sin(m_theta), cos(m_theta)*cos(m_alpha), -cos(m_theta)*sin(m_alpha), m_a*sin(m_theta)),
        (zero, sin(m_alpha), cos(m_alpha), m_d),
        (zero, zero, zero, one) );

    std::cout << m_transform;

    switch(joint_type) {
        case REVOLUTE:
            m_transform = m_transform[zero == 0, one == 1,
                                      m_d == d, m_a == a, m_alpha == alpha];
            break;
        case PRISMATIC:
            m_transform = m_transform[zero == 0, one == 1,
                                      m_theta == theta, m_a == a, m_alpha == alpha];
            break;
        case STATIC:
            m_transform = m_transform[zero == 0, one == 1,
                                      m_theta == theta, m_d == d, m_a == a, m_alpha == alpha];
            break;
        default:
            // TODO : throw
            break;
    }

    std::cout << m_transform;
}

Transform::~Transform(){
}

Symbolic Transform::get_actuated_joint(){
    switch(m_joint_type) {
        case REVOLUTE:
            return m_theta;
            break;
        case PRISMATIC:
            return m_d;
            break;
        default:
            // TODO : throw
            break;
    }
}


class Arm {
public:
    Symbolic m_forward_kinematics;
    Symbolic m_inverse_differential_kinematics_3d;
    Symbolic m_inverse_differential_kinematics_6d;

    std::vector<Symbolic> m_differential_kinematics;

    std::vector<Symbolic> m_actuated_joints;

    Arm(std::vector<Transform> transforms);
    ~Arm();
    void forward_kinematics(std::vector<double>);
    void differential_kinematics(std::vector<double>);
};

Arm::Arm(std::vector<Transform> transforms){
    bool first = true;
    for (auto T : transforms) {
        if (T.m_joint_type == REVOLUTE ||
            T.m_joint_type == PRISMATIC){
            m_actuated_joints.push_back(T.get_actuated_joint());
        }

        if (first) {
            m_forward_kinematics = T.m_transform;
            first = false;
            continue;
        }
        m_forward_kinematics = m_forward_kinematics*T.m_transform;
    }
    std::cout<< m_forward_kinematics;

    for (auto joint : m_actuated_joints) {
        Symbolic diff_kin("d_dq");
        diff_kin = df(m_forward_kinematics, joint);
        m_differential_kinematics.push_back(diff_kin);
    }

    std::cout<< m_differential_kinematics[0];

    Symbolic differential_3d("3d",m_differential_kinematics.size(),3);
    int index = 0;
    for (auto dq : m_differential_kinematics) {
        differential_3d[index][0] = dq[0][3];
        differential_3d[index][1] = dq[1][3];
        differential_3d[index][2] = dq[2][3];
    }
    
}

Arm::~Arm(){
}


int main (int argc, char* argv[]) {
    Transform T1(1,1,1,1,REVOLUTE,1);
    Transform T2(1,1,1,1,REVOLUTE,2);
    Transform T3(1,1,1,1,REVOLUTE,3);
    std::vector<Transform> transforms = {T1, T2, T3};


    Arm arm(transforms);
    return 0;
}






