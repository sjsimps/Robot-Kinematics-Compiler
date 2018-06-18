
#ifndef TRANSFORM_H
#define TRANSFORM_H

#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <regex>
#include <algorithm>
#include <set>
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
    bool is_actuated();
    std::vector<std::vector<double>> evaluate(double joint=0);
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
            throw out_of_range("Transforms must be either REVOLUTE, PRISMATIC, or STATIC");
            break;
    }

    m_transform = (
        (cos(m_theta), -sin(m_theta)*cos(m_alpha), sin(m_theta)*sin(m_alpha), m_a*cos(m_theta)),
        (sin(m_theta), cos(m_theta)*cos(m_alpha), -cos(m_theta)*sin(m_alpha), m_a*sin(m_theta)),
        (zero, sin(m_alpha), cos(m_alpha), m_d),
        (zero, zero, zero, one) );

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
            throw out_of_range("Transforms must be either REVOLUTE, PRISMATIC, or STATIC");
            break;
    }
}

Transform::~Transform(){
}

std::vector<std::vector<double>> Transform::evaluate(double joint_value) {
    Symbolic transform = m_transform;
    std::vector<std::vector<double>> retval { {1, 0, 0, 0},
                                              {0, 1, 0, 0},
                                              {0, 0, 1, 0},
                                              {0, 0, 0, 1} };

    if (is_actuated()) {
        Symbolic joint = get_actuated_joint();
        transform = transform[joint == joint_value];
    }

    for (int r = 0; r < 4; r++) {
        for (int c = 0; c < 4; c++) {
            retval[r][c] = double(transform(r, c));
        }
    }

    return retval;
}

bool Transform::is_actuated() {
    return (m_joint_type == REVOLUTE || m_joint_type == PRISMATIC);
}

Symbolic Transform::get_actuated_joint() {
    if (!is_actuated()) {
        throw logic_error("Cannot get actuated joint. Joint is not actuated.");
    }
    switch(m_joint_type) {
        case REVOLUTE:
            return m_theta;
        case PRISMATIC:
            return m_d;
        default:
            return 0;
    }
}

#endif
