
#ifndef ARM_H
#define ARM_H

#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <regex>
#include <algorithm>
#include <set>

#include "symbolicc++.h"

#include "transform.h"
#include "expressiontree.h"

static void replace(std::string* in, std::string pattern, std::string replacement){
    *in = std::regex_replace( *in, std::regex(pattern), replacement );
}

static std::vector<std::string> split(const std::string& str, const std::string& delim)
{
    std::vector<std::string> tokens;
    size_t prev = 0, pos = 0;
    do {
        pos = str.find(delim, prev);
        if (pos == string::npos) pos = str.length();
        std::string token = str.substr(prev, pos-prev);
        token.erase(std::remove_if(token.begin(), token.end(), ::isspace), token.end());
        if (!token.empty()) tokens.push_back(token);
        prev = pos + delim.length();
    }
    while (pos < str.length() && prev < str.length());
    return tokens;
}

static std::string get_name(const Symbolic& symb){
    std::string name;
    std::ostringstream nstream;
    nstream << symb;
    name = nstream.str();
    return name;
}

static std::ostream &operator<<(std::ostream &os, std::vector<std::vector<double>> const &matrix) {
    std::cout << "[\n";
    for (auto x : matrix) {
        std::cout << "\t[";
        for (auto y : x) {
            std::cout << " " << y;
        }
        std::cout << " ]\n";
    }
    std::cout << "]\n";
}

static std::vector<std::vector<double>>
multiply_transforms(const std::vector<std::vector<double>>& T_a, const std::vector<std::vector<double>>& T_b) {
    if (T_a.size() != 4 || T_b.size() != 4) {
        throw length_error("Transforms must be 4x4");
    }
    std::vector<std::vector<double>> retval { {0,0,0,0},
                                              {0,0,0,0},
                                              {0,0,0,0},
                                              {0,0,0,0} };

    for (int c = 0; c < 4; c++) {
        for (int r = 0; r < 4; r++) {
            for (int i = 0; i < 4; i++) {
                retval[r][c] += T_a[r][i] * T_b[i][c];
            }
        }
    }
    return retval;
}

class Arm {
public:
    Symbolic m_forward_kinematics;
    std::vector<Symbolic> m_differential_kinematics;
    std::vector<Symbolic> m_actuated_joints;
    std::vector<Transform> m_transforms;

    Arm(const std::vector<Transform>& transforms);
    ~Arm();
    void export_expressions();
    std::vector<std::vector<std::vector<double>>> get_positions(std::vector<double> joints);
};

Arm::Arm(const std::vector<Transform>& transforms){
    m_transforms = transforms;
    for (auto T : m_transforms) {
        if (T.is_actuated()){
            m_actuated_joints.push_back(T.get_actuated_joint());
        }
    }
}

Arm::~Arm(){
}

void Arm::export_expressions(){

    ////////////
    // Generating kinematic chain
    // Generates symbolic expressions for kinematics
    std::cout << "Generating kinematic chain ... " << std::flush;
    bool first = true;
    for (auto T : m_transforms) {
        if (first) {
            m_forward_kinematics = T.m_transform;
            first = false;
            continue;
        }
        m_forward_kinematics = m_forward_kinematics*T.m_transform;
    }

    for (auto joint : m_actuated_joints) {
        std::string name = get_name(joint);
        Symbolic diff_kin("d"+name);
        diff_kin = df(m_forward_kinematics, joint);
        m_differential_kinematics.push_back(diff_kin);
    }
    std::cout << "Done\n" << std::flush;

    ////////////
    // Compiling kinematics into executable C++ code
    // Simplifies trigonometric expressions
    std::cout << "Compiling kinematic expressions ..." << std::flush;
    std::string kin_str;
    std::vector<std::string> dif_str;
    std::ostringstream stream;
    std::ofstream outfile ("out.cpp", std::ofstream::binary);
    stream << m_forward_kinematics;
    kin_str = stream.str();

    for (int index = 0; index < m_actuated_joints.size(); index++) {
        stream.str(std::string());
        stream << m_differential_kinematics[index];
        dif_str.push_back(stream.str());
        replace(&dif_str[index], " * ", " ");
        replace(&dif_str[index], "\\[*\\]*", "");
    }

    replace(&kin_str, " * ", " ");
    replace(&kin_str, "\\[*\\]*", "");
    for (auto joint : m_actuated_joints) {
        std::string name = get_name(joint);

        replace(&kin_str, "e\\+", "P");
        replace(&kin_str, "e\\-", "N");
        replace(&kin_str, "sin\\("+name+"\\)", "s_"+name);
        replace(&kin_str, "cos\\("+name+"\\)", "c_"+name);
        for (int index = 0; index < m_actuated_joints.size(); index++) {
            replace(&dif_str[index], "e\\+", "P");
            replace(&dif_str[index], "e\\-", "N");
            replace(&dif_str[index], "sin\\("+name+"\\)", "s_"+name);
            replace(&dif_str[index], "cos\\("+name+"\\)", "c_"+name);
        }
    }

    outfile << "#include <iostream>\n"
            << "#include <string>\n"
            << "#include <vector>\n"
            << "#include <math.h>\n"
            << "#include <time.h>\n";

    ////////////
    // Compile Forward Kinematics:
    std::vector<std::string> expressions = split (kin_str, " ");
    std::set<std::string> new_variables;

    for (int expr_idx = 0; expr_idx < expressions.size(); expr_idx++) {
        // Simplify the kinematics expressions
        ExpressionTree tree (expressions[expr_idx]);
        auto variables = tree.simplify();
        new_variables.insert(variables.begin(), variables.end());
        stream.str(std::string());
        stream << tree;
        expressions[expr_idx] = stream.str();
        replace(&expressions[expr_idx], "P", "e+");
        replace(&expressions[expr_idx], "N", "e-");
    }

    outfile << "static std::vector<double> forward_kinematics(";
    for (auto joint = m_actuated_joints.begin(); joint != m_actuated_joints.end(); joint++) {
        std::string name = get_name(*joint);
        outfile << "double " << name << ((joint == m_actuated_joints.end()-1) ? ") {\n" : ", ");
    }
    for (auto joint : m_actuated_joints) {
        std::string name = get_name(joint);
        outfile << "    double c_" << name << " = cos(" << name << ");\n"
                << "    double s_" << name << " = sin(" << name << ");\n";
    }
    for (auto var : new_variables) {
        outfile << "    " << var;
    }
    outfile << "    std::vector<double> kinematics;\n";
    outfile << "    double result;\n";
    for (auto expr : expressions) {
        outfile << "    result = " << expr << ";\n";
        outfile << "    kinematics.push_back(result);\n";
    }
    outfile << "    return kinematics;\n"
            << "}\n";

    ////////////
    // Compile Differential Kinematics:
    for (int index = 0; index < m_actuated_joints.size(); index++) {
        std::string joint_name = get_name(m_actuated_joints[index]);
        expressions = split (dif_str[index], " ");
        new_variables.clear();
        for (int expr_idx = 0; expr_idx < expressions.size(); expr_idx++) {
            // Simplify the kinematics expressions
            ExpressionTree tree (expressions[expr_idx]);
            auto variables = tree.simplify();
            new_variables.insert(variables.begin(), variables.end());
            stream.str(std::string());
            stream << tree;
            expressions[expr_idx] = stream.str();
            replace(&expressions[expr_idx], "P", "e+");
            replace(&expressions[expr_idx], "N", "e-");
        }

        outfile << "static std::vector<double> differential_kinematics_d" << joint_name << "(";
        for (auto joint = m_actuated_joints.begin(); joint != m_actuated_joints.end(); joint++) {
            std::string name = get_name(*joint);
            outfile << "double " << name << ((joint == m_actuated_joints.end()-1) ? ") {\n" : ", ");
        }
        for (auto joint : m_actuated_joints) {
            std::string name = get_name(joint);
            outfile << "    double c_" << name << " = cos(" << name << ");\n"
                    << "    double s_" << name << " = sin(" << name << ");\n";
        }
        for (auto var : new_variables) {
            outfile << "    " << var;
        }
        outfile << "    std::vector<double> kinematics;\n";
        outfile << "    double result;\n";
        for (auto expr : expressions) {
            outfile << "    result = " << expr << ";\n";
            outfile << "    kinematics.push_back(result);\n";
        }
        outfile << "    return kinematics;\n"
                << "}\n";
    }


    ////////////
    // Test output:
    outfile << "int main (int argc, char* argv[]) {\n"
            << "    clock_t timer = clock();\n"
            << "    std::vector<double> result = forward_kinematics(";
    for (auto joint = m_actuated_joints.begin(); joint != m_actuated_joints.end(); joint++) {
        outfile << "1.0" << ((joint == m_actuated_joints.end()-1) ? ");\n" : ", ");
    }
    outfile << "    for (auto r : result) { std::cout << r << \"\\n\"; }\n"
            << "    timer = clock() - timer;\n"
            << "    std::cout << \"Forward Kinematics Computation Time : \" << ((float)timer)/CLOCKS_PER_SEC << \" seconds\\n\";\n"
            << "}\n";


    std::cout << "Done\n" << std::flush;
}

std::vector<std::vector<std::vector<double>>>
Arm::get_positions(std::vector<double> joints) {
    std::vector<std::vector<std::vector<double>>> retval;
    int joint_index = 0;

    // TODO : add some safety around number of actuated joints
    std::vector<std::vector<double>> T_prev;
    for (auto T : m_transforms) {
        if (T_prev.size() == 0) {
            T_prev = T.evaluate(joints[joint_index]);
        } else {
            T_prev = multiply_transforms(T_prev, T.evaluate(joints[joint_index]));
        }
        if (T.is_actuated()) {
            joint_index++;
        }

        retval.push_back(T_prev);
    }
    return retval;
}


#endif
