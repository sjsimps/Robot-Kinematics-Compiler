
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

///////////////////////////////////////////////////////////////////////////////

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

///////////////////////////////////////////////////////////////////////////////


struct MultiplyExpression {
    bool positive;
    std::vector<std::string> elements;
};
struct SumExpression {
    std::vector<MultiplyExpression> elements;
};

class ExpressionTree {
public:
    ExpressionTree(std::string expr);
    ~ExpressionTree();
    std::set<std::string> simplify();

    SumExpression m_expr;
};

std::ostream &operator<<(std::ostream &os, MultiplyExpression const &mult_exp) {
    if (mult_exp.elements.size() == 0) {
        return os;
    }
    os << ((mult_exp.positive) ? "+" : "-");
    os << mult_exp.elements.front();
    for (auto it = mult_exp.elements.begin()+1; it != mult_exp.elements.end(); ++it) {
        os << "*" << *it;
    }
    return os;
}
std::ostream &operator<<(std::ostream &os, SumExpression const &sum_exp) {
    for (auto mult_exp : sum_exp.elements) {
        os << mult_exp;
    }
    return os;
}
std::ostream &operator<<(std::ostream &os, ExpressionTree const &tree) {
    os << tree.m_expr;
    return os;
}

ExpressionTree::ExpressionTree(std::string expr) {
    std::string element = "";
    MultiplyExpression mult;
    mult.positive = true;

    auto add_element_to_tree = [&] () {
        if (element.size()) {
            mult.elements.push_back(element);
            element.clear();
        }
        if (mult.elements.size()) {
            m_expr.elements.push_back(mult);
            mult.elements.clear();
        }
    };

    for (char c : expr) {
        switch (c) {
            case '+':
                add_element_to_tree();
                mult.positive = true;
                break;
            case '-':
                add_element_to_tree();
                mult.positive = false;
                break;
            case '*':
                if (element.size()) {
                    mult.elements.push_back(element);
                    element.clear();
                }
                break;
            default:
                element.append(1,c);
                break;
        }
    }
    add_element_to_tree();
}

ExpressionTree::~ExpressionTree() {}

std::set<std::string> ExpressionTree::simplify() {

    std::set<std::string> declared_variables;

    auto get_scalar = [](const MultiplyExpression& expr) {
        for (std::string element : expr.elements) {
            if (element.size() > 0 &&
                   ( (element[0] >= '0' && element[0] <= '9') || element[0] == '.' )) {
                return element;
            }
        }
        return std::string("");
    };

    auto get_common_elements = [](const MultiplyExpression& expr1, const MultiplyExpression& expr2,
                                   std::vector<std::string>* expr1_elements, std::vector<std::string>* expr2_elements) {
        std::vector<std::string> common;
        for (auto elem1 = expr1.elements.begin(); elem1 != expr1.elements.end(); elem1++) {
            auto found = std::find(expr2.elements.begin(), expr2.elements.end(), *elem1);
            if (found == expr2.elements.end()) {
                expr1_elements->push_back(*elem1);
                continue;
            }
            common.push_back(*elem1);
        }
        for (auto elem2 = expr2.elements.begin(); elem2 != expr2.elements.end(); elem2++) {
            auto found = std::find(expr1.elements.begin(), expr1.elements.end(), *elem2);
            if (found == expr1.elements.end()) {
                expr2_elements->push_back(*elem2);
            }
        }
        return common;
    };

    auto angle_sum_difference = [get_common_elements](const MultiplyExpression& expr1, const MultiplyExpression& expr2, std::string* new_variable_declare) {

        std::vector<std::string> common, expr1_elem, expr2_elem;
        std::string a1, b1, a2, b2;
        char op11, op12, op21, op22;
        MultiplyExpression retval;

        common = get_common_elements(expr1, expr2, &expr1_elem, &expr2_elem);

        if (expr1_elem.size() != 2 || expr2_elem.size() != 2) {
            return retval;// NOTHING!
        }

        a1 = expr1_elem[0].substr(1);
        b1 = expr1_elem[1].substr(1);
        a2 = expr2_elem[0].substr(1);
        b2 = expr2_elem[1].substr(1);

        op11 = expr1_elem[0][0];
        op12 = expr1_elem[1][0];
        op21 = expr2_elem[0][0];
        op22 = expr2_elem[1][0];

        // cos(a+b) == cos(a)cos(b) - sin(a)sin(b)
        // check operators
        if ( (op11 == op12 && op21 == op22) &&
             ((op11 == 'c' && op22 == 's') || (op11 == 's' && op22 == 'c')) &&
             expr1.positive != expr2.positive ) {
            // check operands
            if ( (a1 == a2 && b1 == b2) ||
                 (a1 == b2 && b1 == a2) ) {
                // SIMPLIFY!
                retval.positive = ( (op11 == 'c' && expr1.positive) ||
                                    (op22 == 'c' && expr2.positive) );
                retval.elements = common;
                std::string simple ("c" + a1 + b1);
                retval.elements.push_back(simple);

                // Declaring new needed variable
                std::string sum = a1.substr(1) + b1;
                std::replace(sum.begin(), sum.end(), '_', '+');
                *new_variable_declare = "double " + simple + " = cos(" + sum + ");\n";
                return retval;
            }
        }

        // sin(a+b) == sin(a)cos(b) + cos(a)sin(b)
        // check operators
        if ( (op11 != op12 && op21 != op22) &&
             (op11 == 'c' || op11 == 's') &&
             (op12 == 'c' || op12 == 's') &&
             (op21 == 'c' || op21 == 's') &&
             (op22 == 'c' || op22 == 's') &&
             expr1.positive == expr2.positive ) {
            // check operands
            if ( (a1 == a2 && b1 == b2) ||
                 (a1 == b2 && b1 == a2) ) {
                // SIMPLIFY!
                retval.positive = expr1.positive;
                retval.elements = common;
                std::string simple ("s" + a1 + b1);
                retval.elements.push_back(simple);

                // Declaring new needed variable
                std::string sum = a1.substr(1) + b1;
                std::replace(sum.begin(), sum.end(), '_', '+');
                *new_variable_declare = "double " + simple + " = sin(" + sum + ");\n";
                return retval;
            }
        }
        return retval;
    };

    auto common_factor = [get_common_elements](const MultiplyExpression& expr1, const MultiplyExpression& expr2) {
        MultiplyExpression retval, factor1, factor2;
        std::string new_element;
        std::ostringstream nstream;

        retval.elements = get_common_elements(expr1, expr2, &factor1.elements, &factor2.elements);

        if (retval.elements.size() == 0) {
            return retval;
        }

        retval.positive = true;
        factor1.positive = expr1.positive;
        factor2.positive = expr2.positive;

        nstream << "("
                << factor1 << ((factor1.elements.size() == 0) ? ((factor1.positive) ? "+1" : "-1" ) : "")
                << factor2 << ((factor2.elements.size() == 0) ? ((factor2.positive) ? "+1" : "-1" ) : "")
                << ")";
        new_element = nstream.str();
        retval.elements.push_back(new_element);

        return retval;
    };

    // Angle-sum difference simplification
    bool simplifying = false;
    do {
        for (auto expr1 = m_expr.elements.begin(); expr1 != m_expr.elements.end(); expr1++) {
            std::string scalar1 = get_scalar(*expr1);
            for (auto expr2 = (expr1+1); expr2 != m_expr.elements.end(); expr2++) {
                std::string scalar2 = get_scalar(*expr2);
                if (scalar1 == scalar2) {
                    std::string new_variable;
                    auto simplified = angle_sum_difference(*expr1, *expr2, &new_variable);
                    if (simplified.elements.size() > 0) {
                        m_expr.elements.erase(expr2);
                        m_expr.elements.erase(expr1);
                        m_expr.elements.push_back(simplified);
                        simplifying = true;
                        declared_variables.insert(new_variable);
                        break;
                    }
                }
                simplifying = false;
            }
            if (simplifying) {
                break;
            }
        }
    } while (simplifying);

    // Common factoring - Not guaranteed to be minimal!
    // Could add some searching to improve how expressions are factored
    simplifying = false;
    do {
        for (auto expr1 = m_expr.elements.begin(); expr1 != m_expr.elements.end(); expr1++) {
            for (auto expr2 = (expr1+1); expr2 != m_expr.elements.end(); expr2++) {
                auto simplified = common_factor(*expr1, *expr2);
                if (simplified.elements.size() > 0) {
                    m_expr.elements.erase(expr2);
                    m_expr.elements.erase(expr1);
                    m_expr.elements.push_back(simplified);
                    simplifying = true;
                    break;
                }
                simplifying = false;
            }
            if (simplifying) {
                break;
            }
        }
    } while (simplifying);

    return declared_variables;
}

///////////////////////////////////////////////////////////////////////////////

static void replace(std::string* in, std::string pattern, std::string replacement){
    *in = std::regex_replace( *in, std::regex(pattern), replacement );
}

std::vector<std::string> split(const std::string& str, const std::string& delim)
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



class Arm {
public:
    Symbolic m_forward_kinematics;
    std::vector<Symbolic> m_differential_kinematics;
    std::vector<Symbolic> m_actuated_joints;

    Arm(std::vector<Transform> transforms);
    ~Arm();
    void export_expressions();
};

Arm::Arm(std::vector<Transform> transforms){
    std::cout << "Generating kinematic chain ... " << std::flush;
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

    for (auto joint : m_actuated_joints) {
        std::string name = get_name(joint);
        Symbolic diff_kin("d"+name);
        diff_kin = df(m_forward_kinematics, joint);
        m_differential_kinematics.push_back(diff_kin);
    }
    std::cout << "Done\n" << std::flush;
}

Arm::~Arm(){
}

void Arm::export_expressions(){
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

        replace(&kin_str, "sin\\("+name+"\\)", "s_"+name);
        replace(&kin_str, "cos\\("+name+"\\)", "c_"+name);
        for (int index = 0; index < m_actuated_joints.size(); index++) {
            replace(&dif_str[index], "sin\\("+name+"\\)", "s_"+name);
            replace(&dif_str[index], "cos\\("+name+"\\)", "c_"+name);
        }
    }

    outfile << "#include <iostream>\n"
            << "#include <string>\n"
            << "#include <vector>\n"
            << "#include <math.h>\n"
            << "#include <time.h>\n";

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

            //std::cout << " TREE : \n" << expressions[expr_idx] << "\n" << tree;

            stream.str(std::string());
            stream << tree;
            expressions[expr_idx] = stream.str();
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

///////////////////////////////////////////////////////////////////////////////

int main (int argc, char* argv[]) {
    Transform T1(1,1,1,1,REVOLUTE,1);
    Transform T2(1,1,1,1,REVOLUTE,2);
    Transform T3(1,1,1,1,REVOLUTE,3);
    Transform T4(1,1,1,1,REVOLUTE,4);
    Transform T5(1,1,1,1,REVOLUTE,5);
    Transform T6(1,1,1,1,REVOLUTE,6);
    std::vector<Transform> transforms = {T1, T2, T3, T4, T5, T6};


    Arm arm(transforms);
    arm.export_expressions();
    return 0;
}






