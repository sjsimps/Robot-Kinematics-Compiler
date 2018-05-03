
#include <iostream>
#include <string>
#include <vector>
#include <regex>
#include <algorithm>

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
    void simplify();

    SumExpression m_expr;
};

std::ostream &operator<<(std::ostream &os, ExpressionTree const &tree) {
    for (auto mult : tree.m_expr.elements) {
        os << ((mult.positive) ? "+" : "-");
        os << mult.elements.front();
        for (auto it = mult.elements.begin()+1; it != mult.elements.end(); ++it) {
            os << "*" << *it;
        }
    }
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

void ExpressionTree::simplify() {
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

    for (auto expr1 = m_expr.elements.begin(); expr1 != m_expr.elements.end(); expr1++) {
        std::string scalar1 = get_scalar(*expr1);
        for (auto expr2 = (expr1+1); expr2 != m_expr.elements.end(); expr2++) {
            std::string scalar2 = get_scalar(*expr2);
            if (scalar1 == scalar2) {
                std::vector<std::string> expr1_elem, expr2_elem;
                get_common_elements(*expr1, *expr2, &expr1_elem, &expr2_elem);
                std::cout << "\nS1:" << scalar1 << " / S2: " << scalar2 << " / EXPR1ELEM: ";
                for (auto s: expr1_elem) { std::cout << s << " "; }
                std::cout << " / EXPRE2ELEM: ";
                for (auto s: expr2_elem) { std::cout << s << " "; }
                std::cout << "\n";
            }
        }
    }
}

///////////////////////////////////////////////////////////////////////////////

class Arm {
public:
    Symbolic m_forward_kinematics;
    Symbolic m_inverse_differential_kinematics_3d;
    Symbolic m_inverse_differential_kinematics_6d;

    std::vector<Symbolic> m_differential_kinematics;

    std::vector<Symbolic> m_actuated_joints;

    Arm(std::vector<Transform> transforms);
    ~Arm();
    void export_expressions();
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

    for (auto joint : m_actuated_joints) {
        //std::string name = std::string(name(joint));
        Symbolic diff_kin("d_d");//+name);
        diff_kin = df(m_forward_kinematics, joint);
        m_differential_kinematics.push_back(diff_kin);
    }
}

Arm::~Arm(){
}

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

void Arm::export_expressions(){
    std::string kin_str;
    std::vector<std::string> dif_str;
    std::ostringstream stream;
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
    std::cout << kin_str;
    std::cout << dif_str[0];


    std::vector<std::string> expressions = split (kin_str, " ");
    for (auto e : expressions) {
        ExpressionTree tree (e);
        tree.simplify();
        std::cout << " TREE : \n" << e << "\n" << tree;
    }
    std::cout << "\n//////////////////////////////\n"
        << "#include <iostream>\n"
        << "#include <string>\n"
        << "#include <vector>\n"
        << "#include <math.h>\n"
        << "static std::vector<double> forward_kinematics(";
    for (auto joint = m_actuated_joints.begin(); joint != m_actuated_joints.end(); joint++) {
        std::string name = get_name(*joint);
        std::cout << "double " << name << ((joint == m_actuated_joints.end()-1) ? ") {\n" : ", ");
    }
    for (auto joint : m_actuated_joints) {
        std::string name = get_name(joint);
        std::cout << "    double c_" << name << " = cos(" << name << ");\n"
                  << "    double s_" << name << " = sin(" << name << ");\n";
    }
    std::cout << "    std::vector<double> kinematics;\n";
    std::cout << "    double result;\n";
    for (auto expr : expressions) {
        std::cout << "    result = " << expr << ";\n";
        std::cout << "    kinematics.push_back(result);\n";
    }
    std::cout << "    return kinematics;\n"
              << "}\n"
              << "int main (int argc, char* argv[]) {\n"
              << "    std::vector<double> result = forward_kinematics(";
    for (auto joint = m_actuated_joints.begin(); joint != m_actuated_joints.end(); joint++) {
        std::cout << "1.0" << ((joint == m_actuated_joints.end()-1) ? ");\n" : ", ");
    }
    std::cout << "    for (auto r : result) { std::cout << \"\\n\" << r; }\n"
              << "}\n";

}

///////////////////////////////////////////////////////////////////////////////

int main (int argc, char* argv[]) {
    Transform T1(1,1,1,1,REVOLUTE,1);
    Transform T2(1,1,1,1,REVOLUTE,2);
    Transform T3(1,1,1,1,REVOLUTE,3);
    std::vector<Transform> transforms = {T1, T2, T3};


    Arm arm(transforms);
    arm.export_expressions();
    return 0;
}






