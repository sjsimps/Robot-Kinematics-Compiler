
#ifndef EXPRESSION_TREE_H
#define EXPRESSION_TREE_H

#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <regex>
#include <algorithm>
#include <set>
#include "symbolicc++.h"

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
    /* TODO : Ensure this terminates!
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
    */
    return declared_variables;
}



#endif
