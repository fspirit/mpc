//
// Created by Stanislav Olekhnovich on 10/08/2017.
//

#ifndef MPC_POLYNOMIAL_H
#define MPC_POLYNOMIAL_H

#include <vector>

#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/Core"
#include <cppad/cppad.hpp>

class Polynomial
{
public:

    Polynomial(Eigen::VectorXd& x, Eigen::VectorXd& y);
    const Eigen::VectorXd& GetCoeffs() const {  return coeffs; }

    template<typename T>
    T GetValue(T x) const
    {
        T result = 0.0;
        for (int i = 0; i < coeffs.size(); i++)
            result += coeffs[i] * CppAD::pow(x, i);
        return result;
    }

    template<typename T>
    T GetFirstDerivativeValue(T x) const
    {
        return 3.0 * coeffs[3] * x * x + 2.0 * coeffs[2] * x + coeffs[1];
    }

private:
    Eigen::VectorXd coeffs;
};


#endif //MPC_POLYNOMIAL_H
