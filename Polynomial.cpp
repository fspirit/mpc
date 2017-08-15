//
// Created by Stanislav Olekhnovich on 10/08/2017.
//

#include <cmath>
#include <iostream>

#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"

#include "Polynomial.h"

const int Degree = 3;

Polynomial::Polynomial(Eigen::VectorXd& x, Eigen::VectorXd& y)
{
    Eigen::MatrixXd A(x.size(), Degree + 1);

    for (int i = 0; i < x.size(); i++)
        A(i, 0) = 1.0;

    for (int j = 0; j < x.size(); j++)
        for (int i = 0; i < Degree; i++)
            A(j, i + 1) = A(j, i) * x(j);

    auto Q = A.householderQr();
    coeffs = Q.solve(y);
}


