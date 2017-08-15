//
// Created by Stanislav Olekhnovich on 14/08/2017.
//

#include "Setup.h"
#include <cppad/ipopt/solve.hpp>
#include "MPCController.h"
#include "CostAndConstraintsCalculator.h"

const double CTEAbsValueWeight = 1000.0;
const double PsiErrorAbsValueWeight = 1000.0;
const double SteeringAngleAbsValueWeight = 20.0;
const double ThrottleAbsValueWeight = 20.0;
const double SteeringAngleDiffWeight = 100.0;
const double ThrottleDiffValueWeight = 20.0;

        void CostAndConstraintsCalculator::operator()(ADvector& fg, const ADvector& x)
{
    fg[0] = CalculateCost(x);

    SetFirstStepState(fg, x);

    // constraints based on our kinematic model
    for (int t = 1; t < N; t++)
    {
        // prev state and actuations
        auto x0 = x[xStartingIndex + t - 1];
        auto y0 = x[yStartingIndex + t - 1];
        auto psi0 = x[psiStartingIndex + t - 1];
        auto v0 = x[vStartingIndex + t - 1];
        auto cte0 = x[cteStartingIndex + t - 1];
        auto epsi0 = x[psiErrorStartingIndex + t - 1];
        auto delta0 = x[steeringAngleStartingIndex + t - 1];
        auto a0 = x[throttleStartingIndex + t - 1];

        // next state
        auto x1 = x[xStartingIndex + t];
        auto y1 = x[yStartingIndex + t];
        auto psi1 = x[psiStartingIndex + t];
        auto v1 = x[vStartingIndex + t];
        auto cte1 = x[cteStartingIndex + t];
        auto epsi1 = x[psiErrorStartingIndex + t];

        // desired py and psi, according to ref path
        auto py_desired = polyfit.GetValue(x0);
        auto psi_desired = CppAD::atan(polyfit.GetFirstDerivativeValue(x0));

        // predicted next state
        auto x1_p = x0 + v0 * CppAD::cos(psi0) * dt;
        auto y1_p = y0 + v0 * CppAD::sin(psi0) * dt;
        auto psi1_p = psi0 + v0 * (-delta0) / Lf * dt;
        auto v1_p = v0 + a0 * dt;
        auto cte1_p = py_desired - y0 + v0 * CppAD::sin(epsi0) * dt;
        auto epsi1_p = psi0 - psi_desired + v0 * (-delta0) / Lf * dt;

        fg[1 + xStartingIndex + t] = x1 - x1_p;
        fg[1 + yStartingIndex + t] = y1 - y1_p;
        fg[1 + psiStartingIndex + t] = psi1 - psi1_p;
        fg[1 + vStartingIndex + t] = v1 - v1_p;
        fg[1 + cteStartingIndex + t] = cte1 - cte1_p;
        fg[1 + psiErrorStartingIndex + t] = epsi1 - epsi1_p;
    }
}

AD<double> CostAndConstraintsCalculator::CalculateCost(const ADvector& vars)
{
    AD<double> cost = PenaliseForTrajectoryError(vars);
    cost += PenaliseForLargeActuatorValues(vars);
    cost += PenaliseForJerks(vars);
    return cost;
}

AD<double> CostAndConstraintsCalculator::PenaliseForJerks(const ADvector &vars) const
{
    AD<double> cost = 0.0;
    for (int t = 0; t < N - 2; t++)
    {
        cost += pow(vars[steeringAngleStartingIndex + t + 1] - vars[steeringAngleStartingIndex + t], 2) * SteeringAngleDiffWeight;
        cost += pow(vars[throttleStartingIndex + t + 1] - vars[throttleStartingIndex + t], 2)  * ThrottleDiffValueWeight;
    }
    return cost;
}

AD<double> CostAndConstraintsCalculator::PenaliseForLargeActuatorValues(const ADvector &vars) const
{
    AD<double> cost = 0.0;
    for (int t = 0; t < N - 1; t++)
    {
        cost += pow(vars[steeringAngleStartingIndex + t], 2) * SteeringAngleAbsValueWeight;
        cost += pow(vars[throttleStartingIndex + t], 2) * ThrottleAbsValueWeight;
    }
    return cost;
}

AD<double> CostAndConstraintsCalculator::PenaliseForTrajectoryError(const ADvector &vars) const
{
    AD<double> cost = 0.0;
    for (int t = 0; t < N; t++)
    {
        cost += pow(vars[cteStartingIndex + t], 2) * CTEAbsValueWeight;
        cost += pow(vars[psiErrorStartingIndex + t], 2) * PsiErrorAbsValueWeight;
        cost += pow(vars[vStartingIndex + t] - MaxVelocity, 2);
    }
    return cost;
}

void CostAndConstraintsCalculator::SetFirstStepState(ADvector &fg, const ADvector &x) const
{
    fg[1 + xStartingIndex] = x[xStartingIndex];
    fg[1 + yStartingIndex] = x[yStartingIndex];
    fg[1 + psiStartingIndex] = x[psiStartingIndex];
    fg[1 + vStartingIndex] = x[vStartingIndex];
    fg[1 + cteStartingIndex] = x[cteStartingIndex];
    fg[1 + psiErrorStartingIndex] = x[psiErrorStartingIndex];
}