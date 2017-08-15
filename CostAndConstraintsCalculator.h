//
// Created by Stanislav Olekhnovich on 14/08/2017.
//

#ifndef MPC_COSTANDCONSTRAINTSCALCULATOR_H
#define MPC_COSTANDCONSTRAINTSCALCULATOR_H

#include "Polynomial.h"

using CppAD::AD;

class CostAndConstraintsCalculator
{
public:
    typedef CPPAD_TESTVECTOR(AD<double>) ADvector;

    const Polynomial& polyfit;
    CostAndConstraintsCalculator(const Polynomial& polyfit): polyfit(polyfit) {}
    void operator()(ADvector& fg, const ADvector& x);

private:
    AD<double> CalculateCost(const ADvector& vars);
    AD<double> PenaliseForJerks(const ADvector &vars) const;
    AD<double> PenaliseForLargeActuatorValues(const ADvector &vars) const;
    AD<double> PenaliseForTrajectoryError(const ADvector &vars) const;
    void SetFirstStepState(ADvector &fg, const ADvector &x) const;
};

#endif //MPC_COSTANDCONSTRAINTSCALCULATOR_H
