//
// Created by Stanislav Olekhnovich on 10/08/2017.
//

#ifndef MPC_MPCCONTROLLER_H
#define MPC_MPCCONTROLLER_H


#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve_result.hpp>
#include "ControlData.h"
#include "ControllerOutput.h"
#include "Polynomial.h"

using CppAD::AD;

class MPCController
{
public:
    typedef CPPAD_TESTVECTOR(double) Dvector;
    typedef CppAD::ipopt::solve_result<Dvector> IPOPTSolution;

    ControllerOutput Process(ControlData data);
private:
    IPOPTSolution SearchForOptimalSteerAndThrottle(const Polynomial &polyfit, const Eigen::VectorXd &state);
    ControllerOutput ComposeOutputFromSolution(IPOPTSolution solution, const Polynomial &polyfit);
    void ConvertReferencePathPointsToVehicleCoords(ControlData data, Eigen::VectorXd& x, Eigen::VectorXd& y);
    Eigen::VectorXd GetCurrentStateWithDelay(const ControlData& data, const Polynomial& referencePath);
    Dvector CreateOptVariablesVector(Eigen::VectorXd startingState);
    std::tuple<std::vector<double>, std::vector<double>> GenerateReferancePathPoints(const Polynomial& polyfit);
    std::tuple<Dvector, Dvector> GetConstraintsBounds(Eigen::VectorXd startingState);
    std::string GetSolverOptions() const;
    std::tuple<std::vector<double>, std::vector<double>> GetPredictedPathPoints(const IPOPTSolution& solution);
    std::tuple<Dvector, Dvector> GetVariablesBounds();
};


#endif //MPC_MPCCONTROLLER_H
