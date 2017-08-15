//
// Created by Stanislav Olekhnovich on 10/08/2017.
//

#include "MPCController.h"

#include <cppad/ipopt/solve.hpp>
#include <thread>

#include "Setup.h"
#include "CostAndConstraintsCalculator.h"

ControllerOutput MPCController::Process(ControlData data)
{
    Eigen::VectorXd vehiclePointsX(data.ReferencePathX.size());
    Eigen::VectorXd vehiclePointsY(data.ReferencePathY.size());

    ConvertReferencePathPointsToVehicleCoords(data, vehiclePointsX, vehiclePointsY);

    Polynomial polyfit(vehiclePointsX, vehiclePointsY);

    Eigen::VectorXd state = GetCurrentStateWithDelay(data, polyfit);

    auto solution = SearchForOptimalSteerAndThrottle(polyfit, state);

    // comment out the lines below to debug!

//    bool ok = true;
//    auto cost = solution.obj_value;
//
//    ok &= solution.status == CppAD::ipopt::solve_result<Dvector>::success;
//
//    if (ok)
//        std::cout << "OK! Cost:" << cost << std::endl;
//    else
//    {
//        std::cout << "SOMETHING IS WRONG! " << cost << std::endl;
//        exit(-1);
//    }

    std::this_thread::sleep_for(std::chrono::milliseconds(LatencyInMillis));

    return ComposeOutputFromSolution(solution, polyfit);
}

void MPCController::ConvertReferencePathPointsToVehicleCoords(ControlData data, Eigen::VectorXd& x, Eigen::VectorXd& y)
{
    for(int i = 0; i < data.ReferencePathX.size(); ++i)
    {
        // Vectors relative to vehicle location in global coords
        double dx = data.ReferencePathX[i] - data.X;
        double dy = data.ReferencePathY[i] - data.Y;

        // Rotate to get coords in vehicle related coords system
        x[i] = dx * cos(-data.Psi) - dy * sin(-data.Psi);
        y[i] = dy * cos(-data.Psi) + dx * sin(-data.Psi);
    }
}

Eigen::VectorXd MPCController::GetCurrentStateWithDelay(const ControlData& data, const Polynomial& referencePath)
{
    // Current CTE relative to vehicle is reference path polinomial at x = 0.0
    double cte = referencePath.GetValue(0.0);
    // Current error psi is tangential angle
    // of the ref trajectory f evaluated at x = 0.0
    double epsi = -atan(referencePath.GetFirstDerivativeValue(0.0));

    double x_next = 0.0 + data.V * dt;
    double y_next = 0.0;
    double psi_next = 0.0 + data.V  * (-data.SteeringAngle) / Lf * dt;
    double v_next = data.V + data.Throttle * dt;
    double cte_next = cte + data.V * sin(epsi) * dt;
    double epsi_next = epsi + data.V * (-data.SteeringAngle) / Lf * dt;

    Eigen::VectorXd state(StateSize);
    state << x_next, y_next, psi_next, v_next, cte_next, epsi_next;

    return state;
}

MPCController::IPOPTSolution MPCController::SearchForOptimalSteerAndThrottle(const Polynomial &polyfit,
                                                              const Eigen::VectorXd &state)
{
    auto x = CreateOptVariablesVector(state);
    auto xBounds = GetVariablesBounds();

    auto gBounds = GetConstraintsBounds(state);

    CostAndConstraintsCalculator cc(polyfit);
    auto options = GetSolverOptions();

    IPOPTSolution solution;

    CppAD::ipopt::solve<Dvector, CostAndConstraintsCalculator>(
            options,
            x,
            std::get<0>(xBounds),
            std::get<1>(xBounds),
            std::get<0>(gBounds),
            std::get<1>(gBounds),
            cc,
            solution);


    return solution;
}

MPCController::Dvector MPCController::CreateOptVariablesVector(Eigen::VectorXd startingState)
{
    MPCController::Dvector x(VariablesVectorSize);

    for (int i = 0; i < VariablesVectorSize; i++)
        x[i] = 0.0;

    x[xStartingIndex] = startingState[0];
    x[yStartingIndex] = startingState[1];
    x[psiStartingIndex] = startingState[2];
    x[vStartingIndex] = startingState[3];
    x[cteStartingIndex] = startingState[4];
    x[psiErrorStartingIndex] = startingState[5];

    return x;
}

std::tuple<MPCController::Dvector, MPCController::Dvector> MPCController::GetVariablesBounds()
{
    Dvector xL(VariablesVectorSize);
    Dvector xU(VariablesVectorSize);

    for (int i = 0; i < steeringAngleStartingIndex; ++i)
    {
        xL[i] = -1.0e10;
        xU[i] = 1.0e10;
    }

    for (int i = steeringAngleStartingIndex; i < throttleStartingIndex; ++i)
    {
        xL[i] = -0.85;
        xU[i] = 0.85;
    }

    for (int i = throttleStartingIndex; i < VariablesVectorSize; i++)
    {
        xL[i] = -1.0;
        xU[i] = 1.0;
    }

    return std::make_tuple(xL, xU);
}

std::tuple<MPCController::Dvector, MPCController::Dvector> MPCController::GetConstraintsBounds(Eigen::VectorXd startingState)
{
    Dvector gL(ConstraintsVectorSize);
    Dvector gU(ConstraintsVectorSize);

    for (int i = 0; i < ConstraintsVectorSize; ++i)
    {
        gL[i] = 0.0;
        gU[i] = 0.0;
    }

    gL[xStartingIndex] = startingState[0];
    gL[yStartingIndex] = startingState[1];
    gL[psiStartingIndex] = startingState[2];
    gL[vStartingIndex] = startingState[3];
    gL[cteStartingIndex] = startingState[4];
    gL[psiErrorStartingIndex] = startingState[5];

    gU[xStartingIndex] = startingState[0];
    gU[yStartingIndex] = startingState[1];
    gU[psiStartingIndex] = startingState[2];
    gU[vStartingIndex] = startingState[3];
    gU[cteStartingIndex] = startingState[4];
    gU[psiErrorStartingIndex] = startingState[5];

    return std::make_tuple(gL, gU);
}

std::tuple<std::vector<double>, std::vector<double>> MPCController::GenerateReferancePathPoints(const Polynomial& polyfit)
{
    const double step = 5.0;

    std::vector<double> x(N);
    std::vector<double> y(N);

    for (int i = 0; i < N; ++i)
    {
        x[i] = step * i;
        y[i] = polyfit.GetValue(x[i]);
    }

    return std::make_tuple(x, y);
}

ControllerOutput MPCController::ComposeOutputFromSolution(MPCController::IPOPTSolution solution, const Polynomial& polyfit)
{
    double steeringAngle = solution.x[steeringAngleStartingIndex];
    double throttle = solution.x[throttleStartingIndex];

//    std::cout << steeringAngle << std::endl;
//    std::cout << throttle << std::endl;

    auto predictedPathPoints = GetPredictedPathPoints(solution);

    auto refPathPoints = GenerateReferancePathPoints(polyfit);

    return {throttle,
            steeringAngle,
            std::get<0>(predictedPathPoints),
            std::get<1>(predictedPathPoints),
            std::get<0>(refPathPoints),
            std::get<1>(refPathPoints)};

}

std::string MPCController::GetSolverOptions() const
{
    // options for IPOPT solver
    std::string options;
    // Uncomment this if you'd like more print information
    options += "Integer print_level  0\n";
    // NOTE: Setting sparse to true allows the solver to take advantage
    // of sparse routines, this makes the computation MUCH FASTER.
    options += "Sparse  true        forward\n";
    options += "Sparse  true        reverse\n";
    // NOTE: Currently the solver has a maximum time limit of 0.5 seconds.
    options += "Numeric max_cpu_time          0.5\n";

    return options;
}

std::tuple<std::vector<double>, std::vector<double>> MPCController::GetPredictedPathPoints(const MPCController::IPOPTSolution& solution)
{
    std::vector<double> x(N);
    std::vector<double> y(N);

    for (int i = 0; i < N; ++i)
    {
        x.emplace_back(solution.x[xStartingIndex + i]);
        y.emplace_back(solution.x[yStartingIndex + i]);
    }

    return std::make_tuple(x, y);
};


