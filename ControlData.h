//
// Created by Stanislav Olekhnovich on 10/08/2017.
//

#ifndef MPC_CONTROLDATA_H
#define MPC_CONTROLDATA_H

#include <vector>

struct ControlData
{
    ControlData(const std::vector<double> &referencePathX, const std::vector<double> &referencePathY, double Psi,
                double X, double Y, double V, double Throttle, double SteeringAngle) : ReferencePathX(referencePathX),
                                                ReferencePathY(referencePathY),
                                                Psi(Psi), X(X), Y(Y), V(V), Throttle(Throttle), SteeringAngle(SteeringAngle) {}

    std::vector<double> ReferencePathX;
    std::vector<double> ReferencePathY;

    double Psi;
    double X;
    double Y;
    double V;
    double Throttle;
    double SteeringAngle;
};

#endif //MPC_CONTROLDATA_H
