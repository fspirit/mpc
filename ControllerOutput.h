//
// Created by Stanislav Olekhnovich on 10/08/2017.
//

#ifndef MPC_CONTROLLEROUTPUT_H
#define MPC_CONTROLLEROUTPUT_H

#include <vector>

struct ControllerOutput
{
    ControllerOutput(double Throttle, double SteeringAngle, const std::vector<double> &PredictedPathX,
                     const std::vector<double> &PredictedPathY, const std::vector<double> &ReferencePathX,
                     const std::vector<double> &ReferencePathY) : Throttle(Throttle), SteeringAngle(SteeringAngle),
                                                                  PredictedPathX(PredictedPathX),
                                                                  PredictedPathY(PredictedPathY),
                                                                  ReferencePathX(ReferencePathX),
                                                                  ReferencePathY(ReferencePathY) {}

    double Throttle;
    double SteeringAngle;
    std::vector<double> PredictedPathX;
    std::vector<double> PredictedPathY;
    std::vector<double> ReferencePathX;
    std::vector<double> ReferencePathY;
};

#endif //MPC_CONTROLLEROUTPUT_H
