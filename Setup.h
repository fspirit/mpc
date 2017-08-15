//
// Created by Stanislav Olekhnovich on 14/08/2017.
//

#ifndef MPC_SETUP_H
#define MPC_SETUP_H

const int N = 10;
const double dt = 0.1;
const int LatencyInMillis = dt * 1000;
const double Lf = 2.67;
const double MaxVelocity = 50.0;

const int StateSize = 6;
const int ActuatorsNumber = 2;
const int VariablesVectorSize = StateSize * N + ActuatorsNumber * (N - 1);
const int ConstraintsVectorSize = (StateSize) * N;

const int xStartingIndex = 0;
const int yStartingIndex = xStartingIndex + N;
const int psiStartingIndex = yStartingIndex + N;
const int vStartingIndex = psiStartingIndex + N;
const int cteStartingIndex = vStartingIndex + N;
const int psiErrorStartingIndex = cteStartingIndex + N;
const int steeringAngleStartingIndex = psiErrorStartingIndex + N;
const int throttleStartingIndex = steeringAngleStartingIndex + (N - 1);

#endif //MPC_SETUP_H
