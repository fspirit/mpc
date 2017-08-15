## The Model

At each timestep state of the model described by
1. x coordinate
2. y coordinate
3. phi - angle between x-axis and vehicle heading direction
4. v - speed of the vehicle
5. cte - distance from reference trajectory f(x) and y
6. ephi - difference between phi and reference trajectory orientation (which is calculated as a the tangential angle
of the ref trajectory f evaluated at x = arctan(f'(x)) )
                                                                                                ​                                                                                                ​​
and 2 actuator values
7. delta, Steering angle
8. a, Throttle ( from -1 to 1). Negative value means we hit on break.

The transition between states T=0 and T+1=1 (time elapsed between states is dt) is described by a set of equations:
x1 = x0 + v0 * cos(phi0) * dt;
y1 = y0 + v0 * sin(phi0) * dt;
phi1 = phi0 + v0 / Lf * delta0 * dt;
v1 = v0 + a * dt;
cte1 = v0 + sin(ephi0) * dt;
ephi1 = ephi0 + v0 / Lf * delta0 * dt

## Timestep Length and Elapsed Duration (N & dt)

I took 10 adn .1 and didnt change that.

## Polynomial Fitting and MPC Preprocessing

Polynomial fitting and optimization is done in vehicle-based coords. So,  at first, all of the waypoints given by the sim
in the global map system were translated to vehicle system, that is, x, y and phi were made all zero (as we assume at time 0 we are
at the center of the vehicle system and heading vector coinsides with vehicle x-axis).

## Model Predictive Control with Latency

To incorporate latency into calculations I moved state at time 0, according to the following rules:

```
    // Current CTE relative to vehicle is reference path value at x = 0.0
    double cte = referencePath.GetValue(0.0);
    // Current error psi is tangential angle
    // of the ref trajectory f evaluated at x = 0.0
    double epsi = -atan(referencePath.GetFirstDerivativeValue(0.0));

    double x_next = 0.0 + v * dt;
    double y_next = 0.0;
    double psi_next = 0.0 + v * (-delta) / Lf * dt;
    double v_next = v+ a * dt;
    double cte_next = cte + v * sin(epsi) * dt;
    double epsi_next = epsi + v * (-delta) / Lf * dt;
```

Calculations of all the following states from 1 to N are based on this first state and therefore are also in vehicle's system.

To get optimal steering & throttle values for next N steps I did the following:
1. Set first state as described ealier
2. Set optimization constraints: state_expected (calculated starting from state 0, using formulas from above) - state_given_by_optimizer = 0
3. Set variable constraints so that steering_angle and throttle stay in defined ranges
4. Set cost computation to penalize for CTE & ephi (errors), slow speed, large values of throttle and steering.
5. Set weights for different components of cost to make sticking to ref path (keeping low errors) more preferable for optimization algorithm


