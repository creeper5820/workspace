//
// Created by soulde on 2023/3/29.
//

#include "../include/PID.h"

double PID::calc(double err) {

    double ret = pFactor * (err - lastErr) + iFactor * err + dFactor * (err - 2 * lastErr + twoLastErr);

    twoLastErr = lastErr;
    lastErr = err;
    I += ret;
    if (I > max) {
        I = max;
    } else if (I < min) {
        I = min;
    }
    return I;
}

PID::PID(double Kp, double Ki, double Kd, double max, double min) : pFactor(Kp), iFactor(Ki), dFactor(Kd),
                                                                    min(min), max(max), I(0), lastErr(0),twoLastErr(0) {

};

