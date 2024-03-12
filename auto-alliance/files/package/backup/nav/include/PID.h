//
// Created by soulde on 2023/3/29.
//

#ifndef PATH_FOLLOWER_PID_H
#define PATH_FOLLOWER_PID_H


class PID {
public:
    PID(double Kp, double Ki, double Kd, double max, double min);

    double calc(double err);

private:
    double max, min;
    double I, lastErr, twoLastErr;
    double pFactor;
    double iFactor;
    double dFactor;
};


#endif //PATH_FOLLOWER_PID_H
