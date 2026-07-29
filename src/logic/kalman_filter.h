#ifndef KALMAN_FILTER_H
#define KALMAN_FILTER_H

// Minimal scalar Kalman filter over the gyro rate reading. There's no
// absolute yaw reference available (no usable magnetometer near the
// motors), so this can't correct long-term drift/bias - it only smooths
// noisy per-sample gyro readings, trusting the running estimate more than
// any single noisy measurement.
struct KalmanState {
    float rate;
    float p; // error covariance
};

inline KalmanState kalmanInit(float initialRate) {
    return {initialRate, 1.0f};
}

inline KalmanState kalmanUpdate(KalmanState state, float measurement, float processNoise, float measurementNoise) {
    float pPred = state.p + processNoise;
    float k = pPred / (pPred + measurementNoise);
    float rate = state.rate + k * (measurement - state.rate);
    float p = (1.0f - k) * pPred;
    return {rate, p};
}

#endif // KALMAN_FILTER_H
