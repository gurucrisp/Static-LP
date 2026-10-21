typedef struct {

    volatile float *input;
    volatile float *setpoint;
    volatile float *output;
    float Kp;
    float Ki; // units: output/(volt * second)
    float Kd;
    float outputMin;
    float outputMax;
    float integral;
    float integralMin;
    float integralMax;
    float derivative;
    float lastError;
    float derivFilterAlpha;
    float measFiltered;
    float rmsFilterAlpha;
    uint32_t lastTime;
    uint32_t timeStepMs;
    uint8_t enabled;
    float lastOutput;  // <-- Added to hold last PID output for freeze

} AutoPID;
