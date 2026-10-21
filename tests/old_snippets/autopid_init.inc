void AutoPID_Init(AutoPID *pid, volatile float *input, volatile float *setpoint, volatile float *output,
                  float Kp, float Ki, float Kd, float outputMin, float outputMax,
                  uint32_t timeStepMs)
{
    pid->input = input;
    pid->setpoint = setpoint;
    pid->output = output;

    // Set gains (Ki here is in units of output-per-volt-per-second)
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;

    // actuator limits
    pid->outputMin = outputMin;
    pid->outputMax = outputMax;

    // integrator limits and state
    pid->integral = 0.0f;
    pid->integralMin = outputMin; // prevent windup (tune if needed)
    pid->integralMax = outputMax;

    // derivative & filtering state
    pid->derivative = 0.0f;
    pid->lastError = (*(setpoint) - *(input));
    pid->derivFilterAlpha = 0.90f; // 0.85-0.95 recommended

    // RMS (measurement) smoothing state
    pid->measFiltered = *input;
    pid->rmsFilterAlpha = 0.95f; // 0.9-0.98 recommended

    // timing & enabling
    pid->lastTime = HAL_GetTick();
    pid->timeStepMs = timeStepMs;
    pid->enabled = 1;

    // initialize lastOutput to current output value
    pid->lastOutput = *output;
}
