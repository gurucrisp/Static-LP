            static uint32_t ramp_last_ms = 0U;
            static uint32_t softstart_begin_ms = 0U;
            static uint8_t was_armed = 0U;

            uint8_t armed = (gbIsSystemNormalState == 1U &&
                             Mreceivedflag_pulse_on == 1U &&
                             pwm_allowed == 1U);

            if (armed && !was_armed)
            {
                uint32_t now = HAL_GetTick();
                /*
                 * Initialize the ramp*d PID setpoint to the present inpu*
                 * voltage so the initial cor*ection demand is close to zero.
              *  */
                __disable_irq();
                voltage_setpoint_pid = vrms_in;
                __enable_irq();

                /*
                 * Reset P*D states for bumpless startup.
               * */
                pid.integral = 0.0f;
                pid.derivative = 0.0f;
                pid.lastOutput = 0.0f;
                pwm_output = 0.0f;

                /*
                 * Initialize the stored PID error according to the same
                 * buck/boost convention used in AutoPID_Compute().
                 */
                if (vrms_in < voltage_setpoint)
                {
                    pid.lastError = voltage_setpoint_pid - vrms_out;
                }
                else
                {
                    pid.lastError = vrms_out - voltage_setpoint_pid;
                }

                /*
                 * Initialize the ramp timestamps.
                 */
                ramp_last_ms = now;
                softstart_begin_ms = now;

                was_armed = 1U;
            }

            if (armed)
            {
                uint32_t softstart_elapsed_ms;
                float voltage_difference;
                float selected_slope;

                softstart_elapsed_ms =
                    (uint32_t)(HAL_GetTick() - softstart_begin_ms);

                voltage_difference =
                    voltage_setpoint_pid - voltage_setpoint;

                /*
                 * Obtain absolute voltage difference without using fabsf().
                 */
                if (voltage_difference < 0.0f)
                {
                    voltage_difference = -voltage_difference;
                }

                /*
                 * Stage 1:
                 * Use a slow 5 V/s slope for the first one second.
                 */
                if (softstart_elapsed_ms < SOFTSTART_SLOW_TIME_MS)
                {
                    selected_slope = SOFTSTART_SLOW_SLOPE_VPS;
                }
                /*
                 * Final stage:
                 * Slow down when the setpoint is within 3 V of the target.
                 */
                else if (voltage_difference <= SOFTSTART_FINAL_BAND_V)
                {
                    selected_slope = SOFTSTART_FINAL_SLOPE_VPS;
                }
                /*
                 * Stage 2:
                 * Use the faster slope after the first one second.
                 */
                else
                {
                    selected_slope = SOFTSTART_FAST_SLOPE_VPS;
                }

                ramp_setpoint_slope_ts(&voltage_setpoint_pid,
                                       voltage_setpoint,
                                       selected_slope,
                                       &ramp_last_ms);
            }
            else
            {
                /*
                 * Reset the soft-start state whenever PWM is not armed.
                 */
                was_armed = 0U;
                ramp_last_ms = 0U;
                softstart_begin_ms = 0U;

                __disable_irq();
                voltage_setpoint_pid = vrms_in;
                __enable_irq();
            }

