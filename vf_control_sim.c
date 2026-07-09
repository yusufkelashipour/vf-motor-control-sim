/*
 * V/f (Volts-per-Hertz) Open-Loop Motor Control Simulation
 * ----------------------------------------------------------
 * Simulates the core control strategy used by a variable frequency drive
 * (VFD) to control the speed of an AC induction motor: as the commanded
 * frequency increases, output voltage is scaled proportionally to keep
 * the volts-per-hertz ratio (and therefore magnetic flux) roughly
 * constant across the operating range.
 *
 * This models the CONTROL STRATEGY itself (V/f law + acceleration ramp +
 * low-frequency voltage boost) -- the same logic a VFD's firmware
 * computes every control cycle -- rather than a full electromagnetic
 * motor model.
 */

#include <stdio.h>

/* ---- Motor / drive parameters ---- */
#define RATED_FREQ_HZ      60.0f
#define RATED_VOLTAGE_V     460.0f
#define V_PER_HZ           (RATED_VOLTAGE_V / RATED_FREQ_HZ)  /* ~7.67 V/Hz */

#define VOLTAGE_BOOST_V     25.0f   /* extra voltage at low frequency to
                                        overcome stator resistance so the
                                        motor can still produce torque
                                        near standstill */
#define BOOST_TAPER_HZ      10.0f   /* boost fades to zero by this freq */

#define TARGET_FREQ_HZ      60.0f   /* commanded target speed */
#define RAMP_TIME_S          5.0f   /* time to accelerate 0 -> target */

#define MOTOR_TIME_CONST_S   0.6f   /* how quickly actual motor speed
                                        "catches up" to commanded
                                        frequency -- models mechanical
                                        inertia / slip lag */

#define SIM_TIME_S           8.0f
#define DT                   0.01f  /* simulation time step (s) */

/* ---- Function prototypes ---- */
float commanded_frequency(float t);
float commanded_voltage(float freq_hz);

int main(void) {
    float t = 0.0f;
    float motor_speed_hz = 0.0f;   /* actual motor speed, starts at rest */

    printf("Time(s) | Freq_cmd(Hz) | Voltage_cmd(V) | V/f ratio | Motor speed(Hz)\n");
    printf("------------------------------------------------------------------------\n");

    /* Print a data row roughly once per second of simulated time */
    float next_print_time = 0.0f;
    const float print_interval = 1.0f;

    while (t <= SIM_TIME_S) {
        float freq_cmd = commanded_frequency(t);
        float voltage_cmd = commanded_voltage(freq_cmd);

        /* First-order lag: motor speed gradually tracks the commanded
           frequency rather than jumping instantly, modeling mechanical
           inertia and motor slip. */
        motor_speed_hz += (freq_cmd - motor_speed_hz) * (DT / MOTOR_TIME_CONST_S);

        if (t >= next_print_time) {
            /* V/f ratio is only meaningful once frequency is past the
               boost region -- below that, boost voltage dominates a
               near-zero frequency and the ratio isn't representative. */
            if (freq_cmd > 2.0f) {
                float vf_ratio = voltage_cmd / freq_cmd;
                printf("%7.2f | %12.2f | %14.2f | %9.2f | %15.2f\n",
                       t, freq_cmd, voltage_cmd, vf_ratio, motor_speed_hz);
            } else {
                printf("%7.2f | %12.2f | %14.2f | %9s | %15.2f\n",
                       t, freq_cmd, voltage_cmd, "--", motor_speed_hz);
            }
            next_print_time += print_interval;
        }

        t += DT;
    }

    return 0;
}

/*
 * Linear ramp from 0 Hz to TARGET_FREQ_HZ over RAMP_TIME_S seconds, then
 * holds at TARGET_FREQ_HZ. This is the frequency reference a VFD's
 * firmware sends to the inverter stage at each control cycle.
 */
float commanded_frequency(float t) {
    if (t >= RAMP_TIME_S) {
        return TARGET_FREQ_HZ;
    }
    return (TARGET_FREQ_HZ / RAMP_TIME_S) * t;
}

/*
 * Applies the V/f law: voltage scales linearly with frequency to keep
 * magnetic flux roughly constant. Below BOOST_TAPER_HZ, extra voltage
 * (boost) is added, tapering to zero, to compensate for the voltage
 * drop across the stator winding's resistance at low speed.
 */
float commanded_voltage(float freq_hz) {
    float base_voltage = V_PER_HZ * freq_hz;
    float boost = 0.0f;

    if (freq_hz < BOOST_TAPER_HZ) {
        float boost_fraction = 1.0f - (freq_hz / BOOST_TAPER_HZ);
        boost = VOLTAGE_BOOST_V * boost_fraction;
    }

    float voltage = base_voltage + boost;

    if (voltage > RATED_VOLTAGE_V) {
        voltage = RATED_VOLTAGE_V;   /* never exceed rated voltage */
    }

    return voltage;
}
