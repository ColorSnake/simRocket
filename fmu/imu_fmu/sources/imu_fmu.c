/**
 * FMI 2.0 Co-Simulation implementation of the simRocket IMU Sensor.
 *
 * Inputs (ValueReferences 0-5):  true angular_velocity XYZ, true accel XYZ
 * Parameters (VR 6-7):           gyro_noise_std, accel_noise_std
 * Outputs (VR 8-13):             measured (noisy) gyro XYZ, measured accel XYZ
 *
 * Noise model: additive Gaussian white noise drawn via Box-Muller transform
 * (no rand_r, srand - seeded from fmi2SetupExperiment startTime for reproducibility).
 */

#define FMI2_EXPORT __attribute__((visibility("default")))

#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <stdio.h>

#include "fmi2Functions.h"

/* ── Value references matching modelDescription.xml ── */
#define VR_IN_GYRO_X      0
#define VR_IN_GYRO_Y      1
#define VR_IN_GYRO_Z      2
#define VR_IN_ACCEL_X     3
#define VR_IN_ACCEL_Y     4
#define VR_IN_ACCEL_Z     5
#define VR_PARAM_GYRO_N   6
#define VR_PARAM_ACCEL_N  7
#define VR_OUT_GYRO_X     8
#define VR_OUT_GYRO_Y     9
#define VR_OUT_GYRO_Z     10
#define VR_OUT_ACCEL_X    11
#define VR_OUT_ACCEL_Y    12
#define VR_OUT_ACCEL_Z    13

/* ── Internal state of the FMU instance ── */
typedef struct {
    /* Inputs */
    double in_gyro[3];
    double in_accel[3];
    /* Parameters */
    double gyro_noise_std;
    double accel_noise_std;
    /* Outputs */
    double out_gyro[3];
    double out_accel[3];
    /* PRNG state (xorshift64 – lightweight, no stdlib dependency) */
    unsigned long long rng_state;
} ImuFmuInstance;

/* ── Lightweight noise generator (Box-Muller + xorshift64) ── */
static unsigned long long xorshift64(unsigned long long* state) {
    unsigned long long x = *state;
    x ^= x << 13;
    x ^= x >> 7;
    x ^= x << 17;
    *state = x;
    return x;
}

static double uniform01(unsigned long long* state) {
    return (double)(xorshift64(state) >> 11) * (1.0 / (double)(1ULL << 53));
}

static double gaussian(unsigned long long* state, double std) {
    double u1 = uniform01(state);
    double u2 = uniform01(state);
    if (u1 < 1e-20) u1 = 1e-20;
    double z = sqrt(-2.0 * log(u1)) * cos(2.0 * 3.14159265358979323846 * u2);
    return z * std;
}

/* ── FMI 2.0 Mandatory functions ── */

FMI2_EXPORT const char* fmi2GetTypesPlatform(void) {
    return "default";
}

FMI2_EXPORT const char* fmi2GetVersion(void) {
    return "2.0";
}

FMI2_EXPORT fmi2Component fmi2Instantiate(
    fmi2String instanceName,
    fmi2Type   fmuType,
    fmi2String fmuGUID,
    fmi2String fmuResourceLocation,
    const fmi2CallbackFunctions* functions,
    fmi2Boolean visible,
    fmi2Boolean loggingOn)
{
    (void)instanceName; (void)fmuType; (void)fmuGUID;
    (void)fmuResourceLocation; (void)functions; (void)visible; (void)loggingOn;

    ImuFmuInstance* inst = (ImuFmuInstance*)calloc(1, sizeof(ImuFmuInstance));
    if (!inst) return NULL;

    inst->gyro_noise_std  = 0.005;
    inst->accel_noise_std = 0.05;
    inst->rng_state       = 0xDEADBEEFCAFEBABEULL; /* default seed */

    return (fmi2Component)inst;
}

FMI2_EXPORT void fmi2FreeInstance(fmi2Component c) {
    free(c);
}

FMI2_EXPORT fmi2Status fmi2SetupExperiment(
    fmi2Component c,
    fmi2Boolean   toleranceDefined,
    fmi2Real      tolerance,
    fmi2Real      startTime,
    fmi2Boolean   stopTimeDefined,
    fmi2Real      stopTime)
{
    (void)toleranceDefined; (void)tolerance; (void)stopTimeDefined; (void)stopTime;
    ImuFmuInstance* inst = (ImuFmuInstance*)c;
    /* Seed PRNG from startTime for reproducibility across runs */
    unsigned long long seed = 0;
    memcpy(&seed, &startTime, sizeof(seed));
    inst->rng_state = (seed == 0) ? 0xDEADBEEFCAFEBABEULL : seed;
    return fmi2OK;
}

FMI2_EXPORT fmi2Status fmi2EnterInitializationMode(fmi2Component c) {
    (void)c;
    return fmi2OK;
}

FMI2_EXPORT fmi2Status fmi2ExitInitializationMode(fmi2Component c) {
    ImuFmuInstance* inst = (ImuFmuInstance*)c;
    memcpy(inst->out_gyro, inst->in_gyro, sizeof(inst->out_gyro));
    memcpy(inst->out_accel, inst->in_accel, sizeof(inst->out_accel));
    return fmi2OK;
}

FMI2_EXPORT fmi2Status fmi2Terminate(fmi2Component c) {
    (void)c;
    return fmi2OK;
}

FMI2_EXPORT fmi2Status fmi2Reset(fmi2Component c) {
    ImuFmuInstance* inst = (ImuFmuInstance*)c;
    memset(inst->in_gyro,  0, sizeof(inst->in_gyro));
    memset(inst->in_accel, 0, sizeof(inst->in_accel));
    memset(inst->out_gyro, 0, sizeof(inst->out_gyro));
    memset(inst->out_accel,0, sizeof(inst->out_accel));
    return fmi2OK;
}

FMI2_EXPORT fmi2Status fmi2GetReal(
    fmi2Component c,
    const fmi2ValueReference vr[],
    size_t nvr,
    fmi2Real value[])
{
    ImuFmuInstance* inst = (ImuFmuInstance*)c;
    for (size_t i = 0; i < nvr; ++i) {
        switch (vr[i]) {
            case VR_IN_GYRO_X:    value[i] = inst->in_gyro[0];       break;
            case VR_IN_GYRO_Y:    value[i] = inst->in_gyro[1];       break;
            case VR_IN_GYRO_Z:    value[i] = inst->in_gyro[2];       break;
            case VR_IN_ACCEL_X:   value[i] = inst->in_accel[0];      break;
            case VR_IN_ACCEL_Y:   value[i] = inst->in_accel[1];      break;
            case VR_IN_ACCEL_Z:   value[i] = inst->in_accel[2];      break;
            case VR_PARAM_GYRO_N: value[i] = inst->gyro_noise_std;   break;
            case VR_PARAM_ACCEL_N:value[i] = inst->accel_noise_std;  break;
            case VR_OUT_GYRO_X:   value[i] = inst->out_gyro[0];      break;
            case VR_OUT_GYRO_Y:   value[i] = inst->out_gyro[1];      break;
            case VR_OUT_GYRO_Z:   value[i] = inst->out_gyro[2];      break;
            case VR_OUT_ACCEL_X:  value[i] = inst->out_accel[0];     break;
            case VR_OUT_ACCEL_Y:  value[i] = inst->out_accel[1];     break;
            case VR_OUT_ACCEL_Z:  value[i] = inst->out_accel[2];     break;
            default:              value[i] = 0.0;                     break;
        }
    }
    return fmi2OK;
}

FMI2_EXPORT fmi2Status fmi2SetReal(
    fmi2Component c,
    const fmi2ValueReference vr[],
    size_t nvr,
    const fmi2Real value[])
{
    ImuFmuInstance* inst = (ImuFmuInstance*)c;
    for (size_t i = 0; i < nvr; ++i) {
        switch (vr[i]) {
            case VR_IN_GYRO_X:    inst->in_gyro[0]      = value[i]; break;
            case VR_IN_GYRO_Y:    inst->in_gyro[1]      = value[i]; break;
            case VR_IN_GYRO_Z:    inst->in_gyro[2]      = value[i]; break;
            case VR_IN_ACCEL_X:   inst->in_accel[0]     = value[i]; break;
            case VR_IN_ACCEL_Y:   inst->in_accel[1]     = value[i]; break;
            case VR_IN_ACCEL_Z:   inst->in_accel[2]     = value[i]; break;
            case VR_PARAM_GYRO_N: inst->gyro_noise_std  = value[i]; break;
            case VR_PARAM_ACCEL_N:inst->accel_noise_std = value[i]; break;
            default: break;
        }
    }
    return fmi2OK;
}

/* Minimal stubs for integer/boolean/string (not used by this model) */
FMI2_EXPORT fmi2Status fmi2GetInteger(fmi2Component c, const fmi2ValueReference vr[], size_t nvr, fmi2Integer v[]) { (void)c;(void)vr;(void)nvr;(void)v; return fmi2OK; }
FMI2_EXPORT fmi2Status fmi2SetInteger(fmi2Component c, const fmi2ValueReference vr[], size_t nvr, const fmi2Integer v[]) { (void)c;(void)vr;(void)nvr;(void)v; return fmi2OK; }
FMI2_EXPORT fmi2Status fmi2GetBoolean(fmi2Component c, const fmi2ValueReference vr[], size_t nvr, fmi2Boolean v[]) { (void)c;(void)vr;(void)nvr;(void)v; return fmi2OK; }
FMI2_EXPORT fmi2Status fmi2SetBoolean(fmi2Component c, const fmi2ValueReference vr[], size_t nvr, const fmi2Boolean v[]) { (void)c;(void)vr;(void)nvr;(void)v; return fmi2OK; }
FMI2_EXPORT fmi2Status fmi2GetString(fmi2Component c, const fmi2ValueReference vr[], size_t nvr, fmi2String v[]) { (void)c;(void)vr;(void)nvr;(void)v; return fmi2OK; }
FMI2_EXPORT fmi2Status fmi2SetString(fmi2Component c, const fmi2ValueReference vr[], size_t nvr, const fmi2String v[]) { (void)c;(void)vr;(void)nvr;(void)v; return fmi2OK; }
FMI2_EXPORT fmi2Status fmi2SetDebugLogging(fmi2Component c, fmi2Boolean loggingOn, size_t nCategories, const fmi2String categories[]) { (void)c;(void)loggingOn;(void)nCategories;(void)categories; return fmi2OK; }

/**
 * fmi2DoStep – The core computation step.
 * Reads inputs, adds Gaussian noise, writes to outputs.
 */
FMI2_EXPORT fmi2Status fmi2DoStep(
    fmi2Component c,
    fmi2Real      currentCommunicationPoint,
    fmi2Real      communicationStepSize,
    fmi2Boolean   noSetFMUStatePriorToCurrentPoint)
{
    (void)currentCommunicationPoint;
    (void)communicationStepSize;
    (void)noSetFMUStatePriorToCurrentPoint;

    ImuFmuInstance* inst = (ImuFmuInstance*)c;

    for (int i = 0; i < 3; ++i) {
        inst->out_gyro[i]  = inst->in_gyro[i]  + gaussian(&inst->rng_state, inst->gyro_noise_std);
        inst->out_accel[i] = inst->in_accel[i] + gaussian(&inst->rng_state, inst->accel_noise_std);
    }

    return fmi2OK;
}

/* FMU state serialization stubs (not required for this PoC) */
FMI2_EXPORT fmi2Status fmi2GetFMUstate(fmi2Component c, fmi2FMUstate* s) { (void)c;(void)s; return fmi2Error; }
FMI2_EXPORT fmi2Status fmi2SetFMUstate(fmi2Component c, fmi2FMUstate s)  { (void)c;(void)s; return fmi2Error; }
FMI2_EXPORT fmi2Status fmi2FreeFMUstate(fmi2Component c, fmi2FMUstate* s){ (void)c;(void)s; return fmi2Error; }
FMI2_EXPORT fmi2Status fmi2SerializedFMUstateSize(fmi2Component c, fmi2FMUstate s, size_t* sz) { (void)c;(void)s;(void)sz; return fmi2Error; }
FMI2_EXPORT fmi2Status fmi2SerializeFMUstate(fmi2Component c, fmi2FMUstate s, fmi2Byte b[], size_t n) { (void)c;(void)s;(void)b;(void)n; return fmi2Error; }
FMI2_EXPORT fmi2Status fmi2DeSerializeFMUstate(fmi2Component c, const fmi2Byte b[], size_t n, fmi2FMUstate* s) { (void)c;(void)b;(void)n;(void)s; return fmi2Error; }
FMI2_EXPORT fmi2Status fmi2GetDirectionalDerivative(fmi2Component c, const fmi2ValueReference vr[], size_t nvr, const fmi2ValueReference vk[], size_t nk, const fmi2Real dv[], fmi2Real dz[]) { (void)c;(void)vr;(void)nvr;(void)vk;(void)nk;(void)dv;(void)dz; return fmi2Error; }
