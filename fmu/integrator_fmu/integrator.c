// Minimal stateful test model: dy/dt = gain * drive (held input).
// Deliberately rejects lifecycle/time errors that a stateless sensor would hide.
#include "fmi2Functions.h"
#include <stdlib.h>
#include <math.h>
#include <string.h>
typedef struct { double u, gain, y, time; int phase; } Model;
FMI2_Export const char* fmi2GetVersion(void) { return "2.0"; }
FMI2_Export const char* fmi2GetTypesPlatform(void) { return "default"; }
FMI2_Export fmi2Component fmi2Instantiate(fmi2String name, fmi2Type type, fmi2String guid,
    fmi2String resources, const fmi2CallbackFunctions* cb, fmi2Boolean visible, fmi2Boolean log) {
    (void)name; (void)resources; (void)cb; (void)visible; (void)log;
    if (type != fmi2CoSimulation || strcmp(guid,"simrocket-integrator-1")) return NULL;
    Model* m=calloc(1,sizeof(Model)); if(m) m->gain=1; return m;
}
FMI2_Export void fmi2FreeInstance(fmi2Component c) { free(c); }
FMI2_Export fmi2Status fmi2SetupExperiment(fmi2Component c,fmi2Boolean td,fmi2Real tol,fmi2Real start,fmi2Boolean sd,fmi2Real stop) {
    (void)td;(void)tol;(void)sd;(void)stop; Model* m=c; if(m->phase) return fmi2Error; m->time=start; return fmi2OK;
}
FMI2_Export fmi2Status fmi2EnterInitializationMode(fmi2Component c) { Model* m=c; if(m->phase) return fmi2Error; m->phase=1; return fmi2OK; }
FMI2_Export fmi2Status fmi2ExitInitializationMode(fmi2Component c) { Model* m=c; if(m->phase!=1) return fmi2Error; m->phase=2; return fmi2OK; }
FMI2_Export fmi2Status fmi2Terminate(fmi2Component c) { ((Model*)c)->phase=3; return fmi2OK; }
FMI2_Export fmi2Status fmi2Reset(fmi2Component c) { Model* m=c; memset(m,0,sizeof(*m));m->gain=1;return fmi2OK; }
FMI2_Export fmi2Status fmi2SetReal(fmi2Component c,const fmi2ValueReference vr[],size_t n,const fmi2Real v[]) {
    Model* m=c; for(size_t i=0;i<n;++i) {
        if(vr[i]==0) m->u=v[i];
        else if(vr[i]==1 && m->phase<2) m->gain=v[i]; else return fmi2Error;
    } return fmi2OK;
}
FMI2_Export fmi2Status fmi2GetReal(fmi2Component c,const fmi2ValueReference vr[],size_t n,fmi2Real v[]) {
    Model* m=c; for(size_t i=0;i<n;++i) {
        if(vr[i]==0) v[i]=m->u; else if(vr[i]==1) v[i]=m->gain; else if(vr[i]==2) v[i]=m->y; else return fmi2Error;
    } return fmi2OK;
}
FMI2_Export fmi2Status fmi2DoStep(fmi2Component c,fmi2Real time,fmi2Real dt,fmi2Boolean rollback) {
    (void)rollback; Model* m=c;
    if(m->phase!=2 || fabs(m->time-time)>1e-9 || dt<=0) return fmi2Error;
    if(m->gain<0) return fmi2Discard; // deterministic error-path fixture
    m->y += m->gain*m->u*dt; m->time+=dt; return fmi2OK;
}
