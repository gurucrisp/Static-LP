/* Compile-time test of the autotune switches' DEFAULTS (slave_features.h): the expected values arrive as -DX_AT, -DX_TM, -DX_AP, -DX_EN, -DX_IO. See check_defaults.sh. */
#include "slave_features.h"
#if (SLAVE_AUTOTUNE != X_AT) || (SLAVE_TUNE_MODE != X_TM) || (SLAVE_TUNE_APPLY != X_AP) || (SLAVE_TUNE_ENFORCE != X_EN) || (SLAVE_IOUT != X_IO)
#error "the defaults are not what the test expects"
#endif
int main(void) { return 0; }
