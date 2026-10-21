/* The example frames of the Examples sheet of slave_link_protocol_v2_draft.xlsx (the approved draft), copied by script. Do not edit by hand: change the workbook, then regenerate. */
static const struct { const char *name; const char *frame; } EXAMPLES[] =
{
    { "S status (existing, frozen)", "$S,R,12,2301,2298,5002,0,19,0,1871*2F" },
    { "E extended status", "$E,7,2300,3,3,2020,2700*76" },
    { "H hello", "$H,2,803,800,3100,0,1,3200*57" },
    { "T autotune progress", "$T,4,320,2301,48,0*41" },
    { "T autotune finished", "$T,10,1855,2301,230,1*70" },
    { "R response, calibration done", "$R,5,0,10152*4C" },
    { "R response, calibration refused", "$R,6,3,0*4B" },
    { "M control", "$M,41,1,2300,3,300*54" },
    { "K command: hello", "$K,2,HELLO*3B" },
    { "K command: input calibration", "$K,5,CALV,2302*49" },
    { "K command: start autotune", "$K,7,TUNE*76" },
};
#define N_EXAMPLES ((int)(sizeof(EXAMPLES) / sizeof(EXAMPLES[0])))
