#ifndef APP_MAIN_H
#define APP_MAIN_H

// Test mode: 1 = simulated test injection, 0 = real UART5 command
#define APP_TEST_MODE 0

#ifdef __cplusplus
extern "C"
{
#endif

void App_ControlInit(void);
void App_ControlStep(void);
void App_TestInjectStep(void);

#ifdef __cplusplus
}
#endif

#endif
