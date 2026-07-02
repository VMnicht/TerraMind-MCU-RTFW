#ifndef APP_MAIN_H
#define APP_MAIN_H

#ifdef __cplusplus
extern "C"
{
#endif

void App_Init(void);
void App_TaskStep(void);

void App_TestInit(void);
void App_TestStep(void);

#ifdef __cplusplus
}
#endif

#endif
