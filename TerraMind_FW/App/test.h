#ifndef APP_TEST_H
#define APP_TEST_H

#ifdef __cplusplus
extern "C"
{
#endif

void AppTest_M3508_Init(void);
void AppTest_M3508_TaskStep(void);
void AppTest_PwmMotor_Init(void);
void AppTest_PwmMotor_TaskStep(void);
void AppTest_ServoA_Init(void);
void AppTest_ServoA_TaskStep(void);
void AppTest_EscA_Init(void);
void AppTest_EscA_TaskStep(void);
void AppTest_EscB_Init(void);
void AppTest_EscB_TaskStep(void);

#ifdef __cplusplus
}
#endif

#endif
