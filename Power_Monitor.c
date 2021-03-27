#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/fpu.h"
#include "driverlib/interrupt.h"
#include "driverlib/timer.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/rom_map.h"
#include "driverlib/adc.h"
#include "inc/hw_adc.h"
#include "inc/hw_ints.h"
#include "grlib/grlib.h"
#include "grlib/widget.h"
#include "grlib/canvas.h"
#include "grlib/checkbox.h"
#include "grlib/container.h"
#include "grlib/pushbutton.h"
#include "grlib/radiobutton.h"
#include "grlib/slider.h"
#include "drivers/Kentec320x240x16_SSD2119_SPI.h"
#include "drivers/touch.h"
//#include "utils/ustdlib.h"

#include <math.h>
#include "Average.h"

#define ADC_SAMPLE_BUF_SIZE 64
#define SampleFreq 5000 // 5kHz
#define LCD_hold 1      // 1Hz
#define LCD_fps 5
#define Vref 3.3
#define PI 3.1415928353
int mVperAmp = 55; // ACS711KLCTR-25AB-T
int ACSoffset = 1650;
#define R1 560.0  //K ohm
#define R2 33.0 //k ohm
#define denominator (R2 / (R1 + R2))

#define TX_ON GPIO_PIN_3

extern tCanvasWidget g_sTOP;
extern tCanvasWidget g_sIndicator;
extern tCanvasWidget g_sResult_Slow;
extern tCanvasWidget g_sResult_Quick;
extern tPushButtonWidget g_sPushBtn;

uint32_t pui32ADC0Value[ADC_SAMPLE_BUF_SIZE];
float Voltage;
float SinVoltage;
float CosVoltage;
float Angle;
uint32_t ADC_CurrentBuffer1[ADC_SAMPLE_BUF_SIZE];
volatile uint32_t CountT0 = 0;
volatile uint32_t CountT1 = 0;
volatile uint32_t CountT5 = 0;
char Str1[10];
char Str2[10];
char Str3[10];

Average<float> ave_VDC(ADC_SAMPLE_BUF_SIZE);
Average<float> ave_ADC(ADC_SAMPLE_BUF_SIZE);

volatile uint32_t uiCount = 0;

// Timer 0 for display the ADC peak per 1s
void Timer0Init(uint32_t ui32SysClock)
{
       SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
       TimerConfigure(TIMER0_BASE, TIMER_CFG_A_PERIODIC);
       // TimerDisable(TIMER0_BASE, TIMER_B);
       TimerLoadSet(TIMER0_BASE, TIMER_A, ui32SysClock / LCD_hold - 1);
       TimerEnable(TIMER0_BASE, TIMER_A);
       IntEnable(INT_TIMER0A);
}
// Timer 1 for display FPS
void Timer1Init(uint32_t ui32SysClock)
{
       SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER1);
       TimerConfigure(TIMER1_BASE, TIMER_CFG_A_PERIODIC);
       TimerLoadSet(TIMER1_BASE, TIMER_A, ui32SysClock / LCD_fps - 1);
       TimerEnable(TIMER1_BASE, TIMER_A);
       IntEnable(INT_TIMER1A);
}
// Timer 5 for trigger ADC
void Timer5Init(uint32_t ui32SysClock)
{
       SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER5);
       TimerConfigure(TIMER5_BASE, TIMER_CFG_A_PERIODIC);
       TimerLoadSet(TIMER5_BASE, TIMER_A, ui32SysClock / SampleFreq - 1);
       TimerControlTrigger(TIMER5_BASE, TIMER_A, 1);
       TimerEnable(TIMER5_BASE, TIMER_A);
       IntEnable(INT_TIMER5A);
}

void OnButtonPress(tWidget *pWidget);

Canvas(g_sDriver, &g_sIndicator, 0, 0,
       &g_sKentec320x240x16_SSD2119, 20, 190, 90, 23,
       (CANVAS_STYLE_FILL | CANVAS_STYLE_OUTLINE | CANVAS_STYLE_TEXT),
       ClrGreenYellow, ClrGreenYellow, ClrBlack, &g_sFontCm22b, "Driver ", 0, 0);

Canvas(g_sBridge, &g_sIndicator, 0, &g_sDriver,
       &g_sKentec320x240x16_SSD2119, 20, 160, 90, 23,
       (CANVAS_STYLE_FILL | CANVAS_STYLE_OUTLINE | CANVAS_STYLE_TEXT),
       ClrGreenYellow, ClrGreenYellow, ClrBlack, &g_sFontCm22b, "Bridge ", 0, 0);

Canvas(g_sIndicator, WIDGET_ROOT, 0, &g_sBridge,
       &g_sKentec320x240x16_SSD2119, 20, 160, 90, 46,
       CANVAS_STYLE_FILL, ClrBlack, 0, 0, 0, 0, 0, 0);

Canvas(g_sResult_Slow, WIDGET_ROOT, 0, 0,
       &g_sKentec320x240x16_SSD2119, 180, 196, 130, 24,
       (CANVAS_STYLE_FILL | CANVAS_STYLE_OUTLINE | CANVAS_STYLE_TEXT),
       ClrBlack, ClrWhite, ClrCyan, &g_sFontCm22b, "Ipp ", 0, 0);

Canvas(g_sCurrent, &g_sResult_Quick, 0, 0,
       &g_sKentec320x240x16_SSD2119, 180, 170, 130, 24,
       (CANVAS_STYLE_FILL | CANVAS_STYLE_OUTLINE | CANVAS_STYLE_TEXT),
       ClrBlack, 0, ClrCyan, &g_sFontCm22b, "Irms ", 0, 0);

Canvas(g_sVolt, &g_sResult_Quick, 0, &g_sCurrent,
       &g_sKentec320x240x16_SSD2119, 180, 144, 130, 24,
       (CANVAS_STYLE_FILL | CANVAS_STYLE_OUTLINE | CANVAS_STYLE_TEXT),
       ClrBlack, 0, ClrCyan, &g_sFontCm22b, "Vdc ", 0, 0);

Canvas(g_sResult_Quick, WIDGET_ROOT, 0, &g_sVolt,
       &g_sKentec320x240x16_SSD2119, 180, 144, 130, 24 * 2,
       CANVAS_STYLE_FILL, ClrBlack, 0, 0, 0, 0, 0, 0);

Canvas(g_sHeading, &g_sTOP, 0, &g_sPushBtn,
       &g_sKentec320x240x16_SSD2119, 0, 0, 320, 23,
       (CANVAS_STYLE_FILL | CANVAS_STYLE_OUTLINE | CANVAS_STYLE_TEXT),
       ClrBlack, ClrWhite, ClrRed, &g_sFontCm22b, "PWM Amplify Control", 0, 0);

Canvas(g_sTOP, WIDGET_ROOT, 0, &g_sHeading,
       &g_sKentec320x240x16_SSD2119, 0, 23, 320, (240 - 80),
       CANVAS_STYLE_FILL, ClrBlack, 0, 0, 0, 0, 0, 0);

RectangularButton(g_sPushBtn, &g_sHeading, 0, 0,
                  &g_sKentec320x240x16_SSD2119, 60, 40, 200, 80,
                  (PB_STYLE_OUTLINE | PB_STYLE_TEXT_OPAQUE | PB_STYLE_TEXT |
                   PB_STYLE_FILL),
                  ClrWhite, ClrWhite, ClrRed, ClrRed,
                  &g_sFontCm22b, "TX ON", 0, 0, 0, 0, OnButtonPress);

bool g_RedLedOn = false;

void OnButtonPress(tWidget *pWidget)
{
       g_RedLedOn = !g_RedLedOn;

       if (g_RedLedOn)
       {
              GPIOPinWrite(GPIO_PORTF_BASE, TX_ON, 0x08);
              PushButtonFillColorSet(&g_sPushBtn, ClrRed);
              PushButtonTextColorSet(&g_sPushBtn, ClrWhite);
              PushButtonTextSet(&g_sPushBtn, "TX ON");
              PushButtonOutlineColorSet(&g_sPushBtn, ClrWhite);
       }
       else
       {
              GPIOPinWrite(GPIO_PORTF_BASE, TX_ON, 0x00);
              PushButtonFillColorSet(&g_sPushBtn, ClrWhite);
              PushButtonTextColorSet(&g_sPushBtn, ClrRed);
              PushButtonTextSet(&g_sPushBtn, "TX Off");
              PushButtonOutlineColorSet(&g_sPushBtn, ClrRed);
       }
}

void Timer0IntHandler(void)
{
       TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
       CountT0++;
       //Radian
       // Angle = atan2(CosVoltage,SinVoltage)*(180/PI) + 180;
       //Degree
       Angle = atan2(CosVoltage, SinVoltage) + PI;
       snprintf(Str1, sizeof(Str1), "%.2f °", Angle);
       CanvasTextSet(&g_sResult_Slow, Str1);
       WidgetPaint((tWidget *)&g_sResult_Slow);
}

void Timer1IntHandler(void)
{
       TimerIntClear(TIMER1_BASE, TIMER_TIMA_TIMEOUT);
       CountT1++;
       snprintf(Str2, sizeof(Str2), "%.2f Vcos", CosVoltage);
       CanvasTextSet(&g_sVolt, Str2);
       snprintf(Str3, sizeof(Str3), "%.2f Vsin", SinVoltage);
       CanvasTextSet(&g_sCurrent, Str3);
       WidgetPaint((tWidget *)&g_sResult_Quick);
}

void Timer5IntHandler(void)
{
       TimerIntClear(TIMER5_BASE, TIMER_TIMA_TIMEOUT);
       CountT5++;
}

void ADCInit(void)
{
       SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
       SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
       GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_3 | GPIO_PIN_2);
       SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
       GPIOPinTypeADC(GPIO_PORTD_BASE, GPIO_PIN_3 | GPIO_PIN_2);
       // ADCHardwareOversampleConfigure(ADC0_BASE, 64);
       ADCSequenceConfigure(ADC0_BASE, 1, ADC_TRIGGER_TIMER, 0);
       ADCSequenceStepConfigure(ADC0_BASE, 1, 0, ADC_CTL_CH0 | ADC_CTL_D);
       ADCSequenceStepConfigure(ADC0_BASE, 1, 1, ADC_CTL_CH2 | ADC_CTL_D);
       ADCSequenceStepConfigure(ADC0_BASE, 1, 2, ADC_CTL_CH0 | ADC_CTL_D);
       ADCSequenceStepConfigure(ADC0_BASE, 1, 3, ADC_CTL_CH2 | ADC_CTL_D | ADC_CTL_IE | ADC_CTL_END);
       IntEnable(INT_ADC0SS1);
       ADCSequenceEnable(ADC0_BASE, 1);
       ADCIntClear(ADC0_BASE, 1);
}

void ADC0SS1IntHandler(void)
{
       // TimerDisable(TIMER5_BASE, TIMER_A);
       ADCIntClear(ADC0_BASE, 1);
       uiCount++;
       ADCSequenceDataGet(ADC0_BASE, 1, pui32ADC0Value);
       CosVoltage = (pui32ADC0Value[0] * (2 * Vref / 4096)) - Vref;
       SinVoltage = (pui32ADC0Value[1] * (2 * Vref / 4096)) - Vref;
       // Voltage = (*pui32ADC0Value * (2 * Vref / 4096)) - Vref;
       // Voltage = Voltage / denominator;
       // Amps = ((Voltage - ACSoffset) / mVperAmp);
       // TimerEnable(TIMER5_BASE, TIMER_A);
}
void GPIOinit(void)
{
       SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
       GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, TX_ON); // Output - TX_ON
       GPIOPinTypeGPIOInput(GPIO_PORTF_BASE, GPIO_PIN_0);  // Input - VDRV
       GPIOPinWrite(GPIO_PORTF_BASE, TX_ON, 0x00);
       SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
       GPIOPinTypeGPIOInput(GPIO_PORTE_BASE, GPIO_PIN_0);  //Input PwrAmp
       SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
       GPIOPinTypeGPIOInput(GPIO_PORTB_BASE, GPIO_PIN_2);  //Input nFAULT
}

int main(void)
{
       tContext sContext;
       uint32_t ui32SysClock;
       ave_ADC.clear();
       ave_VDC.clear();

       FPUEnable();
       FPULazyStackingEnable();

       // Set the clock to 80Mhz derived from the PLL and the external oscillator
       MAP_SysCtlClockSet(SYSCTL_SYSDIV_5 | SYSCTL_USE_PLL | SYSCTL_XTAL_16MHZ |
                          SYSCTL_OSC_MAIN);

       ui32SysClock = MAP_SysCtlClockGet();

       GPIOinit();
       Timer0Init(ui32SysClock);
       Timer1Init(ui32SysClock);
       Timer5Init(ui32SysClock);
       ADCInit();

       Kentec320x240x16_SSD2119Init(ui32SysClock);

       GrContextInit(&sContext, &g_sKentec320x240x16_SSD2119);

       TouchScreenInit(ui32SysClock);
       TouchScreenCallbackSet(WidgetPointerMessage);

       WidgetAdd(WIDGET_ROOT, (tWidget *)&g_sTOP);
       WidgetPaint((tWidget *)&g_sTOP);

       WidgetAdd(WIDGET_ROOT, (tWidget *)&g_sIndicator);
       WidgetPaint((tWidget *)&g_sIndicator);

       WidgetAdd(WIDGET_ROOT, (tWidget *)&g_sResult_Slow);
       WidgetPaint((tWidget *)&g_sResult_Slow);

       WidgetAdd(WIDGET_ROOT, (tWidget *)&g_sResult_Quick);
       WidgetPaint((tWidget *)&g_sResult_Quick);

       TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
       TimerIntEnable(TIMER1_BASE, TIMER_TIMA_TIMEOUT);
       TimerIntEnable(TIMER5_BASE, TIMER_TIMA_TIMEOUT);
       ADCIntEnable(ADC0_BASE, 1);
       IntMasterEnable();

       while (1)
       {
              WidgetMessageQueueProcess();
       }
}
