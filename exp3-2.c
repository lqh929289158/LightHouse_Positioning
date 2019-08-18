#include <stdio.h>
#include <inttypes.h>
#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include "hw_memmap.h"
#include "debug.h"
#include "gpio.h"
#include "hw_i2c.h"
#include "hw_types.h"
#include "i2c.h"
#include "pin_map.h"
#include "sysctl.h"
#include "systick.h"
#include "interrupt.h"
#include "hw_timer.h"
#include "timer.h"
#include "uart.h"

#define SYSTICK_FREQUENCY		1000			//1000hz

/*new definition and functions*/

#define GPIO_SYS SYSCTL_PERIPH_GPION
#define TIMER_BASE TIMER0_BASE
#define TIMER_SYS SYSCTL_PERIPH_TIMER0
#define M_PI   3.14159265358979323846264338327950288
uint32_t GPIO_BASE = GPIO_PORTN_BASE;
uint8_t PIN_N = GPIO_PIN_4;
uint32_t INT_PIN = GPIO_INT_PIN_4;
void GPIO_Init(void);
void TIMER_Init(void);
void my_handler_0(void);
void my_handler_1(void);
void my_handler_2(void);
void my_handler_3(void);
void my_handler_4(void);

void getAngle(int);
void getXYZ(double);
void UARTStringPut(const char *cMessage);

void 		Delay(uint32_t value);
void 		S800_GPIO_Init(void);

void 		S800_UART_Init(void);
//systick software counter define
volatile uint16_t systick_10ms_couter,systick_100ms_couter;
volatile uint8_t	systick_10ms_status,systick_100ms_status;

volatile uint8_t result,cnt,key_value,gpio_status;
volatile uint8_t rightshift = 0x01;
uint32_t ui32SysClock;

uint8_t uart_receive_char;

/*new variables*/
char array[300];
double data[36];
bool falling_rising[5]={false,false,false,false,false};
uint32_t falling_t[5][36];
uint32_t rising_t[5][36];
uint32_t time_cnt = 0;
int end[5]={0,0,0,0,0};
double angle[4];
double U0[3] = {2.898177 ,2.350530 ,-0.727328 };
double U1[3] = {-1.912441 ,2.312013 ,-1.165975};
double M0[9] = {-0.060465 ,-0.539917 ,0.839544 ,0.221673 ,0.812817 ,0.538693 ,-0.973245 ,0.218677 ,0.070538};
double M1[9] = {0.019864 ,0.676978 ,-0.735735 ,0.037503 ,0.734858 ,0.677183 ,0.999099 ,-0.041044 ,-0.010791};
double XX,YY,ZZ;

//Calibration
	double b0h;
	double b0v;
	double b1h;
	double b1v;
	double v0h[3];
	double v0v[3];
	double v1h[3];
	double v1v[3];
	double v0[3];
	double v1[3];
	double g0[3];
	double g1[3];
	double W0[3];
bool displayAngle = true;
bool displayXYZ = true;
bool displayPP = false;
	double a,b,c,d,e,denom,ss,tt;
//double aa,bb,cc,dd,ee;
uint32_t rising,falling;

double XX_1,XX_2,XX_3,XX_4,XX_5;
double YY_1,YY_2,YY_3,YY_4,YY_5;
double ZZ_1,ZZ_2,ZZ_3,ZZ_4,ZZ_5;
double angle_offset = 0.073;
int main(void)
{
	int tmp;
	int i;
	uint32_t status;
	//volatile uint16_t	i2c_flash_cnt,gpio_flash_cnt;
	//use internal 16M oscillator, PIOSC
   //ui32SysClock = SysCtlClockFreqSet((SYSCTL_XTAL_16MHZ |SYSCTL_OSC_INT |SYSCTL_USE_OSC), 16000000);		
	//ui32SysClock = SysCtlClockFreqSet((SYSCTL_XTAL_16MHZ |SYSCTL_OSC_INT |SYSCTL_USE_OSC), 8000000);		
	//use external 25M oscillator, MOSC
   //ui32SysClock = SysCtlClockFreqSet((SYSCTL_XTAL_25MHZ |SYSCTL_OSC_MAIN |SYSCTL_USE_OSC), 25000000);		

	//use external 25M oscillator and PLL to 120M
   ui32SysClock = SysCtlClockFreqSet((SYSCTL_XTAL_25MHZ |SYSCTL_OSC_MAIN | SYSCTL_USE_PLL |SYSCTL_CFG_VCO_480), 120000000);		
	//ui32SysClock = SysCtlClockFreqSet((SYSCTL_XTAL_16MHZ |SYSCTL_OSC_INT |SYSCTL_USE_OSC), 16000000);		
	
	//S800_GPIO_Init();
	//S800_I2C0_Init();
	
	S800_UART_Init();
	//UARTStringPut("Start!\n");
	GPIO_Init();
	TIMER_Init();
	
	UARTStringPut("Start!\n");
	//status = GPIOIntTypeGet(GPIO_BASE,GPIO_PIN_4);
	//sprintf(array,"%d",ui32SysClock);
	//UARTStringPut(array);UARTStringPut("\n");
	while (1)
	{
		
		end[0] = 0;end[1] = 0;end[2] = 0;end[3] = 0;end[4] = 0;
		GPIOIntClear(GPIO_PORTP_BASE,GPIO_INT_PIN_0);
		GPIOIntClear(GPIO_PORTP_BASE,GPIO_INT_PIN_1);
		//GPIOIntClear(GPIO_PORTP_BASE,GPIO_INT_PIN_2);
		//GPIOIntClear(GPIO_PORTP_BASE,GPIO_INT_PIN_3);
		//GPIOIntClear(GPIO_PORTP_BASE,GPIO_INT_PIN_4);
		
		//GPIOIntClear(GPIO_PORTN_BASE,GPIO_INT_PIN_5);
		//GPIOIntClear(GPIO_PORTA_BASE,GPIO_INT_PIN_5);
		//GPIOIntClear(GPIO_PORTA_BASE,GPIO_INT_PIN_4);
		//GPIOIntClear(GPIO_PORTC_BASE,GPIO_INT_PIN_5);
		
		GPIOIntEnable(GPIO_PORTP_BASE,GPIO_INT_PIN_0);
		GPIOIntEnable(GPIO_PORTP_BASE,GPIO_INT_PIN_1);
		//GPIOIntEnable(GPIO_PORTP_BASE,GPIO_INT_PIN_2);
		//GPIOIntEnable(GPIO_PORTP_BASE,GPIO_INT_PIN_3);
		//GPIOIntEnable(GPIO_PORTP_BASE,GPIO_INT_PIN_4);
		
		//GPIOIntEnable(GPIO_PORTN_BASE,GPIO_INT_PIN_5);
		//GPIOIntEnable(GPIO_PORTA_BASE,GPIO_INT_PIN_5);
		//GPIOIntEnable(GPIO_PORTA_BASE,GPIO_INT_PIN_4);
		//GPIOIntEnable(GPIO_PORTC_BASE,GPIO_INT_PIN_5);
		Delay(4000000);
		
		GPIOIntDisable(GPIO_PORTP_BASE,GPIO_INT_PIN_0);
		GPIOIntDisable(GPIO_PORTP_BASE,GPIO_INT_PIN_1);
		//GPIOIntDisable(GPIO_PORTP_BASE,GPIO_INT_PIN_2);
		//GPIOIntDisable(GPIO_PORTP_BASE,GPIO_INT_PIN_3);
		//GPIOIntDisable(GPIO_PORTP_BASE,GPIO_INT_PIN_4);
		
		//GPIOIntDisable(GPIO_PORTN_BASE,GPIO_INT_PIN_5);
		//GPIOIntDisable(GPIO_PORTA_BASE,GPIO_INT_PIN_5);
		//GPIOIntDisable(GPIO_PORTA_BASE,GPIO_INT_PIN_4);
		//GPIOIntDisable(GPIO_PORTC_BASE,GPIO_INT_PIN_5);
		getAngle(0);
		getXYZ(0.0);
		XX_1 = XX;
		YY_1 = YY;
		ZZ_1 = ZZ;
		
		sprintf(array,"%.10lf %.10lf %.10lf %.10lf\n",angle[0],angle[1],angle[2],angle[3]);
		UARTStringPut(array);
		
		getAngle(1);
		getXYZ(0.0);
		XX_2 = XX;
		YY_2 = YY;
		ZZ_2 = ZZ;
		
		sprintf(array,"%.10lf %.10lf %.10lf %.10lf\n",angle[0],angle[1],angle[2],angle[3]);
		UARTStringPut(array);
		
		getAngle(2);
		getXYZ(0.0);
		XX_3 = XX;
		YY_3 = YY;
		ZZ_3 = ZZ;
		
		sprintf(array,"%.10lf %.10lf %.10lf %.10lf\n",angle[0],angle[1],angle[2],angle[3]);
		UARTStringPut(array);
		
		getAngle(3);
		getXYZ(0.0);
		XX_4 = XX;
		YY_4 = YY;
		ZZ_4 = ZZ;
		
		sprintf(array,"%.10lf %.10lf %.10lf %.10lf\n",angle[0],angle[1],angle[2],angle[3]);
		UARTStringPut(array);
		
		getAngle(4);
		getXYZ(0.0);
		XX_5 = XX;
		YY_5 = YY;
		ZZ_5 = ZZ;
		
		sprintf(array,"%.10lf %.10lf %.10lf %.10lf\n",angle[0],angle[1],angle[2],angle[3]);
		UARTStringPut(array);
		
		sprintf(array,"%.10lf %.10lf %.10lf %.10lf %.10lf %.10lf %.10lf %.10lf %.10lf %.10lf %.10lf %.10lf %.10lf %.10lf %.10lf\n",
						XX_1,YY_1,ZZ_1,
						XX_2,YY_2,ZZ_2,
						XX_3,YY_3,ZZ_3,
						XX_4,YY_4,ZZ_4,
						XX_5,YY_5,ZZ_5);
		UARTStringPut(array);
		
	}

	

}
void getXYZ(double offset)
{
		double llen;
	//double offset = 0.073;
	//double offset = 0.0;
	 b0h = angle[1]-offset;
	 b0v = angle[0]-offset;
	 b1h = angle[3]-offset;
	 b1v = angle[2]-offset;

	v0h[0]=0;v0h[1]=cos(b0h);v0h[2]=sin(b0h);
	v0v[0]=cos(b0v);v0v[1]=0;v0v[2]=-sin(b0v);
	v1h[0]=0;v1h[1]=cos(b1h);v1h[2]=sin(b1h);
	v1v[0]=cos(b1v);v1v[1]=0;v1v[2]=-sin(b1v);

	//v = hor x ver
	v0[0]= v0h[1]*v0v[2]-v0v[1]*v0h[2]; v0[1]= v0v[0]*v0h[2]-v0h[0]*v0v[2]; v0[2]= v0h[0]*v0v[1] - v0v[0]*v0h[1];
	v1[0]= v1h[1]*v1v[2]-v1v[1]*v1h[2]; v1[1]= v1v[0]*v1h[2]-v1h[0]*v1v[2]; v1[2]= v1h[0]*v1v[1] - v1v[0]*v1h[1];
	
	llen = sqrt(v0[0]*v0[0]+v0[1]*v0[1]+v0[2]*v0[2]);
	v0[0] = v0[0]/llen;v0[1]=v0[1]/llen;v0[2]=v0[2]/llen;
	
	llen = sqrt(v1[0]*v1[0]+v1[1]*v1[1]+v1[2]*v1[2]);
	v1[0] = v1[0]/llen;v1[1]=v1[1]/llen;v1[2]=v1[2]/llen;
	
	//g = M * v //convert to global coordinate
	//g is directional vector
	
	g0[0] = M0[0]*v0[0]+M0[1]*v0[1]+M0[2]*v0[2]; g0[1] = M0[3]*v0[0]+M0[4]*v0[1]+M0[5]*v0[2]; g0[2] = M0[6]*v0[0]+M0[7]*v0[1]+M0[8]*v0[2];
	g1[0] = M1[0]*v1[0]+M1[1]*v1[1]+M1[2]*v1[2]; g1[1] = M1[3]*v1[0]+M1[4]*v1[1]+M1[5]*v1[2]; g1[2] = M1[6]*v1[0]+M1[7]*v1[1]+M1[8]*v1[2];
	
	//g0[0] = M0[0]*v0[0]+M0[3]*v0[1]+M0[6]*v0[2]; g0[1] = M0[1]*v0[0]+M0[4]*v0[1]+M0[7]*v0[2]; g0[2] = M0[2]*v0[0]+M0[5]*v0[1]+M0[8]*v0[2];
	//g1[0] = M1[0]*v1[0]+M1[3]*v1[1]+M1[6]*v1[2]; g1[1] = M1[1]*v1[0]+M1[4]*v1[1]+M1[7]*v1[2]; g1[2] = M1[2]*v1[0]+M1[5]*v1[1]+M1[8]*v1[2];
	//W0 = U0-U1 //U0,U1 are global coordinate of base stations
	//a = g0 * g0
	//b = g0 * g1
	//c = g1 * g1
	//d = g0 * W0
	//e = g1 * W0
	W0[0] = U0[0]-U1[0]; W0[1] = U0[1]-U1[1]; W0[2] = U0[2]-U1[2];
	a = g0[0]*g0[0] + g0[1]*g0[1] + g0[2]*g0[2];
	b = g0[0]*g1[0] + g0[1]*g1[1] + g0[2]*g1[2];
	c = g1[0]*g1[0] + g1[1]*g1[1] + g1[2]*g1[2];
	d = g0[0]*W0[0] + g0[1]*W0[1] + g0[2]*W0[2];
	e = g1[0]*W0[0] + g1[1]*W0[1] + g1[2]*W0[2];

	//denominator must be > 0
	denom = a*c - b*b;

	ss = (b*e - c*d)/denom;
	tt = (a*e - b*d)/denom;
	
	
	if(displayPP){
		UARTStringPut("U0 ");
	sprintf(array,"%.10lf",U0[0]);UARTStringPut(array);UARTStringPut(" ");
	sprintf(array,"%.10lf",U0[1]);UARTStringPut(array);UARTStringPut(" ");
	sprintf(array,"%.10lf",U0[2]);UARTStringPut(array);UARTStringPut("\n");
		
		UARTStringPut("g0 ");
	sprintf(array,"%.10lf",g0[0]);UARTStringPut(array);UARTStringPut(" ");
	sprintf(array,"%.10lf",g0[1]);UARTStringPut(array);UARTStringPut(" ");
	sprintf(array,"%.10lf",g0[2]);UARTStringPut(array);UARTStringPut("\n");
		
		UARTStringPut("U1 ");
	sprintf(array,"%.10lf",U1[0]);UARTStringPut(array);UARTStringPut(" ");
	sprintf(array,"%.10lf",U1[1]);UARTStringPut(array);UARTStringPut(" ");
	sprintf(array,"%.10lf",U1[2]);UARTStringPut(array);UARTStringPut("\n");
		
		UARTStringPut("g1 ");
	sprintf(array,"%.10lf",g1[0]);UARTStringPut(array);UARTStringPut(" ");
	sprintf(array,"%.10lf",g1[1]);UARTStringPut(array);UARTStringPut(" ");
	sprintf(array,"%.10lf",g1[2]);UARTStringPut(array);UARTStringPut("\n");
		
	UARTStringPut("pp0 ");
	sprintf(array,"%.10lf",U0[0]+ss*g0[0]);UARTStringPut(array);UARTStringPut(" ");
	sprintf(array,"%.10lf",U0[1]+ss*g0[1]);UARTStringPut(array);UARTStringPut(" ");
	sprintf(array,"%.10lf",U0[2]+ss*g0[2]);UARTStringPut(array);UARTStringPut("\n");
	
	UARTStringPut("pp1 ");
	sprintf(array,"%.10lf",U1[0]+tt*g1[0]);UARTStringPut(array);UARTStringPut(" ");
	sprintf(array,"%.10lf",U1[1]+tt*g1[1]);UARTStringPut(array);UARTStringPut(" ");
	sprintf(array,"%.10lf",U1[2]+tt*g1[2]);UARTStringPut(array);UARTStringPut("\n");
	
	//make height to 0
	ss = (0.027-U0[1])/g0[1];
	tt = (0.027-U1[1])/g1[1];

	UARTStringPut("p0 ");
	sprintf(array,"%.10lf",U0[0]+ss*g0[0]);UARTStringPut(array);UARTStringPut(" ");
	sprintf(array,"%.10lf",U0[1]+ss*g0[1]);UARTStringPut(array);UARTStringPut(" ");
	sprintf(array,"%.10lf",U0[2]+ss*g0[2]);UARTStringPut(array);UARTStringPut("\n");
	
	UARTStringPut("p1 ");
	sprintf(array,"%.10lf",U1[0]+tt*g1[0]);UARTStringPut(array);UARTStringPut(" ");
	sprintf(array,"%.10lf",U1[1]+tt*g1[1]);UARTStringPut(array);UARTStringPut(" ");
	sprintf(array,"%.10lf",U1[2]+tt*g1[2]);UARTStringPut(array);UARTStringPut("\n");
}
	XX = (U0[0] + ss*g0[0] + U1[0] + tt*g1[0])/2.0;
	YY = (U0[1] + ss*g0[1] + U1[1] + tt*g1[1])/2.0;
	ZZ = (U0[2] + ss*g0[2] + U1[2] + tt*g1[2])/2.0;


}
void getAngle(int idx)
{
	bool flag = true;
	int start = 0;
	double tmp;
	bool a[4];
	int i,j;
	for(i = 0;i<36;i++){
		data[i] = (rising_t[idx][(end[idx]+i)%36]-falling_t[idx][(end[idx]+i)%36])/120.0;
	}
	
	for(i = 0;i<=24;i++){
		flag = true;
		for(j = 0;j<4;j++){
			if(data[i+j*3+0]>20.0 && data[i+j*3+1]>20.0 && data[i+j*3+2]<20.0){
				//OK
			}
			else{
				flag = false;
				break;
			}
		}
		if(flag){start = i;break;}
	}
	//if(!flag)UARTStringPut("FAIL ");
	//sprintf(array,"%d",start);
	//UARTStringPut(array);UARTStringPut("\n");
	for(j = 0;j<4;j++){
		//BS1
		if(data[start+j*3]<data[start+j*3+1]){
				a[j] = false;
				angle[j] = ((falling_t[idx][(end[idx]+start+j*3+2)%36] - falling_t[idx][(end[idx]+start+j*3)%36])/120.0 - 4000.0)/8333.0 * M_PI;
				//sprintf(array,"%x",(falling_t[(end+start+j*3+2)%36] - falling_t[(end+start+j*3)%36]));
				//UARTStringPut(array);UARTStringPut("\n");
		}
		//BS2
		else{
			a[j] = true;
			
			angle[j] = ((falling_t[idx][(end[idx]+start+j*3+2)%36] - falling_t[idx][(end[idx]+start+j*3+1)%36])/120.0 - 4000.0)/8333.0 * M_PI;
			//sprintf(array,"%x",(falling_t[(end+start+j*3+2)%36] - falling_t[(end+start+j*3+1)%36]));
				//UARTStringPut(array);UARTStringPut("\n");
		}
		//sprintf(array,"%.5lf",angle[j]);
				//UARTStringPut(array);UARTStringPut("\n");
	}
	
	//0 0 1 1
	
	//1 0 0 1
	if(a[0] && a[3]){
		tmp = angle[0];
		angle[0] = angle[1];
		angle[1] = angle[2];
		angle[2] = angle[3];
		angle[3] = tmp;
	}
	//1 1 0 0
	else if(a[0] && a[1]){
		tmp = angle[0];
		angle[0] = angle[2];
		angle[2] = tmp;
		
		tmp = angle[1];
		angle[1] = angle[3];
		angle[3] = tmp;
	}
	
	//0 1 1 0
	else if(a[1]&&a[2]){
		tmp = angle[3];
		angle[3] = angle[2];
		angle[2] = angle[1];
		angle[1] = angle[0];
		angle[0] = tmp;
	}
}

void TIMER_Init(void)
{
	SysCtlPeripheralEnable(TIMER_SYS);                      // enable peripheral
	SysCtlDelay(5);
	SysCtlPeripheralReset(TIMER_SYS);
	SysCtlDelay(5);
	TimerDisable(TIMER_BASE, TIMER_A);
	//TimerDisable(TIMER_BASE, TIMER_BOTH);
	TimerConfigure(TIMER_BASE,TIMER_CFG_A_PERIODIC_UP);
	//TimerConfigure(TIMER_BASE,TIMER_CFG_PERIODIC_UP);
	TimerClockSourceSet(TIMER_BASE,TIMER_CLOCK_SYSTEM);
	//TimerPrescaleSet(TIMER_BASE,TIMER_BOTH,1);
	TimerPrescaleSet(TIMER_BASE,TIMER_A,1);
	TimerLoadSet(TIMER_BASE,TIMER_A,0xFFFFFFFF);
	//TimerLoadSet64(TIMER_BASE,0xFFFFFFFFFFFFFFFF);
	TimerEnable(TIMER_BASE,TIMER_A);
	
}
void GPIO_Init(void)
{
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPION);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOK);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOP);
	
	GPIOPinTypeGPIOInput(GPIO_PORTP_BASE,GPIO_PIN_0);
	GPIOPinTypeGPIOInput(GPIO_PORTP_BASE,GPIO_PIN_1);
	GPIOPinTypeGPIOInput(GPIO_PORTP_BASE,GPIO_PIN_2);
	GPIOPinTypeGPIOInput(GPIO_PORTP_BASE,GPIO_PIN_3);
	GPIOPinTypeGPIOInput(GPIO_PORTP_BASE,GPIO_PIN_4);
	
	GPIOPinTypeGPIOInput(GPIO_PORTN_BASE,GPIO_PIN_4);
	GPIOPinTypeGPIOInput(GPIO_PORTN_BASE,GPIO_PIN_5);
	
	GPIOPinTypeGPIOInput(GPIO_PORTK_BASE,GPIO_PIN_4);
	GPIOPinTypeGPIOInput(GPIO_PORTK_BASE,GPIO_PIN_5);
	GPIOPinTypeGPIOInput(GPIO_PORTK_BASE,GPIO_PIN_6);
	
	GPIOPinTypeGPIOInput(GPIO_PORTA_BASE,GPIO_PIN_4);
	GPIOPinTypeGPIOInput(GPIO_PORTA_BASE,GPIO_PIN_5);
	
	GPIOPinTypeGPIOInput(GPIO_PORTC_BASE,GPIO_PIN_4);
	GPIOPinTypeGPIOInput(GPIO_PORTC_BASE,GPIO_PIN_5);
	
	GPIOPadConfigSet(GPIO_PORTP_BASE,GPIO_PIN_0,GPIO_STRENGTH_2MA,GPIO_PIN_TYPE_STD_WPU);
	GPIOPadConfigSet(GPIO_PORTP_BASE,GPIO_PIN_1,GPIO_STRENGTH_2MA,GPIO_PIN_TYPE_STD_WPU);
	GPIOPadConfigSet(GPIO_PORTP_BASE,GPIO_PIN_2,GPIO_STRENGTH_2MA,GPIO_PIN_TYPE_STD_WPU);
	GPIOPadConfigSet(GPIO_PORTP_BASE,GPIO_PIN_3,GPIO_STRENGTH_2MA,GPIO_PIN_TYPE_STD_WPU);
	GPIOPadConfigSet(GPIO_PORTP_BASE,GPIO_PIN_4,GPIO_STRENGTH_2MA,GPIO_PIN_TYPE_STD_WPU);
	
	GPIOPadConfigSet(GPIO_PORTN_BASE,GPIO_PIN_4,GPIO_STRENGTH_2MA,GPIO_PIN_TYPE_STD_WPU);
	GPIOPadConfigSet(GPIO_PORTN_BASE,GPIO_PIN_5,GPIO_STRENGTH_2MA,GPIO_PIN_TYPE_STD_WPU);
	
	GPIOPadConfigSet(GPIO_PORTK_BASE,GPIO_PIN_4,GPIO_STRENGTH_2MA,GPIO_PIN_TYPE_STD_WPU);
	GPIOPadConfigSet(GPIO_PORTK_BASE,GPIO_PIN_5,GPIO_STRENGTH_2MA,GPIO_PIN_TYPE_STD_WPU);
	GPIOPadConfigSet(GPIO_PORTK_BASE,GPIO_PIN_6,GPIO_STRENGTH_2MA,GPIO_PIN_TYPE_STD_WPU);

	GPIOPadConfigSet(GPIO_PORTA_BASE,GPIO_PIN_4,GPIO_STRENGTH_2MA,GPIO_PIN_TYPE_STD_WPU);
	GPIOPadConfigSet(GPIO_PORTA_BASE,GPIO_PIN_5,GPIO_STRENGTH_2MA,GPIO_PIN_TYPE_STD_WPU);
	
	GPIOPadConfigSet(GPIO_PORTC_BASE,GPIO_PIN_4,GPIO_STRENGTH_2MA,GPIO_PIN_TYPE_STD_WPU);
	GPIOPadConfigSet(GPIO_PORTC_BASE,GPIO_PIN_5,GPIO_STRENGTH_2MA,GPIO_PIN_TYPE_STD_WPU);
	
	GPIOIntDisable(GPIO_PORTP_BASE,GPIO_INT_PIN_0);
	GPIOIntDisable(GPIO_PORTP_BASE,GPIO_INT_PIN_1);
	GPIOIntDisable(GPIO_PORTP_BASE,GPIO_INT_PIN_2);
	GPIOIntDisable(GPIO_PORTP_BASE,GPIO_INT_PIN_3);
	GPIOIntDisable(GPIO_PORTP_BASE,GPIO_INT_PIN_4);
	
	GPIOIntDisable(GPIO_PORTN_BASE,GPIO_INT_PIN_4);
	GPIOIntDisable(GPIO_PORTN_BASE,GPIO_INT_PIN_5);
	
	GPIOIntDisable(GPIO_PORTK_BASE,GPIO_INT_PIN_4);
	GPIOIntDisable(GPIO_PORTK_BASE,GPIO_INT_PIN_5);
	GPIOIntDisable(GPIO_PORTK_BASE,GPIO_INT_PIN_6);
	
	GPIOIntDisable(GPIO_PORTA_BASE,GPIO_INT_PIN_4);
	GPIOIntDisable(GPIO_PORTA_BASE,GPIO_INT_PIN_5);
	
	GPIOIntDisable(GPIO_PORTC_BASE,GPIO_INT_PIN_4);
	GPIOIntDisable(GPIO_PORTC_BASE,GPIO_INT_PIN_5);
	
	GPIOIntTypeSet(GPIO_PORTP_BASE,GPIO_PIN_0,GPIO_FALLING_EDGE);
	GPIOIntTypeSet(GPIO_PORTP_BASE,GPIO_PIN_1,GPIO_FALLING_EDGE);
	GPIOIntTypeSet(GPIO_PORTP_BASE,GPIO_PIN_2,GPIO_FALLING_EDGE);
	GPIOIntTypeSet(GPIO_PORTP_BASE,GPIO_PIN_3,GPIO_FALLING_EDGE);
	GPIOIntTypeSet(GPIO_PORTP_BASE,GPIO_PIN_4,GPIO_FALLING_EDGE);
	
	GPIOIntTypeSet(GPIO_PORTN_BASE,GPIO_PIN_4,GPIO_FALLING_EDGE);
	GPIOIntTypeSet(GPIO_PORTN_BASE,GPIO_PIN_5,GPIO_FALLING_EDGE);
	
	GPIOIntTypeSet(GPIO_PORTK_BASE,GPIO_PIN_4,GPIO_FALLING_EDGE);
	GPIOIntTypeSet(GPIO_PORTK_BASE,GPIO_PIN_5,GPIO_FALLING_EDGE);
	GPIOIntTypeSet(GPIO_PORTK_BASE,GPIO_PIN_6,GPIO_FALLING_EDGE);
	
	GPIOIntTypeSet(GPIO_PORTA_BASE,GPIO_PIN_4,GPIO_FALLING_EDGE);
	GPIOIntTypeSet(GPIO_PORTA_BASE,GPIO_PIN_5,GPIO_FALLING_EDGE);
	
	GPIOIntTypeSet(GPIO_PORTC_BASE,GPIO_PIN_4,GPIO_FALLING_EDGE);
	GPIOIntTypeSet(GPIO_PORTC_BASE,GPIO_PIN_5,GPIO_FALLING_EDGE);

	//GPIOIntRegister(GPIO_PORTN_BASE,my_handler);
	//GPIOIntRegister(GPIO_PORTK_BASE,my_handler);
	//GPIOIntRegister(GPIO_PORTA_BASE,my_handler);
	//GPIOIntRegister(GPIO_PORTC_BASE,my_handler);
	//GPIOIntRegister(GPIO_PORTP_BASE,my_handler_0);
	
	GPIOIntRegisterPin(GPIO_PORTP_BASE,0,my_handler_0);
	GPIOIntRegisterPin(GPIO_PORTP_BASE,1,my_handler_1);
	GPIOIntRegisterPin(GPIO_PORTP_BASE,2,my_handler_2);
	GPIOIntRegisterPin(GPIO_PORTP_BASE,3,my_handler_3);
	GPIOIntRegisterPin(GPIO_PORTP_BASE,4,my_handler_4);
	
	GPIOIntClear(GPIO_PORTP_BASE,GPIO_INT_PIN_0);
	GPIOIntClear(GPIO_PORTP_BASE,GPIO_INT_PIN_1);
	GPIOIntClear(GPIO_PORTP_BASE,GPIO_INT_PIN_2);
	GPIOIntClear(GPIO_PORTP_BASE,GPIO_INT_PIN_3);
	GPIOIntClear(GPIO_PORTP_BASE,GPIO_INT_PIN_4);
	
	
	GPIOIntClear(GPIO_PORTN_BASE,GPIO_INT_PIN_4);
	GPIOIntClear(GPIO_PORTN_BASE,GPIO_INT_PIN_5);
	
	GPIOIntClear(GPIO_PORTK_BASE,GPIO_INT_PIN_4);
	GPIOIntClear(GPIO_PORTK_BASE,GPIO_INT_PIN_5);
	GPIOIntClear(GPIO_PORTK_BASE,GPIO_INT_PIN_6);
	
	GPIOIntClear(GPIO_PORTA_BASE,GPIO_INT_PIN_4);
	GPIOIntClear(GPIO_PORTA_BASE,GPIO_INT_PIN_5);
	
	GPIOIntClear(GPIO_PORTC_BASE,GPIO_INT_PIN_4);
	GPIOIntClear(GPIO_PORTC_BASE,GPIO_INT_PIN_5);
	
	//GPIOIntEnable(GPIO_PORTN_BASE,GPIO_INT_PIN_4);
	//status = GPIOIntTypeGet(GPIO_BASE,GPIO_PIN_4);
	//sprintf(array,"%x",status);
	//UARTStringPut(array);
}
/*
void my_handler(void)
{
	time_cnt = TimerValueGet(TIMER_BASE,TIMER_A);
	GPIOIntClear(GPIO_BASE,INT_PIN);
		if(!falling_rising[0]){
			falling = time_cnt;
			falling_t[0][end[0]%36] = time_cnt;
			GPIOIntTypeSet(GPIO_BASE,PIN_N,GPIO_RISING_EDGE);
			
		}
		else{
			rising_t[end%36] = time_cnt;
			end++;
			rising = time_cnt;
			GPIOIntTypeSet(GPIO_BASE,PIN_N,GPIO_FALLING_EDGE);
		}
}
*/
void my_handler_0(void)
{
	time_cnt = TimerValueGet(TIMER_BASE,TIMER_A);
	GPIOIntClear(GPIO_PORTP_BASE,GPIO_INT_PIN_0);
		if(!falling_rising[0]){
			falling_t[0][end[0]%36] = time_cnt;
			GPIOIntTypeSet(GPIO_PORTP_BASE,GPIO_PIN_0,GPIO_RISING_EDGE);
			falling_rising[0]=true;
		}
		else{
			rising_t[0][end[0]%36] = time_cnt;
			end[0]++;
			GPIOIntTypeSet(GPIO_PORTP_BASE,GPIO_PIN_0,GPIO_FALLING_EDGE);
			falling_rising[0]=false;
		}
		//UARTStringPut("Here0000!\n");
}

void my_handler_1(void)
{
	time_cnt = TimerValueGet(TIMER_BASE,TIMER_A);
	GPIOIntClear(GPIO_PORTP_BASE,GPIO_INT_PIN_1);
		if(!falling_rising[1]){
			falling_t[1][end[1]%36] = time_cnt;
			GPIOIntTypeSet(GPIO_PORTP_BASE,GPIO_PIN_1,GPIO_RISING_EDGE);
			falling_rising[1]=true;
		}
		else{
			rising_t[1][end[1]%36] = time_cnt;
			end[1]++;
			GPIOIntTypeSet(GPIO_PORTP_BASE,GPIO_PIN_1,GPIO_FALLING_EDGE);
			falling_rising[1]=false;
		}
		//UARTStringPut("Here1111!\n");
}

void my_handler_2(void)
{
	time_cnt = TimerValueGet(TIMER_BASE,TIMER_A);
	GPIOIntClear(GPIO_PORTP_BASE,GPIO_INT_PIN_2);
		if(!falling_rising[2]){
			falling_t[2][end[2]%36] = time_cnt;
			GPIOIntTypeSet(GPIO_PORTP_BASE,GPIO_PIN_2,GPIO_RISING_EDGE);
			falling_rising[2]=true;
		}
		else{
			rising_t[2][end[2]%36] = time_cnt;
			end[2]++;
			GPIOIntTypeSet(GPIO_PORTP_BASE,GPIO_PIN_2,GPIO_FALLING_EDGE);
			falling_rising[2]=false;
		}
		//UARTStringPut("Here2222!\n");
}

void my_handler_3(void)
{
	time_cnt = TimerValueGet(TIMER_BASE,TIMER_A);
	GPIOIntClear(GPIO_PORTP_BASE,GPIO_INT_PIN_3);
		if(!falling_rising[3]){
			falling_t[3][end[3]%36] = time_cnt;
			GPIOIntTypeSet(GPIO_PORTP_BASE,GPIO_PIN_3,GPIO_RISING_EDGE);
			falling_rising[3]=true;
		}
		else{
			rising_t[3][end[3]%36] = time_cnt;
			end[3]++;
			GPIOIntTypeSet(GPIO_PORTP_BASE,GPIO_PIN_3,GPIO_FALLING_EDGE);
			falling_rising[3]=false;
		}
		//UARTStringPut("Here3333!\n");
}
void my_handler_4(void)
{
	time_cnt = TimerValueGet(TIMER_BASE,TIMER_A);
	GPIOIntClear(GPIO_PORTP_BASE,GPIO_INT_PIN_4);
		if(!falling_rising[4]){
			falling_t[4][end[4]%36] = time_cnt;
			GPIOIntTypeSet(GPIO_PORTP_BASE,GPIO_PIN_4,GPIO_RISING_EDGE);
			falling_rising[4]=true;
		}
		else{
			rising_t[4][end[4]%36] = time_cnt;
			end[4]++;
			GPIOIntTypeSet(GPIO_PORTP_BASE,GPIO_PIN_4,GPIO_FALLING_EDGE);
			falling_rising[4]=false;
		}
		//UARTStringPut("Here4444!\n");
}
void Delay(uint32_t value)
{
	uint32_t ui32Loop;
	for(ui32Loop = 0; ui32Loop < value; ui32Loop++){};
}


void UARTStringPut(const char *cMessage)
{
	while(*cMessage!='\0')
		UARTCharPut(UART2_BASE,*(cMessage++));
}

void S800_UART_Init(void)
{
	SysCtlPeripheralEnable(SYSCTL_PERIPH_UART2);
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);						//Enable PortA
	while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOD));			//Wait for the GPIO moduleA ready

	GPIOPinConfigure(GPIO_PD4_U2RX);												// Set GPIO A0 and A1 as UART pins.
  GPIOPinConfigure(GPIO_PD5_U2TX);    			

  GPIOPinTypeUART(GPIO_PORTD_BASE, GPIO_PIN_4 | GPIO_PIN_5);

	// Configure the UART for 115,200, 8-N-1 operation.
  UARTConfigSetExpClk(UART2_BASE, ui32SysClock,1500000,(UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |UART_CONFIG_PAR_NONE));
	UARTStringPut((uint8_t *)"\r\nHello, world!\r\n");
}
void S800_GPIO_Init(void)
{
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);						//Enable PortF
	while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOF));			//Wait for the GPIO moduleF ready
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOJ);						//Enable PortJ	
	while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOJ));			//Wait for the GPIO moduleJ ready	
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPION);						//Enable PortN	
	while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPION));			//Wait for the GPIO moduleN ready		
	
  GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_0);			//Set PF0 as Output pin
  GPIOPinTypeGPIOOutput(GPIO_PORTN_BASE, GPIO_PIN_0);			//Set PN0 as Output pin

	GPIOPinTypeGPIOInput(GPIO_PORTJ_BASE,GPIO_PIN_0 | GPIO_PIN_1);//Set the PJ0,PJ1 as input pin
	GPIOPadConfigSet(GPIO_PORTJ_BASE,GPIO_PIN_0 | GPIO_PIN_1,GPIO_STRENGTH_2MA,GPIO_PIN_TYPE_STD_WPU);
}


/*
	Corresponding to the startup_TM4C129.s vector table systick interrupt program name
*/
void SysTick_Handler(void)
{
	if (systick_100ms_couter	!= 0)
		systick_100ms_couter--;
	else
	{
		systick_100ms_couter	= SYSTICK_FREQUENCY/10;
		systick_100ms_status 	= 1;
	}
	
	if (systick_10ms_couter	!= 0)
		systick_10ms_couter--;
	else
	{
		systick_10ms_couter		= SYSTICK_FREQUENCY/100;
		systick_10ms_status 	= 1;
	}
	if (GPIOPinRead(GPIO_PORTJ_BASE,GPIO_PIN_0) == 0)
	{
		systick_100ms_status	= systick_10ms_status = 0;
		GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_0,GPIO_PIN_0);		
	}
	else
		GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_0,0);		
}
