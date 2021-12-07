/*------------------------------------
 * Pin configuration
 * PORTA: 0000001 0x03
 * RA0: MotorCurrent_L
 * RA1: MotorCurrent_R
 * 
 * PORTB: 00110011 0x33
 * RB0: toggle swith input :deploy
 * RB1: toggle swith input :pull in 
 * RB2: CANTX
 * RB3: CANRX
 * RB4: toggle swith input :deploy cable
 * RB6: toggle swith input :pull in cable
 * 
 * PORTC: 00000000 0x00
 * RC0: MotorDriver_L EN
 * RC2: MotorDriver_L PWM1
 * RC6: MotorDriver_L PWM2
 * 
 * RC1: MotorDriver_R EN
 * RC7: MotorDriver_R PWM1
 * RB5: MotorDriver_R PWM2
 * 
 * 
 * 
 * CAN message
 * Receive
 * 0x70 5byte set target 0: mode-direct, speed, torque, free. 1-4: singed int target value- pwm duty, velosity, torque 
 * 0x71 4byte set torque-current coeff 0-3: float
 * 
 * Send
 * 0x78 1byte Heart beat
 * 0x79 set tension roller command  0: mode-direct, speed, torque, free. 1-4: singed int target value- pwm duty, velosity, torque
 * 0x7a 8byte motor state 0-3: float velosity, 4-7: float torque
 *   
 * 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
 * https://www.pololu.com/product/2997
//------------------------------------*/

#include <p18lf25k80.h>
#include "ECAN.h"
#include "myCAN.h"
#include "timers.h"
#include "usart.h"
#include "portb.h"
#include "i2c.h"
#include "adc.h"
//#include "thDrive.h"
#include "delays.h"
#include "INA226.h"
#include "pwm.h"

#pragma config FOSC = HS2	
#pragma config PLLCFG = ON	//PLL40MHz
#pragma config XINST = OFF
#pragma config CANMX = PORTB
#pragma config SOSCSEL = DIG
#pragma config WDTEN = ON
//#pragma config LVP = OFF

#define TMR0IF INTCONbits.TMR0IF
#define RBIF INTCONbits.RBIF

#define STAGE_DEPLOY PORTCbits.RB0
#define STAGE_PULL_IN PORTCbits.RB1
#define CABLE_DEPLOY PORTCbits.RB6
#define CABLE_PULL_IN PORTCbits.RB7

#define M1_EN PORTCbits.RC0
#define M1_PWM1 PORTCbits.RC2
#define M1_PWM2 PORTCbits.RC6

#define M2_EN PORTCbits.RC1
#define M2_PWM1 PORTCbits.RC7
#define M2_PWM2 PORTBbits.RB5

#define MCHP_C18
#define NODE1

//Functions prototype===============================================================
void isr(void);
void my_putc(unsigned char a,unsigned char port);
void init(void);
int getADC(inc ch);
void motorupdate(int duty);
void motorfree(void);
void controllupdate(char mode, int power, float target_speed, float target_torque);
void setMotorDuty(int pwm1,int pwm2);
float setMotorVelosity(float targetSpeed, float currentSpeed);
void Int2Bytes(int input, BYTE* target, BYTE len);
//===================================================================

BOOL TimerFlag = FALSE;
BOOL LEDFlag = TRUE;
BOOL DEADTIMEFLAG = TRUE;
int T_Counter = 0;
int LED_Counter = 0;
unsigned int TimeOut_Counter = 0;
unsigned int led_TimeOut_Counter = 0;
BYTE test = 0;
//CAN??
const unsigned long ID_SET_TENSION_ROLLER_COMMAND = 0x80;
const unsigned long ID_STATUS = 0x79;

const unsigned int TIMEOUT = 4;

int cnt=0;

long speedCounter = 0;
//float  speed = 0;
float  torque = 0;
float  target_speed = 0;
float  target_torque = 0;
int pwm_duty = 0;
float reso = 4;
float intCycle = 20;
float torque2current = 1.0;
const float VOLTAGE2CURRENT = 2.0;
float reelRadius = 0.12;//[m]]
float reelRadius_ = 0.07;//[m]]
float rollerRadius = 0.015;//[m]]
char control_mode = 0;
float _speed = 0;

union IntAndFloat{
    long ival;
    float fval;
};
union IntAndFloat speed;
union IntAndFloat targetspeed;
union IntAndFloat targetspeed_tension;
union IntAndFloat current_m1;
union IntAndFloat current_m2;



typedef struct {
    unsigned long ID;
    BYTE data[8];
    unsigned int len;
} CANmessage;

CANmessage set_tension;
CANmessage send_Status;

typedef struct {
    float error;
    float k;
    float I;
    float D;
    float accum;
    float dt;
} controller;

controller m1_Controller;
controller m2_Controller;

long sc = 0;
int PWM_power = 0;

int lastDuty_m1 = 0;
int lastDuty_m2 = 0;
int dir = 1;
//????????==============================================================
#pragma code compatible_vvector=0x08
void compatible_interrupt(void){
	_asm
		GOTO isr
	_endasm
}
#pragma code
//
//??????
#pragma interruptlow isr
void isr(void){
	if(TMR0IF){
		TMR0IF = 0;
		T_Counter++;
        LED_Counter++;
		if(T_Counter >= 61){//20Hz
			T_Counter = 0;
			TimerFlag = TRUE;
            _speed = speedCounter / reso * intCycle * 2 * 3.14 / 51;//[rad/sec]
            sc = speedCounter;
            speedCounter = 0;
            
            TimeOut_Counter++;
		}
        if(LED_Counter >= 610){//2Hz
			LED_Counter = 0;
			LEDFlag = TRUE;
		}
	}
    /*
    if(RBIF){
        RBIF = 0;
        if(ENC_A){
            speedCounter +=1;
        } 
    }
     */
}
//?????=========================================================================
void main()
{	
    speed.fval = 0.0;
    m1_Controller.k = 15;
    m1_Controller.I = 18;
    m1_Controller.dt = 1.0 / intCycle;
    m1_Controller.accum = 0;
    
    m2_Controller.k = 15;
    m2_Controller.I = 18;
    m2_Controller.dt = 1.0 / intCycle;
    m2_Controller.accum = 0;
    targetspeed.fval = 0;
    
    set_tension.ID = ID_SET_TENSION_ROLLER_COMMAND;
    set_tension.len = 5;
    send_Status.ID = ID_STATUS;
    send_Status.len = 8;
    
    init();
	
    while(1){		
		if(ECANReceiveMessage(&id, data_R, &datalen_R, &flags)){
            //while(!ECANSendMessage(id, data_T,2, ECAN_TX_STD_FRAME));
			switch(id){
                case 0x70:
                    TimeOut_Counter = 0;
                    switch(data_R[0]){
                        case 0://direct
                            control_mode = 0;
                            pwm_duty = ((data_R[1] << 8 ) | data_R[2]);
                            break;
                        case 1://speed controll
                            control_mode = 1;
                            targetspeed.ival = data_R[1];
                            targetspeed.ival = targetspeed.ival << 8 | data_R[2];
                            targetspeed.ival = targetspeed.ival << 8 | data_R[3];
                            targetspeed.ival = targetspeed.ival << 8 | data_R[4];
                            //tension roller command sending
                            if(targetspeed.fval > 0){
                                targetspeed_tension.fval = targetspeed.fval * reelRadius / rollerRadius;
                            }else{
                                targetspeed_tension.fval = targetspeed.fval * reelRadius_ / rollerRadius;
                            }
                            
                            //targetspeed_tension.fval = targetspeed.fval * reelRadius / rollerRadius;
                            //Int2Bytes(targetspeed_tension.ival, set_tension.data, set_tension.len);
                            set_tension.data[0] = 1;
                            set_tension.data[1] = (targetspeed_tension.ival >> 24) & 0xff;
                            set_tension.data[2] = (targetspeed_tension.ival >> 16) & 0xff;
                            set_tension.data[3] = (targetspeed_tension.ival >> 8) & 0xff;
                            set_tension.data[4] = (targetspeed_tension.ival) & 0xff;
                            
                            while(!ECANSendMessage(set_tension.ID, set_tension.data, set_tension.len, ECAN_TX_STD_FRAME));
                            
                            break;
                        case 2:
                            control_mode = 2;
                            target_torque = ((data_R[1] << 24 ) | (data_R[2] << 16) | (data_R[3] << 8) | data_R[4] );
                            break;
                        default:
                            break;
                    }
                break;
                case 0x71:
                    torque2current = ((data_R[0] << 24 ) | (data_R[1] << 16) | (data_R[2] << 8) | data_R[3] );
                    break;
                default:
                    break;        
            }
        }
		//LED flashing
        if(LEDFlag){
            LEDFlag =FALSE;
            //while(!ECANSendMessage(0xee, tstMessage,4, ECAN_TX_STD_FRAME));
        }
		if(TimerFlag){
			TimerFlag = FALSE;
            
            if(targetspeed.fval < 0){
                speed.fval = _speed * -1.0;
            }else{
                speed.fval = _speed;
            }
            
            if(TimeOut_Counter > TIMEOUT){
                targetspeed.fval = 0.0;
                motorfree();
                m1_Controller.accum = 0;
                m2_Controller.accum = 0;
            }else{
                //setMotorDuty((int)setMotorVelosity( targetspeed.fval, speed.fval) );
                setMotorDuty();
            }
             
            current_m1.fval = getADC(1) *  5.0 / 4096 * VOLTAGE2CURRENT;
            current_m2.fval = getADC(2) *  5.0 / 4096 * VOLTAGE2CURRENT;
            
            send_Status.data[0] = (current_m1.ival >> 24) & 0xff;
            send_Status.data[1] = (current_m1.ival >> 16) & 0xff;
            send_Status.data[2] = (current_m1.ival >> 8) & 0xff;
            send_Status.data[3] = (current_m1.ival) & 0xff;
            
            send_Status.data[4] = (current_m2.ival >> 24) & 0xff;
            send_Status.data[5] = (current_m2.ival >> 16) & 0xff;
            send_Status.data[6] = (current_m2.ival >> 8) & 0xff;
            send_Status.data[7] = (current_m2.ival) & 0xff;
            
            while(!ECANSendMessage(send_Status.ID, send_Status.data,send_Status.len, ECAN_TX_STD_FRAME));
		}
	}
}

float setMotorVelosity_m1(float targetSpeed, float currentSpeed){
    float s = 0;
    m1_Controller.error = targetSpeed - currentSpeed;
    m1_Controller.accum += m1_Controller.error * m1_Controller.dt;
    
    s = m1_Controller.k * ( m1_Controller.error + (m1_Controller.I * m1_Controller.accum) );
    //s = speedController.k * speedController.error;
    if(m1_Controller.accum > 1000){
        m1_Controller.accum = 1000;
    }else if(m1_Controller.accum < -1000){
        m1_Controller.accum = -1000;
    }
    return s;
}
float setMotorVelosity_m2(float targetSpeed, float currentSpeed){
    float s = 0;
    m2_Controller.error = targetSpeed - currentSpeed;
    m2_Controller.accum += m2_Controller.error * m2_Controller.dt;
    
    s = m2_Controller.k * ( m2_Controller.error + (m2_Controller.I * m2_Controller.accum) );
    //s = speedController.k * speedController.error;
    if(m2_Controller.accum > 1000){
        m2_Controller.accum = 1000;
    }else if(m2_Controller.accum < -1000){
        m2_Controller.accum = -1000;
    }
    return s;
}
void setMotorDuty(int pwm1, int pwm2){
    M1_EN = 1;
    if((lastDuty_m1 & 0x8000) != (pwm1 & 0x8000) ){
        SetDCPWM3(0);
        SetDCPWM4(0);
        lastDuty_m1 = pwm1;
        return;
    }
    if(pwm1 > 0){
        SetDCPWM2(pwm1);
        SetDCPWM3(0);
    }else{
        SetDCPWM2(0);
        SetDCPWM3(pwm1 * -1);
    }
    lastDuty_m1 = pwm1;
    
    M2_EN = 1;
    if((lastDuty_m2 & 0x8000) != (pwm2 & 0x8000) ){
        SetDCPWM4(0);
        SetDCPWM5(0);
        lastDuty_m2 = pwm2;
        return;
    }
    if(pwm2 > 0){
        SetDCPWM4(pwm1);
        SetDCPWM5(0);
    }else{
        SetDCPWM4(0);
        SetDCPWM5(pwm1 * -1);
    }
    lastDuty_m2 = pwm2;
}
//UART_1byte??
void my_putc(unsigned char a,unsigned char port){
	if(port ==0x01){
		putc1USART(a);
		while(!PIR1bits.TX1IF){}
	}else if(port ==0x02){
		putc2USART(a);
		while(!PIR3bits.TX2IF){}
	}
}
void init(void){
	//0:out 1:in
	TRISA = 0x01;
	TRISB = 0x10;
	TRISC = 0x00;
	ANCON0 = 0x00;
	ANCON1 = 0x00;
    
	ECANInitialize();
    
    //Timer and interrupt 
	OpenTimer0(TIMER_INT_ON & T0_8BIT & T0_SOURCE_INT & T0_PS_1_32);//20MHZ/4/256/256 = 76.29Hz;
	INTCONbits.GIE = 1;
	INTCONbits.PEIE = 1;
	INTCONbits.RBIE = 1;
	IOCB = 0x10;//RB4
   
    //PWM
    T2CON = 0b00000011;
    SetDCPWM3(0x0000);
    SetDCPWM4(0x0000);
	OpenPWM3(0xff,1);
    OpenPWM4(0xff,1);
    
    //I2C
    //OpenI2C(MASTER,SLEW_ON);
    //SSPCON1bits.SSPEN = 1;
    //SSPADD = 255;
    
    //ADC initialize
    OpenADC(ADC_FOSC_32 & ADC_RIGHT_JUST & ADC_20_TAD,ADC_CH0 & ADC_INT_OFF ,0);
}
int getADC(void){
    ConvertADC();
    while(BusyADC()){}
    return ReadADC();
}
void motorupdate(int duty){
    if(duty >0){
        SetDCPWM3(0x7fff & duty);
    }else if(duty != 0){
        SetDCPWM4(0x7fff & duty);
    }
}
void motorfree(void){
    SetDCPWM2(0x00);
    SetDCPWM3(0x00);
    SetDCPWM4(0x00);
    SetDCPWM5(0x00);
    M1_EN = 0;
    M2_EN = 0;
    
}
void controllupdate(char mode, int power, float target_speed, float target_torque){
    int duty;
    switch(mode){
        case 0:
            duty = power;
            break;
        case 1:
            duty = (target_speed - speed.fval) * 1;
            break;
        case 2:
            break;
        default:
            break;
    }
    motorupdate(duty);
}

void Int2Bytes(int input, BYTE* target, BYTE len){
    int i = 0;
    for(i; i < len; i++){
        target[i] = input << (8 * i);
    }
}