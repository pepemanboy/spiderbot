/* 
 * File:   main.c
 * Author: Pepemanboy
 *
 * Created on 25 de julio de 2016, 21:20
 * DSPIC30F5015
 */

int robot = 1;

// <editor-fold defaultstate="collapsed" desc="LIBRARIES">

#include <xc.h>
#include <stdio.h>
#include <stdlib.h>
#include "system.h"
#include <libpic30.h>
#include <math.h>
#include <i2c.h>

// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="I/O">

#define S1 PORTDbits.RD10
#define S2 PORTFbits.RF5
#define S3 PORTFbits.RF4
#define S4 PORTBbits.RB15
#define S5 PORTBbits.RB14
#define S6 PORTBbits.RB13
#define S7 PORTBbits.RB12

#define S14 PORTBbits.RB2
#define S15 PORTBbits.RB3
#define S16 PORTBbits.RB4
#define S17 PORTBbits.RB5
#define S18 PORTGbits.RG8
#define S19 PORTGbits.RG7
#define S20 PORTDbits.RD11

#define S8 11
#define S9 10
#define S10 9
#define S11 8
#define S12 0
#define S13 1

#define IR_SENSOR PORTBbits.RB6

#define LEFT1 LATCbits.LATC14
#define LEFT2 LATCbits.LATC13
#define LEFT_STBY LATDbits.LATD1

#define RIGHT1 LATDbits.LATD5
#define RIGHT2 LATDbits.LATD4
#define RIGHT_STBY LATDbits.LATD6

#define ENC_RIGHT PORTDbits.RD8
#define ENC_LEFT PORTFbits.RF6

#define TRIGGER PORTGbits.RG9

#define BUTTON1 PORTEbits.RE5
#define BUTTON2 PORTEbits.RE6
#define BUTTON3 PORTEbits.RE7

#define LED1 LATEbits.LATE2
#define LED2 LATEbits.LATE3
#define LED3 LATEbits.LATE4

#define PWM_LEFT 1
#define PWM_RIGHT 4
#define PWM_FAN 3

// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="VARIABLES">

//Fuzzy Logic
#define MAX(x, y) (((x) > (y)) ? (x) : (y))
#define MIN(x, y) (((x) < (y)) ? (x) : (y))

double kpMOM;
double kdMOM;
double bsMOM;

//Settings
int setting = 0;

//PWM
int period = 3199;
int period_fan = 5000;
int fanSpeed = 250;

//Main
int temp;

//ADC
int analogReading;
unsigned whiteTreshold = 700;

//Bluetooth
double aux;

//Gyro
int greading;
int gyroThreshold = 30000;


//Bluetooth
unsigned bt_char;
unsigned bt_num;

//PID Parameters
int baseSpeedC = 100;
int maxSpeedC = 100;
int kpC = 150;
int kdC = 900;
int baseSpeedS = 100;
int maxSpeedS = 100;
int kpS = 30;
int kdS = 800;
int baseSpeed = 100;
int maxSpeed = 100;
int kp = 30;
int kd = 800;
int minSpeed = 0;

//Control
double previousError = 0;
double error = 0;
double derivative = 0;
int deltaSpeed;
double leftMotor = 0;
double rightMotor = 0;
int curvature = 0;
int sensorCount = 0;

//I2C
char GYRO = 0b11010110;
int GYRO1 = 0b11010111;
int LOW_ODR = 0x39;
int CTRL4 = 0x23;
int CTRL1 = 0x20;
int ZL = 0x2C;
int ZH = 0x2D;

//Fan on
int previousTrigger = 0;

// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="LEDS">

/**
 * LED flashing to acknowledge something
 */
void acknowledge(int times){
    int i;
    for(i = 0; i < times*2; i++){
        LED1 = ~LED1;
        __delay_ms(200);
    } 
}

/**
 * Reset LEDs
 */
void resetLEDs(){
    LED1 = LED2 = LED3 = 1;
}

// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="PWM">

/**
 * Setup PWM
 */
void setupPWM(){
    T2CONbits.TON = 0; //Timer off
    T2CONbits.TCKPS = 0b00; //Prescaler of 1
    TMR2 = 0x00; //Reset timer
    PR2 = period; //Period
    T2CONbits.TON = 1; //Timer on    
    
    OC1CONbits.OCM = 0b000; //OC1 off
    OC1R = 0; //Initial duty 
    OC1RS = 0; //Duty
    OC1CONbits.OCTSEL = 0; //Select timer 2
    OC1CONbits.OCM = 0b110; //OC1 on in PWM mode
    
    OC4CONbits.OCM = 0b000; //OC4 off
    OC4R = 0; //Initial duty
    OC4RS = 0; //Duty
    OC4CONbits.OCTSEL = 0; //Select timer 2
    OC4CONbits.OCM = 0b110; //OC4 on in PWM mode
    
    T3CONbits.TON = 0; //Timer off
    T3CONbits.TCKPS = 0b10; //Prescaler of 64
    TMR3 = 0x00;
    PR3 = period_fan;
    T3CONbits.TON = 1;
    
    OC3CONbits.OCM = 0b000; //OC3 off
    OC3R = 0; //Initial duty
    OC3RS = 0; //Duty
    OC3CONbits.OCTSEL = 1;
    OC3CONbits.OCM = 0b110;
}

/**
 * Write duty to PWM channel
 * @param channel
 * @param duty
 */
void writePWM(int channel, double duty){
    switch(channel){
        case PWM_LEFT: OC1RS = duty*period; break; //Duty
        case PWM_RIGHT: OC4RS = duty*period; break;
    }
}

/**
 * Initialize Brushless ESC
 */
void initFan(){
    OC3RS = 300;
    __delay_ms(4000);
}

void enableFan(){
    int i = 0;
    for(i = 200; i < 200+fanSpeed; i ++){
        OC3RS = i;
        __delay_ms(20);
    }
}

void disableFan(){
    OC3RS = 200;
}

// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="MOTORS">

/**
Set direction for robot
 * 0 forward
 * 1 backward
 * 2 turn right
 * 3 turn left
**/
void setDirection(int direction){
    switch(direction){
        case 0: LEFT1 = 1; LEFT2 = 0; RIGHT1 = 0; RIGHT2 = 1;break;
        case 1: LEFT1 = 0; LEFT2 = 1; RIGHT1 = 1; RIGHT2 = 0;break;
        case 2: LEFT1 = 1; LEFT2 = 0; RIGHT1 = 1; RIGHT2 = 0;break;
        case 3: LEFT1 = 0; LEFT2 = 1; RIGHT1 = 0; RIGHT2 = 1;break;
    }    
}
/**
 * Enable drive motors
 */
void enableMotors(){
    LEFT_STBY = 1;
    RIGHT_STBY = 1;    
}

/**
 * Disable drive motors
 */
void disableMotors(){
    LEFT_STBY = 0;
    RIGHT_STBY = 0;
}

/**
Write speed for motors
**/
void writeSpeed(double left, double right){
    double max = maxSpeed/100.0;
    double min = minSpeed/100.0;
    setDirection(0);
    if(right > max){
        right = max;    
    }else if(right < min){
        right = min;
    }
    
    if(left > max){
        left = max;
    }else if(left < min){
        left = min;
    }
    if(right < 0 && left < 0){
        setDirection(1);
        right = -right;
        left = -left;
    }else if(right < 0){
        right = -right;
        setDirection(2);
    }else if(left < 0){
        left = -left;
        setDirection(3);
    }
    
    writePWM(PWM_RIGHT,right); 
    writePWM(PWM_LEFT,left); 
}

// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="I2C">

/**
 * Setup I2C comm
 */
void setupI2C(){
    int config1 = (I2C_ON & I2C_IDLE_CON & I2C_CLK_HLD &
             I2C_IPMI_DIS & I2C_7BIT_ADD &
             I2C_SLW_EN & I2C_SM_DIS &
             I2C_GCALL_DIS & I2C_STR_DIS &
             I2C_NACK & I2C_ACK_DIS & I2C_RCV_DIS &
             I2C_STOP_DIS & I2C_RESTART_DIS &
             I2C_START_DIS);
    int config2 = 0x91; //100 khz    
    OpenI2C(config1,config2);
    IdleI2C();
}

/**
 * Wait slave to acknowledge I2C reading
 */
void waitAckI2C(){    
    while(I2CSTATbits.TBF);  // 8 clock cycles
    while(!IFS0bits.MI2CIF); // Wait for 9th clock cycle
    IFS0bits.MI2CIF = 0;     // Clear interrupt flag
    while(I2CSTATbits.ACKSTAT);
}


/**
 * Write dat in reg in address
 * @param address
 * @param reg
 * @param dat
 * @return 0 if there was a problem, 1 if everything OK
 */
int writeByteI2C(int address,int reg, int dat){
    //Start
    IdleI2C();
    StartI2C();
    IdleI2C();
    //Write address
    if(MasterWriteI2C(address) != 0){
        return 0;
    }
    waitAckI2C();
    //Write register
    if(MasterWriteI2C(reg) != 0){
        return 0;
    }
    waitAckI2C();
    //Write data
    if(MasterWriteI2C(dat) != 0){
        return 0;
    }
    waitAckI2C();
    //Stop
    StopI2C();
    IdleI2C();
    return 1;
}

//Read byte from (address), registry (reg), with not acknowledge signal
int readByte(int address, int reg){
    unsigned char reading = 0;
    //Start
    IdleI2C();
    StartI2C();
    IdleI2C();
    //Write address
    MasterWriteI2C(address);
    waitAckI2C();
    //Write register
    MasterWriteI2C(reg);
    waitAckI2C();
    //Restart
    IdleI2C();
    StopI2C();
    IdleI2C();
    StartI2C();
    IdleI2C();
    //Write address (read)    
    MasterWriteI2C(address+1);
    waitAckI2C();
    //Read
    reading = MasterReadI2C();    	
    IdleI2C();
    //Not acknowledge signal
    NotAckI2C();
    IdleI2C();
    //Stop
    StopI2C();
    IdleI2C();
    return reading;
}

/**
 * Twos complement
 * @param msb
 * @param lsb
 * @return 
 */
int combineTwosComplement(int msb, int lsb){
    int number = 256*msb + lsb;
    if(number >= 32768){
        return number - 65536;
    }
    return number;
}

/**
 * Initialize gyro L3GD20H
 */
int initGyro(){
    //Low speed ODR disabled
     if(writeByteI2C(GYRO,LOW_ODR,0x00) == 0){
         return 0;
     }
     //+-250 dps full scale
     if(writeByteI2C(GYRO,CTRL4,0b00010000) == 0){
         return 0;
     }
     //DR = 01 (200 Hz ODR); BW = 10 (50 Hz bandwidth); PD = 1 (normal mode); Zen = 1 Yen = Xen = 0 (z enabled only)
     if(writeByteI2C(GYRO,CTRL1,0b01101100) == 0){
         return 0;
     }
     return 1;
}

int readGyro(){
    return combineTwosComplement(readByte(GYRO, ZH), readByte(GYRO ,ZL));
}

// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="UART">

/**
 * Setup UART
 */
void UART_Init(){
    U1MODEbits.STSEL = 0; //1 stop bit
    U1MODEbits.PDSEL = 0; //No parity, 8 data bits
    U1MODEbits.ABAUD = 0; //Auto baud disabled
    U1MODEbits.USIDL = 0; //Continue operation in IDLE mode
    U1MODEbits.LPBACK = 0; //Loopback disabled
    U1BRG = 103; //For baudrate of 9600
    U1MODEbits.UARTEN = 1; //Enable UART
    U1STAbits.UTXEN = 1; //Enable transmit    
}

/**
 * Read transmission flag
 */
int UART_Busy(){
    if(!IFS0bits.U1TXIF){
        return 1;
    }else{
        IFS0bits.U1TXIF = 0; //Clear flag
        return 0;
    }
}

/**
 * Write a byte to UART
 */
void UART_Write(char data){
    U1TXREG = data; //Write data to the register
    while(UART_Busy()); //Wait for transmission 
}

/**
 * Write a String to UART
 * @param String
 */
void UART_Write_Text(char *String){
    while (*String){
        UART_Write(*String++);
    }
}

unsigned UART_Read(){
    return U1RXREG;    
}

int UART_Data_Ready(){
    return U1STAbits.URXDA == 1;
}

// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="ANALOG INPUTS">

void ADC_Init(){
    ADCON1bits.ADON = 0; //Turn off ADC module
    ADCON3bits.ADRC = 0; //ADC clock derived from system clock
    ADCON3bits.SAMC = 0b1111; //Auto sample time at 31 TAD
    ADCON3bits.ADCS = 0b00010; // Conversion clock
    
    ADCON2bits.CSCNA = 0; //Scanning inputs off
    ADCON2bits.BUFS = 0; //One 16 bit buffer
    ADCON2bits.SMPI = 0b0000; 
    ADCON2bits.BUFM = 1; 
    ADCON2bits.ALTS = 0;
    
    ADCON1bits.ADSIDL = 0;
    ADCON1bits.FORM = 0b00;
    ADCON1bits.SSRC = 0b0000;
    ADCON1bits.ASAM = 0;   
}

/**
 * Select channel for ADC
 * @param channel
 */
void ADC_Select_Channel(unsigned char channel){
    ADCON1bits.ADON = 0;
    __delay_us(100);
    ADCHSbits.CH0SA = (channel & 0b00011111);
    __delay_us(100);
    ADCON1bits.ADON = 1;
}

/**
 * Sample an ADC Channel
 * @param channel
 * @return 
 */
unsigned int ADC_Get_Sample(unsigned char channel){
    ADC_Select_Channel(channel);
    ADCON1bits.SAMP = 1;
    while(!ADCON1bits.SAMP);
    ADCON1bits.SAMP = 0;
    while(ADCON1bits.SAMP);
    while(!ADCON1bits.DONE);
    return ADCBUF0;
}

// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="SENSORS">

/**
Read an analog sensor
**/
double readAnalog(unsigned char channel){
    analogReading = ADC_Get_Sample(channel);
    if(analogReading < whiteTreshold){
        return 0;
    }else{
        return (double)analogReading/1024.0;
    }
}

/**
Read a sensor
**/
double readSensor(unsigned char sensor){
    switch(sensor){
        case 1: if(S1) return 1; return 0;
        case 2: if(S2) return 1; return 0;
        case 3: if(S3) return 1; return 0;
        case 4: if(S4) return 1; return 0;
        case 5: if(S5) return 1; return 0;
        case 6: if(S6) return 1; return 0;
        case 7: if(S7) return 1; return 0;
        case 8: if(robot == 3) return readAnalog(S11); return readAnalog(S8);
        case 9: return readAnalog(S9);
        case 10: return readAnalog(S10);
        case 11: return readAnalog(S11);
        case 12: return readAnalog(S12);
        case 13: return readAnalog(S13);
        case 14: if(S14) return 1; return 0;
        case 15: if(S15) return 1; return 0;
        case 16: if(S16) return 1; return 0;
        case 17: if(S17) return 1; return 0;
        case 18: if(S18) return 1; return 0;
        case 19: if(S19) return 1; return 0;
        case 20: if(S20) return 1; return 0;
    }
    return 0;
}

// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="BLUETOOTH">

/**
Write bluetooth for diagnostics
**/
void bluetoothWrite(){
    int i;
    //Sensors
    UART_Write('s');
    for(i = 1; i <= 20; i ++){
        if(i == 10 || i == 11){
             continue;
        }
        aux = readSensor(i);
        if(aux != 0){
            UART_Write(1);
        }else{
            UART_Write(0);
        }
    }   
}

/**
Bluetooth read

STICK TO THE PROTOCOL
**/
void bluetoothRead(){
    int i, j;
    if(UART_Data_Ready() == 1){
        //Read character
        bt_char = UART_Read();
        //Make number
        bt_num = 0;
        for(i = 0, j = 100; i < 3; i ++, j/=10){
            while(UART_Data_Ready() == 0);
            bt_num += (UART_Read()-48)*j;
        }
        switch (bt_char){
          case 'a': baseSpeedC = bt_num; break;
          case 'b': maxSpeedC = bt_num; break;
          case 'c': kpC = bt_num; break;
          case 'd': kdC = bt_num; break;
          case 'e': baseSpeedS = bt_num; break;
          case 'f': maxSpeedS = bt_num; break;
          case 'g': kpS = bt_num; break;
          case 'h': kdS = bt_num; break;
          case 'j': fanSpeed = bt_num; break;
      }
        acknowledge(2);
    }
}

// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="SETUP">

/**
Configure I/O pins and analog inputs
**/
void configurePins(){
    TRISB = 0b1111111111111111; //0xFFF
    TRISC = 0;
    TRISD = 0b110100000000; //0xD00
    TRISE = 0b11100000; //0xE0
    TRISF = 0b1110100; //0x74
    TRISG = 0b1110000000; //0x380
    ADPCFG = 0b1111000010111100; //0xF0BC
}

/**
 * Initialize everything
 */
void init(){
    configurePins();
    resetLEDs();
    setupPWM();
    ADC_Init();
    UART_Init();
    setDirection(0);
    disableMotors(); 
    acknowledge(3);
    if(robot == 1 || 2){
        setupI2C();
        if(initGyro() == 1){
            acknowledge(3);
        }else{
            acknowledge(1);
        }
    }
    initFan();
    acknowledge(2);
}

// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="FUZZY">
//Outputs
double kpbajo(double kp){
    if(kp<5){
        return 1;
    }
    else if(kp<10){
        return -(1.0/5.0)*(kp-10);
    }else{
        return 0;
    }
}

double kpmedio(double kp){
    if(kp <5){
        return 0;
    }
    else if(kp<10){
        return 1.0/5.0*(kp-5);
    }
    else if(kp < 15){
        return -1.0/5.0*(kp-15);
    }
    else{
        return 0;
    }
}

double kpalto(double kp){
    if(kp<10){
        return 0;
    }
    else if(kp < 15){
        return 1.0/5.0*(kp-10);
    }
    else{
        return 1;
    }
}

double kdbajo(double kd){
    if(kd < 775){
        return 1;
    }
    else if(kd < 825){
        return -1.0/50.0*(kd-825);
    }
    else{
        return 0;
    }
}

double kdalto(double kd){
    if(kd < 775){
        return 0;
    }
    else if(kd < 825){
        return 1.0/50.0*(kd-775);
    }
    else{
        return 1;
    }
}

double speedbajo(double speed){
    if(speed < 30){
        return 1;
    }else if(speed < 60){
        return -1.0/30.0*(speed-60);
    }else{
        return 0;
    }
}

double speedmedio(double speed){
    if(speed < 60){
        return 0;
    }
    else if(speed < 70){
        return 1.0/10*(speed-60);
    }
    else if(speed < 80){
        return 1;
    }
    else if(speed < 90){
        return -1.0/10*(speed-90);
    }
    else{
        return 0;
    }
}

double speedalto(double speed){
    if(speed < 80){
        return 0;
    }
    else if(speed < 90){
        return 1.0/10*(speed-80);
    }
    else{
        return 1;
    }
}

// Inputs
double gBajo (double inputG) {
	if (inputG < 10000) return 1.0;
	else if (inputG >= 10000 && inputG < 20000) return (-1.0/10000*(inputG-20000)); //Ecuación
	else return 0.0;
}

double gAlto (double inputG) {
	if (inputG < 10000) return 0.0;
	else if (inputG >= 10000 && inputG < 20000) return (1.0/10000 *(inputG-10000)); //Ecuación
	else return 1.0;
}

double eBajo (double inputE) {
	if (inputE < 3.0) return 1.0;
	else if (inputE >= 3.0 && inputE < 5.0) return (-0.5 * inputE) + 2.5; //Ecuación
	else return 0.0;
}

double eMedio (double inputE) {
	if(inputE < 3){
		return 0;
	}else if(inputE < 5){
		return (1.0/2*(inputE-3));
	}else if(inputE < 8){
		return (-1.0/3*(inputE-8));
	}else{
		return 0;
	}
}

double eAlto (double inputE) {
	if(inputE < 5){
		return 0;
	}else if(inputE < 8){
		return (1.0/3*(inputE-5));
	}else{
		return 1;
	}
}

void fuzzy(double g, double e){
    
    double gbajo = gBajo(g);
    double galto = gAlto(g);
    double ebajo = eBajo(e);
    double emedio = eMedio(e);
    double ealto = eAlto(e);
    
    
    
    int i;
    
    //Kd
    
    double kd_r1 = gbajo;
    double kd_r2 = galto;
    
    double suma = 0;
    double elementos = 0;
    
    if(kd_r1 >= kd_r2){
        for(i = 700; i <= 900; i ++){
            if(kdbajo(i) >= kd_r1){
                suma += i;
               elementos ++;
            }
        }
    }
	if(kd_r2 >= kd_r1){        
        for(i = 700; i <= 900; i ++){
            if(kdalto(i) >= kd_r2){
                suma += i;
                elementos ++;
            }
        }
    }
    kd = suma/elementos;
    
    //Kp
    double kp_r1 = MAX(MAX(ebajo,emedio),gbajo);
    double kp_r2 = ealto;
    double kp_r3 = MAX(MAX(ebajo,emedio),galto);
    
    
    double max = MAX(MAX(kp_r1,kp_r2),kp_r3);
    
    suma = 0;
    elementos = 0;
    if(max == kp_r2){
        for(i = 3; i <= 20; i ++){
            if(kpbajo(i) >= kp_r2){
                suma += i;
               elementos ++;
            }
        }
    }
    if(max == kp_r1){
        for(i = 3; i <= 20; i ++){
            if(kpmedio(i) >= kp_r1){
                suma += i;
               elementos ++;
            }
        }
    }
	if(max == kp_r3){
        for(i = 3; i <= 20; i ++){
            if(kpalto(i) >= kp_r3){
                suma += i;
               elementos ++;
            }
        }
    }
    kp = suma/elementos;
    
    //Base speed
    
    //Base speed
    double bs_r1 = MAX(gbajo,ebajo);
    double bs_r2 = MAX(galto,emedio);
    double bs_r3 = ealto;
    
    max = MAX(MAX(bs_r1,bs_r2),bs_r3);
    suma = 0;
    elementos = 0;
    if(max == bs_r3){
        for(i = 0; i < 100; i ++){
            if(speedbajo(i) >= bs_r3){
                suma += i;
               elementos ++;
            }
        }
    }
    if(max == bs_r1){
        for(i = 0; i < 100; i ++){
            if(speedalto(i) >= bs_r1){
                suma += i;
               elementos ++;
            }
        }
    }
	if(max == bs_r2){
        for(i = 0; i < 100; i ++){
            if(speedmedio(i) >= bs_r2){
                suma += i;
               elementos ++;
            }
        }
    }
    baseSpeed = suma/elementos*0.8; 
}

// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="CONTROL">

/**
Read all sensors
**/
double weighedAverage(){
    int i;
    double weight = 0;
    double value = 0;
    for(i = 1; i <= 20; i++){
        if(i == 10 || i == 11){
            continue;
        }
        double reading = readSensor(i);
        weight += reading;
        if(i < 10){
            value += reading*(i-10);
        }else{                                                                                                            
            value += reading*(i-11);
        }
    }
    sensorCount = weight;
    if(weight != 0){
        return value/weight;
    }else{
        if (previousError<2 && previousError>-2){
            return 0;
        }
        return previousError;        
    }
}

void adjustCurves(){
    greading=0;
     if(greading > 30000 || greading < -30000 || error > 3 || error < -3){
        LED1 = 0;
        kd = kdC;
        kp = kpC;
        baseSpeed = baseSpeedC;
        maxSpeed = maxSpeedC;
        minSpeed = -100;
        if(error > 6 || error < 6){
            kp = 5;
            baseSpeed = 50;
        }
     }else{
        LED1 = 1;
        kd = kdS;
        kp = kpS;
        baseSpeed = baseSpeedS;
        maxSpeed = maxSpeedS;
        minSpeed = 0;
     }
}

/**
Control loop
**/
void controller(){
    error = weighedAverage();
    adjustCurves();
    fuzzy(abs(readGyro()), abs(error));
    derivative = error-previousError;
    deltaSpeed = kp*error+derivative*kd;
    writeSpeed((baseSpeed+deltaSpeed)*0.01,(baseSpeed-deltaSpeed)*0.01);
    previousError = error;
}

// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="CHANGE SETTINGS">

//CHANGE SETTINGS FOR ROBOT 1
void changeSetting1(){
    if(setting==3){
        setting = -1;
    }
    setting ++;
    
    if(setting == 0){
        baseSpeedC = 100;
        baseSpeedS = 100;
        
        LED1 = 1;
        LED2 = 1;
        LED3 = 0;
    }
    
    if(setting == 1){
        baseSpeedC = 90;
        baseSpeedS = 90;
        
        LED1 = 1;
        LED2 = 0;
        LED3 = 1;
    }
    
    if(setting == 2){
        baseSpeedC = 80;
        baseSpeedS = 80;
        
        LED1 = 1;
        LED2 = 0;
        LED3 = 0;
    }
    
    if(setting == 3){
        baseSpeedC = 70;
        baseSpeedS = 70;
        
        LED1 = 0;
        LED2 = 1;
        LED3 = 1;
    }
}

void checkButton(){
    if(BUTTON1 == 0){
            changeSetting1();
            __delay_ms(200);
    }
}


// </editor-fold>

/**
 * Main program
 */
int main(int argc, char** argv) {
    init();
    while(1){ 
        if(TRIGGER){
            if(previousTrigger == 0){
                enableFan();                    
                enableMotors();
                previousTrigger = 1;
            }
            controller(); 
        }else{
            disableMotors();
            bluetoothRead();      
            bluetoothWrite();
            disableFan();  
            __delay_ms(10);
            previousTrigger = 0;
            checkButton();
        }
    }
    return (EXIT_SUCCESS);
}

