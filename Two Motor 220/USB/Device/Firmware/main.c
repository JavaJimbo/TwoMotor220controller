/***************************************************************************************
 * Project:     Two Motor 220 - adapted from Quad Motor
 *              For PIC 32MX220F032D and Two Motor Board Rev 2.0 with USB
 * FileName:    main.c 
 *   
 * 8-25-17 JBS: `A IO pin changes from Quad Board
 *               Both motors and D flips tested, both UARTS work, USB works.
 * 8-26-17 JBS:  Got AD and switch inputs working and interrupt on change
 *               Note that ConfigAd() must be initialized before digital inputs.
 * 8-28-17 JBS:  Everything has been tested except I2C. USB clobbers Atmel Memory input.
 *               ConfigAd() must be called before initializing IO.
 ****************************************************************************************/
// #define USE_USB
#define USE_PWM
#define USE_AD

#define LED         LATBbits.LATB13
#define DISABLE_OUT PORTBbits.RB4
#define FAULT1_IN PORTCbits.RC7
#define FAULT2_IN PORTCbits.RC8

#define FORWARD 0
#define REVERSE 1

#define EncoderOne TMR1
#define EncoderTwo TMR4


#define ENC1_DIR PORTBbits.RB15
#define ENC2_DIR PORTAbits.RA10

#define PWM1 OC4RS
#define PWM2 OC3RS

#define DIR1_OUT LATAbits.LATA7
#define DIR2_OUT LATCbits.LATC5


#define STX 36
#define ETX 13
#define DLE 16
#define MAXPACKET 80
#define MAXVELOCITY 500
#define DRIVEDIRECT 145
#define ROOMBA 0
#define RASPI 240
#define ROBOTNIK 19
#define SETPID 69
#define START 128
#define STOP 173
#define POWERDOWN 133
#define RESET 7
#define SAFE 131
#define FULL 132
#define QUIT 128
#define SHUTDOWN 160

#define STANDBY 0
#define RUN 111


#define SYS_FREQ 60000000
#define GetPeripheralClock() SYS_FREQ 


#include "USB/usb.h"
#include "USB/usb_function_cdc.h"
#include "HardwareProfile.h"
#include "Delay.h"
#include "AT45DB161.h"
#include <plib.h>
#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>


/** CONFIGURATION **************************************************/
#pragma config UPLLEN   = ON            // USB PLL Enabled
#pragma config FPLLMUL  = MUL_15        // PLL Multiplier for 220 - yields 60 Mhz
#pragma config UPLLIDIV = DIV_2         // USB PLL Input Divider
#pragma config FPLLIDIV = DIV_2         // PLL Input Divider
#pragma config FPLLODIV = DIV_1         // PLL Output Divider
#pragma config FPBDIV   = DIV_1         // Peripheral Clock divisor
#pragma config FWDTEN   = OFF           // Watchdog Timer
#pragma config WDTPS    = PS1           // Watchdog Timer Postscale
#pragma config FCKSM    = CSDCMD        // Clock Switching & Fail Safe Clock Monitor
#pragma config OSCIOFNC = OFF           // CLKO Enable
#pragma config POSCMOD  = HS            // Primary Oscillator
#pragma config IESO     = OFF           // Internal/External Switch-over
#pragma config FSOSCEN  = OFF           // Secondary Oscillator Enable (KLO was off)
#pragma config FNOSC    = PRIPLL        // Oscillator Selection
#pragma config CP       = OFF           // Code Protect
#pragma config BWP      = OFF           // Boot Flash Write Protect
#pragma config PWP      = OFF           // Program Flash Write Protect
#pragma config ICESEL   = ICS_PGx3      // ICE/ICD Comm Channel Select
#pragma config JTAGEN   = OFF           // Use JTAG pins for normal IO
#pragma config DEBUG    = OFF            // Enable/disable debugging


/*** DEFINES *****************************************************************/
#define HOSTuart UART2
#define SYS_FREQ 60000000  // With 8 Mhz crystal and FPLLMUL = MUL_15
#define HOSTbits U2STAbits
#define HOST_VECTOR _UART_2_VECTOR     
#define MAXBUFFER 64

#define false FALSE
#define true TRUE

#define START_ONE 80
#define START_TWO 80
#define START_THREE 20
#define START_FOUR 20
#define TIMEOUT 200
#define UART_TIMEOUT 400
#define MAXBITLENGTH 20

#define STX 36
#define ETX 13
#define DLE 16

/** I N C L U D E S **********************************************************/
#include "GenericTypeDefs.h"
#include "Compiler.h"
#include "usb_config.h"
#include "USB/usb_device.h"
#include "USB/usb.h"
#include "HardwareProfile.h"


/** V A R I A B L E S ********************************************************/
#define MAXDATABYTES 64
unsigned char arrData[MAXDATABYTES];
unsigned char HOSTRxBuffer[MAXBUFFER + 1];
unsigned char HOSTTxBuffer[MAXBUFFER + 1];
// unsigned char HOSTRxBufferFull = FALSE;
unsigned int RxDataLength = 0;

unsigned char USBRxBuffer[MAXBUFFER];
unsigned char USBTxBuffer[MAXBUFFER];

unsigned short RxLength = 0;
unsigned short TxLength = 0;

unsigned short previousExpected = 0, numExpectedBytes = 0;
unsigned char error = 0;
unsigned char RXstate = 0;
unsigned char timeoutFlag = FALSE;
unsigned short numBytesReceived = 0;

/** P R I V A T E  P R O T O T Y P E S ***************************************/
// extern unsigned short CRCcalculate(unsigned char *message, unsigned char nBytes);
extern unsigned char getCRC8(unsigned char *ptrMessage, short numBytes);
static void InitializeSystem(void);
void ProcessIO(void);
void USBDeviceTasks(void);
void YourHighPriorityISRCode();
void YourLowPriorityISRCode();
// void USBCBSendResume(void);
void BlinkUSBStatus(void);
void UserInit(void);
void ConfigAd(void);

unsigned char PCAReadByte(unsigned char device, unsigned char PCAcontrolRegister, unsigned char *ptrData);
unsigned char PCAWriteByte(unsigned char device, unsigned char PCAcontrolRegister, unsigned char data);

unsigned char setPCA9685outputs(unsigned char device, unsigned short channel, unsigned short turnON, unsigned short turnOFF);
unsigned char initializePCA9685(unsigned char device);
unsigned int decodePacket(unsigned char *ptrInPacket, unsigned int numInBytes, unsigned char *ptrData);

void Halt(void);
unsigned char testRUN(short PWMvalue, unsigned char direction);
long getPositionError(short i, short targetSpeed);
long PIDcontrol(long error);
// void initializeErrorArrays(void);
unsigned char setMotorPWM(short side, short PWMvalue, unsigned char direction);
void putch(unsigned char ch);

unsigned int decodePacket(unsigned char *ptrInPacket, unsigned int inPacketSize, unsigned char *ptrData);

#define PWM_OFFSET 800

long kP = 50, kI = 0, kD = 0;

union convertType {
    unsigned char byte[2];
    short integer;
} convert;

#define NUM_ENCODERS 2
#define LEFT 0
#define RIGHT 1
long actualPos[NUM_ENCODERS], targetPos[NUM_ENCODERS];

long errorLeft, errorRight, Lcorr, Rcorr, PWMleft, PWMright, posLeft, posRight;

#define MAXPWM 1000
unsigned short dataLength = 0;
unsigned char hostChar = 0, UART1char = 0;
#define NUM_AD_INPUTS 2
unsigned short arrADreading[NUM_AD_INPUTS];

// 
unsigned short Timer5Counter = 0;
unsigned short PORTBreg = 0;
#define ATMEL_BUFFER 2

int main(void) {
    long wheel1 = 0, wheel2 = 0;
    long velocity1 = 0, velocity2 = 0;
    short displayCounter = 0;
    unsigned char displayMode = false;
    unsigned char ch, strPrint[MAXBUFFER];
    short strLength = 0;
    short outTestLength;
    unsigned char AtmelRAM[PAGESIZE] = "\r#2 Checking ATmel reads and writes";
    unsigned char AtmelFetchRAM[PAGESIZE];
    short pageNum = 0;
    unsigned short i;
    unsigned char runMode = false;

#ifdef USE_USB    
    USBDeviceInit();
#endif    
    UserInit();    
    
    outTestLength = strlen(AtmelRAM);

    DelayMs(10);
    printf("\r\rTWO MOTOR BOARD REV 2.0 FIRMWARE VERSION 1.0");

    DelayMs(10);

    printf("\rInitializing SPI");
    initAtmelSPI();
    /*
    printf("\rErasing Sector #0");
    EraseFLASHsector(0);
    printf("\rWriting page #%d to Buffer %d:", pageNum, ATMEL_BUFFER);
    WriteAtmelBuffer(ATMEL_BUFFER, AtmelRAM);
    printf("\rProgramming flash");
    ProgramFLASH(ATMEL_BUFFER, pageNum);
    */
    printf("\rTransferring flash");
    TransferFLASH(ATMEL_BUFFER, pageNum);
    printf("\rReading Buffer %d:", ATMEL_BUFFER);
    ReadAtmelBuffer(ATMEL_BUFFER, AtmelFetchRAM);
    printf(" %s", AtmelFetchRAM);
    printf("\rTESTING EVERYTHING ELSE");

    LED = 0;
    PWM1 = PWM2 = 0;
    DIR1_OUT = DIR2_OUT = FORWARD;

    DelayMs(200);

    strLength = sprintf(strPrint, "\rTESTING UART #1 COMMUNICATION");
    if (strLength < MAXBUFFER) {
        for (i = 0; i < strLength; i++) {
            while (!UARTTransmitterIsReady(UART1));
            ch = strPrint[i];
            UARTSendDataByte(UART1, ch);
        }
    }

    if (UART1char) {
        strLength = sprintf(strPrint, "\rUART 1: %c", UART1char);
        if (strLength < MAXBUFFER) {
            for (i = 0; i < strLength; i++) {
                while (!UARTTransmitterIsReady(UART1));
                UART1char = strPrint[i];
                UARTSendDataByte(UART1, UART1char);
            }
        }
    }

    PORTBreg = PORTB & 0x0003;
    while (1) {
        if (!Timer5Counter) {
            Timer5Counter = 10;
            // printf("\rPORTB: %d, AD #0: %d", PORTBreg, arrADreading[0]);
            // printf("\rFAULT :%X", PORTC & 0b110000000);
            if (runMode){
                PWM1 = PWM2 = arrADreading[0];
                mAD1IntEnable(INT_ENABLED);
            }
            else PWM1 = PWM2 = 0;
        }

        if (UART1char) {
            strLength = sprintf(strPrint, "\rCH: %c", UART1char);
            UART1char = 0;
            if (strLength < MAXBUFFER) {
                for (i = 0; i < strLength; i++) {
                    while (!UARTTransmitterIsReady(UART1));
                    ch = strPrint[i];
                    UARTSendDataByte(UART1, ch);
                }
            }
        }

        if (hostChar) {
            printf("\rCH: %c", hostChar);

            switch (hostChar) {
                case 'F':
                    runMode = true;
                    DIR1_OUT = DIR2_OUT = FORWARD;
                    printf(" FORWARD");
                    break;

                case 'R':
                    runMode = true;
                    DIR1_OUT = DIR2_OUT = REVERSE;
                    printf(" REVERSE");
                    break;

                case 'D':
                    if (displayMode) displayMode = FALSE;
                    else displayMode = TRUE;
                    if (displayMode) printf("\rDISPLAY MODE OFF");
                    else printf("\rDISPLAY MODE ON");
                    break;

                case ' ':
                    runMode = false;
                    PWM1 = PWM2 = 0;
                    printf(" HALT");
                    break;
            }
            hostChar = 0;
        }


        long tempEncoder;

        tempEncoder = (long) EncoderOne;
        EncoderOne = 0;
        velocity1 = velocity1 + tempEncoder;
        if (ENC1_DIR == FORWARD) wheel1 = wheel1 + tempEncoder;
        else wheel1 = wheel1 - tempEncoder;

        tempEncoder = (long) EncoderTwo;
        EncoderTwo = 0;
        velocity2 = velocity2 + tempEncoder;
        if (ENC2_DIR == FORWARD) wheel2 = wheel2 + tempEncoder;
        else wheel2 = wheel2 - tempEncoder;
        
        DelayMs(1);
        if (displayMode) {
            displayCounter++;
#define MAXLOOP 128
            if (displayCounter >= MAXLOOP) {
                printf("\rM1: %d V: %d, M2: %d V: %d", wheel1, velocity1, wheel2, velocity2);
                displayCounter = 0;
                velocity1 = velocity2 = 0;
            }
        } else displayCounter = 0;

#ifdef USE_USB           
#if defined(USB_INTERRUPT)
        if (USB_BUS_SENSE && (USBGetDeviceState() == DETACHED_STATE)) {
            USBDeviceAttach();
        }
#endif

#if defined(USB_POLLING)
        // Check bus status and service USB interrupts.
        USBDeviceTasks(); // Interrupt or polling method.  If using polling, must call
        // this function periodically.  This function will take care
        // of processing and responding to SETUP transactions 
        // (such as during the enumeration process when you first
        // plug in).  USB hosts require that USB devices should accept
        // and process SETUP packets in a timely fashion.  Therefore,
        // when using polling, this function should be called 
        // regularly (such as once every 1.8ms or faster** [see 
        // inline code comments in usb_device.c for explanation when
        // "or faster" applies])  In most cases, the USBDeviceTasks() 
        // function does not take very long to execute (ex: <100 
        // instruction cycles) before it returns.
#endif				  
        // Application-specific tasks.
        // Application related code may be added here, or in the ProcessIO() function.
        ProcessIO();
#endif
    } // end while(1)

#ifdef TEST_EEPROM    
    ANSELBbits.ANSB2 = 0;
    ANSELBbits.ANSB3 = 0;
    DelayMs(200);

#endif    
} // end main()

void putch(unsigned char ch) {
    while (!IFS1bits.U2TXIF); // set when register is empty 
    U2TXREG = ch;
}

/*
#define MULTIPLIER 100
long getPositionError(short wheel, short targetSpeed) {
    long error, newPos, lngTargetSpeed;

    if (wheel >= NUM_ENCODERS) return (0);

    if (wheel == LEFT) {
        newPos = (long) LEFTREARENC;
        //LEFTREARENC = 0;
    } else {
        newPos = (long) RIGHTREARENC;
        //RIGHTREARENC = 0;        
    }

    lngTargetSpeed = (long) abs(targetSpeed);

    actualPos[wheel] = (newPos * (long) MULTIPLIER);
    targetPos[wheel] = targetPos[wheel] + lngTargetSpeed;

    if (wheel == LEFT) posLeft = actualPos[wheel];
    else posRight = actualPos[wheel];

    error = actualPos[wheel] - targetPos[wheel];

    if (wheel == LEFT) {
        errorLeft = error / MULTIPLIER;
    } else {
        errorRight = error / MULTIPLIER;
    }

    return (error);
}

#define DIVIDER 10000

long PIDcontrol(long error) {
    long PIDcorrection;
    long PCorr = 0;

    PCorr = error * kP;
    PIDcorrection = PCorr;
    PIDcorrection = PIDcorrection / DIVIDER;
    if (PIDcorrection > PWM_MAX) PIDcorrection = PWM_MAX;
    if (PIDcorrection < -PWM_MAX) PIDcorrection = -PWM_MAX;

    return (PIDcorrection);
}
 */

/******************************************************************************
 * Function:        void UserInit(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        This routine should take care of all of the demo code
 *                  initialization that is required.
 *
 * Note:            
 * PPSInput(2,U2RX,RPA9);       // Assign U2RX to pin RPA09
 * PPSInput(3,U2CTS,RPC3);      // Assign U2CTS to pin RPC3
 * PPSOutput(4,RPC4,U2TX);      // Assign U2TX to pin RPC4
 * PPSOutput(1,RPB15,U2RTS);    // Assign U2RTS to pin RPB15    
 *
 *****************************************************************************/


void UserInit(void) {
    unsigned char ch;
    unsigned short dummyRead;

    mJTAGPortEnable(false);

#ifdef USE_AD    
    ConfigAd();
#endif            

    // DIGITAL OUTPUTS: 
    PORTSetPinsDigitalOut(IOPORT_A, BIT_7);
    PORTSetPinsDigitalOut(IOPORT_B, BIT_4 | BIT_5 | BIT_7 | BIT_13 | BIT_14); // Added BIT 13 for LED out and BIT 7 for Atmel CS
    PORTSetPinsDigitalOut(IOPORT_C, BIT_5 | BIT_9); // Added BIT 9 for Atmel WP write protect    

    PPSOutput(2, RPB5, SDO1);    
    PPSInput(2, SDI1, RPA9);

    ATMEL_WRITE_PROTECT = 1; // Enable EEPROM write protection at startup, and disable chip select
    ATMEL_CS = 1;

    DIR1_OUT = DIR2_OUT = 0;

    // Enable analog inputs
    ANSELCbits.ANSC0 = 1; // AN6  
    ANSELCbits.ANSC1 = 1; // AN7     

    // Disable analog on digital inputs:    
    ANSELBbits.ANSB15 = 0;
    ANSELBbits.ANSB14 = 0;
    ANSELBbits.ANSB0 = 0;
    ANSELBbits.ANSB1 = 0;
    ANSELBbits.ANSB2 = 0;
    ANSELCbits.ANSC3 = 0;


    PORTSetPinsDigitalIn(IOPORT_A, BIT_10); // For Encoder 2 Direction input `A        
    PORTSetPinsDigitalIn(IOPORT_B, BIT_0 | BIT_1 | BIT_15); // Removed BIT_13   `A    
    PORTSetPinsDigitalIn(IOPORT_C, BIT_7 | BIT_8); // Changed to input  `A          

    LED = 0;
    DISABLE_OUT = 0;

    // Set up UART #1
    PPSOutput(1, RPB3, U1TX);
    PPSInput(3, U1RX, RPC3);

    UARTConfigure(UART1, UART_ENABLE_HIGH_SPEED | UART_ENABLE_PINS_TX_RX_ONLY);
    UARTSetFifoMode(UART1, UART_INTERRUPT_ON_RX_NOT_EMPTY); //  | UART_INTERRUPT_ON_TX_DONE  
    UARTSetLineControl(UART1, UART_DATA_SIZE_8_BITS | UART_PARITY_NONE | UART_STOP_BITS_1);
    UARTSetDataRate(UART1, SYS_FREQ, 57600);
    UARTEnable(UART1, UART_ENABLE_FLAGS(UART_PERIPHERAL | UART_RX | UART_TX));

    // Configure UART #1 Interrupts
    INTEnable(INT_U1TX, INT_DISABLED);
    INTEnable(INT_U1RX, INT_ENABLED);
    INTSetVectorPriority(INT_VECTOR_UART(UART1), INT_PRIORITY_LEVEL_2);
    INTSetVectorSubPriority(INT_VECTOR_UART(UART1), INT_SUB_PRIORITY_LEVEL_0);

    // Set up UART #2 as HOST UART
    PPSOutput(4, RPC2, U2TX);
    PPSInput(2, U2RX, RPA8);

    UARTConfigure(HOSTuart, UART_ENABLE_HIGH_SPEED | UART_ENABLE_PINS_TX_RX_ONLY);
    UARTSetFifoMode(HOSTuart, UART_INTERRUPT_ON_RX_NOT_EMPTY); //  | UART_INTERRUPT_ON_TX_DONE  
    UARTSetLineControl(HOSTuart, UART_DATA_SIZE_8_BITS | UART_PARITY_NONE | UART_STOP_BITS_1);
    UARTSetDataRate(HOSTuart, SYS_FREQ, 57600);
    UARTEnable(HOSTuart, UART_ENABLE_FLAGS(UART_PERIPHERAL | UART_RX | UART_TX));

    // Configure UART #2 Interrupts
    INTEnable(INT_U2TX, INT_DISABLED);
    INTEnable(INT_U2RX, INT_ENABLED);
    INTSetVectorPriority(INT_VECTOR_UART(HOSTuart), INT_PRIORITY_LEVEL_2);
    INTSetVectorSubPriority(INT_VECTOR_UART(HOSTuart), INT_SUB_PRIORITY_LEVEL_0);

    do {
        ch = UARTGetDataByte(HOSTuart);
    } while (ch);

    // Set counter inputs    
    PPSInput(3, T4CK, RPB2);

    // Set up timers as counters
    T1CON = 0x00;
    T1CONbits.TCS = 1; // Use external counter as input source (motor encoder)
    T1CONbits.TCKPS1 = 0; // 1:1 Prescaler
    T1CONbits.TCKPS0 = 0;
    T1CONbits.TSYNC = 1;
    PR1 = 0xFFFF;
    T1CONbits.TON = 1; // Let her rip 

    T4CON = 0x00;
    T4CONbits.TCS = 1; // Use external counter as input source (motor encoder)
    T4CONbits.TCKPS2 = 0; // 1:1 Prescaler
    T4CONbits.TCKPS1 = 0;
    T4CONbits.TCKPS0 = 0;
    T4CONbits.T32 = 0; // TMRx and TMRy form separate 16-bit timers
    PR4 = 0xFFFF;
    T4CONbits.TON = 1; // Let her rip     

    // Set up Timer 5 interrupt with a priority of 2
    ConfigIntTimer5(T5_INT_ON | T5_INT_PRIOR_2);
    OpenTimer5(T5_ON | T5_SOURCE_INT | T5_PS_1_8, 7500);

    // Set up Timer 2 for PWM time base    
    T2CON = 0x00;
    T2CONbits.TCKPS2 = 0; // 1:1 Prescaler
    T2CONbits.TCKPS1 = 0;
    T2CONbits.TCKPS0 = 0;
    PR2 = 3000; // Use 50 microsecond rollover for 20 khz
    T2CONbits.TON = 1; // Let her rip       

    // Set up PWM OC3
    PPSOutput(4, RPC4, OC3);
    OC3CON = 0x00;
    OC3CONbits.OC32 = 0; // 16 bit PWM
    OC3CONbits.ON = 1; // Turn on PWM
    OC3CONbits.OCTSEL = 0; // Use Timer 2 as PWM time base
    OC3CONbits.OCM2 = 1; // PWM mode enabled, no fault pin
    OC3CONbits.OCM1 = 1;
    OC3CONbits.OCM0 = 0;
    OC3RS = 0;

    // Set up PWM OC4 on D6 on the Olimex 220 board:
    PPSOutput(3, RPC6, OC4);
    OC4CON = 0x00;
    OC4CONbits.OC32 = 0; // 16 bit PWM
    OC4CONbits.ON = 1; // Turn on PWM
    OC4CONbits.OCTSEL = 0; // Use Timer 2 as PWM time base
    OC4CONbits.OCM2 = 1; // PWM mode enabled, no fault pin
    OC4CONbits.OCM1 = 1;
    OC4CONbits.OCM0 = 0;
    OC4RS = 0;

    // Set up interrupt on change for the PORT B input pins RB0 and RB1
    CNCONBbits.ON = 1; // CN is enabled
    CNCONBbits.SIDL = 0; // CPU Idle does not affect CN operation
    CNENBbits.CNIEB0 = 1; // Enable RB0 change notice    
    CNENBbits.CNIEB1 = 1; // Enable RB1 change notice

    // Read port B to clear mismatch condition
    dummyRead = PORTB;

    // Clear CN interrupt flags
    mCNSetIntPriority(2);
    mCNSetIntSubPriority(2);
    mCNBClearIntFlag();
    mCNBIntEnable(BIT_0 | BIT_1);

    // Turn on the interrupts
    INTEnableSystemMultiVectoredInt();


}//end UserInit

void __ISR(HOST_VECTOR, ipl2) IntHostUartHandler(void) {
    static unsigned char startFlag = false;
    static unsigned char escapeFlag = false;
    static unsigned short RxIndex = 0;
    unsigned char dummy;

    if (INTGetFlag(INT_SOURCE_UART_RX(HOSTuart))) {
        INTClearFlag(INT_SOURCE_UART_RX(HOSTuart));
        if (HOSTbits.OERR || HOSTbits.FERR) {
            if (UARTReceivedDataIsAvailable(HOSTuart))
                dummy = UARTGetDataByte(HOSTuart);
            HOSTbits.OERR = 0;
            RxIndex = 0;
        }

        if (UARTReceivedDataIsAvailable(HOSTuart)) {
            hostChar = toupper(UARTGetDataByte(HOSTuart));
            /*
            if (RxIndex < MAXBUFFER) HOSTRxBuffer[RxIndex++] = ch;              
            if (ch == '\r') {
                HOSTRxBufferFull = TRUE;
                HOSTRxBuffer[RxIndex] = '\0';
                RxIndex = 0;
            }
             */
        }

    }
}

#define UART1bits U1STAbits

void __ISR(_UART_1_VECTOR, ipl2) IntUart1Handler(void) {
    unsigned char dummy;

    if (INTGetFlag(INT_SOURCE_UART_RX(UART1))) {
        INTClearFlag(INT_SOURCE_UART_RX(UART1));
        if (UART1bits.OERR || UART1bits.FERR) {
            if (UARTReceivedDataIsAvailable(UART1))
                dummy = UARTGetDataByte(UART1);
            UART1bits.OERR = 0;
        }

        if (UARTReceivedDataIsAvailable(UART1)) {
            UART1char = toupper(UARTGetDataByte(UART1));
        }
    }
}

/*
        if (UARTReceivedDataIsAvailable(HOSTuart)) {
            ch = UARTGetDataByte(HOSTuart);

            // Store next char if packet is valid and board number matches
            if (startFlag && RxIndex < MAXBUFFER) HOSTRxBuffer[RxIndex] = ch;

            // If preceding character wasn't an escape char:
            // check whether it is STX, ETX or DLE,
            // otherwise if board number matches then store and advance for next char
            if (escapeFlag == false || startFlag == false) {
                if (ch == DLE) escapeFlag = true;
                else if (ch == STX) {
                    RxIndex = 0;
                    startFlag = true;
                } else if (ch == ETX) {
                    startFlag = false;
                    dataLength = RxIndex;
                    RxIndex = 0;
                } else if (startFlag) RxIndex++;
            }// Otherwise if preceding character was an escape char:	
            else {
                escapeFlag = false;
                if (startFlag) RxIndex++;
            }
        }
        if (INTGetFlag(INT_SOURCE_UART_TX(HOSTuart))) {
            INTClearFlag(INT_SOURCE_UART_TX(HOSTuart));
        }
    }
}*/

/********************************************************************
 * Function:        void ProcessIO(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        This function is a place holder for other user
 *                  routines. It is a mixture of both USB and
 *                  non-USB tasks.
 *
 * Note:            None
 *******************************************************************/
#ifdef USE_USB

#define MAXUSBBUFFER 64

void ProcessIO(void) {
    static unsigned char ch, USBTxIndex = 0;
    unsigned short i, length;
    BYTE numBytesRead;
    unsigned char strUSBxmit[MAXUSBBUFFER], strUSBRxPacket[MAXUSBBUFFER];
    static unsigned short USBCounter = 0;

    // Blink the LEDs according to the USB device status
    // BlinkUSBStatus();

    // User Application USB tasks
    if ((USBDeviceState < CONFIGURED_STATE) || (USBSuspendControl == 1)) {
        LED = 0;
        return;
    }

    USBCounter++;
    if (USBCounter > 200) {
        USBCounter = 0;
        if (LED) LED = 0;
        else LED = 1;
    }

    numBytesRead = getsUSBUSART(strUSBRxPacket, MAXUSBBUFFER);
    if (numBytesRead) {
        for (i = 0; i < numBytesRead; i++) {
            ch = strUSBRxPacket[i];
            if (USBTxIndex < MAXBUFFER - 2) HOSTTxBuffer[USBTxIndex++] = ch;
            if (ch == '\r') {
                HOSTTxBuffer[USBTxIndex] = '\0';
                printf("\rUSB RECEIVED: %s", HOSTTxBuffer);
                USBTxIndex = 0;
                if (USBUSARTIsTxTrfReady()) {
                    length = sprintf(strUSBxmit, "\rUSB received: %s", HOSTTxBuffer);
                    if (length < MAXUSBBUFFER) putUSBUSART(strUSBxmit, length);
                }
            }
        }
        numBytesRead = 0;
    }
    CDCTxService();
} //end ProcessIO




#endif

/********************************************************************
 * Function:        static void InitializeSystem(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        InitializeSystem is a centralize initialization
 *                  routine. All required USB initialization routines
 *                  are called from here.
 *
 *                  User application initialization routine should
 *                  also be called from here.                  
 *
 * Note:            None
 *******************************************************************/
static void InitializeSystem(void) {
#ifdef USE_USB    
#if defined(USE_USB_BUS_SENSE_IO)
    tris_usb_bus_sense = INPUT_PIN; // See HardwareProfile.h
#endif

#if defined(USE_SELF_POWER_SENSE_IO)
    tris_self_power = INPUT_PIN; // See HardwareProfile.h
#endif
#endif

#ifdef USE_USB
    USBDeviceInit();
#endif
    
    UserInit();
}//end InitializeSystem

void USBCBSuspend(void) {
    ;
}

void USBCBWakeFromSuspend(void) {
    ;
}

void USBCB_SOF_Handler(void) {
    ;
}

void USBCBErrorHandler(void) {
    ;
}

void USBCBCheckOtherReq(void) {
    USBCheckCDCRequest();
}

void USBCBStdSetDscHandler(void) {
    ;
}

void USBCBInitEP(void) {
    CDCInitEP();
}

/*
void USBCBSendResume(void) {
    static WORD delay_count;

    if (USBGetRemoteWakeupStatus() == TRUE) {
        if (USBIsBusSuspended() == TRUE) {
            USBMaskInterrupts();
            USBCBWakeFromSuspend();
            USBSuspendControl = 0;
            USBBusIsSuspended = FALSE; //So we don't execute this code again, 
            //until a new suspend condition is detected.
            delay_count = 3600U;
            do {
                delay_count--;
            } while (delay_count);
            USBResumeControl = 1; // Start RESUME signaling
            delay_count = 1800U; // Set RESUME line for 1-13 ms
            do {
                delay_count--;
            } while (delay_count);
            USBResumeControl = 0; //Finished driving resume signalling

            USBUnmaskInterrupts();
        }
    }
}
 */

#if defined(ENABLE_EP0_DATA_RECEIVED_CALLBACK)

void USBCBEP0DataReceived(void) {
}
#endif

BOOL USER_USB_CALLBACK_EVENT_HANDLER(int event, void *pdata, WORD size) {
    switch (event) {
        case EVENT_TRANSFER:
            //Add application specific callback task or callback function here if desired.
            break;
        case EVENT_SOF:
            USBCB_SOF_Handler();
            break;
        case EVENT_SUSPEND:
            USBCBSuspend();
            break;
        case EVENT_RESUME:
            USBCBWakeFromSuspend();
            break;
        case EVENT_CONFIGURED:
            USBCBInitEP();
            break;
        case EVENT_SET_DESCRIPTOR:
            USBCBStdSetDscHandler();
            break;
        case EVENT_EP0_REQUEST:
            USBCBCheckOtherReq();
            break;
        case EVENT_BUS_ERROR:
            USBCBErrorHandler();
            break;
        case EVENT_TRANSFER_TERMINATED:
            break;
        default:
            break;
    }
    return TRUE;
}


#ifdef USE_AD

void ConfigAd(void) {

    //mPORTCSetPinsAnalogIn(BIT_0 | BIT_1);
    //mPORTBSetPinsDigitalOut(BIT_0 | BIT_1);

    // ---- configure and enable the ADC ----

    // ensure the ADC is off before setting the configuration
    CloseADC10();

    // define setup parameters for OpenADC10
    //                 Turn module on | ouput in integer | trigger mode auto | enable autosample
#define PARAM1  ADC_MODULE_ON | ADC_FORMAT_INTG | ADC_CLK_AUTO | ADC_AUTO_SAMPLING_ON

    // ADC ref external    | disable offset test    | enable scan mode | perform  samples | use dual buffers | use only mux A
#define PARAM2  ADC_VREF_AVDD_AVSS | ADC_OFFSET_CAL_DISABLE | ADC_SCAN_ON | ADC_SAMPLES_PER_INT_4 | ADC_ALT_BUF_ON | ADC_ALT_INPUT_OFF

    //                   use ADC internal clock | set sample time
#define PARAM3  ADC_CONV_CLK_INTERNAL_RC | ADC_SAMPLE_TIME_31

    //  set AM7 (A1 on Olimex 220 board) input to analog
    // #define PARAM4    ENABLE_AN0_ANA | ENABLE_AN1_ANA| ENABLE_AN2_ANA | ENABLE_AN3_ANA
#define PARAM4    ENABLE_AN6_ANA | ENABLE_AN7_ANA 


    // USE AN6 and AN7    
#define PARAM5 SKIP_SCAN_AN0 | SKIP_SCAN_AN1 |SKIP_SCAN_AN2 | SKIP_SCAN_AN3 |\
    SKIP_SCAN_AN4 | SKIP_SCAN_AN5 |\
    SKIP_SCAN_AN8 | SKIP_SCAN_AN9 | SKIP_SCAN_AN10 |\
    SKIP_SCAN_AN11 | SKIP_SCAN_AN12 | SKIP_SCAN_AN13 | SKIP_SCAN_AN14 | SKIP_SCAN_AN15


    // set negative reference to Vref for Mux A
    SetChanADC10(ADC_CH0_NEG_SAMPLEA_NVREF);

    // open the ADC
    OpenADC10(PARAM1, PARAM2, PARAM3, PARAM4, PARAM5);

    ConfigIntADC10(ADC_INT_PRI_2 | ADC_INT_SUB_PRI_2 | ADC_INT_ON);

    // clear the interrupt flag
    mAD1ClearIntFlag();

    // Enable the ADC
    EnableADC10();
}

void __ISR(_ADC_VECTOR, ipl6) ADHandler(void) {
    unsigned short offSet;
    unsigned char i;

    mAD1IntEnable(INT_DISABLED);
    mAD1ClearIntFlag();

    // Determine which buffer is idle and create an offset
    offSet = 8 * ((~ReadActiveBufferADC10() & 0x01));

    for (i = 0; i < NUM_AD_INPUTS; i++)
        arrADreading[i] = (unsigned short) ReadADC10(offSet + i); // read the result of channel 0 conversion from the idle buffer
}
#endif

void __ISR(_TIMER_5_VECTOR, ipl2) Timer5Handler(void) {
    mT5ClearIntFlag(); // Clear interrupt flag
    if (Timer5Counter) Timer5Counter--;
}

void __ISR(_CHANGE_NOTICE_VECTOR, ipl2) ChangeNotice_Handler(void) {

    // Step #1 - always clear the mismatch condition first
    PORTBreg = PORTB & 0x0003;

    // Step #2 - then clear the interrupt flag
    mCNBClearIntFlag();

}