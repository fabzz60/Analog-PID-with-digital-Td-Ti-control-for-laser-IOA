#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_memmap.h"
#include "inc/hw_ssi.h"
#include "driverlib/ssi.h"
#include "inc/hw_types.h"
#include "driverlib/debug.h"
#include "driverlib/fpu.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "utils/uartstdio.h"
#include "driverlib/interrupt.h"
#include "inc/hw_ints.h"
#include "grlib/grlib.h"
#include "driverlib/flash.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>     /* atoi */
/**********************************************************************************/
//PID control for Lock Laser IOA Fabrice Wiotte Atelier electronique Mars 2022//
       //programmed with IAR embedded Workbench IDE for ARM 8.32.1//
/*********************************************************************************/
//Table 1: Character LCD pins with 1 Controller
//**********************************************//
//1	VSS	Power supply (GND)
//2	VCC	Power supply (+5V)
//3	VEE	Contrast adjust
//4	RS	0 = Instruction input
//1 = Data input
//5	R/W	0 = Write to LCD module
//1 = Read from LCD module
//6	EN	Enable signal
//7	D0	Data bus line 0 (LSB)
//8	D1	Data bus line 1
//9	D2	Data bus line 2
//10	D3	Data bus line 3
//11	D4	Data bus line 4
//12	D5	Data bus line 5
//13	D6	Data bus line 6
//14	D7	Data bus line 7 (MSB)


//************ IF lcd  4 x 16**********************//
//  80 81 82 83 84 85 86 87 88 89 8A 8B 8C 8D 8F
//  C0 C1 C2 C3 C4 C5 C6 C7 C8 C9 CA CB CC CD CF
//  90 91 92 93 94 95 96 97 98 99 9A 9B 9C 9D 9F
//  D0 D1 D2 D3 D4 D5 D6 D7 D8 D9 DA DB DC DE DF
//***********************************************//
//*********************************************//
//#include "inc/tm4c123gh6pm.h"
//*****************************************************************************
// GPIO for port A
#define GPIO_PORTA_DATA_R       (*((volatile unsigned long *)0x400043FC)) // bits 7-0
#define GPIO_PORTA_DIR_R        (*((volatile unsigned long *)0x40004400))
#define GPIO_PORTA_AFSEL_R      (*((volatile unsigned long *)0x40004420))
#define GPIO_PORTA_DEN_R        (*((volatile unsigned long *)0x4000451C))
#define GPIO_PORTA_AMSEL_R      (*((volatile unsigned long *)0x40004528))
#define GPIO_PORTA_DR8R_R       (*((volatile unsigned long *)0x40004508))
#define GPIO_PORTA_PCTL_R       (*((volatile unsigned long *)0x4000452C))

// GPIO for port B
#define GPIO_PORTB_DATA_R       (*((volatile unsigned long *)0x400053FC))
#define GPIO_PORTB_DIR_R        (*((volatile unsigned long *)0x40005400))
#define GPIO_PORTB_PUR_R        (*((volatile unsigned long *)0x40005510))
#define GPIO_PORTB_DEN_R        (*((volatile unsigned long *)0x4000551C))
#define GPIO_PORTB_AMSEL_R      (*((volatile unsigned long *)0x40005528))
#define GPIO_PORTB_LOCK_R       (*((volatile unsigned long *)0x40005520))
#define GPIO_PORTB_CR_R         (*((volatile unsigned long *)0x40005524))
#define GPIO_PORTB_PCTL_R       (*((volatile unsigned long *)0x4000552C))
#define SYSCTL_RCGC2_R          (*((volatile unsigned long *)0x400FE108))
#define GPIO_PORTB_AFSEL_R      (*((volatile unsigned long *)0x40005420))

// GPIO for port C
#define GPIO_PORTC_DATA_R       (*((volatile unsigned long *)0x400063FC))
#define GPIO_PORTC_DIR_R        (*((volatile unsigned long *)0x40006400))
#define GPIO_PORTC_PUR_R        (*((volatile unsigned long *)0x40006510))
#define GPIO_PORTC_DEN_R        (*((volatile unsigned long *)0x4000651C))
#define GPIO_PORTC_AMSEL_R      (*((volatile unsigned long *)0x40006528))
#define GPIO_PORTC_LOCK_R       (*((volatile unsigned long *)0x40006520))
#define GPIO_PORTC_CR_R         (*((volatile unsigned long *)0x40006524))
#define GPIO_PORTC_PCTL_R       (*((volatile unsigned long *)0x4000652C))
#define SYSCTL_RCGC2_R          (*((volatile unsigned long *)0x400FE108))
#define GPIO_PORTC_AFSEL_R      (*((volatile unsigned long *)0x40006420))

// GPIO for port D
#define GPIO_PORTD_DATA_BITS_R  ((volatile unsigned long *)0x40007000)
#define GPIO_PORTD_DATA_R       (*((volatile unsigned long *)0x400073FC))
#define GPIO_PORTD_DIR_R        (*((volatile unsigned long *)0x40007400))
#define GPIO_PORTD_IS_R         (*((volatile unsigned long *)0x40007404))
#define GPIO_PORTD_IBE_R        (*((volatile unsigned long *)0x40007408))
#define GPIO_PORTD_IEV_R        (*((volatile unsigned long *)0x4000740C))
#define GPIO_PORTD_IM_R         (*((volatile unsigned long *)0x40007410))
#define GPIO_PORTD_RIS_R        (*((volatile unsigned long *)0x40007414))
#define GPIO_PORTD_MIS_R        (*((volatile unsigned long *)0x40007418))
#define GPIO_PORTD_ICR_R        (*((volatile unsigned long *)0x4000741C))
#define GPIO_PORTD_AFSEL_R      (*((volatile unsigned long *)0x40007420))
#define GPIO_PORTD_DR2R_R       (*((volatile unsigned long *)0x40007500))
#define GPIO_PORTD_DR4R_R       (*((volatile unsigned long *)0x40007504))
#define GPIO_PORTD_DR8R_R       (*((volatile unsigned long *)0x40007508))
#define GPIO_PORTD_ODR_R        (*((volatile unsigned long *)0x4000750C))
#define GPIO_PORTD_PUR_R        (*((volatile unsigned long *)0x40007510))
#define GPIO_PORTD_PDR_R        (*((volatile unsigned long *)0x40007514))
#define GPIO_PORTD_SLR_R        (*((volatile unsigned long *)0x40007518))
#define GPIO_PORTD_DEN_R        (*((volatile unsigned long *)0x4000751C))
#define GPIO_PORTD_LOCK_R       (*((volatile unsigned long *)0x40007520))
#define GPIO_PORTD_CR_R         (*((volatile unsigned long *)0x40007524))
#define GPIO_PORTD_AMSEL_R      (*((volatile unsigned long *)0x40007528))
#define GPIO_PORTD_PCTL_R       (*((volatile unsigned long *)0x4000752C))
#define GPIO_PORTD_ADCCTL_R     (*((volatile unsigned long *)0x40007530))
#define GPIO_PORTD_DMACTL_R     (*((volatile unsigned long *)0x40007534))

// GPIO for port E
#define GPIO_PORTE_DATA_BITS_R  ((volatile uint32_t *)0x40024000)
#define GPIO_PORTE_DATA_R       (*((volatile uint32_t *)0x400243FC))
#define GPIO_PORTE_DIR_R        (*((volatile uint32_t *)0x40024400))
#define GPIO_PORTE_IS_R         (*((volatile uint32_t *)0x40024404))
#define GPIO_PORTE_IBE_R        (*((volatile uint32_t *)0x40024408))
#define GPIO_PORTE_IEV_R        (*((volatile uint32_t *)0x4002440C))
#define GPIO_PORTE_IM_R         (*((volatile uint32_t *)0x40024410))
#define GPIO_PORTE_RIS_R        (*((volatile uint32_t *)0x40024414))
#define GPIO_PORTE_MIS_R        (*((volatile uint32_t *)0x40024418))
#define GPIO_PORTE_ICR_R        (*((volatile uint32_t *)0x4002441C))
#define GPIO_PORTE_AFSEL_R      (*((volatile uint32_t *)0x40024420))
#define GPIO_PORTE_DR2R_R       (*((volatile uint32_t *)0x40024500))
#define GPIO_PORTE_DR4R_R       (*((volatile uint32_t *)0x40024504))
#define GPIO_PORTE_DR8R_R       (*((volatile uint32_t *)0x40024508))
#define GPIO_PORTE_ODR_R        (*((volatile uint32_t *)0x4002450C))
#define GPIO_PORTE_PUR_R        (*((volatile uint32_t *)0x40024510))
#define GPIO_PORTE_PDR_R        (*((volatile uint32_t *)0x40024514))
#define GPIO_PORTE_SLR_R        (*((volatile uint32_t *)0x40024518))
#define GPIO_PORTE_DEN_R        (*((volatile uint32_t *)0x4002451C))
#define GPIO_PORTE_LOCK_R       (*((volatile uint32_t *)0x40024520))
#define GPIO_PORTE_CR_R         (*((volatile uint32_t *)0x40024524))
#define GPIO_PORTE_AMSEL_R      (*((volatile uint32_t *)0x40024528))
#define GPIO_PORTE_PCTL_R       (*((volatile uint32_t *)0x4002452C))
#define GPIO_PORTE_ADCCTL_R     (*((volatile uint32_t *)0x40024530))
#define GPIO_PORTE_DMACTL_R     (*((volatile uint32_t *)0x40024534))

// GPIO for port F
#define GPIO_PORTF_DATA_R       (*((volatile unsigned long *)0x400253FC))
#define GPIO_PORTF_DIR_R        (*((volatile unsigned long *)0x40025400))
#define GPIO_PORTF_PUR_R        (*((volatile unsigned long *)0x40025510))
#define GPIO_PORTF_DEN_R        (*((volatile unsigned long *)0x4002551C))
#define GPIO_PORTF_AMSEL_R      (*((volatile unsigned long *)0x40025528))
#define GPIO_PORTF_LOCK_R       (*((volatile unsigned long *)0x40025520))
#define GPIO_PORTF_CR_R         (*((volatile unsigned long *)0x40025524))
#define GPIO_PORTF_PCTL_R       (*((volatile unsigned long *)0x4002552C))
#define SYSCTL_RCGC2_R          (*((volatile unsigned long *)0x400FE108))
#define GPIO_PORTF_AFSEL_R      (*((volatile unsigned long *)0x40025420))

#define PA0 (*((volatile unsigned long *)0x40004004))
#define PA1 (*((volatile unsigned long *)0x40004008))
#define PA2 (*((volatile unsigned long *)0x40004010))
#define PA3 (*((volatile unsigned long *)0x40004020))
#define PA4 (*((volatile unsigned long *)0x40004040))
#define PA6 (*((volatile unsigned long *)0x40004100))
#define PA7 (*((volatile unsigned long *)0x40004200))

#define PF0 (*((volatile unsigned long *)0x40025004))
#define PF1 (*((volatile unsigned long *)0x40025008))    
#define PF2 (*((volatile unsigned long *)0x40025010))     
#define PF3 (*((volatile unsigned long *)0x40025020))
#define PF4 (*((volatile unsigned long *)0x40025040))

#define PC4 (*((volatile unsigned long *)0x40006040))
#define PC5 (*((volatile unsigned long *)0x40006080))  

#define PD0 (*((volatile unsigned long *)0x40007004))
#define PD1 (*((volatile unsigned long *)0x40007008))
#define PD2 (*((volatile unsigned long *)0x40007010))  
#define PD3 (*((volatile unsigned long *)0x40007020))   
#define PD4 (*((volatile unsigned long *)0x40007040))
#define PD5 (*((volatile unsigned long *)0x40007080))
#define PD7 (*((volatile unsigned long *)0x40007200))  

#define PE0 (*((volatile unsigned long *)0x40024004))   
#define PE1 (*((volatile unsigned long *)0x40024008))    
#define PE2 (*((volatile unsigned long *)0x40024010))   
#define PE3 (*((volatile unsigned long *)0x40024020))

#define RS (*((volatile unsigned long *)0x40004200))
#define E  (*((volatile unsigned long *)0x40004100))
#define D0 (*((volatile unsigned long *)0x40005004))
#define D1 (*((volatile unsigned long *)0x40005008))
#define D2 (*((volatile unsigned long *)0x40005010))
#define D3 (*((volatile unsigned long *)0x40005020))
#define D4 (*((volatile unsigned long *)0x40005040))
#define D5 (*((volatile unsigned long *)0x40005080))
#define D6 (*((volatile unsigned long *)0x40005100))
#define D7 (*((volatile unsigned long *)0x40005200))

// Flash ROM addresses must be 1k byte aligned, e.g., 0x8000, 0x8400, 0x8800...
#define FLASH                   0x10000  // location in flash to write; make sure no program code is in this block
#define FLASH1                  0x10010
#define FLASH2                  0x10020  
#define FLASH3                  0x10030
#define FLASH4                  0x10040  
#define FLASH5                  0x10050
//fonctions//
void interrupt_portD(void);
void interrupt_portE(void);
int Lcd_Cmd(int portB);
int lcd_display(char *disp);
int Lcd_Clear();
void Lcd_Init(void);
void start_affichage();
int i;
int j;
//*****************************************************************************
#ifdef DEBUG
void
__error__(char *pcFilename, uint32_t ui32Line)
{
}
#endif
void PortA_Init(void)
{
  volatile unsigned long delay;
  SYSCTL_RCGC2_R |= 0x00000001;     // 1) activate clock for Port A
  delay = SYSCTL_RCGC2_R;           // allow time for clock to start
  //GPIO_PORTA_AMSEL_R &= ~0x80;      // 3) disable analog on PA7
  GPIO_PORTA_PCTL_R &= ~0xF0000000; // 4) PCTL GPIO on PA7
  //GPIO_PORTA_DIR_R |= 0xC0;       // 5) PA7-PA6  out PA0 PA1 in
  GPIO_PORTA_DIR_R |= 0xFF; 
  //GPIO_PORTA_DIR_R |= 0xC8;          // PA7 PA6 PA3 out
  //GPIO_PORTA_AFSEL_R &= ~0x80;      // 6) disable alt funct on PA7
  GPIO_PORTA_DEN_R |= 0xFF;         // 7) enable digital I/O on PA7-PA6 
  //GPIO_PORTA_DEN_R |= 0xC8;
}
void PortB_Init(void)
{
  volatile unsigned long delay;
  SYSCTL_RCGC2_R |= 0x00000002;     // 1) activate clock for Port B
  delay = SYSCTL_RCGC2_R;           // allow time for clock to start
  GPIO_PORTB_AMSEL_R = 0x00;        // 3) disable analog on PB
  GPIO_PORTB_PCTL_R = 0x00000000;   // 4) PCTL GPIO on PB0
  GPIO_PORTB_DIR_R |= 0xFF;         // 5) PB0-PB7 is out
  //GPIO_PORTB_AFSEL_R &= ~0x01;      // 6) disable alt funct on PB0
  GPIO_PORTB_AFSEL_R &= ~0xFF;      // 6) disable alt funct on PB0-PB7
  GPIO_PORTB_DEN_R |= 0xFF;         // 7) enable digital I/O on PB0-PB7
}
void PortC_Init(void)
{
  volatile unsigned long delay;
  SYSCTL_RCGC2_R |= 0x00000004; // port C Clock Gating Control
  delay = SYSCTL_RCGC2_R;           // allow time for clock to start
  GPIO_PORTC_DIR_R |= 0x30; // make PC5 PC4 output (PC5 built-in LED)
  GPIO_PORTC_DEN_R |= 0x30; // enable digital I/O on PC5 PC4
}
void PortD_Init(void)
{
  volatile unsigned long delay;
  SYSCTL_RCGC2_R |= 0x00000008;     // 1) activate clock for Port D
  delay = SYSCTL_RCGC2_R;           // allow time for clock to start
                                    // 2) no need to unlock PD3-0
  GPIO_PORTD_AMSEL_R &= ~0x0F;      // 3) disable analog functionality on PD3-0
  GPIO_PORTD_AFSEL_R &= ~0x0F;      // 6) regular port function 
  GPIO_PORTD_PCTL_R &= ~0xF000FFFF; // 4) GPIO
  GPIO_PORTD_LOCK_R = 0x4C4F434B;   // 2) unlock GPIO Port D
  GPIO_PORTD_CR_R = 0x80;           // allow changes to PD7
  //GPIO_PORTD_DIR_R |= 0x00;         // 5) make PD3-0 in
  GPIO_PORTD_DIR_R |= 0x80;           //PD1 PD0  PD4 PD2 PD3 input
  //GPIO_PORTD_DEN_R |= 0x0F;         // 7) enable digital I/O on PD3-0
  GPIO_PORTD_DEN_R |= 0x9F;
}
void PortE_Init(void)
{
  volatile unsigned long delay;
  SYSCTL_RCGC2_R |= 0x10;           // activate clock for Port E
  delay = SYSCTL_RCGC2_R;           // wait 3-5 bus cycles
  GPIO_PORTE_DIR_R |= 0xFF;         // 
  GPIO_PORTE_AFSEL_R &= ~0xFF;      // not alternative
  //GPIO_PORTE_AMSEL_R &= ~0x0F;      // no analog
  GPIO_PORTE_PCTL_R &= ~0x0000FFFF; // bits for PE3,PE2,PE1,PE0
  GPIO_PORTE_DEN_R |= 0x0F;         // enable PE3,PE2,PE1,PE0
}
//// PF4 is input SW1 and PF2 is output Blue LED
void PortF_Init(void)
{ 
  volatile unsigned long delay;
  SYSCTL_RCGC2_R |= 0x00000020;     // 1) activate clock for Port F
  delay = SYSCTL_RCGC2_R;           // allow time for clock to start
  GPIO_PORTF_LOCK_R = 0x4C4F434B;   // 2) unlock GPIO Port F
  GPIO_PORTF_CR_R = 0x1F;           // allow changes to PF4-0
  // only PF0 needs to be unlocked, other bits can't be locked
  GPIO_PORTF_AMSEL_R = 0x00;        // 3) disable analog on PF
  GPIO_PORTF_PCTL_R = 0x00000000;   // 4) PCTL GPIO on PF4-0
  //GPIO_PORTF_DIR_R = 0x0E;          // 5) PF4,PF0 in, PF3-PF1 out 
  GPIO_PORTF_DIR_R = 0x0A;          // 5) PF4,PF2,PF0 in, PF3-1 out
  GPIO_PORTF_AFSEL_R = 0x00;        // 6) disable alt funct on PF7-0
  //GPIO_PORTF_PUR_R = 0x11;          // enable pull-up on PF0 and PF4
  GPIO_PORTF_DEN_R = 0x1F;          // 7) enable digital I/O on PF4-0
}
void PeripheralEnableInit(void)
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
}
int Lcd_Port(int portB)
{
     GPIO_PORTB_DATA_R = portB;
     return 0;
}  
//  Function for sending command to LCD
int Lcd_Cmd(int portB)
{
     RS = 0x00;             // => RS = 0
     Lcd_Port(portB);
     E  = 0x40;             // Enable = 1
     SysCtlDelay(15000);
     E  = 0x00;             // Enable = 0
     return 0;
}
//  Function for sending data to LCD
int Lcd_data(int portB)
{
    Lcd_Port(portB);
    RS   = 0x80;  // RS = 1
    E   = 0x40;    //Enable = 1
    SysCtlDelay(15000);
    E   = 0x00;    // Enable = 0
    return 0;
}
int Lcd_Clear()
{
	Lcd_Cmd(0);
        Lcd_Cmd(1);
        return 0;
}
void Lcd_display_off()
{
	Lcd_Cmd(0);
	Lcd_Cmd(0x0C);
}
void Lcd_Shift_Right()
{
	Lcd_Cmd(0x01);
	Lcd_Cmd(0x18);
}

void Lcd_Shift_Left()
{
	Lcd_Cmd(0x01);
	Lcd_Cmd(0x1C);
}
void LCD_busy()
{
 
     D7   = 0x80;           //Make D7th bit of LCD as i/p
     E   = 0x40;    //Enable = 1
     RS   = 0x80;  // RS = 1        //Selected command register
     //rw   = 1;           //We are reading
 
     while(D7)
     {          //read busy flag again and again till it becomes 0
            E   = 0x00;    //     //Enable H->L
            E   = 0x40;    //Enable = 1
     }
 
}
//  Function for initializing LCD
void Lcd_Init()
{ 
  SysCtlDelay(10000); 
  Lcd_Cmd(0x38);     //Function set: 2 Line, 8-bit, 5x7 dots
  SysCtlDelay(10000); 
  //Lcd_Cmd(0x0f);     // Display on Cursor blinking
  //SysCtlDelay(10000);
  Lcd_Cmd(0x0c);
  SysCtlDelay(10000);
  Lcd_Cmd(0x01);     //Clear LCD
  SysCtlDelay(10000);
  Lcd_Cmd(0x06);     //Entry mode, auto increment with no shift
  SysCtlDelay(10000);
  Lcd_Cmd(0x83);  // DDRAM addresses 0x80..0x8F + 0xC0..0xCF are used.
  SysCtlDelay(10000); 
}
//  Function for sending string to LCD
int lcd_display(char *disp)
{
    int x=0;
    while(disp[x]!=0)
    {
        Lcd_data(disp[x]);
        x++;
    }
    return 0;
}
int main(void)
{  
    SysCtlClockSet(SYSCTL_SYSDIV_2_5 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);  // Set the clocking to run at 80MHz from the PLL.
    PeripheralEnableInit();
    PortF_Init();
    PortA_Init();
    PortB_Init();
    PortC_Init();
    PortD_Init();
    PortE_Init();
    Lcd_Init();
    PE0 = 0x00;
    PE1 = 0x00;
    PE2 = 0x00;
    PE3 = 0x00;
    PD7 = 0x00;
    SysCtlDelay(20000);
    Lcd_Clear();
    SysCtlDelay(400000);
    Lcd_Cmd(0x80);
    lcd_display("   Welcome to");
    Lcd_Cmd(0xc0);
    lcd_display("CNRS-LPL-IG-UP13");
    SysCtlDelay(10000000);  //37.5ns * 50000000 = 1,875 s
    //Lcd_Clear();
    SysCtlDelay(400000);
    Lcd_Cmd(0x90);
    lcd_display("     Atelier");
    Lcd_Cmd(0xd2);
    lcd_display("Electronique");
    SysCtlDelay(5000000);  //37.5ns * 50000000 = 1,875 s
    
    
  while(1)
  {
    //SELECT rc intégration
    if(PD0 ==0x00 && PD1 ==0x00 && PD2 ==0x00)
    {
    PE3 = 0x00;
    PE2 = 0x00;
    PE1 = 0x00;
    PE0 = 0x00;
    PD7 = 0x00;
    Lcd_Clear();
    Lcd_Cmd(0x80);
    lcd_display(" RC integrateur");
    Lcd_Cmd(0xc0);
    lcd_display("  select 1->5");
    Lcd_Cmd(0x90);
    lcd_display("    Ti in s ");
    Lcd_Cmd(0xd2);
    lcd_display("Carte PID IOA");
    SysCtlDelay(40000000);
    }
    
    
    //R =1M and C =100nF  code 1
    if(PD0 ==0x01 && PD1 ==0x00 && PD2 ==0x00)
    {
    PE3 = 0x08;
    PE2 = 0x00;
    PE1 = 0x00;
    PE0 = 0x00;
    PD7 = 0x00;
    Lcd_Clear();
    Lcd_Cmd(0x80);
    lcd_display(" RC integrateur");
    Lcd_Cmd(0xc0);
    lcd_display("  R=1M  C=100nF");
    Lcd_Cmd(0x90);
    lcd_display("   Ti = 0.1s ");
    Lcd_Cmd(0xd1);
    lcd_display("Carte PID IOA");
    SysCtlDelay(40000000);
    }
    //R =1M and C =1uF code 2
    if(PD1 ==0x02 && PD0 ==0x00 && PD2 ==0x00)
    {
    PE2 = 0x04;
    PE3 = 0x00;
    PE1 = 0x00;
    PE0 = 0x00;
    PD7 = 0x00;
    Lcd_Clear();
    Lcd_Cmd(0x80);
    lcd_display(" RC integrateur");
    Lcd_Cmd(0xc0);
    lcd_display("  R=1M  C=1uF");
    Lcd_Cmd(0x90);
    lcd_display("    Ti = 1s ");
    Lcd_Cmd(0xd1);
    lcd_display("Carte PID IOA");
    SysCtlDelay(40000000);
    }
    //R =1M and C =3.3uF code 3
    if(PD0 ==0x01 && PD1 ==0x02 && PD2 ==0x00)
    {
    PE1 = 0x02;
    PE3 = 0x00;
    PE2 = 0x00;
    PE0 = 0x00;
    PD7 = 0x00;
    Lcd_Clear();
    Lcd_Cmd(0x80);
    lcd_display(" RC integrateur");
    Lcd_Cmd(0xc0);
    lcd_display("  R=1M  C=3.3uF");
    Lcd_Cmd(0x90);
    lcd_display("    Ti = 3.3s ");
    Lcd_Cmd(0xd1);
    lcd_display("Carte PID IOA");
    SysCtlDelay(40000000);
    }
    //R =1M and C =10uF code 4
    if(PD2 ==0x04 && PD0 ==0x00 && PD1 ==0x00)
    {
    PE0 = 0x01;
    PE1 = 0x00;
    PE3 = 0x00;
    PE2 = 0x00;
    PD7 = 0x00;
    Lcd_Clear();
    Lcd_Cmd(0x80);
    lcd_display(" RC integrateur");
    Lcd_Cmd(0xc0);
    lcd_display("  R=1M  C=10uF");
    Lcd_Cmd(0x90);
    lcd_display("    Ti = 10s ");
    Lcd_Cmd(0xd1);
    lcd_display("Carte PID IOA");
    SysCtlDelay(40000000);
    }
    //R =1M and C =33uF  code 5
    if(PD0 ==0x01 && PD1 ==0x00 && PD2 ==0x04) 
    {
    PD7 = 0x80;
    PE1 = 0x00;
    PE3 = 0x00;
    PE2 = 0x00;
    PE0 = 0x00;
    Lcd_Clear();
    Lcd_Cmd(0x80);
    lcd_display(" RC integrateur");
    Lcd_Cmd(0xc0);
    lcd_display("  R=1M  C=33uF");
    Lcd_Cmd(0x90);
    lcd_display("    Ti = 33s ");
    Lcd_Cmd(0xd1);
    lcd_display("Carte PID IOA");
    SysCtlDelay(40000000);
    }    

    //SELECT rc dérivateur
    if(PD3 ==0x00 && PD4 ==0x00 && PD5 ==0x00)
    {
    PA0 = 0x00;
    PA1 = 0x00;
    PA2 = 0x00;
    PA3 = 0x00;
    PA4 = 0x00;
    Lcd_Clear();
    Lcd_Cmd(0x80);
    lcd_display(" RC derivateur");
    Lcd_Cmd(0xc0);
    lcd_display("  select 1->5");
    Lcd_Cmd(0x90);
    lcd_display("    Td in s ");
    Lcd_Cmd(0xd1);
    lcd_display(" Carte PID IOA");
    SysCtlDelay(40000000);
    }
    //R =1M and C =33nF  code 1
    if(PD3 ==0x08 && PD4 ==0x00 && PD5 ==0x00)
    {
    PA0 = 0x01;
    PA1 = 0x00;
    PA2 = 0x00;
    PA3 = 0x00;
    PA4 = 0x00;
    Lcd_Clear();
    Lcd_Cmd(0x80);
    lcd_display(" RC derivateur");
    Lcd_Cmd(0xc0);
    lcd_display("  R=1M  C=33nF");
    Lcd_Cmd(0x90);
    lcd_display("  Td = 0.033s ");
    Lcd_Cmd(0xd1);
    lcd_display("Carte PID IOA");
    SysCtlDelay(40000000);
    }
    
    //SELECT rc dérivateur
    //R =1M and C =100nF  code 2
    if(PD3 ==0x00 && PD4 ==0x10 && PD5 ==0x00)
    {
    PA0 = 0x00;
    PA1 = 0x02;
    PA2 = 0x00;
    PA3 = 0x00;
    PA4 = 0x00;
    Lcd_Clear();
    Lcd_Cmd(0x80);
    lcd_display(" RC derivateur");
    Lcd_Cmd(0xc0);
    lcd_display("  R=1M  C=100nF");
    Lcd_Cmd(0x90);
    lcd_display("   Td = 0.1s ");
    Lcd_Cmd(0xd1);
    lcd_display("Carte PID IOA");
    SysCtlDelay(40000000);
    }
    
    //R =1M and C =330nF  code 3
    if(PD3 ==0x08 && PD4 ==0x10 && PD5 ==0x00)
    {
    PA0 = 0x00;
    PA1 = 0x00;
    PA2 = 0x04;
    PA3 = 0x00;
    PA4 = 0x00;
    Lcd_Clear();
    Lcd_Cmd(0x80);
    lcd_display(" RC derivateur");
    Lcd_Cmd(0xc0);
    lcd_display("  R=1M  C=330nF");
    Lcd_Cmd(0x90);
    lcd_display("   Td = 0.33s ");
    Lcd_Cmd(0xd1);
    lcd_display("Carte PID IOA");
    SysCtlDelay(40000000);
    }
    //R =1M and C =3uF  code 4
    if(PD3 ==0x00 && PD4 ==0x00 && PD5 ==0x20)
    {
    PA0 = 0x00;
    PA1 = 0x00;
    PA2 = 0x00;
    PA3 = 0x08;
    PA4 = 0x00;
    Lcd_Clear();
    Lcd_Cmd(0x80);
    lcd_display(" RC derivateur");
    Lcd_Cmd(0xc0);
    lcd_display("  R=1M  C=1uF");
    Lcd_Cmd(0x90);
    lcd_display("   Td = 1s ");
    Lcd_Cmd(0xd1);
    lcd_display("Carte PID IOA");
    SysCtlDelay(40000000);
    }
    //R =1M and C =3.3uF  code 5
    if(PD3 ==0x08 && PD4 ==0x00 && PD5 ==0x20)
    {
    PA0 = 0x00;
    PA1 = 0x00;
    PA2 = 0x00;
    PA3 = 0x00;
    PA4 = 0x10;
    Lcd_Clear();
    Lcd_Cmd(0x80);
    lcd_display(" RC derivateur");
    Lcd_Cmd(0xc0);
    lcd_display("  R=1M  C=3.3uF");
    Lcd_Cmd(0x90);
    lcd_display("   Td = 3.3s ");
    Lcd_Cmd(0xd1);
    lcd_display("Carte PID IOA");
    SysCtlDelay(40000000);
    }
  }
} 
