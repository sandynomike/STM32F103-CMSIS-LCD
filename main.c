//  STM32F103-CMSIS-LC

//  Target Microcontroller: STM32F103 (Blue Pill)
//  Mike Shegedin, 04/2023 (edit as needed)
//
//  Simple functionality to control a 16x2 LCD via the Blue Pill.
//
//  HARDWARE SETUP
//  ==============
//  The 16x2 LCD module requires 5V signals to operate but the
//  STM32 operates at 3.3V. Fortunately most STM32 GPIO pins are
//  5V tolerant, so if they are set to floating output, they can
//  be tied to 5V with pullup resistors. Other values of
//  resistors may work, but 5.6 k ohm resisters were verified as
//  working.
//
//  Needless to say, this means that an external 5V supply is
//  required. One option is to use the 5V line on the Blue Pill
//  when it is powered via the USB port.
//
//     Blue Pill  16x2 LCD  INLINE  5V/GND
//     =========  ========  ======  ======
//        GND ----- VSS ------------- GND
//         5V ----- VDD ------------- 5V
//                                /-- 5V
//                   VO -[10k Pot]**     (LCD contrast control)
//                                \-- GND
//         B14 ----- RS -- [100K]* -- 5V
//                   RW ------------- GND
//         B15 ----- EN -- [100K] --- 5V
//                   D0 ------------- GND
//                   D1 ------------- GND
//                   D2 ------------- GND
//                   D3 ------------- GND
//         A8 ------ D4 -- [100K]* -- 5V
//         A9 ------ D5 -- [100K]* -- 5V
//        A10 ------ D6 -- [100K]* -- 5V
//        A11 ------ D7 -- [100K]* -- 5V
//                    A -- [1.0K] --- 5V
//                    K ------------- GND
//
//  Alternative Hardware Setup
//  ==========================
//  This might not work for everyone, but at least for some LCD
//  modules, the pullup resistors may not be required, except
//  for the one between 5V and the LCD RS line. Also, instead of
//  a bulky potentiometer for the contrast control to V0, a 2K
//  resistor between V0 and 5V may be good enough to maintain
//  good contrast.
//
// ** Pot may be replaced by a 2k resistor between V0 and GND.
//         VO -- [ 2k ] --- GND (LCD contrast control)
//        
//  * These pullup resistors to 5V might not be required.
//    Note that the RS line seems to require the pullup resistor.
//        B14 ------ RS
//         A8 ------ D4
//         A9 ------ D5
//        A10 ------ D6
//        A11 ------ D7



#include "stm32f103x6.h"  // Primary CMSIS header file

#define LCD_RS_BIT (1<<14)
#define LCD_EN_BIT (1<<15)

#define LCD_RS_ON()  GPIOB->ODR |=  LCD_RS_BIT
#define LCD_EN_ON()  GPIOB->ODR |=  LCD_EN_BIT
#define LCD_RS_OFF() GPIOB->ODR &= ~LCD_RS_BIT
#define LCD_EN_OFF() GPIOB->ODR &= ~LCD_EN_BIT

#define LCD_CLEAR                0x01
#define LCD_HOME                 0x02
#define LCD_OFF                  0x08
#define LCD_ON_NO_CURSOR         0x0C
#define LCD_ON_BLINK_CURSOR      0x0D
#define LCD_ON_LINE_CURSOR       0x0E 
#define LCD_ON_LINE_BLINK_CURSOR 0x0F
#define LCD_4B_58F_2L            0x2B
#define LCD_1ST_LINE             0x80
#define LCD_2ND_LINE             0xC0


//  writeLowerNibble
//  Puts the lower 4 bits of 'data' onto GPIO pins A[11:7]
void
LCD_writeLowerNibble( uint8_t data )
{
  GPIOA->ODR &= 0xF0FF;    // Clear GPIO A[11:8]
  // Mask off the lower nibble of data, shift 8 bits to the left, and write those bits
  // to the newly cleared nibble from A[11:8]
  GPIOA->ODR |= (data & 0x0F) << 8;
}


//  writeUpperNibble
//  Writes the upper 4 bits of 'data' onto GPIO pins A[11:8]
void
LCD_writeUpperNibble( uint8_t data )
{
  GPIOA->ODR &= 0xF0FF;    // Clear GPIO A[11:8]
  // Mask off the upper nibble of data, shift 4 bits to the left, and write those bits
  // to the newly cleared nibble from A[11:8]
  GPIOA->ODR |= (data & 0xF0) << 4;
}


//  Pause
//  Actually halts the program here by entering an endless loop
//  For debugging.
void
pause( void )
{
  while(1);
}


//  delay_ms
//  Input: uint16_t d
//  Causes a delay for approx d ms
//  ** Only works at clock speed of 8 MHz!
void
delay_ms( uint16_t d )
{
  for( uint16_t x=0; x<d ; x++ )
  {
    for( uint16_t y=0; y<800; y++) ; 
  }
}


//  LCD_pulse_EN
//  Make EN high, wait for 1 ms, and then bring it down low.
void
LCD_pulse_EN(void)
{
  LCD_EN_ON();
  delay_ms(1);
  LCD_EN_OFF();
}


//  LCD_cmd
//  Send a 4-bit command to the LCD
void
LCD_cmd( uint8_t data )
{
  // Make sure EN and RS are low
  LCD_EN_OFF();
  LCD_RS_OFF();
  // Delay
  delay_ms(1);
  // Place upper nibble of command on GPIOA[11:8]
  LCD_writeUpperNibble( data );
  // Pulse EN pin to set this nibble
  LCD_pulse_EN();
  // Place lower nibble of command on GPIOA[11:8]
  LCD_writeLowerNibble( data );
  // Pulse EN pin to set this nibble
  LCD_pulse_EN();
}


//  LCD_putc
//  Send a character to the LCD
void
LCD_putc( char data )
{
  // Set EN low, and RS high
  LCD_EN_OFF();
  LCD_RS_ON();
  delay_ms(1);
  // Place upper nibble of character on GPIOA[11:8]
  LCD_writeUpperNibble( data );
  // Pulse EN pin to set this nibble
  LCD_pulse_EN();
  // Place lower nibble of command on GPIOA[11:8]
  LCD_writeLowerNibble( data );
  // Pulse EN pin to set this nibble
  LCD_pulse_EN();
  delay_ms(2);
}


//  LCD_puts
//  Takes a pointer to a null-terminated string and displays that string
//  from the current LCD cursor position. Does not check for LCD line/string
//  overflow.
void
LCD_puts( char *data )
{
  uint8_t j=0;

  while( data[j] != 0 )
  {
    LCD_putc( data[j] );
    j++;
  }
}


//  LCD_init
//  Initializes LCD by initializing required GPIO ports and pins used to talk
//  to the LCD. Also initializes the LCD screen itself to be in 4-bit mode.
//  This setup uses GPIO A8, A9, A10, A11 for LCD data pins 4, 5, 6, 7 respectively.
//  GPIO B14 is LCD RS and GPIO B15 is the LCD EN pin.
//  Note that these GPIO pins are set as floating in order to allow external resistor-
//  pullups to 5V. This is okay as these are 5V-tolerant pins.
void
LCD_init( void )
{
  RCC->APB2ENR |= RCC_APB2ENR_IOPAEN |
                  RCC_APB2ENR_IOPBEN;           // Enable GPIO Ports A and B

  GPIOA->CRH &= ~( 0b11 << GPIO_CRH_CNF8_Pos  |
                   0b11 << GPIO_CRH_CNF9_Pos  |
                   0b11 << GPIO_CRH_CNF10_Pos |
                   0b11 << GPIO_CRH_CNF11_Pos ); // Clear CNF Bits

  GPIOB->CRH &= ~( 0b11 << GPIO_CRH_CNF14_Pos |
                   0b11 << GPIO_CRH_CNF15_Pos ); // Clear CNF Bits

  GPIOA->CRH |= ( 0b01 << GPIO_CRH_MODE8_Pos  |
                  0b01 << GPIO_CRH_CNF8_Pos   |
                  0b01 << GPIO_CRH_MODE9_Pos  |
                  0b01 << GPIO_CRH_CNF9_Pos   |
                  0b01 << GPIO_CRH_MODE10_Pos |
                  0b01 << GPIO_CRH_CNF10_Pos  |
                  0b01 << GPIO_CRH_MODE11_Pos |
                  0b01 << GPIO_CRH_CNF11_Pos  ); // Set Mode bits to 01

  GPIOB->CRH |= ( 0b01 << GPIO_CRH_MODE14_Pos |
                  0b01 << GPIO_CRH_CNF14_Pos  |
                  0b01 << GPIO_CRH_MODE15_Pos |
                  0b01 << GPIO_CRH_CNF15_Pos  ); // Set Mode bits to 01

  // Start with delay to make sure the LCD module is fully powered up
  delay_ms(20);
  // Set EN and RS low
  LCD_EN_OFF();
  LCD_RS_OFF();
  // Delay
  delay_ms(1);
  // Write 0x02 to data pins to start 4-bit mode
  LCD_writeLowerNibble( 0x2 );
  // Pulse EN pin to set this nibble
  LCD_pulse_EN();

  LCD_cmd( LCD_4B_58F_2L );     // 4-bit, 5x8 font, 2 lines
  LCD_cmd( LCD_CLEAR );         // Clear display
  LCD_cmd( LCD_HOME );          // Home
  LCD_cmd( LCD_ON_NO_CURSOR );  // Display on, no cursor
}


int
main()
{
  char myString[] = "Hello World!";

  LCD_init();
  LCD_puts( "Mike" );
  LCD_cmd( LCD_1ST_LINE + 8);   // Postion to 8th char of 1st line
  LCD_putc( '1' );
  LCD_putc( '2' );
  LCD_putc( '3' );
  LCD_putc( '4' );
  LCD_cmd( LCD_2ND_LINE + 2 );  // Position to 2nd char of 2nd line
  LCD_puts( myString );

  return 1;
}  
