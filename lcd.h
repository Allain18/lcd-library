#ifndef _LCD_H_
#define _LCD_H_
/*************************************************************************
   Title	:   C include file for the HD44780U LCD library (lcd.c)
   Author:    Alain Girard
   File:	    lcd.h
   Software:  STM32 libraries
   Hardware:  STM32F0
 ***************************************************************************/

/**
   @mainpage
   Collection of libraries for GCC
   @author Alain Girard

   @file
   @defgroup LCD library <lcd.h>
   @code #include <lcd.h> @endcode

   @brief Basic routines for interfacing a HD44780U-based character LCD display

   LCD character displays can be found in many devices, like espresso machines, laser printers.
   The Hitachi HD44780 controller and its compatible controllers like Samsung KS0066U have become an industry standard for these types of displays.


   @author Alain Girard

   @version   1.0

 */

#include "stm32f0xx.h"


/**@{*/

/*
 * LCD and target specific definitions below can be defined in a separate include file with name lcd_definitions.h instead modifying this file
 * by adding _LCD_DEFINITIONS_FILE to the options of Keil
 * All definitions added to the file lcd_definitions.h will override the default definitions from lcd.h
 */
#ifdef _LCD_DEFINITIONS_FILE
#include "lcd_definitions_file.h"
#endif

/**
 * @name  Definitions for Display Size
 * Change these definitions to adapt setting to your display
 */
#ifndef LCD_LINES
#define LCD_LINES                   2 /**< number of visible lines of the display */
#endif
#ifndef LCD_DISP_LENGTH
#define LCD_DISP_LENGTH             16 /**< visibles characters per line of the display */
#endif

/**
 * @name Definitions for 4-bit IO mode
 *
 * The four LCD data lines and the three control lines RS, RW, E can be on the
 * same port or on different ports.
 * Change LCD_RS_PORT, LCD_RW_PORT, LCD_E_PORT if you want the control lines on
 * different ports.
 */
#ifndef LCD_GPIO_Port
#define LCD_GPIO_Port               (GPIOA) /**< port for the LCD lines   */
#endif
#ifndef LCD_DB4
#define LCD_DB4                     (GPIO_Pin_0) /**< pin for 4bit data bit 4  */
#endif
#ifndef LCD_DB5
#define LCD_DB5                     (GPIO_Pin_1) /**< pin for 4bit data bit 5  */
#endif
#ifndef LCD_DB6
#define LCD_DB6                     (GPIO_Pin_2) /**< pin for 4bit data bit 6  */
#endif
#ifndef LCD_DB7
#define LCD_DB7                     (GPIO_Pin_3) /**< pin for 4bit data bit 7  */
#endif

#ifndef LCD_RS
#define LCD_RS                      (GPIO_Pin_4) /**< pin  for RS line         */
#endif
#ifndef LCD_RW
#define LCD_RW                      (GPIO_Pin_5) /**< pin  for RW line         */
#endif
#ifndef LCD_E
#define LCD_E                       (GPIO_Pin_6) /**< pin  for Enable line     */
#endif

/**
 * @name Definitions for LCD command instructions
 * The constants define the various LCD controller instructions which can be passed to the
 * function lcd_command(), see HD44780 data sheet for a complete description.
 */

/* instruction register bit positions, see HD44780U data sheet */
#define LCD_CLEAR_DISPLAY           ((uint8_t) (0x01))

#define LCD_RETURN_HOME             ((uint8_t) (0x02))

#define LCD_ENTRY_MODE_SET          ((uint8_t) (0x04))
#define LCD_ENTRY_MODE_SET_ID       ((uint8_t) (0x02))
#define LCD_ENTRY_MODE_SET_SH       ((uint8_t) (0x01))

#define LCD_DISPLAY_ONOFF_CONTROL   ((uint8_t) (0x08))
#define LCD_DISPLAY_ONOFF_CONTROL_D ((uint8_t) (0x04))
#define LCD_DISPLAY_ONOFF_CONTROL_C ((uint8_t) (0x02))
#define LCD_DISPLAY_ONOFF_CONTROL_B ((uint8_t) (0x01))

#define LCD_CURSOR_DISPLAY_SHIFT    ((uint8_t) (0x10))
#define LCD_CURSOR_DISPLAY_SHIFT_SC ((uint8_t) (0x08))
#define LCD_CURSOR_DISPLAY_SHIFT_RL ((uint8_t) (0x04))

#define LCD_FUNCTION_SET            ((uint8_t) (0x20))
#define LCD_FUNCTION_SET_DL         ((uint8_t) (0x10))
#define LCD_FUNCTION_SET_N          ((uint8_t) (0x08))
#define LCD_FUNCTION_SET_F          ((uint8_t) (0x04))

#define LCD_SET_CGRAM_ADDRESS       ((uint8_t) (0x40))

#define LCD_SET_DDRAM_ADDRESS       ((uint8_t) (0x80))

// Macros
#define LCD_MOVE_DISP_LEFT()  LCD_WriteInstruction(0x18)
#define LCD_MOVE_DISP_RIGHT() LCD_WriteInstruction(0x1C)

// Functions
void LCD_Setup(void);
void LCD_PowerOn(void);
void LCD_Clear(void);
void LCD_Home(void);
void LCD_GotoXY(uint8_t x, uint8_t y);
void LCD_WriteInstruction(uint8_t value);
void LCD_WriteData(uint8_t value);
void Delay(const int);
void LCD_NewChar(unsigned char, unsigned char *);
void LCD_Putc(uint8_t c);
void LCD_Puts(uint8_t * str);

#endif /* _LCD_H_ */
