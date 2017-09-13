/******************************************************************************
* Project        : HAN ESE PRJ2, PRJ1V & PRJ1D
* File           : HD44780-based LCD implementation
* Copyright      : 2013 HAN Embedded Systems Engineering
******************************************************************************
   Change History:

    Version 1.0 - May 2013
    > Initial revision

******************************************************************************/
#include "lcd.h"

// ----------------------------------------------------------------------------
// Local function prototypes
// ----------------------------------------------------------------------------
void LCD_ToggleE(void);
void write4bits(uint8_t ins);

/**
 * @brief  This function sets up the LCD HD44780 hardware as mentioned in the
 *         hardware description in the header file.
 * @param  None
 * @retval None
 */
void LCD_Setup(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    // Enable GPIO peripheral
    #if (LCD_GPIO_Port == GPIOA)
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
    #elif (LCD_GPIO_Port == GPIOB)
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);
    #elif (LCD_GPIO_Port == GPIOC)
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOC, ENABLE);
    #elif (LCD_GPIO_Port == GPIOD)
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOD, ENABLE);
    #elif (LCD_GPIO_Port == GPIOE)
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOE, ENABLE);
    #endif

    // Configure GPIO pins
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_InitStructure.GPIO_Pin = (LCD_RS |
                                   LCD_RW |
                                   LCD_E  |
                                   LCD_DB4 |
                                   LCD_DB5 |
                                   LCD_DB6 |
                                   LCD_DB7);

    GPIO_ResetBits(LCD_GPIO_Port, LCD_RS | LCD_RW | LCD_E  | LCD_DB4 | LCD_DB5 | LCD_DB6 | LCD_DB7);

    GPIO_Init(LCD_GPIO_Port, &GPIO_InitStructure);
    GPIO_ResetBits(LCD_GPIO_Port, LCD_RW);
}

/**
 * @brief  This function powers on and initializes the HD44780 LCD module by
 *         writing data to 6 instruction registers. The first three
 *         instructions use a delay routine, because the datasheet mentions
 *         that the busyflag cannot be checked. The other three instructions
 *         do not need a delay, because in the function
 *         LCD_WriteInstruction() the busy flag is checked before writing
 *         the instruction.
 *         After executing this function:
 *         - Function_set: 2 lines x 16, display on, cursor off, no blink
 *         - Return home, set DDRAM address 0 in address counter
 * @param  None
 * @retval None
 */
void LCD_PowerOn(void)
{
    // Wait for more then 50 ms after Vdd rises to 4.5V
    Delay(SystemCoreClock / 8 / 20); // ~ 50ms

    // try to set 4-bit mode
    write4bits(0x03);
    Delay(SystemCoreClock / 8 / 222); // ~4.5 ms

    // second try
    write4bits(0x03);
    Delay(SystemCoreClock / 8 / 222); // ~40 ms

    // third go !
    write4bits(0x03);
    Delay(SystemCoreClock / 8 / 6666); // ~150 us

    // finally, set to 4-bit interface
    write4bits(0x02);

    // Function set
    LCD_WriteInstruction(LCD_FUNCTION_SET   |
                         LCD_FUNCTION_SET_N |
                         LCD_FUNCTION_SET_F);

    LCD_WriteInstruction(LCD_DISPLAY_ONOFF_CONTROL |
                         LCD_DISPLAY_ONOFF_CONTROL_D);

    // Delay(SystemCoreClock / 8 / 200); // ~50 ms

    // Display clear
    LCD_Clear();

    // Entry mode set
    // Set the LCD unit to increment the address counter and shift the cursor to
    // the right after each data transaction. The display does not shift.
    LCD_WriteInstruction(LCD_ENTRY_MODE_SET |
                         LCD_ENTRY_MODE_SET_ID);
}

/**
 * @brief  This function clears (0x00) the entire DDRAM and sets DDRAM address
 *         to 0x00.
 * @param  None
 * @retval None
 */
void LCD_Clear(void)
{
    LCD_WriteInstruction(LCD_CLEAR_DISPLAY);
    Delay(SystemCoreClock / 8 / 500);
}

/**
 * @brief  This function sets the cursor to it's home position
 *         x=0, y=0. It also switches a shifted display back to an unshifted
 *         state.
 * @param  None
 * @retval None
 */
void LCD_Home(void)
{
    LCD_WriteInstruction(LCD_RETURN_HOME);
    Delay(SystemCoreClock / 8 / 500);
}

/**
 * @brief  This function sets the cursor to position (x,y):
 *
 *         +----+----+---- ----+----+----+
 *         | 0,0| 1,0|         |38,0|39,0|
 *         +----+----+---- ----+----+----+
 *         | 0,1| 1,1|         |38,1|39,1|
 *         +----+----+---- ----+----+----+
 *
 * @param  x: horizontal position (0-39)
 *         y: vertical position (0-1)
 * @retval None
 */
void LCD_GotoXY(uint8_t x, uint8_t y)
{
    LCD_WriteInstruction(LCD_SET_DDRAM_ADDRESS + (y * 0x40) + x);
}

/**
 * @brief  This function writes an instruction to the LCD. The RS and
 *         RW lines are both reset to logic 0 and then first the high nibble
 *         of the instruction is written to the databus.
 * @param  ins: an instruction that the LCD controller can interpret. For an
 *              overview of all instructions see the header file and/or the
 *              LCD controller datasheet:
 *              http://en.wikipedia.org/wiki/Hitachi_HD44780_LCD_controller
 * @retval None
 */
void LCD_WriteInstruction(uint8_t value)
{
    GPIO_ResetBits(LCD_GPIO_Port, LCD_RS);

    write4bits(value >> 4);
    write4bits(value);
    Delay(SystemCoreClock / 8 / 20000);
}

void write4bits(uint8_t ins)
{
    if ((ins & 0x01) == 0)
    {
        GPIO_ResetBits(LCD_GPIO_Port, LCD_DB4);
    }
    else
    {
        GPIO_SetBits(LCD_GPIO_Port, LCD_DB4);
    }
    if ((ins & 0x02) == 0)
    {
        GPIO_ResetBits(LCD_GPIO_Port, LCD_DB5);
    }
    else
    {
        GPIO_SetBits(LCD_GPIO_Port, LCD_DB5);
    }
    if ((ins & 0x04) == 0)
    {
        GPIO_ResetBits(LCD_GPIO_Port, LCD_DB6);
    }
    else
    {
        GPIO_SetBits(LCD_GPIO_Port, LCD_DB6);
    }
    if ((ins & 0x08) == 0)
    {
        GPIO_ResetBits(LCD_GPIO_Port, LCD_DB7);
    }
    else
    {
        GPIO_SetBits(LCD_GPIO_Port, LCD_DB7);
    }

    LCD_ToggleE();
}

/**
 * @brief  This function writes 8-bit data to the LCD. Data will be
 *         visible at the current cursor position.
 * @param  d: 8-bit data to be written. Verify the characterset how this
 *            8-bit data translates to display characters, for instance:
 *            http://www.msc-ge.com/download/displays/dabla_allg/ge-c1602b-yyh-jt-r.pdf
 * @retval None
 */
void LCD_WriteData(uint8_t value)
{
    GPIO_SetBits(LCD_GPIO_Port, LCD_RS);

    write4bits(value >> 4);
    write4bits(value);
    Delay(SystemCoreClock / 8 / 20000);
}

/**
 * @brief  This function writes an ASCII character to the display at the
 *         current cursor position.
 * @param  c: character to be displayed
 * @retval None
 */
void LCD_Putc(uint8_t c)
{
    LCD_WriteData(c);
}

/**
 * @brief  This function writes a string of characters to the LCD.
 * @param  str: pointer to NULL termineted string to be displayed
 * @retval None
 */
void LCD_Puts(uint8_t * str)
{
    while (*str)
    {
        LCD_Putc(*str++);
    }
}

/**
 * @brief  This function toggles the Enable line for an write operation.
 *
 * @param  None
 * @retval None
 */
void LCD_ToggleE(void)
{
    GPIO_ResetBits(LCD_GPIO_Port, LCD_E);
    // Address setup time
    Delay(SystemCoreClock / 8 / 1000000); // ~1 us

    GPIO_SetBits(LCD_GPIO_Port, LCD_E);

    // Enable line puls width (high level)
    Delay(SystemCoreClock / 8 / 1000000); // ~1 us

    GPIO_ResetBits(LCD_GPIO_Port, LCD_E);

    Delay(SystemCoreClock / 8 / 10000); // ~100 us
}


/**
 * @brief  This function write on the CGRAM a new charater.
 *
 * @param  num: adress of the caractere
 * @param  tab: array with the caratere
 * @retval None
 */
void LCD_NewChar(unsigned char num, unsigned char * tab)
{
    unsigned char i;

    LCD_WriteInstruction(LCD_SET_CGRAM_ADDRESS | num << 3);

    for (i = 0; i < 8; i++)
    {
        LCD_WriteData(tab[i]);
    }
}

/**
 * @brief  This function implements a delay.
 *         If the optimization level is set to -O3, this function takes 8
 *         cycles. To create a delay of 1 second, use the following function
 *         call: Delay(SystemCoreClock/8);
 * @param  d: number delay loops (1 loop takes 8/SystemCoreClock sec.)
 * @retval None
 */
void Delay(const int d)
{
    volatile int i;

    for (i = d; i > 0; i--)
    {
        ;
    }

    return;
}
