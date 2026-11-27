#include "WMMatrixLed.h"

// =============================================================================
// PATCH NOTE: Added delayMicroseconds(2) calls in writeByte() and
// writeBytesToAddress() between SCK transitions. ESP32 at 240 MHz toggles
// digitalWrite() far faster than the TM640B's minimum clock pulse width,
// so without these delays the chip never latches the bits and shows nothing.
// =============================================================================

#define MATRIX_BIT_DELAY_US 10


WMPort::WMPort(void)
{
}


void WMPort::setOutput(uint8_t port)
{
        pinMode(port, OUTPUT);
}

void WMPort::setInput(uint8_t port)
{
        pinMode(port, INPUT);
}

void WMPort::setPuOn(uint8_t port)
{
        pinMode(port, INPUT_PULLUP);
}

bool WMPort::readDPort(uint8_t port)
{
        bool val;
        setInput(port);
        val = digitalRead(port);
        return(val);
}

bool WMPort::readDPuPort(uint8_t port)
{
        bool val;
        setPuOn(port);
        val = digitalRead(port);
        return(val);
}

int16_t WMPort::readAport(uint8_t port)
{
        int16_t val;
        setInput(port);
        val = analogRead(port);
        return(val);
}

void WMPort::writeDport(uint8_t port,bool value)
{
        setOutput(port);
        digitalWrite(port, value);
}

void WMPort::writeAport(uint8_t port,int16_t value)
{
        analogWrite(port, value);
}


WMMatrixLed::WMMatrixLed(void)
{

}

WMMatrixLed::WMMatrixLed(uint8_t port)
{
        u8_SCKPin = wmPort[port].pin1;
        u8_DINPin = wmPort[port].pin2;

        // Lock pinMode once so writeDport hot path can skip it
        pinMode(u8_SCKPin, OUTPUT);
        pinMode(u8_DINPin, OUTPUT);

        writeDport(u8_SCKPin,1);
        writeDport(u8_DINPin,1);

    writeByte(Mode_Address_Auto_Add_1);
    setBrightness(Brightness_5);
    clearScreen();
}

WMMatrixLed::WMMatrixLed(uint8_t SCK_Pin, uint8_t DIN_Pin)
{
        u8_SCKPin = SCK_Pin;
        u8_DINPin = DIN_Pin;

        pinMode(u8_SCKPin, OUTPUT);
        pinMode(u8_DINPin, OUTPUT);

        writeDport(u8_SCKPin,1);
        writeDport(u8_DINPin,1);

    writeByte(Mode_Address_Auto_Add_1);
    setBrightness(Brightness_5);
    clearScreen();
}


void WMMatrixLed::reset(uint8_t port){
    u8_SCKPin = wmPort[port].pin1;
        u8_DINPin = wmPort[port].pin2;

        pinMode(u8_SCKPin, OUTPUT);
        pinMode(u8_DINPin, OUTPUT);

        writeDport(u8_SCKPin,1);
        writeDport(u8_DINPin,1);

    writeByte(Mode_Address_Auto_Add_1);
    setBrightness(Brightness_5);
    clearScreen();
}


void WMMatrixLed::writeByte(uint8_t data)
{
    //Start
        writeDport(u8_SCKPin,1);
        writeDport(u8_DINPin,0);
        delayMicroseconds(MATRIX_BIT_DELAY_US);

    for(char i=0;i<8;i++)
    {
        writeDport(u8_SCKPin, 0);
        writeDport(u8_DINPin, (data & 0x01));
        delayMicroseconds(MATRIX_BIT_DELAY_US);
        writeDport(u8_SCKPin, 1);
        delayMicroseconds(MATRIX_BIT_DELAY_US);
        data = data >> 1;
    }

    //End
    writeDport(u8_SCKPin, 0);
    writeDport(u8_DINPin, 0);
    delayMicroseconds(MATRIX_BIT_DELAY_US);
    writeDport(u8_SCKPin, 1);
    writeDport(u8_DINPin, 1);
    delayMicroseconds(MATRIX_BIT_DELAY_US);
}


void WMMatrixLed::writeBytesToAddress(uint8_t Address, const uint8_t *P_data, uint8_t count_of_data)
{
    uint8_t T_data;

    if(Address > 15 || count_of_data==0)
        return;

    Address = ADDRESS(Address);

    //Start
    writeDport(u8_SCKPin, 1);
    writeDport(u8_DINPin, 0);
    delayMicroseconds(MATRIX_BIT_DELAY_US);

    //write Address
    for(char i=0;i<8;i++)
    {
        writeDport(u8_SCKPin, 0);
        writeDport(u8_DINPin, (Address & 0x01));
        delayMicroseconds(MATRIX_BIT_DELAY_US);
        writeDport(u8_SCKPin, 1);
        delayMicroseconds(MATRIX_BIT_DELAY_US);
        Address = Address >> 1;
    }


    //write data
    for(uint8_t k=0; k<count_of_data; k++)
    {
        T_data = *(P_data + k);

        for(char i=0;i<8;i++)
        {
            writeDport(u8_SCKPin, 0);
            writeDport(u8_DINPin, (T_data & 0x80));
            delayMicroseconds(MATRIX_BIT_DELAY_US);
            writeDport(u8_SCKPin, 1);
            delayMicroseconds(MATRIX_BIT_DELAY_US);
            T_data = T_data << 1;
        }
    }

    //End
    writeDport(u8_SCKPin, 0);
    writeDport(u8_DINPin, 0);
    delayMicroseconds(MATRIX_BIT_DELAY_US);
    writeDport(u8_SCKPin, 1);
    writeDport(u8_DINPin, 1);
    delayMicroseconds(MATRIX_BIT_DELAY_US);
}


void WMMatrixLed::clearScreen()
{
    for(uint8_t i=0;i<LED_BUFFER_SIZE;i++)
    {
        u8_Display_Buffer[i] = 0x00;
    }

    b_Color_Index = 1;
    b_Draw_Str_Flag = 0;

    writeBytesToAddress(0,u8_Display_Buffer,LED_BUFFER_SIZE);
}

void WMMatrixLed::setBrightness(uint8_t Bright)
{
    if((uint8_t)Bright>8)
    {
        Bright = Brightness_8;
    }

    if((uint8_t)Bright != 0)
    {
        Bright = (LED_Matrix_Brightness)((uint8_t)(Bright-1)|0x08);

    }
    writeByte(0x80 | (uint8_t)Bright);

}

void WMMatrixLed::setColorIndex(bool Color_Number)
{
    b_Color_Index = Color_Number;
}

void WMMatrixLed::drawBitmap(int8_t x, int8_t y, uint8_t Bitmap_Width, uint8_t *Bitmap)
{

    if(x>15 || y>7 || Bitmap_Width==0)
        return;


    if(b_Color_Index == 1)
    {
        for(uint8_t k=0;k<Bitmap_Width;k++)
        {
          if(x+k>=0){
            u8_Display_Buffer[x+k] = (u8_Display_Buffer[x+k] & (0xff << (8-y))) | (y>0?(Bitmap[k] >> y):(Bitmap[k] << (-y)));
          }
        }
    }
    else if(b_Color_Index == 0)
    {
        for(uint8_t k=0;k<Bitmap_Width;k++)
        {
            if(x+k>=0){
              u8_Display_Buffer[x+k] = (u8_Display_Buffer[x+k] & (0xff << (8-y))) | (y>0?(~Bitmap[k] >> y):(~Bitmap[k] << (-y)));
            }
        }
    }

    writeBytesToAddress(0,u8_Display_Buffer,LED_BUFFER_SIZE);
}

void WMMatrixLed::drawStr(int16_t X_position, int8_t Y_position, const char *str)
{
    b_Draw_Str_Flag = 1;

    for(i16_Number_of_Character_of_Str = 0; str[i16_Number_of_Character_of_Str] != '\0'; i16_Number_of_Character_of_Str++)
    {
        if(i16_Number_of_Character_of_Str < STRING_DISPLAY_BUFFER_SIZE - 1)
        {
            i8_Str_Display_Buffer[i16_Number_of_Character_of_Str] = str[i16_Number_of_Character_of_Str];
        }
        else
        {
            break;
        }
    }
    i8_Str_Display_Buffer[i16_Number_of_Character_of_Str] = '\0';


    if(X_position < -(i16_Number_of_Character_of_Str * 6))
    {
        X_position = -(i16_Number_of_Character_of_Str * 6);
    }
    else if(X_position > 16)
    {
        X_position = 16;
    }
    i16_Str_Display_X_Position = X_position;


    if(Y_position < -1)
    {
        Y_position = -1;
    }
    else if(Y_position >15)
    {
        Y_position = 15;
    }
    i8_Str_Display_Y_Position = Y_position;

    showStr();

}


void WMMatrixLed::showStr()
{

    uint8_t display_buffer_label = 0;

    if(i16_Str_Display_X_Position > 0)
    {
        for(display_buffer_label = 0; display_buffer_label < i16_Str_Display_X_Position && display_buffer_label < LED_BUFFER_SIZE; display_buffer_label++)
        {
            u8_Display_Buffer[display_buffer_label] = 0x00;
        }

        if(display_buffer_label < LED_BUFFER_SIZE)
        {
            uint8_t num;

            for(uint8_t k=0;display_buffer_label < LED_BUFFER_SIZE && k < i16_Number_of_Character_of_Str;k++)
            {
                for(num=0; pgm_read_byte(&Character_font_6x8[num].Character[0]) != '@'; num++)
                {
                    if(i8_Str_Display_Buffer[k] == pgm_read_byte(&Character_font_6x8[num].Character[0]))
                    {
                        for(uint8_t j=0;j<6;j++)
                        {
                            u8_Display_Buffer[display_buffer_label] = pgm_read_byte(&Character_font_6x8[num].data[j]);
                            display_buffer_label++;

                            if(display_buffer_label >= LED_BUFFER_SIZE)
                            {
                                break;
                            }
                        }
                        break;
                    }
                }

                if(pgm_read_byte(&Character_font_6x8[num].Character[0]) == '@')
                {
                    i8_Str_Display_Buffer[k] = ' ';
                    k--;
                }
            }

            if(display_buffer_label < LED_BUFFER_SIZE)
            {
                for(display_buffer_label = display_buffer_label; display_buffer_label < LED_BUFFER_SIZE; display_buffer_label++)
                {
                    u8_Display_Buffer[display_buffer_label] = 0x00;
                }
            }
        }
    }

    else if(i16_Str_Display_X_Position <= 0)
    {
        if(i16_Str_Display_X_Position == -(i16_Number_of_Character_of_Str * 6))
        {
            for(; display_buffer_label < LED_BUFFER_SIZE; display_buffer_label++)
            {
                u8_Display_Buffer[display_buffer_label] = 0x00;
            }
        }
        else
        {
            int8_t j = (-i16_Str_Display_X_Position) % 6;
            uint8_t num;

            i16_Str_Display_X_Position = -i16_Str_Display_X_Position;

            for(int16_t k=i16_Str_Display_X_Position/6; display_buffer_label < LED_BUFFER_SIZE && k < i16_Number_of_Character_of_Str;k++)
            {
                for(num=0; pgm_read_byte(&Character_font_6x8[num].Character[0]) != '@'; num++)
                {
                    if(i8_Str_Display_Buffer[k] == pgm_read_byte(&Character_font_6x8[num].Character[0]))
                    {
                        for(;j<6;j++)
                        {
                            u8_Display_Buffer[display_buffer_label] = pgm_read_byte(&Character_font_6x8[num].data[j]);
                            display_buffer_label++;

                            if(display_buffer_label >= LED_BUFFER_SIZE)
                            {
                                break;
                            }
                        }
                        j=0;
                        break;
                    }
                }

                if(pgm_read_byte(&Character_font_6x8[num].Character[0]) == '@')
                {
                    i8_Str_Display_Buffer[k] = ' ';
                    k--;
                }

            }

            if(display_buffer_label < LED_BUFFER_SIZE)
            {
                for(; display_buffer_label < LED_BUFFER_SIZE; display_buffer_label++)
                {
                    u8_Display_Buffer[display_buffer_label] = 0x00;
                }
            }

            i16_Str_Display_X_Position = -i16_Str_Display_X_Position;
        }
    }


    if(7 - i8_Str_Display_Y_Position >= 0)
    {
        for(uint8_t k=0; k<LED_BUFFER_SIZE; k++)
        {
            u8_Display_Buffer[k] = u8_Display_Buffer[k] << (7 - i8_Str_Display_Y_Position);
        }
    }
    else
    {
        for(uint8_t k=0; k<LED_BUFFER_SIZE; k++)
        {
            u8_Display_Buffer[k] = u8_Display_Buffer[k] >> (i8_Str_Display_Y_Position - 7);
        }
    }


    if(b_Color_Index == 0)
    {
        for(uint8_t k=0; k<LED_BUFFER_SIZE; k++)
        {
            u8_Display_Buffer[k] = ~u8_Display_Buffer[k];
        }
    }

    writeBytesToAddress(0,u8_Display_Buffer,LED_BUFFER_SIZE);

}


void WMMatrixLed::showClock(uint8_t hour, uint8_t minute, bool point_flag)
{
    u8_Display_Buffer[0]  = pgm_read_byte(&Clock_Number_font_3x8[hour/10].data[0]);
    u8_Display_Buffer[1]  = pgm_read_byte(&Clock_Number_font_3x8[hour/10].data[1]);
    u8_Display_Buffer[2]  = pgm_read_byte(&Clock_Number_font_3x8[hour/10].data[2]);

    u8_Display_Buffer[3]  = 0x00;

    u8_Display_Buffer[4]  = pgm_read_byte(&Clock_Number_font_3x8[hour%10].data[0]);
    u8_Display_Buffer[5]  = pgm_read_byte(&Clock_Number_font_3x8[hour%10].data[1]);
    u8_Display_Buffer[6]  = pgm_read_byte(&Clock_Number_font_3x8[hour%10].data[2]);

    u8_Display_Buffer[9]  = pgm_read_byte(&Clock_Number_font_3x8[minute/10].data[0]);
    u8_Display_Buffer[10] = pgm_read_byte(&Clock_Number_font_3x8[minute/10].data[1]);
    u8_Display_Buffer[11] = pgm_read_byte(&Clock_Number_font_3x8[minute/10].data[2]);

    u8_Display_Buffer[12] = 0x00;

    u8_Display_Buffer[13] = pgm_read_byte(&Clock_Number_font_3x8[minute%10].data[0]);
    u8_Display_Buffer[14] = pgm_read_byte(&Clock_Number_font_3x8[minute%10].data[1]);
    u8_Display_Buffer[15] = pgm_read_byte(&Clock_Number_font_3x8[minute%10].data[2]);


    if(point_flag == PointOn)
    {
        u8_Display_Buffer[7] = 0x28;
        u8_Display_Buffer[8] = 0x28;
    }
    else
    {
        u8_Display_Buffer[7] = 0x00;
        u8_Display_Buffer[8] = 0x00;
    }

    if(b_Color_Index == 0)
    {
        for(uint8_t k=0; k<LED_BUFFER_SIZE; k++)
        {
            u8_Display_Buffer[k] = ~u8_Display_Buffer[k];
        }
    }

    writeBytesToAddress(0,u8_Display_Buffer,LED_BUFFER_SIZE);
}


bool WMMatrixLed::testStatus(int16_t value,int16_t X_position)
{
        bool return_value = false;
        switch(value - X_position)
        {
                case 3:
                case 7:
                case 11:
                case 15:
                        return_value =  true;
                        break;
                default:
                        break;
        }
        return return_value;

}

void WMMatrixLed::setPostionLedOnOff(int16_t posX,int16_t posY,uint8_t flag)
{
        if(flag)
        {
                u8_Display_Buffer[posX] |= (1 << 7 - posY);
        }
        else
        {
                u8_Display_Buffer[posX] &= ~(1 << 7 - posY);
        }
        writeBytesToAddress(0,u8_Display_Buffer,LED_BUFFER_SIZE);
}

// ----- showNum stubs preserved for ABI compatibility (not needed by Gary) -----
void WMMatrixLed::showNum(float value,uint8_t digits) {}
void WMMatrixLed::showNum(int16_t X_position,float value,uint8_t digits) {}
