/*
 * Copyright (c) 2010 by Cristian Maglie <c.maglie@arduino.cc>
 * Copyright (c) 2014 by Paul Stoffregen <paul@pjrc.com> (Transaction API)
 * Copyright (c) 2014 by Matthijs Kooijman <matthijs@stdin.nl> (M5Settings AVR)
 * Copyright (c) 2014 by Andrew J. Kroll <xxxajk@gmail.com> (atomicity fixes)
 * M5 Master library for arduino.
 *
 * This file is free software; you can redistribute it and/or modify
 * it under the terms of either the GNU General Public License version 2
 * or the GNU Lesser General Public License version 2.1, both as
 * published by the Free Software Foundation.
 */

#ifndef _M5_H_INCLUDED
#define _M5_H_INCLUDED

#include <Arduino.h>

// M5_HAS_TRANSACTION means M5 has beginTransaction(), endTransaction(),
// usingInterrupt(), and M5Setting(clock, bitOrder, dataMode)
#define M5_HAS_TRANSACTION 1

// M5_HAS_NOTUSINGINTERRUPT means that M5 has notUsingInterrupt() method
#define M5_HAS_NOTUSINGINTERRUPT 1

// M5_ATOMIC_VERSION means that M5 has atomicity fixes and what version.
// This way when there is a bug fix you can check this define to alert users
// of your code if it uses better version of this library.
// This also implies everything that M5_HAS_TRANSACTION as documented above is
// available too.
#define M5_ATOMIC_VERSION 1

// Uncomment this line to add detection of mismatched begin/end transactions.
// A mismatch occurs if other libraries fail to use M5.endTransaction() for
// each M5.beginTransaction().  Connect an LED to this pin.  The LED will turn
// on if any mismatch is ever detected.
//#define M5_TRANSACTION_MISMATCH_LED 5

#ifndef LSBFIRST
#define LSBFIRST 0
#endif
#ifndef MSBFIRST
#define MSBFIRST 1
#endif

#define M5_CLOCK_DIV4 0x00
#define M5_CLOCK_DIV16 0x01
#define M5_CLOCK_DIV64 0x02
#define M5_CLOCK_DIV128 0x03
#define M5_CLOCK_DIV2 0x04
#define M5_CLOCK_DIV8 0x05
#define M5_CLOCK_DIV32 0x06

#define M5_MODE0 0x00
#define M5_MODE1 0x04
#define M5_MODE2 0x08
#define M5_MODE3 0x0C

#define M5_MODE_MASK 0x0C  // CPOL = bit 3, CPHA = bit 2 on SPCR
#define M5_CLOCK_MASK 0x03  // SPR1 = bit 1, SPR0 = bit 0 on SPCR
#define M5_2XCLOCK_MASK 0x01  // M52X = bit 0 on SPSR

// define M5_AVR_EIMSK for AVR boards with external interrupt pins
#if defined(EIMSK)
  #define M5_AVR_EIMSK  EIMSK
#elif defined(GICR)
  #define M5_AVR_EIMSK  GICR
#elif defined(GIMSK)
  #define M5_AVR_EIMSK  GIMSK
#endif

//M5数据操作宏定义
#define Set_Bit(val,bitn) (val |=(1<<(bitn)))
#define Clr_Bit(val,bitn) (val &=~(1<<(bitn)))
#define Get_Bit(val,bitn) (val &(1<<(bitn)))
#define M328CS1 Set_Bit(PORTB,0)
#define M328CS0 Clr_Bit(PORTB,0)
#define SET_M328CS_OUTPUT  Set_Bit(DDRB,0)
#define SET_M328INT_INPUT  Clr_Bit(DDRD,5)
#define M328INT (PIND&0x20)==0x20
#define M328INT1 Set_Bit(PORTD,5)
#define M328INT0 Clr_Bit(PORTD,5)
#define uchar uint8_t

class M5Settings {
public:
  M5Settings(uint32_t clock, uint8_t bitOrder, uint8_t dataMode) {
    if (__builtin_constant_p(clock)) {
      init_AlwaysInline(clock, bitOrder, dataMode);
    } else {
      init_MightInline(clock, bitOrder, dataMode);
    }
  }
  M5Settings() {
    init_AlwaysInline(4000000, MSBFIRST, M5_MODE0);
  }
private:
  void init_MightInline(uint32_t clock, uint8_t bitOrder, uint8_t dataMode) {
    init_AlwaysInline(clock, bitOrder, dataMode);
  }
  void init_AlwaysInline(uint32_t clock, uint8_t bitOrder, uint8_t dataMode)
    __attribute__((__always_inline__)) {
    // Clock settings are defined as follows. Note that this shows M52X
    // inverted, so the bits form increasing numbers. Also note that
    // fosc/64 appears twice
    // SPR1 SPR0 ~M52X Freq
    //   0    0     0   fosc/2
    //   0    0     1   fosc/4
    //   0    1     0   fosc/8
    //   0    1     1   fosc/16
    //   1    0     0   fosc/32
    //   1    0     1   fosc/64
    //   1    1     0   fosc/64
    //   1    1     1   fosc/128

    // We find the fastest clock that is less than or equal to the
    // given clock rate. The clock divider that results in clock_setting
    // is 2 ^^ (clock_div + 1). If nothing is slow enough, we'll use the
    // slowest (128 == 2 ^^ 7, so clock_div = 6).
    uint8_t clockDiv;

    // When the clock is known at compiletime, use this if-then-else
    // cascade, which the compiler knows how to completely optimize
    // away. When clock is not known, use a loop instead, which generates
    // shorter code.
    if (__builtin_constant_p(clock)) {
      if (clock >= F_CPU / 2) {
        clockDiv = 0;
      } else if (clock >= F_CPU / 4) {
        clockDiv = 1;
      } else if (clock >= F_CPU / 8) {
        clockDiv = 2;
      } else if (clock >= F_CPU / 16) {
        clockDiv = 3;
      } else if (clock >= F_CPU / 32) {
        clockDiv = 4;
      } else if (clock >= F_CPU / 64) {
        clockDiv = 5;
      } else {
        clockDiv = 6;
      }
    } else {
      uint32_t clockSetting = F_CPU / 2;
      clockDiv = 0;
      while (clockDiv < 6 && clock < clockSetting) {
        clockSetting /= 2;
        clockDiv++;
      }
    }

    // Compensate for the duplicate fosc/64
    if (clockDiv == 6)
    clockDiv = 7;

    // Invert the M52X bit
    clockDiv ^= 0x1;

    // Pack into the M5Settings class
    spcr = _BV(SPE) | _BV(MSTR) | ((bitOrder == LSBFIRST) ? _BV(DORD) : 0) |
      (dataMode & M5_MODE_MASK) | ((clockDiv >> 1) & M5_CLOCK_MASK);
    spsr = clockDiv & M5_2XCLOCK_MASK;
  }
  uint8_t spcr;
  uint8_t spsr;
  friend class M5Class;
};


class M5Class {
public:
  // Initialize the M5 library
  static void begin();

  // If M5 is used from within an interrupt, this function registers
  // that interrupt with the M5 library, so beginTransaction() can
  // prevent conflicts.  The input interruptNumber is the number used
  // with attachInterrupt.  If M5 is used from a different interrupt
  // (eg, a timer), interruptNumber should be 255.
  static void usingInterrupt(uint8_t interruptNumber);
  // And this does the opposite.
  static void notUsingInterrupt(uint8_t interruptNumber);
  // Note: the usingInterrupt and notUsingInterrupt functions should
  // not to be called from ISR context or inside a transaction.
  // For details see:
  // https://github.com/arduino/Arduino/pull/2381
  // https://github.com/arduino/Arduino/pull/2449

  // Before using M5.transfer() or asserting chip select pins,
  // this function is used to gain exclusive access to the M5 bus
  // and configure the correct settings.
  inline static void beginTransaction(M5Settings settings) {
	//  begin();
	  if (interruptMode > 0) {
      uint8_t sreg = SREG;
      noInterrupts();

      #ifdef M5_AVR_EIMSK
      if (interruptMode == 1) {
        interruptSave = M5_AVR_EIMSK;
        M5_AVR_EIMSK &= ~interruptMask;
        SREG = sreg;
      } else
      #endif
      {
        interruptSave = sreg;
      }
    }

    #ifdef M5_TRANSACTION_MISMATCH_LED
    if (inTransactionFlag) {
      pinMode(M5_TRANSACTION_MISMATCH_LED, OUTPUT);
      digitalWrite(M5_TRANSACTION_MISMATCH_LED, HIGH);
    }
    inTransactionFlag = 1;
    #endif
	
    SPCR = settings.spcr;
    SPSR = settings.spsr;
	delayMicroseconds(25);
	M328CS0;
	/*
	pinMode(MOSI,OUTPUT);
	pinMode(MISO,INPUT);
	pinMode(SCK,OUTPUT);
	
	*/
}

  // Write to the SPI bus (MOSI pin) and also receive (MISO pin)
  inline static uint8_t transfer(uint8_t data) {
    SPDR = data;
    /*
     * The following NOP introduces a small delay that can prevent the wait
     * loop form iterating when running at the maximum speed. This gives
     * about 10% more speed, even if it seems counter-intuitive. At lower
     * speeds it is unnoticed.
     */
    asm volatile("nop");
    while (!(SPSR & _BV(SPIF))) ; // wait
	asm volatile("nop");
	delayMicroseconds(30);
    return SPDR;
  }
  inline static uint16_t transfer16(uint16_t data) {
    union { uint16_t val; struct { uint8_t lsb; uint8_t msb; }; } in, out;
    in.val = data;
    if (!(SPCR & _BV(DORD))) {
      SPDR = in.msb;
      asm volatile("nop"); // See transfer(uint8_t) function
      while (!(SPSR & _BV(SPIF))) ;
      out.msb = SPDR;
      SPDR = in.lsb;
	  asm volatile("nop"); 
      while (!(SPSR & _BV(SPIF))) ;
      out.lsb = SPDR;
    } else {
      SPDR = in.lsb;
	  asm volatile("nop"); 
      while (!(SPSR & _BV(SPIF))) ;
      out.lsb = SPDR;
      SPDR = in.msb;

	  asm volatile("nop"); 
      while (!(SPSR & _BV(SPIF))) ;
      out.msb = SPDR;
    }
    return out.val;
  }
  inline static void transfer(void *buf, size_t count) {
    if (count == 0) return;
    uint8_t *p = (uint8_t *)buf;
    SPDR = *p;
    while (--count > 0) {
      uint8_t out = *(p + 1);
      while (!(SPSR & _BV(SPIF))) ;
      uint8_t in = SPDR;
      SPDR = out;
      *p++ = in;    
    }
    while (!(SPSR & _BV(SPIF))) ;
    *p = SPDR;
  }
  // After performing a group of transfers and releasing the chip select
  // signal, this function allows others to access the M5 bus
  inline static void endTransaction(void) {
    #ifdef M5_TRANSACTION_MISMATCH_LED
    if (!inTransactionFlag) {
      pinMode(M5_TRANSACTION_MISMATCH_LED, OUTPUT);
      digitalWrite(M5_TRANSACTION_MISMATCH_LED, HIGH);
    }
    inTransactionFlag = 0;
    #endif

    if (interruptMode > 0) {
      #ifdef M5_AVR_EIMSK
      uint8_t sreg = SREG;
      #endif
      noInterrupts();
      #ifdef M5_AVR_EIMSK
      if (interruptMode == 1) {
        M5_AVR_EIMSK = interruptSave;
        SREG = sreg;
      } else
      #endif
      {
        SREG = interruptSave;
      }
    }
	M328CS1;
	 //SPCR &= ~_BV(SPE);
	//end();
	//delayMicroseconds(30);
	/*
	pinMode(MOSI,INPUT);
	pinMode(MISO,INPUT);
	pinMode(SCK,INPUT);
	pinMode(SS,INPUT);
	digitalWrite(MOSI, HIGH);
	digitalWrite(MISO, HIGH);
	digitalWrite(SCK, HIGH);
	*/
	//pinMode(SCK, INPUT);
    //pinMode(MOSI, INPUT);
  }

  // Disable the M5 bus
  static void end();
  
  //Initialize the M5 Board
  static void Init();
  static bool IsFree();
  static bool IsBusy();
  //////////Set M5 device//////////
  static void SetEncoderMode(uint8_t m);
  static void KeyBeepEnable();
  static void KeyBeepDisable();
  static void SetKeyLightTime(uchar t);
  static void ShowRunLogo();
  static void HideRunLogo();
  static void SetRunLogoXY(uint8_t x,uint8_t y);
  static void ShowM5Logo();
  static void HideM5Logo();
  static void SetM5LogoXY(uint8_t x,uint8_t y);
  static void BeepEnable();
  static void BeepDisable();
  static void SetBeepTime(uint8_t t1,uint8_t t2);
  static void Beep();
  static void Beep1(uint8_t t1,uint8_t t2);
  static void SetButtonA(uint8_t length,char* p);
  static void SetButtonB(uint8_t length,char* p);
  static void SetButtonC(uint8_t length,char* p);
  //////////M5 LCD Function/////////
 
    //Adjust the contrast M5 LCD brightness
  static void Contrast(uint8_t ratio);  
  //Adjust the contrast M5 LCD Contrast ratio
  static void Light(uint8_t light);


  //Clear the LCD
  static void ClearScreen();
  static void FullScreen();
    //M5 output to draw point
  static void SetPixel(uint8_t x,uint8_t y);  
  //M5 output to clear point
  static void ClearPixel(uint8_t x,uint8_t y);
  //M5 output characters
  static void PutCh(uint8_t x,uint8_t y,char ch);
  static void PutCh_(uint8_t x,uint8_t y,char ch);
  static void PutCh_2X(uint8_t x,uint8_t y,char ch);
  static void PutCh_2X_(uint8_t x,uint8_t y,char ch);
  static void PutS(uchar x,uchar y,char* s);
  static void PutS_(uchar x,uchar y,char* s);
  static void PutS_2X(uchar x,uchar y,char* s);
  static void PutS_2X_(uchar x,uchar y,char* s);
  static void Line(uchar x1,uchar y1,uchar x2,uchar y2);
  static void Line_(uchar x1,uchar y1,uchar x2,uchar y2);
  static void Rect(uchar x1,uchar y1,uchar x2,uchar y2);
  static void Rect_(uchar x1,uchar y1,uchar x2,uchar y2);
  static void FillRect(uchar x1,uchar y1,uchar x2,uchar y2);
  static void ClearRect(uchar x1,uchar y1,uchar x2,uchar y2);






  //M5 output double characters
  static void PutDoubleChar(uint8_t x,uint8_t y,char ch);
  
  //M5 output character string
  static void PutStr(uint8_t x,uint8_t y,void *buf);
  
  //M5 output double size of character string
  static void PutDoubleStr(uint8_t x,uint8_t y,void *buf);


  //M5 output other side double characters
  static void PutDoubleChar_Other(uint8_t x,uint8_t y,char ch);
  

  
  //M5 output other side double size of character string
  static void PutDoubleStr_Other(uint8_t x,uint8_t y,void *buf); 
  static void PutStr_Other(uint8_t x,uint8_t y,void *buf); 

  
  //M5 output to Draw line
  static void DrawLine(uint8_t x1,uint8_t y1,uint8_t x2,uint8_t y2);
  
  //M5 output to Clear the line
  static void ClearLine(uint8_t x1,uint8_t y1,uint8_t x2,uint8_t y2);
  
  //M5 output to Draw rectangle
  static void DrawRectangle(uint8_t x1,uint8_t y1,uint8_t x2,uint8_t y2);

  //M5 output to clear rectangle
  static void ClearRectangle(uint8_t x1,uint8_t y1,uint8_t x2,uint8_t y2);
  
  //M5 output to Draw Entity rectangle
  static void DrawRect_Ent(uint8_t x1,uint8_t y1,uint8_t x2,uint8_t y2);

  //M5 output to clear Entity rectangle
  static void ClearRect_Ent(uint8_t x1,uint8_t y1,uint8_t x2,uint8_t y2);
  
  //M5 output to Draw chamfer rectangle
  static void DrawRect_cham(uint8_t x1,uint8_t y1,uint8_t x2,uint8_t y2);

  //M5 output to clear chamfer rectangle
  static void ClearRect_cham(uint8_t x1,uint8_t y1,uint8_t x2,uint8_t y2);
  
  //M5 output character string 16*4
  static void PutStrLine(uint8_t x,uint8_t y,void *buf);
  
  //M5 output character string 16*4    //x:行  //y:列
  static void PutStrLine(uint8_t x,uint8_t y,uint16_t data);
  
  //M5 clear character string 16*4    //x:行
  static void ClearStrLine(uint8_t x);
  
  //M5 clear character string 16*4    //x:行//y:列//long:长度
  static void ClearSpace(uint8_t x ,uint8_t y ,uint8_t len);
  
  //M5 output bmp 
  static void DrawFullScreen(void *buf);
  

  void PrintBmp(uint16_t index ,void *buf);
  
  //get anything press key 
  static byte GetKey();
  
  // This function is deprecated.  New applications should use
  // beginTransaction() to configure M5 settings.
  inline static void setBitOrder(uint8_t bitOrder) {
    if (bitOrder == LSBFIRST) SPCR |= _BV(DORD);
    else SPCR &= ~(_BV(DORD));
  }
  // This function is deprecated.  New applications should use
  // beginTransaction() to configure M5 settings.
  inline static void setDataMode(uint8_t dataMode) {
    SPCR = (SPCR & ~M5_MODE_MASK) | dataMode;
  }
  // This function is deprecated.  New applications should use
  // beginTransaction() to configure M5 settings.
  inline static void setClockDivider(uint8_t clockDiv) {
    SPCR = (SPCR & ~M5_CLOCK_MASK) | (clockDiv & M5_CLOCK_MASK);
    SPSR = (SPSR & ~M5_2XCLOCK_MASK) | ((clockDiv >> 2) & M5_2XCLOCK_MASK);
  }
  // These undocumented functions should not be used.  M5.transfer()
  // polls the hardware flag which is automatically cleared as the
  // AVR responds to M5's interrupt
  inline static void attachInterrupt() { SPCR |= _BV(SPIE); }
  inline static void detachInterrupt() { SPCR &= ~_BV(SPIE); }

private:
  static uint8_t initialized;
  static uint8_t interruptMode; // 0=none, 1=mask, 2=global
  static uint8_t interruptMask; // which interrupts to mask
  static uint8_t interruptSave; // temp storage, to restore state
  #ifdef M5_TRANSACTION_MISMATCH_LED
  static uint8_t inTransactionFlag;
  #endif
};

extern M5Class M5;

#endif
