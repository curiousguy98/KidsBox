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

#include "M5.h"

M5Class M5;

M5Settings settingA(2000000,MSBFIRST,M5_MODE0);
uint8_t M5Class::initialized = 0;
uint8_t M5Class::interruptMode = 0;
uint8_t M5Class::interruptMask = 0;
uint8_t M5Class::interruptSave = 0;
#ifdef M5_TRANSACTION_MISMATCH_LED
uint8_t M5Class::inTransactionFlag = 0;
#endif

void M5Class::begin()
{
  uint8_t sreg = SREG;
  noInterrupts(); // Protect from a scheduler and prevent transactionBegin
  if (!initialized) {
    // Set SS to high so a connected chip will be "deselected" by default
    uint8_t port = digitalPinToPort(SS);
    uint8_t bit = digitalPinToBitMask(SS);
    volatile uint8_t *reg = portModeRegister(port);

    // if the SS pin is not already configured as an output
    // then set it high (to enable the internal pull-up resistor)
    if(!(*reg & bit)){
      digitalWrite(SS, HIGH);
    }

    // When the SS pin is set as OUTPUT, it can be used as
    // a general purpose output port (it doesn't influence
    // M5 operations).
    pinMode(SS, OUTPUT);

    // Warning: if the SS pin ever becomes a LOW INPUT then M5
    // automatically switches to Slave, so the data direction of
    // the SS pin MUST be kept as OUTPUT.
    SPCR |= _BV(MSTR);
    SPCR |= _BV(SPE);

    // Set direction register for SCK and MOSI pin.
    // MISO pin automatically overrides to INPUT.
    // By doing this AFTER enabling M5, we avoid accidentally
    // clocking in a single bit since the lines go directly
    // from "input" to M5 control.
    // http://code.google.com/p/arduino/issues/detail?id=888
    pinMode(SCK, OUTPUT);
    pinMode(MOSI, OUTPUT);
  }
  initialized++; // reference count
  SREG = sreg;
}

void M5Class::end() {
  uint8_t sreg = SREG;
  noInterrupts(); // Protect from a scheduler and prevent transactionBegin
  // Decrease the reference counter
  if (initialized)
    initialized--;
  // If there are no more references disable M5
  if (!initialized) {
    SPCR &= ~_BV(SPE);
    interruptMode = 0;
    #ifdef M5_TRANSACTION_MISMATCH_LED
    inTransactionFlag = 0;
    #endif
  }
  SREG = sreg;
}

// mapping of interrupt numbers to bits within M5_AVR_EIMSK
#if defined(__AVR_ATmega32U4__)
  #define M5_INT0_MASK  (1<<INT0)
  #define M5_INT1_MASK  (1<<INT1)
  #define M5_INT2_MASK  (1<<INT2)
  #define M5_INT3_MASK  (1<<INT3)
  #define M5_INT4_MASK  (1<<INT6)
#elif defined(__AVR_AT90USB646__) || defined(__AVR_AT90USB1286__)
  #define M5_INT0_MASK  (1<<INT0)
  #define M5_INT1_MASK  (1<<INT1)
  #define M5_INT2_MASK  (1<<INT2)
  #define M5_INT3_MASK  (1<<INT3)
  #define M5_INT4_MASK  (1<<INT4)
  #define M5_INT5_MASK  (1<<INT5)
  #define M5_INT6_MASK  (1<<INT6)
  #define M5_INT7_MASK  (1<<INT7)
#elif defined(EICRA) && defined(EICRB) && defined(EIMSK)
  #define M5_INT0_MASK  (1<<INT4)
  #define M5_INT1_MASK  (1<<INT5)
  #define M5_INT2_MASK  (1<<INT0)
  #define M5_INT3_MASK  (1<<INT1)
  #define M5_INT4_MASK  (1<<INT2)
  #define M5_INT5_MASK  (1<<INT3)
  #define M5_INT6_MASK  (1<<INT6)
  #define M5_INT7_MASK  (1<<INT7)
#else
  #ifdef INT0
  #define M5_INT0_MASK  (1<<INT0)
  #endif
  #ifdef INT1
  #define M5_INT1_MASK  (1<<INT1)
  #endif
  #ifdef INT2
  #define M5_INT2_MASK  (1<<INT2)
  #endif
#endif

void M5Class::usingInterrupt(uint8_t interruptNumber)
{
  uint8_t mask = 0;
  uint8_t sreg = SREG;
  noInterrupts(); // Protect from a scheduler and prevent transactionBegin
  switch (interruptNumber) {
  #ifdef M5_INT0_MASK
  case 0: mask = M5_INT0_MASK; break;
  #endif
  #ifdef M5_INT1_MASK
  case 1: mask = M5_INT1_MASK; break;
  #endif
  #ifdef M5_INT2_MASK
  case 2: mask = M5_INT2_MASK; break;
  #endif
  #ifdef M5_INT3_MASK
  case 3: mask = M5_INT3_MASK; break;
  #endif
  #ifdef M5_INT4_MASK
  case 4: mask = M5_INT4_MASK; break;
  #endif
  #ifdef M5_INT5_MASK
  case 5: mask = M5_INT5_MASK; break;
  #endif
  #ifdef M5_INT6_MASK
  case 6: mask = M5_INT6_MASK; break;
  #endif
  #ifdef M5_INT7_MASK
  case 7: mask = M5_INT7_MASK; break;
  #endif
  default:
    interruptMode = 2;
    break;
  }
  interruptMask |= mask;
  if (!interruptMode)
    interruptMode = 1;
  SREG = sreg;
}

void M5Class::notUsingInterrupt(uint8_t interruptNumber)
{
  // Once in mode 2 we can't go back to 0 without a proper reference count
  if (interruptMode == 2)
    return;
  uint8_t mask = 0;
  uint8_t sreg = SREG;
  noInterrupts(); // Protect from a scheduler and prevent transactionBegin
  switch (interruptNumber) {
  #ifdef M5_INT0_MASK
  case 0: mask = M5_INT0_MASK; break;
  #endif
  #ifdef M5_INT1_MASK
  case 1: mask = M5_INT1_MASK; break;
  #endif
  #ifdef M5_INT2_MASK
  case 2: mask = M5_INT2_MASK; break;
  #endif
  #ifdef M5_INT3_MASK
  case 3: mask = M5_INT3_MASK; break;
  #endif
  #ifdef M5_INT4_MASK
  case 4: mask = M5_INT4_MASK; break;
  #endif
  #ifdef M5_INT5_MASK
  case 5: mask = M5_INT5_MASK; break;
  #endif
  #ifdef M5_INT6_MASK
  case 6: mask = M5_INT6_MASK; break;
  #endif
  #ifdef M5_INT7_MASK
  case 7: mask = M5_INT7_MASK; break;
  #endif
  default:
    break;
    // this case can't be reached
  }
  interruptMask &= ~mask;
  if (!interruptMask)
    interruptMode = 0;
  SREG = sreg;
}

//Initialize the M5 Board
void M5Class::Init()
{
  delay(3000);
  M5.begin();
  SET_M328CS_OUTPUT;
  M328CS0;
  M5.begin();
  //*************Clear the Bus*****************
  M5.beginTransaction(settingA);  
  M5.transfer(0x0);delayMicroseconds(20);
  M5.transfer(0x0);delayMicroseconds(20);
  M5.endTransaction();delayMicroseconds(50);
  delay(100);return;
  //***********Switch the user mode************
  M5.beginTransaction(settingA); 
  M5.transfer(0xaa);delayMicroseconds(20);
  M5.transfer(0x5);delayMicroseconds(20);
  M5.transfer(0x55);delayMicroseconds(20);
  M5.endTransaction();delay(500);
  //*************Clear the LCD*****************
  M5.beginTransaction(settingA); 
  M5.transfer(0xaa);delayMicroseconds(20);
  M5.transfer(0x12);delayMicroseconds(20);
  M5.transfer(0x55);delayMicroseconds(20);
  M5.endTransaction();delay(500);
  delay(100);
}

  //Adjust the contrast M5 LCD Contrast ratio
void M5Class::Contrast(uint8_t ratio)
{
  M5.beginTransaction(settingA); 
  M5.transfer(0xaa);delayMicroseconds(20);
  M5.transfer(0x10);delayMicroseconds(20);
  M5.transfer(ratio);delayMicroseconds(20);
  M5.transfer(0x55);delayMicroseconds(20);
  M5.endTransaction();delay(500);
	
}

//Adjust the contrast M5 LCD brightness
void M5Class::Adjust(uint8_t LIGHT)
{
  M5.beginTransaction(settingA); 
  M5.transfer(0xaa);delayMicroseconds(20);
  M5.transfer(0x11);delayMicroseconds(20);
  M5.transfer(LIGHT);delayMicroseconds(20);
  M5.transfer(0x55);delayMicroseconds(20);
  M5.endTransaction();delay(500);
	
}

//Switch the user mode
void M5Class::UserMode()
{
  M5.beginTransaction(settingA); 
  M5.transfer(0xaa);delayMicroseconds(20);
  M5.transfer(0x5);delayMicroseconds(20);
  M5.transfer(0x55);delayMicroseconds(20);
  M5.endTransaction();delay(500);	
}

//Clear the LCD
void M5Class::ClearScreen()
{
  M5.beginTransaction(settingA); 
  M5.transfer(0xaa);delayMicroseconds(20);
  M5.transfer(0x12);delayMicroseconds(20);
  M5.transfer(0x55);delayMicroseconds(20);
  M5.endTransaction();delay(500);
}

//The output characters
void M5Class::PutChar(uint8_t x,uint8_t y,char ch)
{
   M5.beginTransaction(settingA); 
   M5.transfer(0xaa);delayMicroseconds(20);
   M5.transfer(0x16);delayMicroseconds(20);
   M5.transfer(x);delayMicroseconds(20);
   M5.transfer(y);delayMicroseconds(20);
   M5.transfer(ch);delayMicroseconds(20);
   M5.transfer(0x55);delayMicroseconds(20);
   M5.endTransaction();	
}
void M5Class::PutS(uchar x,uchar y,char* s)
{
	uchar i=0;
	for(char *p = &s[0]; *p; p++)
	{ 
		if(*s == 0)return;
		M5.PutChar(x+8*i,y,*p);
		i++;
		delay(40);
	}
}
//The output double size characters
void M5Class::PutDoubleChar(uint8_t x,uint8_t y,char ch)
{
   M5.beginTransaction(settingA); 
   M5.transfer(0xaa);delayMicroseconds(20);
   M5.transfer(0x18);delayMicroseconds(20);
   M5.transfer(x);delayMicroseconds(20);
   M5.transfer(y);delayMicroseconds(20);
   M5.transfer(ch);delayMicroseconds(20);
   M5.transfer(0x55);delayMicroseconds(20);
   M5.endTransaction();	
}

//M5 output character string 16*4    
//x:行
//y:列
void M5Class::PutStrLine(uint8_t x,uint8_t y,void *buf)
{
   uint8_t *p = (uint8_t *)buf;
   uint8_t count = 0;
   
   M5.beginTransaction(settingA); 
   M5.transfer(0xaa);delayMicroseconds(20);
   M5.transfer(0x1a);delayMicroseconds(20);
   x = 16*(x-1)+1;
   y = 8*(y-1)+1;
   M5.transfer(y);delayMicroseconds(20);
   M5.transfer(x);delayMicroseconds(20);
   while(p[count]!='\0')
   {
	  M5.transfer(p[count]);
	  delayMicroseconds(20);
	  count++;
   }
   M5.transfer(0x0);delayMicroseconds(20);
   M5.transfer(0x55);delayMicroseconds(20);
   M5.endTransaction();		
}

//M5 output character string 16*4    
//x:行
//y:列
void M5Class::PutStrLine(uint8_t x,uint8_t y,uint16_t data)
{
   uchar p[7]="";
   uint8_t count = 0,index = 0;
   uint8_t a=0 , b=0,Sp = 0;
   
   x = 16*(x-1)+1;
   y = 8*(y-1)+1;
   a = x;
   b = y;
   /*M5.beginTransaction(settingA);
   M5.transfer(0xaa);delayMicroseconds(20);
   M5.transfer(0x1a);delayMicroseconds(20);

   M5.transfer(y);delayMicroseconds(20);
   M5.transfer(x);delayMicroseconds(20);
   
   while(Sp<=5)
   {
	  M5.transfer(' ');
	  delayMicroseconds(20);
	  Sp++;
   }
   
   M5.transfer(0x0);delayMicroseconds(20);
   M5.transfer(0x55);delayMicroseconds(20);
   M5.endTransaction();	*/
   
   
   	  p[0] = data/100000+'0';
	  p[1] = data%100000/10000+'0';
      p[2] = data%10000/1000+'0';
      p[3] = data%1000/100+'0';
      p[4] = data%100/10+'0'; 
      p[5] = data%10+'0';
	  p[6] = '\0';
	  
   while(p[index] == '0')
   {
	  index++;
   }
   if(index>=4)
   {
	  index = 4; 
   }	   
   count = index;
   
   //*********************************************************
   
   M5.beginTransaction(settingA);
   
   M5.transfer(0xaa);delayMicroseconds(20);
   M5.transfer(0x1a);delayMicroseconds(20);

   M5.transfer(y);delayMicroseconds(20);
   M5.transfer(x);delayMicroseconds(20);
   
   while(p[count]!='\0')
   {
	  M5.transfer(p[count]);
	  delayMicroseconds(20);
	  count++;
   }
   M5.transfer(0x0);delayMicroseconds(20);
   M5.transfer(0x55);delayMicroseconds(20);
   M5.endTransaction();		
}

//M5 clear character string 16*4    
//x:行
void M5Class::ClearStrLine(uint8_t x)
{
   uint8_t count = 0,y = 0;
   int a = 0,b = 0;
   a = x;b = x;
   M5.beginTransaction(settingA); 
   M5.transfer(0xaa);delayMicroseconds(20);
   M5.transfer(0x1a);delayMicroseconds(20);
   a = 16*(a-1)+1;
   y = 1;
   M5.transfer(y);delayMicroseconds(20);
   M5.transfer(a);delayMicroseconds(20);
   count = 8;
   while(count!=0)
   {
	  M5.transfer(' ');
	  delayMicroseconds(20);
	  count--;
   }
   M5.transfer(0x0);delayMicroseconds(20);
   M5.transfer(0x55);delayMicroseconds(20);
   M5.endTransaction();	
   delay(100);
//**********************************************
   
   M5.beginTransaction(settingA); 
   M5.transfer(0xaa);delayMicroseconds(20);
   M5.transfer(0x1a);delayMicroseconds(20);
   b = 16*(b-1)+1;
   y = 65;
   M5.transfer(y);delayMicroseconds(20);
   M5.transfer(b);delayMicroseconds(20);
   count = 8;
   while(count!=0)
   {
	  M5.transfer(' ');
	  delayMicroseconds(20);
	  count--;
   }
   M5.transfer(0x0);delayMicroseconds(20);
   M5.transfer(0x55);delayMicroseconds(20);
   M5.endTransaction();	   
}

//M5 clear character string 16*4    
//x:行
//y:列
//long:长度
void M5Class::ClearSpace(uint8_t x ,uint8_t y ,uint8_t len)
{
   uint8_t count = 0;
   int a = 0,b = 0,c = 0;
   a = x,b = y,c = len;

   if(len>12)
   {
	 M5.beginTransaction(settingA); 
     M5.transfer(0xaa);delayMicroseconds(20);
     M5.transfer(0x1a);delayMicroseconds(20);
     a = 16*(a-1)+1;
     b = 8*(b-1)+1;
     M5.transfer(b);delayMicroseconds(20);
     M5.transfer(a);delayMicroseconds(20);
	 count = 12;
     while(count!=0)
     {
	  M5.transfer(' ');
	  delayMicroseconds(20);
	  count--;
     }
     M5.transfer(0x0);delayMicroseconds(20);
     M5.transfer(0x55);delayMicroseconds(20);
     M5.endTransaction();	
     delay(100); 
	 
	 //*********************************
	 
	 M5.beginTransaction(settingA); 
     M5.transfer(0xaa);delayMicroseconds(20);
     M5.transfer(0x1a);delayMicroseconds(20);
	 
     x = 16*(x-1)+1;
     y = 8*(y+12-1)+1;
     M5.transfer(y);delayMicroseconds(20);
     M5.transfer(x);delayMicroseconds(20);
	 
	 c = len - 12;
     while(c!=0)
     {
	  M5.transfer(' ');
	  delayMicroseconds(20);
	  c--;
     }
     M5.transfer(0x0);delayMicroseconds(20);
     M5.transfer(0x55);delayMicroseconds(20);
     M5.endTransaction();
   }
   else if(len<=12)
   {
     M5.beginTransaction(settingA); 
     M5.transfer(0xaa);delayMicroseconds(20);
     M5.transfer(0x1a);delayMicroseconds(20);
     x = 16*(x-1)+1;
     y = 8*(y-1)+1;
     M5.transfer(y);delayMicroseconds(20);
     M5.transfer(x);delayMicroseconds(20);
	 
     while(len!=0)
     {
	    M5.transfer(' ');
	    delayMicroseconds(20);
	    len--;
     }
     M5.transfer(0x0);delayMicroseconds(20);
     M5.transfer(0x55);delayMicroseconds(20);
     M5.endTransaction();	
     delay(150);
   }   
}


//M5 output double size of character string
void M5Class::PutDoubleStr(uint8_t x,uint8_t y,void *buf)
{
   uint8_t *p = (uint8_t *)buf;
   uint8_t count = 0;

   M5.beginTransaction(settingA); 
   M5.transfer(0xaa);delayMicroseconds(20);
   M5.transfer(0x1c);delayMicroseconds(20);
   M5.transfer(x);delayMicroseconds(20);
   M5.transfer(y);delayMicroseconds(20);
   while(p[count]!='\0')
   {
	  M5.transfer(p[count]);
	  delayMicroseconds(20);
	  count++;
   }
   M5.transfer(0x0);delayMicroseconds(20);
   M5.transfer(0x55);delayMicroseconds(20);
   M5.endTransaction();		
}

//M5 output Full screen the LCD
void M5Class::FullScreen()
{
  M5.beginTransaction(settingA); 
  M5.transfer(0xaa);delayMicroseconds(20);
  M5.transfer(0x13);delayMicroseconds(20);
  M5.transfer(0x55);delayMicroseconds(20);
  M5.endTransaction();delay(500);
	
}

//M5 output other side character
void M5Class::PutChar_Other(uint8_t x,uint8_t y,char ch)
{
   M5.beginTransaction(settingA); 
   M5.transfer(0xaa);delayMicroseconds(20);
   M5.transfer(0x17);delayMicroseconds(20);
   M5.transfer(x);delayMicroseconds(20);
   M5.transfer(y);delayMicroseconds(20);
   M5.transfer(ch);delayMicroseconds(20);
   M5.transfer(0x55);delayMicroseconds(20);
   M5.endTransaction();	
	
}

//The output other side and double size characters
void M5Class::PutDoubleChar_Other(uint8_t x,uint8_t y,char ch)
{
   M5.beginTransaction(settingA); 
   M5.transfer(0xaa);delayMicroseconds(20);
   M5.transfer(0x19);delayMicroseconds(20);
   M5.transfer(x);delayMicroseconds(20);
   M5.transfer(y);delayMicroseconds(20);
   M5.transfer(ch);delayMicroseconds(20);
   M5.transfer(0x55);delayMicroseconds(20);
   M5.endTransaction();	
}

//M5 output other side character string
void M5Class::PutStr_Other(uint8_t x,uint8_t y,void *buf)
{
   uint8_t *p = (uint8_t *)buf;
   uint8_t count = 0;

   M5.beginTransaction(settingA); 
   M5.transfer(0xaa);delayMicroseconds(20);
   M5.transfer(0x1b);delayMicroseconds(20);
   M5.transfer(x);delayMicroseconds(20);
   M5.transfer(y);delayMicroseconds(20);
   while(p[count]!='\0')
   {
	  M5.transfer(p[count]);
	  delayMicroseconds(20);
	  count++;
   }
   M5.transfer(0x0);delayMicroseconds(20);
   M5.transfer(0x55);delayMicroseconds(20);
   M5.endTransaction();		
}

//M5 output other side and double size of character string
void M5Class::PutDoubleStr_Other(uint8_t x,uint8_t y,void *buf)
{
   uint8_t *p = (uint8_t *)buf;
   uint8_t count = 0;

   M5.beginTransaction(settingA); 
   M5.transfer(0xaa);delayMicroseconds(20);
   M5.transfer(0x1d);delayMicroseconds(20);
   M5.transfer(x);delayMicroseconds(20);
   M5.transfer(y);delayMicroseconds(20);
   while(p[count]!='\0')
   {
	  M5.transfer(p[count]);
	  delayMicroseconds(20);
	  count++;
   }
   M5.transfer(0x0);delayMicroseconds(20);
   M5.transfer(0x55);delayMicroseconds(20);
   M5.endTransaction();		
}

//M5 output to draw point
void M5Class::DrawPoint(uint8_t x,uint8_t y)
{
   M5.beginTransaction(settingA); 
   M5.transfer(0xaa);delayMicroseconds(20);
   M5.transfer(0x14);delayMicroseconds(20);
   M5.transfer(x);delayMicroseconds(20);
   M5.transfer(y);delayMicroseconds(20);
   M5.transfer(0x55);delayMicroseconds(20);
   M5.endTransaction();	
}

//M5 output to clear point
void M5Class::ClearPoint(uint8_t x,uint8_t y)
{
   M5.beginTransaction(settingA); 
   M5.transfer(0xaa);delayMicroseconds(20);
   M5.transfer(0x15);delayMicroseconds(20);
   M5.transfer(x);delayMicroseconds(20);
   M5.transfer(y);delayMicroseconds(20);
   M5.transfer(0x55);delayMicroseconds(20);
   M5.endTransaction();	
}

//M5 output to Draw line
void M5Class::DrawLine(uint8_t x1,uint8_t y1,uint8_t x2,uint8_t y2)
{
   M5.beginTransaction(settingA); 
   M5.transfer(0xaa);delayMicroseconds(20);
   M5.transfer(0x1e);delayMicroseconds(20);
   M5.transfer(x1);delayMicroseconds(20);
   M5.transfer(y1);delayMicroseconds(20);
   M5.transfer(x2);delayMicroseconds(20);
   M5.transfer(y2);delayMicroseconds(20);
   M5.transfer(0x55);delayMicroseconds(20);
   M5.endTransaction();	
}

//M5 output to Clear the line
void M5Class::ClearLine(uint8_t x1,uint8_t y1,uint8_t x2,uint8_t y2)
{
   M5.beginTransaction(settingA); 
   M5.transfer(0xaa);delayMicroseconds(20);
   M5.transfer(0x1f);delayMicroseconds(20);
   M5.transfer(x1);delayMicroseconds(20);
   M5.transfer(y1);delayMicroseconds(20);
   M5.transfer(x2);delayMicroseconds(20);
   M5.transfer(y2);delayMicroseconds(20);
   M5.transfer(0x55);delayMicroseconds(20);
   M5.endTransaction();	
}

//M5 output to Draw rectangle
void M5Class::DrawRectangle(uint8_t x1,uint8_t y1,uint8_t x2,uint8_t y2)
{
   M5.beginTransaction(settingA); 
   M5.transfer(0xaa);delayMicroseconds(20);
   M5.transfer(0x20);delayMicroseconds(20);
   M5.transfer(x1);delayMicroseconds(20);
   M5.transfer(y1);delayMicroseconds(20);
   M5.transfer(x2);delayMicroseconds(20);
   M5.transfer(y2);delayMicroseconds(20);
   M5.transfer(0x55);delayMicroseconds(20);
   M5.endTransaction();
}

//M5 output to clear rectangle
void M5Class::ClearRectangle(uint8_t x1,uint8_t y1,uint8_t x2,uint8_t y2)
{
   M5.beginTransaction(settingA); 
   M5.transfer(0xaa);delayMicroseconds(20);
   M5.transfer(0x21);delayMicroseconds(20);
   M5.transfer(x1);delayMicroseconds(20);
   M5.transfer(y1);delayMicroseconds(20);
   M5.transfer(x2);delayMicroseconds(20);
   M5.transfer(y2);delayMicroseconds(20);
   M5.transfer(0x55);delayMicroseconds(20);
   M5.endTransaction();
}

//M5 output to Draw Entity rectangle
void M5Class::DrawRect_Ent(uint8_t x1,uint8_t y1,uint8_t x2,uint8_t y2)
{
   M5.beginTransaction(settingA); 
   M5.transfer(0xaa);delayMicroseconds(20);
   M5.transfer(0x22);delayMicroseconds(20);
   M5.transfer(x1);delayMicroseconds(20);
   M5.transfer(y1);delayMicroseconds(20);
   M5.transfer(x2);delayMicroseconds(20);
   M5.transfer(y2);delayMicroseconds(20);
   M5.transfer(0x55);delayMicroseconds(20);
   M5.endTransaction();
}

//M5 output to clear Entity rectangle
void M5Class::ClearRect_Ent(uint8_t x1,uint8_t y1,uint8_t x2,uint8_t y2)
{
   M5.beginTransaction(settingA); 
   M5.transfer(0xaa);delayMicroseconds(20);
   M5.transfer(0x23);delayMicroseconds(20);
   M5.transfer(x1);delayMicroseconds(20);
   M5.transfer(y1);delayMicroseconds(20);
   M5.transfer(x2);delayMicroseconds(20);
   M5.transfer(y2);delayMicroseconds(20);
   M5.transfer(0x55);delayMicroseconds(20);
   M5.endTransaction();
}

//M5 output to Draw  chamfer rectangle
void M5Class::DrawRect_cham(uint8_t x1,uint8_t y1,uint8_t x2,uint8_t y2)
{
   M5.beginTransaction(settingA); 
   M5.transfer(0xaa);delayMicroseconds(20);
   M5.transfer(0x24);delayMicroseconds(20);
   M5.transfer(x1);delayMicroseconds(20);
   M5.transfer(y1);delayMicroseconds(20);
   M5.transfer(x2);delayMicroseconds(20);
   M5.transfer(y2);delayMicroseconds(20);
   M5.transfer(0x0D);delayMicroseconds(20);
   M5.transfer(0x55);delayMicroseconds(20);
   M5.endTransaction();
}

//M5 output to clear  chamfer rectangle
void M5Class::ClearRect_cham(uint8_t x1,uint8_t y1,uint8_t x2,uint8_t y2)
{
   M5.beginTransaction(settingA); 
   M5.transfer(0xaa);delayMicroseconds(20);
   M5.transfer(0x25);delayMicroseconds(20);
   M5.transfer(x1);delayMicroseconds(20);
   M5.transfer(y1);delayMicroseconds(20);
   M5.transfer(x2);delayMicroseconds(20);
   M5.transfer(y2);delayMicroseconds(20);
   M5.transfer(0x0D);delayMicroseconds(20);
   M5.transfer(0x55);delayMicroseconds(20);
   M5.endTransaction();
}

//M5 output bmp 
/*void M5Class::DrawFullScreen(void *buf)
{
   uint8_t *p = (uint8_t *)buf;
   int count = 0;
   M5.beginTransaction(settingA); 
   M5.transfer(0xaa);delayMicroseconds(20);
   M5.transfer(0x26);delayMicroseconds(20);

   while(count<512)
   {
	  M5.transfer(p[count]);
	  delayMicroseconds(100);
	  count++;
   }
   M5.transfer(0xff);delayMicroseconds(20);
   M5.transfer(0x55);delayMicroseconds(20);
   M5.endTransaction();	
}*/

void M5Class::DrawFullScreen(void *buf)
{
   uint8_t *p = (uint8_t *)buf;
   int count = 0,i = 0,j = 0,n = 0;
   uint16_t data_flag = 0,index = 0;
   
   M5.beginTransaction(settingA); 
			  
	 for(i=0;i<128;i++)
	 {
	   for(j=0;j<8;j++)
	   {
		 index = 8*i+j;
		 if(p[index]!=0x00)
		 {
		   for(n=0;n<8;n++)
		   {
		     data_flag = Get_Bit(p[index],n);
	         if(data_flag!=0x00)
		     {
                M5.transfer(0xaa);delayMicroseconds(20);
                M5.transfer(0x14);delayMicroseconds(20);
                M5.transfer(i+1);delayMicroseconds(20);
                M5.transfer((j*8+(n+1)));delayMicroseconds(20);
                M5.transfer(0x55);delayMicroseconds(20);
			    delayMicroseconds(150);
		     }   
		   }
		 }
	   }
	 }
	 
	M5.endTransaction();
}

//M5 output character string 128*64
void M5Class::PutStr(uint8_t x,uint8_t y,void *buf)
{
   uint8_t *p = (uint8_t *)buf;
   uint8_t count = 0;

   M5.beginTransaction(settingA); 
   M5.transfer(0xaa);delayMicroseconds(20);
   M5.transfer(0x1a);delayMicroseconds(20);
   M5.transfer(x);delayMicroseconds(20);
   M5.transfer(y);delayMicroseconds(20);
   while(p[count]!='\0')
   {
	  M5.transfer(p[count]);
	  delayMicroseconds(20);
	  count++;
   }
   M5.transfer(0x0);delayMicroseconds(20);
   M5.transfer(0x55);delayMicroseconds(20);
   M5.endTransaction();		
}


/*void M5Class::DrawFullScreen(void *buf)
{
   uint8_t *p = (uint8_t *)buf;
   uint16_t index = 0,count_x = 0;
   uint8_t  i = 0;
   M5.beginTransaction(settingA);

   while(index<1024)
   {
 
      M5.transfer(0xaa);delayMicroseconds(20);
      M5.transfer(0x26);delayMicroseconds(20);
	  M5.transfer(count_x);delayMicroseconds(20);
	  count_x++;
	  
	  for(i=0;i<8;i++)
	  {
		M5.transfer(p[index]);delayMicroseconds(20); 
        index++;		
	  }
	  M5.transfer(0x55);delayMicroseconds(20);
	  delay(5);
   }
   M5.endTransaction();
}*/

//get anything press key 
byte M5Class::GetKey()
{
   M5.beginTransaction(settingA); 
   
   byte key_data = M5.transfer(0xff);delayMicroseconds(20);
   M5.endTransaction();	
   return key_data;
}

