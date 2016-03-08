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
    if(!(*reg & bit)){
      digitalWrite(SS, HIGH);
    }
    pinMode(SS, OUTPUT);
    SPCR |= _BV(MSTR);
    SPCR |= _BV(SPE);
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
 // SPCR &= ~_BV(SPE);
  //pinMode(SCK, INPUT);
 //pinMode(MOSI, INPUT);
 // pinMode(SS, INPUT);
 

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
  delay(1600);
  SET_M328CS_OUTPUT;
  SET_M328INT_INPUT;
  M328INT0;
  M328CS0;
  M5.begin();
  //*************Clear the Bus*****************
  M5.beginTransaction(settingA);  
  M5.transfer(0x0);
  M5.transfer(0x0);
  M5.transfer(0x0);
  M5.endTransaction();
  //delayMicroseconds(20);
  delay(200);
}
bool M5Class::IsFree()
{
	bool ret;
    M5.beginTransaction(settingA);  
    M5.transfer(0xaa);
    M5.transfer(0xff);
    ret=M5.transfer(0x55);
    M5.endTransaction();
    return ret;
}
bool M5Class::IsBusy()
{
	bool ret;
    M5.beginTransaction(settingA);  
    M5.transfer(0xaa);
    M5.transfer(0xff);
    ret=M5.transfer(0x55);
    M5.endTransaction();
	delay(1);
	return !ret;
	//if(ret==0x00)return false;
	//else return true;
}
void M5Class::SetEncoderMode(uint8_t m)
{
	M5.beginTransaction(settingA); 
    M5.transfer(0xaa);
    M5.transfer(0x13);
	M5.transfer(m);
    M5.transfer(0x55);
    M5.endTransaction();	
    delay(50);
}
//get anything press key 
byte M5Class::GetKey()
{
   M5.beginTransaction(settingA); 
   M5.transfer(0xaa);
   M5.transfer(0x10);
   byte key_data =M5.transfer(0x55);
   M5.endTransaction();	
   delay(20);
   return key_data;
}
void M5Class::KeyBeepEnable()
{
	beginTransaction(settingA);
	transfer(0xaa);
	transfer(0x11);
	transfer(0x01);
	transfer(0x55);
	endTransaction();
	delay(50);
}
void M5Class::KeyBeepDisable()
{
	beginTransaction(settingA);
	transfer(0xaa);
	transfer(0x11);
	transfer(0x00);
	transfer(0x55);
	endTransaction();
	delay(50);
}
void M5Class::SetKeyLightTime(uchar t)
{
	beginTransaction(settingA);
	transfer(0xaa);
	transfer(0x12);
	transfer(t);
	transfer(0x55);
	endTransaction();
	delay(50);
}

void M5Class::ShowRunLogo()
{
  beginTransaction(settingA); 
  transfer(0xaa);
  transfer(0x14);
  transfer(0x01);
  transfer(0x55);
  endTransaction();	
  delay(100);
}

void M5Class::HideRunLogo()
{
  beginTransaction(settingA); 
  transfer(0xaa);
  transfer(0x14);
  transfer(0x00);
  transfer(0x55);
  endTransaction();	
  delay(100);
}

void M5Class::SetRunLogoXY(uint8_t x,uint8_t y)
{
    beginTransaction(settingA); 
	transfer(0xaa);
	transfer(0x15);
	transfer(x);
	transfer(y);
	transfer(0x55);
	endTransaction();	
	delay(100);
}
void M5Class::ShowM5Logo()
{
  beginTransaction(settingA); 
  transfer(0xaa);
  transfer(0x16);
  transfer(0x01);
  transfer(0x55);
  endTransaction();	
  delay(100);
}

void M5Class::HideM5Logo()
{
  beginTransaction(settingA); 
  transfer(0xaa);
  transfer(0x16);
  transfer(0x00);
  transfer(0x55);
  endTransaction();	
  delay(100);
}
void M5Class::SetM5LogoXY(uint8_t x,uint8_t y)
{
    beginTransaction(settingA); 
	transfer(0xaa);
	transfer(0x17);
	transfer(x);
	transfer(y);
	transfer(0x55);
	endTransaction();	
	delay(150);
}
void M5Class::BeepEnable()
{
	beginTransaction(settingA);
	transfer(0xaa);
	transfer(0x18);
	transfer(0x01);
	transfer(0x55);
	endTransaction();
	delay(100);
}
void M5Class::BeepDisable()
{
	beginTransaction(settingA);
	transfer(0xaa);
	transfer(0x18);
	transfer(0x00);
	transfer(0x55);
	endTransaction();
	delay(100);
}
void M5Class::SetBeepTime(uint8_t time1,uint8_t time2)
{
	beginTransaction(settingA);
	transfer(0xaa);
	transfer(0x19);
	transfer(time1);
	transfer(time2);
	transfer(0x55);
	endTransaction();
	delay(50);
}
void M5Class::Beep1(uint8_t time1,uint8_t time2)
{
	beginTransaction(settingA);
	transfer(0xaa);
	transfer(0x1b);
	transfer(time1);
	transfer(time2);
	transfer(0x55);
	endTransaction();
	delayMicroseconds(50);
}
void M5Class::Beep()
{
	beginTransaction(settingA);
	transfer(0xaa);
	transfer(0x1a);
	transfer(0x55);
	endTransaction();
	//delayMicroseconds(50);
	delay(10);
	//delay(50);
}
void M5Class::SetButtonA(uint8_t length,char* p)
{
	beginTransaction(settingA);
	transfer(0xaa);
	transfer(0x1c);
	transfer(length);
	transfer(p[0]);
	transfer(p[1]);
	transfer(p[2]);
	transfer(p[3]);
	transfer(0x55);
	endTransaction();
	//delayMicroseconds(50);
	delay(100);
}
void M5Class::SetButtonB(uint8_t length,char* p)
{
	beginTransaction(settingA);
	transfer(0xaa);
	transfer(0x1d);
	transfer(length);
	transfer(p[0]);
	transfer(p[1]);
	transfer(p[2]);
	transfer(p[3]);
	transfer(0x55);
	endTransaction();
	//delayMicroseconds(50);
	delay(100);
}
void M5Class::SetButtonC(uint8_t length,char* p)
{
	beginTransaction(settingA);
	transfer(0xaa);
	transfer(0x1e);
	transfer(length);
	transfer(p[0]);
	transfer(p[1]);
	transfer(p[2]);
	transfer(p[3]);
	transfer(0x55);
	endTransaction();
	//delayMicroseconds(50);
	delay(100);
}


/////////////////////LCD Display/////////////////////
  //Adjust the contrast M5 LCD Contrast ratio
void M5Class::Contrast(uint8_t ratio)
{
  beginTransaction(settingA); 
  transfer(0xaa);
  transfer(0x20);
  transfer(ratio);
  transfer(0x55);
  endTransaction();
  delay(20);
}

//Adjust the contrast M5 LCD brightness
void M5Class::Light(uint8_t LIGHT)
{
  beginTransaction(settingA); 
  transfer(0xaa);
  transfer(0x21);
  transfer(LIGHT);
  transfer(0x55);
  endTransaction();	
  delay(20);
}
//Clear the LCD
void M5Class::ClearScreen()
{
  M5.beginTransaction(settingA); 
  M5.transfer(0xaa);
  M5.transfer(0x22);
  M5.transfer(0x55);
  M5.endTransaction();
  //while(!M5.IsBusy());
  delay(250);
}
//Clear the LCD
void M5Class::FullScreen()
{
	while(!M5.IsBusy());
  M5.beginTransaction(settingA); 
  M5.transfer(0xaa);
  M5.transfer(0x23);
  M5.transfer(0x55);
  M5.endTransaction();
  //while(!M5.IsBusy());
  delay(250);
}
//M5 output to draw point
void M5Class::SetPixel(uint8_t x,uint8_t y)
{
   M5.beginTransaction(settingA); 
   M5.transfer(0xaa);
   M5.transfer(0x24);
   M5.transfer(x);
   M5.transfer(y);
   M5.transfer(0x55);
   M5.endTransaction();	
   delay(25);
}

//M5 output to clear point
void M5Class::ClearPixel(uint8_t x,uint8_t y)
{
   M5.beginTransaction(settingA); 
   M5.transfer(0xaa);
   M5.transfer(0x25);
   M5.transfer(x);
   M5.transfer(y);
   M5.transfer(0x55);
   M5.endTransaction();	
   delay(25);
}
//The output characters
void M5Class::PutCh(uint8_t x,uint8_t y,char ch)
{
   M5.beginTransaction(settingA); 
   M5.transfer(0xaa);
   M5.transfer(0x26);
   M5.transfer(x);
   M5.transfer(y);
   M5.transfer(ch);
   M5.transfer(0x55);
   M5.endTransaction();	
   delay(10);
}
//M5 output other side character
void M5Class::PutCh_(uint8_t x,uint8_t y,char ch)
{
   M5.beginTransaction(settingA); 
   M5.transfer(0xaa);
   M5.transfer(0x27);
   M5.transfer(x);
   M5.transfer(y);
   M5.transfer(ch);
   M5.transfer(0x55);
   M5.endTransaction();	
	delay(10);
}
//The output characters
void M5Class::PutCh_2X(uint8_t x,uint8_t y,char ch)
{
   M5.beginTransaction(settingA); 
   M5.transfer(0xaa);
   M5.transfer(0x28);
   M5.transfer(x);
   M5.transfer(y);
   M5.transfer(ch);
   M5.transfer(0x55);
   M5.endTransaction();	
   delay(10);
}
//M5 output other side character
void M5Class::PutCh_2X_(uint8_t x,uint8_t y,char ch)
{
   M5.beginTransaction(settingA); 
   M5.transfer(0xaa);
   M5.transfer(0x29);
   M5.transfer(x);
   M5.transfer(y);
   M5.transfer(ch);
   M5.transfer(0x55);
   M5.endTransaction();	
	delay(10);
}

void M5Class::PutS(uchar x,uchar y,char* s)
{   
   M5.beginTransaction(settingA); 
   M5.transfer(0xaa);
   M5.transfer(0x2a);
   M5.transfer(x);
   M5.transfer(y);
   //M5.transfer(0xC4);
   //M5.transfer(0xE3);
   //M5.transfer(0xBA);
   //M5.transfer(0xC3);
   for(uchar i=0;i<16;i++)
   {
	   if(s[i]==0)goto b1;
	   M5.transfer(s[i]);		   
   }
b1:M5.transfer(0);
   M5.transfer(0x55);
   M5.endTransaction();	
   delay(20);	
}
void M5Class::PutS_(uchar x,uchar y,char* s)
{   
   M5.beginTransaction(settingA); 
   M5.transfer(0xaa);
   M5.transfer(0x2b);
   M5.transfer(x);
   M5.transfer(y);
   for(uchar i=0;i<16;i++)
   {
	   if(s[i]==0)goto b1;
	   M5.transfer(s[i]);		   
   }
b1:M5.transfer(0);
   M5.transfer(0x55);
   M5.endTransaction();	
   delay(20);	
}
void M5Class::PutS_2X(uchar x,uchar y,char* s)
{   
   M5.beginTransaction(settingA); 
   M5.transfer(0xaa);
   M5.transfer(0x2c);
   M5.transfer(x);
   M5.transfer(y);
   for(uchar i=0;i<16;i++)
   {
	   if(s[i]==0)goto b1;
	   M5.transfer(s[i]);		   
   }
b1:M5.transfer(0);
   M5.transfer(0x55);
   M5.endTransaction();	
   delay(20);	
}
void M5Class::PutS_2X_(uchar x,uchar y,char* s)
{   
   M5.beginTransaction(settingA); 
   M5.transfer(0xaa);
   M5.transfer(0x2d);
   M5.transfer(x);
   M5.transfer(y);
   for(uchar i=0;i<16;i++)
   {
	   if(s[i]==0)goto b1;
	   M5.transfer(s[i]);		   
   }
b1:M5.transfer(0);
   M5.transfer(0x55);
   M5.endTransaction();	
   delay(20);	
}

void M5Class::Line(uchar x1,uchar y1,uchar x2,uchar y2)
{   
   M5.beginTransaction(settingA); 
   M5.transfer(0xaa);
   M5.transfer(0x2e);
   M5.transfer(x1);
   M5.transfer(y1);
   M5.transfer(x2);
   M5.transfer(y2);
   M5.transfer(0x55);
   M5.endTransaction();	
   delay(50);	
}

void M5Class::Line_(uchar x1,uchar y1,uchar x2,uchar y2)
{   
   M5.beginTransaction(settingA); 
   M5.transfer(0xaa);
   M5.transfer(0x2f);
   M5.transfer(x1);
   M5.transfer(y1);
   M5.transfer(x2);
   M5.transfer(y2);
   M5.transfer(0x55);
   M5.endTransaction();	
   delay(50);	
}
void M5Class::Rect(uchar x1,uchar y1,uchar x2,uchar y2)
{   
   M5.beginTransaction(settingA); 
   M5.transfer(0xaa);
   M5.transfer(0x30);
   M5.transfer(x1);
   M5.transfer(y1);
   M5.transfer(x2);
   M5.transfer(y2);
   M5.transfer(0x55);
   M5.endTransaction();	
   delay(100);	
}
void M5Class::Rect_(uchar x1,uchar y1,uchar x2,uchar y2)
{   
   M5.beginTransaction(settingA); 
   M5.transfer(0xaa);
   M5.transfer(0x31);
   M5.transfer(x1);
   M5.transfer(y1);
   M5.transfer(x2);
   M5.transfer(y2);
   M5.transfer(0x55);
   M5.endTransaction();	
   delay(20);	
}
void M5Class::FillRect(uchar x1,uchar y1,uchar x2,uchar y2)
{   
   M5.beginTransaction(settingA); 
   M5.transfer(0xaa);
   M5.transfer(0x32);
   M5.transfer(x1);
   M5.transfer(y1);
   M5.transfer(x2);
   M5.transfer(y2);
   M5.transfer(0x55);
   M5.endTransaction();	
   delay(100);	
}
void M5Class::ClearRect(uchar x1,uchar y1,uchar x2,uchar y2)
{   
   M5.beginTransaction(settingA); 
   M5.transfer(0xaa);
   M5.transfer(0x33);
   M5.transfer(x1);
   M5.transfer(y1);
   M5.transfer(x2);
   M5.transfer(y2);
   M5.transfer(0x55);
   M5.endTransaction();	
   delay(100);	
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



