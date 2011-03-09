/*--------------------------------------------------------------------------
P89V51Rx2.H
Registers definition for Philips P89V51Rx2

Copyright (c) 2007 Keil Elektronik GmbH and Keil Software, Inc.
All rights reserved.
-------------------------------------------------------------------------- */

/* Byte Addresses */
sfr   P0   	 = 0x80;
sfr   SP   	 = 0x81;
sfr   DPL   	 = 0x82;
sfr   DPH   	 = 0x83;
sfr   WDTD   	 = 0x85;
sfr   SPDAT   	 = 0x86;
sfr   PCON   	 = 0x87;
sfr   TCON   	 = 0x88;
sfr   TMOD   	 = 0x89;
sfr   TL0   	 = 0x8A;
sfr   TL1   	 = 0x8B;
sfr   TH0   	 = 0x8C;
sfr   TH1   	 = 0x8D;
sfr   AUXR   	 = 0x8E;
sfr   P1   	 = 0x90;
sfr   SCON   	 = 0x98;
sfr   SBUF   	 = 0x99;
sfr   P2   	 = 0xA0;
sfr   AUXR1   	 = 0xA2;
sfr   IEN0   	 = 0xA8;
sfr   SADDR   	 = 0xA9;
sfr   SPCFG   	 = 0xAA;
sfr   P3   	 = 0xB0;
sfr   FCF   	 = 0xB1;
sfr   FST   	 = 0xB6;
sfr   IP0H   	 = 0xB7;
sfr   IP0   	 = 0xB8;
sfr   SADEN   	 = 0xB9;
sfr   WDTC   	 = 0xC0;
sfr   T2CON   	 = 0xC8;
sfr   T2MOD   	 = 0xC9;
sfr   RCAP2L   	 = 0xCA;
sfr   RCAP2H   	 = 0xCB;
sfr   TL2   	 = 0xCC;
sfr   TH2        = 0xCD;
sfr   PSW   	 = 0xD0;
sfr   SPCTL   	 = 0xD5;
sfr   CCON   	 = 0xD8;
sfr   CMOD   	 = 0xD9;
sfr   CCAPM0   	 = 0xDA;
sfr   CCAPM1   	 = 0xDB;
sfr   CCAPM2   	 = 0xDC;
sfr   CCAPM3   	 = 0xDD;
sfr   CCAPM4   	 = 0xDE;
sfr   ACC   	 = 0xE0;
sfr   IEN1   	 = 0xE8;
sfr   CL   	 = 0xE9;
sfr   CCAP0L   	 = 0xEA;
sfr   CCAP1L   	 = 0xEB;
sfr   CCAP2L   	 = 0xEC;
sfr   CCAP3L   	 = 0xED;
sfr   CCAP4L   	 = 0xEE;
sfr   B   	 = 0xF0;
sfr   IP1H   	 = 0xF7;
sfr   IP1   	 = 0xF8;
sfr   CH   	 = 0xF9;
sfr   CCAP0H   	 = 0xFA;
sfr   CCAP1H   	 = 0xFB;
sfr   CCAP2H   	 = 0xFC;
sfr   CCAP3H   	 = 0xFD;
sfr   CCAP4H   	 = 0xFE;

/* Bit Addresses */

/* P0 */
sbit   AO   	 = P0^0;
sbit   EXTRAM   	 = P0^1;

/* TCON */
sbit   IT0   	 = TCON^0;
sbit   IE0   	 = TCON^1;
sbit   IT1   	 = TCON^2;
sbit   IE1   	 = TCON^3;
sbit   TR0   	 = TCON^4;
sbit   TF0   	 = TCON^5;
sbit   TR1   	 = TCON^6;
sbit   TF1   	 = TCON^7;

/* P1 */
sbit   T2EX   	 = P1^1;
sbit   ECI   	 = P1^2;
sbit   CEX0   	 = P1^3;
sbit   CEX1   	 = P1^4;
sbit   CEX2   	 = P1^5;
sbit   CEX3   	 = P1^6;
sbit   CEX4   	 = P1^7;

/* SCON */
sbit   RI   	 = SCON^0;
sbit   TI   	 = SCON^1;
sbit   RB8   	 = SCON^2;
sbit   TB8   	 = SCON^3;
sbit   REN   	 = SCON^4;
sbit   SM3   	 = SCON^5;
sbit   SM1   	 = SCON^6;
sbit   SM0   	 = SCON^7;

/* P2 */
sbit   DPS   	 = P2^0;
sbit   GF2   	 = P2^3;

/* IEN0 */
sbit   EX0   	 = IEN0^0;
sbit   ET0   	 = IEN0^1;
sbit   EX1   	 = IEN0^2;
sbit   ET1   	 = IEN0^3;
sbit   ES0   	 = IEN0^4;
sbit   ET2   	 = IEN0^5;
sbit   EC   	 = IEN0^6;
sbit   EA   	 = IEN0^7;

/* P3 */
sbit   RxD   	 = P3^0;
sbit   TxD   	 = P3^1;
sbit   INT0   	 = P3^2;
sbit   INT1   	 = P3^3;
sbit   T0   	 = P3^4;
sbit   T1   	 = P3^5;
sbit   WR   	 = P3^6;
sbit   RD   	 = P3^7;

/* IP0 */
sbit   PX0   	 = IP0^0;
sbit   PT0   	 = IP0^1;
sbit   PX1   	 = IP0^2;
sbit   PT1       = IP0^3;
sbit   PS   	 = IP0^4;
sbit   PT2   	 = IP0^5;
sbit   PPC   	 = IP0^6;

/* WDTC */
sbit   SWDT   	 = WDTC^0;
sbit   WDT   	 = WDTC^1;
sbit   WDTS   	 = WDTC^2;
sbit   WDRE   	 = WDTC^3;
sbit   WDOUT   	 = WDTC^4;

/* T2CON */
sbit   RL2   	 = T2CON^0;
sbit   T2   	 = T2CON^1;
sbit   TR2   	 = T2CON^2;
sbit   EXEN2   	 = T2CON^3;
sbit   TCLK   	 = T2CON^4;
sbit   RCLK   	 = T2CON^5;
sbit   EXF2   	 = T2CON^6;
sbit   TF2   	 = T2CON^7;

/* PSW */
sbit   P   	 = PSW^0;
sbit   F1   	 = PSW^1;
sbit   OV   	 = PSW^2;
sbit   RS0   	 = PSW^3;
sbit   RS1   	 = PSW^4;
sbit   F0   	 = PSW^5;
sbit   AC   	 = PSW^6;
sbit   CY   	 = PSW^7;

/* CCON */
sbit   CCF0   	 = CCON^0;
sbit   CCF1   	 = CCON^1;
sbit   CCF2   	 = CCON^2;
sbit   CCF3   	 = CCON^3;
sbit   CCF4   	 = CCON^4;
sbit   CR   	 = CCON^6;
sbit   CF   	 = CCON^7;

/* IEN1 */
sbit   EBO   	 = IEN1^3;

/* IP1 */
sbit   PBO   	 = IP1^3;
