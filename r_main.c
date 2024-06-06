/***********************************************************************************************************************
* DISCLAIMER
* This software is supplied by Renesas Electronics Corporation and is only intended for use with Renesas products.
* No other uses are authorized. This software is owned by Renesas Electronics Corporation and is protected under all
* applicable laws, including copyright laws. 
* THIS SOFTWARE IS PROVIDED "AS IS" AND RENESAS MAKES NO WARRANTIES REGARDING THIS SOFTWARE, WHETHER EXPRESS, IMPLIED
* OR STATUTORY, INCLUDING BUT NOT LIMITED TO WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
* NON-INFRINGEMENT.  ALL SUCH WARRANTIES ARE EXPRESSLY DISCLAIMED.TO THE MAXIMUM EXTENT PERMITTED NOT PROHIBITED BY
* LAW, NEITHER RENESAS ELECTRONICS CORPORATION NOR ANY OF ITS AFFILIATED COMPANIES SHALL BE LIABLE FOR ANY DIRECT,
* INDIRECT, SPECIAL, INCIDENTAL OR CONSEQUENTIAL DAMAGES FOR ANY REASON RELATED TO THIS SOFTWARE, EVEN IF RENESAS OR
* ITS AFFILIATES HAVE BEEN ADVISED OF THE POSSIBILITY OF SUCH DAMAGES.
* Renesas reserves the right, without notice, to make changes to this software and to discontinue the availability 
* of this software. By using this software, you agree to the additional terms and conditions found by accessing the 
* following link:
* http://www.renesas.com/disclaimer
*
* Copyright (C) 2011, 2021 Renesas Electronics Corporation. All rights reserved.
***********************************************************************************************************************/

/***********************************************************************************************************************
* File Name    : r_main.c
* Version      : CodeGenerator for RL78/G13 V2.05.06.02 [08 Nov 2021]
* Device(s)    : R5F100GE
* Tool-Chain   : CCRL
* Description  : This file implements main function.
* Creation Date: 2024/02/22
***********************************************************************************************************************/

/***********************************************************************************************************************
Includes
***********************************************************************************************************************/
#include "r_cg_macrodriver.h"
#include "r_cg_cgc.h"
#include "r_cg_port.h"
#include "r_cg_serial.h"
#include "r_cg_timer.h"
#include "r_cg_it.h"
/* Start user code for include. Do not edit comment generated here */
#include	<string.h>
#include	<stdio.h>
/* End user code. Do not edit comment generated here */
#include "r_cg_userdefine.h"

/***********************************************************************************************************************
Pragma directive
***********************************************************************************************************************/
/* Start user code for pragma. Do not edit comment generated here */


/* 標準的なもの */
#define LOW			0U
#define HIGH		1U	
#define DISABLE		0U
#define ENABLE		1U							
#define FALSE		0U
#define TRUE		1U
#define CLEAR		0U
#define SET			1U

#define	FREE		0U
#define	BUSY		1U

#define	CLR		0U
#define SET		1U
#define	CHK		9U

#define	TEST	1U

#define	rAMC	0x3586
#define	rASC	0x3581

/* macro */

#define	LOAD165		(P7_bit.no3 = 0)
#define	SIFT165		(P7_bit.no3 = 1)
#define	CLK_LOW		(P7_bit.no4 = 0)
#define	CLK_HIGH	(P7_bit.no4 = 1)

#define	SIFT595		(P7_bit.no1 = 0)
#define	LOAD595		(P7_bit.no1 = 1)
#define	SRCLK_LOW	(P7_bit.no2 = 0)
#define	SRCLK_HIGH	(P7_bit.no2 = 1)

#define		LEDMONI		(P6_bit.no2)


#define		RMAX	9200
#define		RMIN	4000
#define		COMP	6000

#define		NUL		0x00
#define		STX		0x02
#define		ETX		0x03
#define		EOT		0x04
#define		ENQ		0x05
#define		ACK		0x06
#define		NAK		0x15

#define		SIZE	1	
#define		EMPTY		0
#define		AVAILABLE	1
#define		FULL		2

#define		SOLBIT		0b0100000000000000 	// OUT15の位置
#define		ROCKBIT		0b0001000000000000	// TIN13の位置	0=施錠/ 1=解錠
#define		DOOROPEN	0b0010000000000000	// TIN14の位置	0=閉扉/ 1=開扉

#define		BUFLIM		624
#define		ROOMMAX		32

#define		LIFETIME1	4000
#define		LIFETIME2	90000

#define 	START485\
        	{\
            DebugTimer(8);\
			P14_bit.no6 = 1;\
			P14_bit.no7 = 1;\
			DebugTimer(1);\
        	}
			
#define		ENABLE485\
			{\
			P14_bit.no6 = 1;\
			P14_bit.no7 = 1;\
			}
					
#define		DISABLE485\
			{\
			P14_bit.no6 = 0;\
			P14_bit.no7 = 0;\
			}
			
#define		RECV485		(P14_bit.no7 = 0)		


	
/* End user code. Do not edit comment generated here */

/***********************************************************************************************************************
Global variables and functions
***********************************************************************************************************************/
/* Start user code for global. Do not edit comment generated here */

volatile uint8_t	g_tx1_ready_flag =FREE;	//DebugPort
volatile uint8_t	g_tx2_ready_flag =FREE;	//RS485

uint8_t		gbuf[BUFLIM];
uint8_t		gcomflg ='\0';
uint32_t	gremdata= 0;
uint8_t		grcvlrc ='\0';


uint32_t	glifecnt;
uint32_t	glifetime =LIFETIME2;
uint8_t		gloopcnt ='\0';

uint8_t		grockstatus =0;
/*
struct	buf_t{
	
	//uint8_t		buf[20];
	uint16_t	len;
	uint16_t	timer;
	
}tbuf,*ptbuf;
*/

uint8_t		gtxbuf[20];
uint16_t	gtxlen =0;
uint16_t	gtxtimer =0;


uint8_t		grnum[ROOMMAX] ={'\0'};			

uint8_t 	goutp[16] ={'\0'};

uint8_t		gaddr;
uint8_t		grockmode;
uint8_t		gascmode;

struct	ascflg_t{
	
	uint8_t		y;
	uint8_t		p;
	uint8_t		i;
	uint8_t		g;			//初G電文
}gcomm,*pgcomm;


uint16_t	gterminaldata;
uint16_t	golddata;

struct	queue_t{
	
    uint16_t    data[SIZE+1];   /* stac+1個 */
    uint8_t     pop;        	/* head */ 
    uint8_t     push;       	/* tail */
    uint8_t     flag;
}Queue, *pQueue;


struct  remcode{
        uint16_t key;
        uint16_t rnumb;
}
const   remcode[41] ={  0xFF00,0x3031,			/* 01 */
                        0xFE01,0x3036,			/* 06 */
                        0xFD02,0x3042,			/* 0B */
                        0xFC03,0x3130,			/* 10 */
                        0xFB04,0x3032,			/* 02 */
                        0xFA05,0x3037,			/* 07 */
                        0xF906,0x3043,          /* 0C */
                        0xF807,0x3131,          /* 11 */
                        0xF708,0x3033,          /* 03 */
                        0xF609,0x3038,          /* 08 */
                        0xF50A,0x3044,          /* 0D */
                        0xF40B,0x3132,          /* 12 */
                        0xF30C,0x3034,          /* 04 */
                        0xF20D,0x3039,          /* 09 */
                        0xF10E,0x3045,          /* 0E */
                        0xF00F,0x3133,          /* 13 */
                        0xEF10,0x3035,          /* 05 */
                        0xEE11,0x3041,          // 0A,
                        0xED12,0x3046,          // 0F,
                        0xEC13,0x3134,          // 14,
                        0xBF40,0x3135,          // 15,
                        0xBE41,0x3141,          // 1A,
                        0xBD42,0x3146,          // 1F,
                        0xBC43,0x3234,          // 24,
                        0xBB44,0x3136,          // 16,
                        0xBA45,0x3142,          // 1B,
                        0xB946,0x3230,          // 20,
                        0xB847,0x3235,          // 25,
                        0xB748,0x3137,          // 17,
                        0xB649,0x3143,          // 1C,
                        0xB54A,0x3231,          // 21,
                        0xB44B,0x3236,          // 26,
                        0xB34C,0x3138,          // 18,
                        0xB24D,0x3144,          // 1D,
                        0xB14E,0x3232,          // 22,
                        0xB04F,0x3237,          // 27,
                        0xAF50,0x3139,          // 19,
                        0xAE51,0x3145,          // 1E,
                        0xAD52,0x3233,          // 23,
                        0xAC53,0x3238,          // 28,
                        0xACAC,0x3030           // 00
                        };  
struct  aschextable{
    uint8_t    	ascii;
    uint8_t 	hex;
}
const   aschextable[] ={ '0',0x00,
                        '1',0x01,
                        '2',0x02,
                        '3',0x03,
                        '4',0x04,
                        '5',0x05,
                        '6',0x06,
                        '7',0x07,
                        '8',0x08,
                        '9',0x09,
                        'A',0x0A,
                        'B',0x0B,
                        'C',0x0C,
                        'D',0x0D,
                        'E',0x0E,
                        'F',0x0F
};


union	inidata{
	
	uint8_t		all[224];
	uint8_t		room[32][7];
};
#if 1
union inidata pattern ={ 0 };

#else
union inidata pattern ={	0,0,0,0,0,0,0,	//00	テスト用
							0,0,0,0,0,0,0,	//01		
							0,0,0,0,0,0,0,	//02
							0,8,0,0,0,0,0,	//03
							0,0,2,0,0,0,0,	//04
							0,0,8,0,0,0,0,	//05
							0,0,0,2,0,0,0,	//06
							0,0,0,8,0,0,0,	//07
							0,0,0,0,2,0,0,	//08
							0,0,0,0,8,0,0,	//09
							0,0,0,0,0,2,0,	//0A
							0,0,0,0,0,8,0,	//0B
							0,0,0,0,0,0,2,	//0C
							0,0,0,0,0,0,8,	//0D
							0,0,0,0,0,0,0,	//0E
							0,0,0,0,0,0,0,	//0F
							0,0,0,0,0,0,0,	//10
							0,0,0,0,0,0,0,	//11
							0,0,0,0,0,0,0,	//12	
							0,0,0,0,0,0,0,	//13
							0,0,0,0,0,0,0,	//14
							0,0,0,0,0,0,0,	//15
							0,0,0,0,0,0,0,	//16	
							0,0,0,0,0,0,0,	//17
							1,0,0,0,0,0,0,	//18
							0,1,0,0,0,0,0,	//19
							0,0,1,0,0,0,0,	//1A
							0,0,0,1,0,0,0,	//1B	
							0,0,0,0,1,0,0,	//1C
							0,0,0,0,0,1,0,	//1D
							0,0,0,0,0,0,1,	//1E
							0,0,0,0,0,0,0	//1F
						};
#endif


uint8_t		testformat[11] ={0x02,'A','S','C','4','1','0','0',0x03,0x57,'\0'};

extern volatile 	uint16_t  g_uart2_rx_length;

/*

		functions	

*/

void		spiOut(uint32_t value, uint8_t len);
uint8_t		serialPrint1(uint8_t *buf, uint16_t len);
uint8_t		serialPrint2(uint8_t *buf, uint16_t len);

void		serRead2(uint8_t al);
void		startFa(void);
uint32_t	hc165Read(uint8_t  len);
uint32_t	hc165Read2(uint8_t  len);
uint8_t		asciiConvert(uint8_t type, uint8_t al);
uint8_t 	makeLrc(const uint8_t* buf, uint8_t lim);
uint8_t		*dataSwap(uint8_t *buf);
uint8_t 	convData(uint8_t *dest, const uint8_t *buf);
uint16_t	tformdeQueue(uint8_t *buf, uint16_t val);
uint8_t		serComp(uint8_t	cmd, uint8_t val);

uint8_t		addrChk(const uint8_t *buf, uint8_t mode);
uint8_t		stxComm(const uint8_t *buf, struct ascflg_t *pgcomm);
uint8_t		enqComm(const uint8_t *buf, struct queue_t *pQueue);

uint8_t		enqCommY(const uint8_t *buf, struct ascflg_t *pgcomm); 
uint8_t		stxCommY(const uint8_t *buf, struct ascflg_t *pgcomm);

uint8_t		enQueue(struct queue_t *pQueue, uint16_t value);
uint16_t	deQueue(struct queue_t *pQueue);
//void		initQueue(struct queue_t *pQ);

uint8_t		bufCopy(uint8_t *dest, uint8_t *source, uint16_t len);

void 		remRec(uint32_t	pwidth);
uint16_t	select(uint16_t	data);
uint8_t		remConv(uint32_t data, uint8_t *buf);


uint16_t	terminalCap(uint16_t cnt);
uint16_t	terminalOut(uint16_t cnt);
uint8_t		rockStatans(uint16_t data,uint8_t status);

//uint16_t	solCnt(uint16_t sdata, const uint8_t al);
uint8_t		solCnt1(uint8_t al, uint8_t cnt);
uint8_t		solCnt2(uint8_t al, uint8_t cnt);

uint16_t	serOut595(uint8_t *buf);
uint8_t		getMode(void);

int8_t		comPression(const uint8_t *data);
uint8_t		kUnfold(uint8_t *dest, const uint8_t *source);
void    	popEep(uint8_t	*addr);


void    	dataCopy(uint8_t *dest, uint8_t *source, uint8_t num);
void    	dataSetY(uint8_t *dest, uint8_t	*source, uint16_t num);
void    	convDataY(uint8_t *dest, uint8_t *addr, uint8_t lim);
void    	allOrY(uint8_t *dest, uint8_t  *source, uint8_t lim);
uint16_t	serOut595Y(uint8_t *buf);
uint16_t	yajiOut(uint16_t cnt);


void		testMode(uint8_t ctl);
void		asc4100s(uint8_t ctl);
void		asc4100y(uint8_t ctl);

void		serialEnd(void);
void		waitTimer(uint32_t cunt);

void		ledMoni(uint16_t cunt);

void		info(void);
void		pollingOut(void);

uint8_t		serialOut(uint8_t *dest ,uint16_t len);

void	DebugPrint2(uint8_t *s,uint16_t n);

/* End user code. Do not edit comment generated here */
void R_MAIN_UserInit(void);

/***********************************************************************************************************************
* Function Name: main
* Description  : This function implements main function.
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
void main(void)
{
    R_MAIN_UserInit();
    /* Start user code. Do not edit comment generated here */
	
	g_uart2_rx_length =0U;

    pQueue 	=&Queue;
	pQueue->flag =EMPTY;
	pQueue->pop =pQueue->push ='\0';
	
	pgcomm 	=&gcomm;
	pgcomm->y =pgcomm->p =pgcomm->i =pgcomm->g =0;
	
	spiOut(CLR, 16);       		// HC595 ALL"0"CLR
	P14_bit.no7 =0;				// RECV485

	gascmode =getMode();		// b3=LAN b2=YAJI b1=?? b0=TEST　マスク無しで反転してくる
	
    glifetime =(gascmode &0x02)? LIFETIME2 :LIFETIME1;
	glifecnt  =LIFETIME2;		//一旦入れておく
	
	gaddr =(uint8_t)(hc165Read(31) >>8);	//反転してくる
	grockmode =gaddr &0b11000000;
    gaddr &=(gascmode &0x04)? 0b00001111: 0b00111111;	//b2 =YAJI
	
	info();
	testMode(gascmode &0x01);
    startFa();
	
	/* ASC4100 loop */
	asc4100y(gascmode &0x04);
	
	asc4100s(TRUE);
	
    /* End user code. Do not edit comment generated here */
}

/***********************************************************************************************************************
* Function Name: R_MAIN_UserInit
* Description  : This function adds user code before implementing main function.
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
void R_MAIN_UserInit(void)
{
    /* Start user code. Do not edit comment generated here */
	
	R_IT_Start();
	R_TAU0_Channel3_Start();
	
	R_UART1_Stop();		// Monitor
	SDR02 = 0x8A00U;	//115200Bps設定 Baseは76800Bps
	R_UART1_Start();	// Monitor
	
	R_UART2_Start();	// RS485
	
    EI();
    /* End user code. Do not edit comment generated here */
}

/* Start user code for adding. Do not edit comment generated here */
/***********************************************************************************************************************
* Function Name: 
* Description  : 
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
void	info(void){
	
 	uint8_t	buf[30];
						
		sprintf(buf ,"DATE is %s \n" ,__DATE__);	
        serialPrint1(buf,strlen(buf));
}
/***********************************************************************************************************************
* Function Name: 
* Description  : 
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
void	asc4100s(uint8_t ctl){
	
    while (ctl){
		
		static	uint16_t	taskTime0 =10;
		static	uint16_t	taskTime1 =10;
		static	uint8_t		cout =0;
		
		
		if(gloopcnt != cout){			//CountUpは 1mS
			cout =gloopcnt;				//0-9を繰り返す
			
			serialOut(gtxbuf,gtxlen);	//485送信用
			
			ledMoni(500);				//Debug用
			
			
			switch(cout){
				
				case	0:				//10回転で10msec
					terminalCap(taskTime0);
					break;
					
				case	5:
					taskTime1= terminalOut(taskTime1);
					break;
					
				case	9:
					pollingOut();
					break;	
					
				
				default:
					break;
			}
			
			if(gcomflg =='?'){
				gcomflg= '\0';
				
				if(addrChk(gbuf,'S')){
					glifecnt= glifetime;
					enqComm(gbuf, pQueue);
				}
			}
			else if(gcomflg =='!'){
				gcomflg ='\0';
				
				if(addrChk(gbuf,'S')){
					glifecnt= glifetime;
					
					if(stxComm(gbuf,pgcomm)){				//'G'以外は'\0'を返す
						grockstatus =convData(goutp, gbuf);	// gbuf並変->data分割->goutp
						taskTime1 =1;
						pgcomm->g ='G';
					}
					
				}
			}
			
    	}//for 1mSEC loop
		
	}//for while(true)
}//for asc4100S

/***********************************************************************************************************************
* Function Name: 
* Description  : 
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
void	asc4100y(uint8_t ctl){
	
	 while (ctl){
		 
		static	uint16_t	taskTime0= 40;
		static	uint8_t		cout;
		
		if(gloopcnt != cout){			//CountUpは 1mS
			cout =gloopcnt;				//0-9を繰り返す
						
			serialOut(gtxbuf,gtxlen);
			
			ledMoni(500);				//Debug用
			
			switch(cout){
				
				case	0:			// 10回転で10msec
					pollingOut();
				break;
				
				case	5:			// 10回転で10msec
					taskTime0= yajiOut(taskTime0);
				break;
								
				default:
				break;
				
			}// for switch
			
			if(gcomflg=='?'){
				gcomflg ='\0';
				
				if(addrChk(gbuf,'Y')){
					glifecnt  =glifetime;			// addr一致でLIFE更新
					enqCommY(gbuf,pgcomm);
				}
			}
			else if(gcomflg=='!'){
				gcomflg ='\0';
				if(addrChk(gbuf,'Y')){
					glifecnt  =glifetime;
					stxCommY(gbuf,pgcomm);
				}
			}
		}// for if
	 }// for while
}// for asc4100y
/***********************************************************************************************************************
* Function Name: 
* Description  : 
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
void	pollingOut(void){
	
		if(glifecnt==0){
			
			uint16_t i;
			for(i =0;i <16;i++)	goutp[i] ='\0';
			
			pQueue->flag =EMPTY;
			pQueue->pop =pQueue->push ='\0';
			pgcomm->y =pgcomm->p =pgcomm->i ='\0';
			
			glifecnt =glifetime;
		}
}
/***********************************************************************************************************************
* Function Name: 
* Description  : 
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
uint8_t		serialOut(uint8_t *dest, uint16_t len){
	
			
			if(gtxtimer >0){
				--gtxtimer;			//5->4->3->で3mSEC
				
				if(gtxtimer ==2){
					
					P14_bit.no6 = 1;	//ENABLE485;
					P14_bit.no7 = 1;	
				}
				else if(gtxtimer ==1)	serialPrint2(dest,len);
				
				else if(gtxtimer ==0){
					
					P14_bit.no6 = 0;	//DISABLE485;
					P14_bit.no7 = 0;
				}
			}
			return	0;
}
/***********************************************************************************************************************
* Function Name: 
* Description  : 
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
uint8_t		serialPrint2(uint8_t *buf ,uint16_t len){

	
	while(g_tx2_ready_flag != FREE)	__nop();
	
	g_tx2_ready_flag = BUSY;
	R_UART2_Send((uint8_t *)buf, len);
		
	while(g_tx2_ready_flag != FREE)	{ __nop();}
	
	return	0;
}
/***********************************************************************************************************************
* Function Name: 
* Description  : 
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
void	serialWait(uint16_t tc){

	while(tc--) __nop();	
}
/***********************************************************************************************************************
* Function Name: 
* Description  : 
* Arguments    : None
* Return Value : 反転出力
***********************************************************************************************************************/
uint32_t	hc165Read(uint8_t  len){
	
	uint32_t	ans= 0;
	uint8_t		i;
	
	LOAD165;
	NOP();
	SIFT165;
	
	for(i= 0; i< len; i++){
		
		CLK_LOW;
		if(P7_bit.no0)	ans|= 1U;
		ans <<= 1;
		CLK_HIGH;
		NOP();
	}
	return	~ans;
}
/***********************************************************************************************************************
* Function Name: 
* Description  : 
* Arguments    : None
* Return Value : 反転出力
***********************************************************************************************************************/
uint32_t	hc165Read2(uint8_t  len){
	
	uint32_t	ans= 0;
	uint8_t		i;
	
	LOAD165;
	NOP();
	SIFT165;
	
	for(i= 0; i< len; i++){
		
		CLK_LOW;
		if(P7_bit.no0)	ans|= 0x80000000;
		ans >>= 1;
		CLK_HIGH;
		NOP();
	}
	return	~ans;
}
/***********************************************************************************************************************
* Function Name: 
* Description  : 
* Arguments    : number
* Return Value : returntype'H' return(hex), type 'A' return(ASCII)
***********************************************************************************************************************/
uint8_t	asciiConvert(uint8_t type, uint8_t al){
	
	uint8_t	i;
	int8_t	ans =-1;
	
	if(type== 'A'){
		
		for(i=0; i< 16; i++){
        	if(aschextable[i].hex == al){
            	ans= aschextable[i].ascii;		//0x30-0x39,0x41-0x46
            	break;
        	}
    	}
	}
	else if(type== 'H'){
		
		for(i=0; i<16; i++){ 
        	if(aschextable[i].ascii == al){
            	ans= aschextable[i].hex;		//0x00-0x0F
            	break;
        	}
    	}
	}
	return	ans;
}
/***********************************************************************************************************************
* Function Name: 
* Description  : 
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
void	startFa(void){

		uint8_t	i;
		uint8_t buf[2];
		
		while(TRUE){						//0x05待ち
		
			if(gcomflg =='?') break;		//誰か呼ばれている
			if(gcomflg =='!') break;		//誰か呼ばれている
			//if(0x04 == gbuf2[i++]) break;	//0x04が無 = 親機起動無
			//i &= 64;						//64 = 0x40	gbufのsize
		}
		buf[0] =0xfa;
		
		ENABLE485;
		waitTimer(2);
		
		for(i =0;i<30;i++)	serialPrint2(buf,1);
		
		waitTimer(1);
		DISABLE485;
}
/***********************************************************************************************************************
* Function Name: 
* Description  : 
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
void	serialEnd(void){

	g_tx2_ready_flag =FREE;
	//DISABLE485;
}
/***********************************************************************************************************************
* Function Name: 
* Description  : 
* Arguments    : Code
* Return Value : None
***********************************************************************************************************************/
void	serRead2(uint8_t al){
	
	static	uint16_t 	i= 0;
	static	uint8_t 	rxflg = 0;
	
	if(gcomflg !=0) return;			// busy
	
	gbuf[i] =al;
	
	if(rxflg ==0x03){					//* 0x03受信後のLRCで受信完了
		rxflg =0;
		gcomflg ='!';
		grcvlrc =al;					//0x03受信後のalはLRC
		return;
	}
	
	if(gbuf[i] ==0x03 &&rxflg ==0x02)	rxflg =0x03; 	//* 0x02 受信後の 0x03
	
	else if(gbuf[i] ==0x02 /*&&rxflg ==0*/){	//* 0x02受信
		i =0;
		gbuf[i] =rxflg =0x02;
	}
		
	else if(gbuf[i] ==ENQ /*&&rxflg ==0*/){		//* ENQ受信
		gcomflg =i =0;					//gcomflgもclrしてみるか？
		gbuf[i] =rxflg =ENQ;
	}
	else if(rxflg ==ENQ &&i ==2){		//* ENQ後 2BYTEでENQ電文受信完了
		gcomflg ='?';
		rxflg =0;					
	}
	
	if(++i >BUFLIM) i =gcomflg =rxflg =0;
}
/***********************************************************************************************************************
* Function Name: 
* Description  : 
* Arguments    : Code
* Return Value : None
***********************************************************************************************************************/
uint8_t		addrChk(const uint8_t *buf, uint8_t mode){
	
	uint8_t	addr;
	
	if(mode=='S'){
	
		addr = asciiConvert('H',buf[2]);
		addr|= (asciiConvert('H',buf[1]) << 4);
		return (gaddr==addr)? TRUE: FALSE;
	}
    if(mode=='Y'){
		
		if('D'!= buf[1]) return FALSE;
		addr = (asciiConvert('H',buf[2])& 0b00001111);
		return (gaddr==addr)? TRUE: FALSE;
	}
    return  FALSE;
}
/***********************************************************************************************************************
* Function Name: ENQ応答の処理
* Description  : 
* Arguments    : Code
* Return Value : None
***********************************************************************************************************************/
uint8_t		enqComm(const uint8_t *buf, struct queue_t *pQueue){

		uint8_t		lbuf[20];
	
	if(gcomm.y == '\0')   return	0;	//参加許可前の返信禁止

	if(pQueue->flag ==AVAILABLE){

		golddata =tformdeQueue(lbuf, deQueue(pQueue));		//送信bufに各dataをsetしてくる
		dataSwap(lbuf);
		if(rASC ==(uint16_t)gremdata)	gremdata= remConv(gremdata, (lbuf+14));
        *(lbuf+17)= makeLrc(lbuf, 0x03);
		*(lbuf+18)= '\0';	//Debug用
		bufCopy(gtxbuf,lbuf,18);
	}
	else if(rASC ==(uint16_t)gremdata){
		
		tformdeQueue(lbuf, gterminaldata);
		dataSwap(lbuf);			// buf[4]-buf[11]並び変え
		gremdata= remConv(gremdata, (lbuf+14));
		*(lbuf+17)= makeLrc(lbuf, 0x03);
		*(lbuf+18)= '\0';	//Debug用
		bufCopy(gtxbuf,lbuf,18);
		serialPrint1(lbuf, 18);	// ******TEST******
	}
	else{
		*lbuf = EOT;            // 状態変化無
		*(lbuf+1)= '\0';		//Debug用
		bufCopy(gtxbuf,lbuf,1);
	}
	return	0;
}
/***********************************************************************************************************************
* Function Name: STX応答の処理
* Description  : 
* Arguments    : txdbuf,combuf
* Return Value : None
***********************************************************************************************************************/
uint8_t		stxComm(const uint8_t *buf, struct ascflg_t *command){
	
	uint8_t		lbuf[10];
	uint8_t		ans= '\0';

	
	if(buf[3]== 'V'){
		
		*lbuf  	= 0x02;
		*(lbuf+1)	= 'F';		//strcpy(txbuf+1,"FFx00");
		*(lbuf+2)	= 'F';
		*(lbuf+3)	= 'x';
		*(lbuf+4)	= '0';
		*(lbuf+5)	= '0';
		*(lbuf+6)	= 0x03;
		*(lbuf+7)	= 0x7B;
		*(lbuf+8)	= '\0';		//Debug用
		bufCopy(gtxbuf,lbuf,8);
	}
	else if(buf[3]== 'Y'){
		
		*lbuf= ACK;
		
		command->y ='Y';
		command->i =0;
		command->g =0;
		
		Queue.flag =EMPTY;
		Queue.pop =Queue.push ='\0';
		
	}
	else if(buf[3]=='G' && buf[13]== makeLrc(buf, 0x03)){	// G電文は出力指示 buf[13]か,rcvlrcか
	
		ans= 'G';
		*lbuf= ACK;
		*(lbuf+1)	= '\0';		//Debug用
		bufCopy(gtxbuf,lbuf,1);
	}
	else{
		
		*lbuf= NAK;
		*(lbuf+1)	= '\0';		//Debug用
		bufCopy(gtxbuf,lbuf,1);
	}
	return	ans;
}
/***********************************************************************************************************************
* Function Name: 
* Description  : 
* Arguments    : Code
* Return Value : None
***********************************************************************************************************************/
uint8_t 	makeLrc(const uint8_t* buf, uint8_t lim){

    uint8_t tmp;

    tmp= *(++buf);	//0x02 pass

    do{
		
        tmp^= *(++buf);
		
    }while(*buf != lim);
	
    return  tmp;
}


uint8_t	calcLrc(const uint8_t* buf, uint8_t delim){
	
	uint8_t		al = 0;				//'D' ^ dipaddr; 
	uint16_t	i;
		
	al =buf[1];						// buf[0]=0x02はPass
	
	for(i =2; i <590; i++){			// 全 dataは 582
		
		al^= buf[i];				// al = al ^ data[i];
		if(buf[i]== delim) break;
	}
	if(i >588) return NAK;
	
	return(buf[++i]== al)? ACK: NAK;// 0x03の次はLRC
}
/***********************************************************************************************************************
* Function Name: conversion data Multiplex stxComm()から呼ばれる
* Description  : 受信した*bufは１byteに2dataが圧縮されている、これを1byteずつ*out「」に展開する
* Arguments    : Code
* Return Value : None
***********************************************************************************************************************/
uint8_t convData(uint8_t *dest, const uint8_t *buf){
	
	uint8_t	i,j,ans;
	uint8_t temp[8];		//b[3]'G' b[4]3-4 b[5]1-2 b[6]7-8 b[7]5-6 b[8]11-12 b[9]9-10 b[10]15-16 b[11]13-14
	
	temp[0]= buf[5];		//並び変え
    temp[1]= buf[4];
    temp[2]= buf[7]; 
    temp[3]= buf[6];
    temp[4]= buf[9];
    temp[5]= buf[8];
    temp[6]= buf[11];
    temp[7]= buf[10];
	
	
	for(i =0,j =0; j <8; j++){
		
		temp[j] =asciiConvert('H', temp[j]);
		
        dest[i++]= temp[j] &0x03;
        dest[i++]= temp[j] >>2;
    }
	
	if(grockmode==0b00000000)	dest[14] =0x00;
	if(dest[14] ==0x02)	dest[14] =0x00;	//ROCKはBLINKさせない
	
	
	ans= dest[14];					// Rock指示だけ抽出して返す
	
	for(i= 0; i< 15+1; i++){     	// buf[4]- buf[11]はoutp[0]- outp[15]に変わった
                                	// 更にOneShot用の加工の為
        if(dest[i]== 0x03){      	// 0x03=OneShot -> 0x05に置き換え
            dest[i] =0x05;       	// 5->4 この間0.4SEC
        } 
	}
	return ans;
}

/***********************************************************************************************************************
* Function Name: 
* Description  : 
* Arguments    : Code
* Return Value : None
***********************************************************************************************************************/
uint16_t	terminalCap(uint16_t cnt){
	
	static	uint16_t	last3data,last2data,last1data,last0data;
	static 	uint16_t	lcnt =0;
			uint8_t		moni;
	

	if((lcnt = ++lcnt % cnt) ==0){
		
		last3data =last2data;
		last2data =last1data;
        last1data =last0data;
        last0data =((uint16_t)(hc165Read2(31)) &(~0xC000));	// inputは14本しかない
		
		if(last3data==last2data &&last2data==last1data &&last1data==last0data){  // 3回一致->LEVEL検出

			gterminaldata =last0data;
			
			if(golddata !=last0data){
				
				if(gcomm.i <5)	golddata =last0data;
				else	moni =enQueue(pQueue, last0data);
				
				DebugPrint2("Queue=",(uint16_t)moni);
			}
            /**/
        }
	}
	return	cnt;
}
/***********************************************************************************************************************
* Function Name: 
* Description  : 
* Arguments    : Code
* Return Value : None
***********************************************************************************************************************/
void	initQueue(struct queue_t *pQ){
    
    int i;
    
    for(i= 0; i< SIZE; i++)	pQ->data[i]= 0; //キューの中身を0埋め
        
    pQ->pop= 0;
    pQ->push= 0;
    pQ->flag= EMPTY;
}
/***********************************************************************************************************************
* Function Name: 
* Description  : 
* Arguments    : Code
* Return Value : None
***********************************************************************************************************************/
uint8_t		enQueue(struct queue_t *pQueue, uint16_t value){
	
	uint8_t		ans =FALSE;
	
	if(pQueue->flag ==EMPTY){
		
		pQueue->data[0] = value;
		pQueue->flag =AVAILABLE;
		ans =TRUE;
	}
	
	return	ans;
}
/***********************************************************************************************************************
* Function Name: 
* Description  : 
* Arguments    : Code
* Return Value : None
***********************************************************************************************************************/
uint16_t	deQueue(struct queue_t *pQueue){
	
    pQueue->flag =EMPTY;  		//AVAILABLEだから来た
	return	pQueue->data[0]; 	//dequeue操作
}
/***********************************************************************************************************************
* Function Name: 
* Description  : deQueue -> Transform
* Arguments    : Code
* Return Value : None
***********************************************************************************************************************/
uint16_t	tformdeQueue(uint8_t *buf, uint16_t val){

	static	uint16_t	lastdata =0;
			uint16_t    chengebit;
            uint32_t    mask;
			uint8_t     i;
			
			
			chengebit =lastdata ^val;
    		lastdata =val;
						
			for(i= 4; i< 16; i++)   buf[i]= 0x00; //準備 4-15
			
			mask= 1U;			//入力変化Bit
			for(i =4; i <8; i++){
				
				if((chengebit &mask) !=0)   buf[i] =0x01;
        		if((chengebit &(mask <<=1)) !=0)   buf[i] |=0x02;
        		if((chengebit &(mask <<=1)) !=0)   buf[i] |=0x04;
        		if((chengebit &(mask <<=1)) !=0)   buf[i] |=0x08;
        		mask<<= 1;
				
				buf[i] =asciiConvert('A',buf[i]);
			}
			mask= 1U;			//入力状態Bit  IN1-IN14 
			for(i =8; i <12; i++){
				
        		if((lastdata &mask) !=0)   buf[i] =0x01;
        		if((lastdata &(mask <<=1)) !=0)   buf[i] |=0x02;
        		if((lastdata &(mask <<=1)) !=0)   buf[i] |=0x04;
        		if((lastdata &(mask <<=1)) !=0)   buf[i] |=0x08;
        		mask<<= 1;
				
				buf[i] =asciiConvert('A',buf[i]);
    		}
			buf[0]= 0x02;
    		buf[1]= 'F';      //0x46
    		buf[2]= 'F';
    		buf[3]= 'g';
			//buf[4]
			//buf[5]
			//buf[6]
			//buf[7]
			//buf[8]
			//buf[9]
			//buf[10]
			//buf[11]
			buf[12]='0';
			buf[13]= rockStatans(lastdata,grockstatus);
			buf[14]='0';		//rem
			buf[15]='0';		//rem
			buf[16]= 0x03;
			
			return	lastdata;
}
/***********************************************************************************************************************
* Function Name: 
* Description  :  gterminaldata
* Arguments    : 
* Return Value : 解錠信号
***********************************************************************************************************************/
uint8_t	rockStatans(uint16_t termnal, uint8_t stat){

		
	if(grockmode ==0b11000000)	return (termnal&ROCKBIT)? '1': '0'; //0=施錠 1=解錠　裏menu	
	//if(grockmode ==0b11000000)	return (stat)? '0': '1';	//0=施錠 1=解錠　裏menu	
	if(grockmode ==0b01000000)	return (termnal&ROCKBIT)? '1': '0';	//0=施錠 1=解錠　通常
	//if(grockmode ==0b01000000)	return (stat)? '0': '1';	//0=施錠 1=解錠　通常
	if(grockmode ==0b10000000)	return '0';	//"解錠表示信号無し"は固定返信
	if(grockmode ==0b00000000)	return '0';	//"電気錠無し"は固定返信
	
	return '0';
}
/***********************************************************************************************************************
* Function Name: 
* Description  : buf2byte 
* Arguments    : Code
* Return Value : CLR flag
***********************************************************************************************************************/
uint8_t		remConv(uint32_t data, uint8_t *buf){
	
	uint16_t	ans= 0x3030;
	
	
	if(rASC ==(uint16_t)gremdata)	ans= select((uint16_t)(data >>16));
	
	buf[0]= (uint8_t)(ans>> 8);
    buf[1]= (uint8_t)ans;
	
	return	0;
}
/***********************************************************************************************************************
* Function Name: 
* Description  : 
* Arguments    : Code
* Return Value : None
***********************************************************************************************************************/
uint16_t	select(uint16_t	data){
	
	uint16_t	ans= 0;
	uint16_t	i;
	
	for(i= 0; i< 41; i++){
    	
		if(remcode[i].key == data){
        	ans = remcode[i].rnumb;
        	break;
        }
    }
    return  ans;
}
/***********************************************************************************************************************
* Function Name: 
* Description  : 旧Array();
* Arguments    : Code		dest <- source
* Return Value : None
***********************************************************************************************************************/
uint8_t *dataSwap(uint8_t *buf){
	
	static uint8_t temp[4];

        temp[1]=  buf[4];	//1-4
        temp[0]=  buf[5];	//5-8
        temp[3]=  buf[6];	//9-12
        temp[2]=  buf[7];	//13-16
        
        buf[4]= temp[0];	//5-8
        buf[5]= temp[1];	//1-4
        buf[6]= temp[2];	//13-16
        buf[7]= temp[3];	//9-12
		

        temp[1]=  buf[8];
        temp[0]=  buf[9];
        temp[3]=  buf[10];
        temp[2]=  buf[11];
        
        buf[8]= temp[0];
        buf[9]= temp[1];
        buf[10]= temp[2];
        buf[11]= temp[3];

	return buf;
}
/***********************************************************************************************************************
* Function Name: 
* Description  :
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
void 	remRec(uint32_t	pwidth){
	
	static	uint8_t		count;
	static	uint32_t	uldata;
	
	if(pwidth > RMAX  || pwidth < RMIN ){
		count = uldata = 0;
	}
	else{
		if(pwidth > COMP)	uldata |= 0x80000000;
		
		if ( count >= 31){
			count = 0;
			gremdata= uldata;	// 受信OK
		}
		else{
			count++;
			uldata = uldata >> 1;
		}
	}	
}
/***********************************************************************************************************************
* Function Name: 
* Description  :
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
uint16_t	terminalOut(uint16_t cnt){
	
	static 	uint16_t	lcnt= 0;
	
	if((lcnt= ++lcnt % cnt)== 0){
	
		if(pgcomm->g=='G')	spiOut(serOut595(goutp),16);
		
		if(pgcomm->i <100) gcomm.i++;
		
	}
	return 40;	
}
/***********************************************************************************************************************
* Function Name: 
* Description  : buf[]から74HC595のdata(16bit)化
* Arguments    : uint8_t *buf[]
* Return Value : uint16_t
***********************************************************************************************************************/
uint16_t	serOut595(uint8_t *buf){
	
	static	uint16_t	serdata;
			uint16_t	mask;
			uint8_t		i;
			
	
	for(i=0, mask= 1; i< 14+1; i++, mask<<= 1){
		
		//if(i==14) DebugPrint2("i=",(uint16_t)buf[i]);
		
		switch(buf[i]){
			
			case	0:
			
				if(grockmode==0xC0 &&i==14){
					solCnt1(buf[i],CLR);
					buf[14] =(gterminaldata&ROCKBIT)? 'P': 4;		//開いてたら0
					if(buf[14]==4)	serdata |= mask;
					else	serdata &= ~mask;
				}
				else serdata &= ~mask;
				break;
				
			case	1:
			
				if(grockmode==0xC0 &&i==14){
					solCnt2(buf[i],CLR);
					buf[14] =(gterminaldata&ROCKBIT)? 8: 'P';	//開いているのを閉めたい
					if(buf[14]==8)	serdata |= mask;
					else	serdata &= ~mask;
				}
				else	serdata |= mask;
				break;
				
			case	2:
				serdata ^= mask;
				break;
				
			case	3:		//nomal=0,1,2,3(oneshot)
				serdata &= ~mask;
				if(grockmode==0xC0 &&i==14)	buf[i] =solCnt1(buf[i],SET);		//裏メニュー
				else	buf[i] =0;
				break;
				
			case	4:		//OFF
				serdata &= ~mask;
				buf[i]--;		// 4通過後に3を通過させる
				break;
				
			case	5:		//ON
			case	6:		//ON
			
				if(grockmode==0xC0 &&i==14){
					if(gterminaldata&ROCKBIT)	buf[14] ='P';
					else{
						serdata |= mask;
						buf[i]--;		// 5->4 1回転分のoneshot パルス幅	
					}
				}
				else{
					serdata |= mask;
					buf[i]--;			// 5->4 1回転分のoneshot パルス幅
				}
				break;
				
				
			case	7:
				if(grockmode==0xC0 &&i==14){
					serdata &= ~mask;
					if(grockmode==0xC0 &&i==14)	buf[14] =solCnt2(buf[14],SET);		//裏メニュー
				}
				break;
				
			case	8:
				if(grockmode==0xC0 &&i==14){
					serdata &= ~mask;
					buf[14]--;
				}
				break;
				
			case	9:
			
				if((gterminaldata&ROCKBIT)==0)	buf[14] ='P';
				else{
					serdata|= mask;
					buf[i]--;		// 6-5-> 2回転分のoneshot パルス幅	
				}
				break;	
				
			default:			
				break;
		}
	}
	return serdata;
}

/***********************************************************************************************************************
* Function Name: 
* Description  : 
* Arguments    : 
* Return Value : 解錠なら'P'
***********************************************************************************************************************/
uint8_t	solCnt1(uint8_t al,uint8_t cnt){
	
	static	uint8_t j =0;
	
	if(cnt==0){
		j =0;
		return al;
	}

	if(gterminaldata & DOOROPEN)	return	al;			// DoorOpen中の為,一旦回避
	
	if((gterminaldata &ROCKBIT)==1){	// 開いていたら終わる	
		j =0;
		return 'P';
	}
	if(++j >4){
		j =0;
		return 'P';
	}				
	return 5;							//Re LOAD 5から繰り返し
}

/***********************************************************************************************************************
* Function Name: 
* Description  : 
* Arguments    : 
* Return Value : 
***********************************************************************************************************************/
uint8_t	solCnt2(uint8_t al,uint8_t cnt){
	
	static	uint8_t j =0;
	
	if(cnt==0){
		j =0;
		return al;
	}

	if(gterminaldata & DOOROPEN)	return	al;			// DoorOpen中の為,一旦回避
	
	if((gterminaldata&ROCKBIT)==0){		// 閉まっていたら終わる	
		j =0;
		return 'P';
	}
	if(++j >4){
		j =0;
		return 'P';
	}				
	return 9;							//Re LOAD 9-8-7を繰り返し
}
/***********************************************************************************************************************
* Function Name: 
* Description  : spiOut 最終的な出力関数 これより先はFET
* Arguments    : None 
* Return Value : None
***********************************************************************************************************************/
void	spiOut(uint32_t value, uint8_t len){
	
	uint8_t	i;
	
	SIFT595;
	
	for( i= 0; i< len; i++){
		
		SRCLK_LOW;
		P3_bit.no0 = (value & 0x8000)? 1: 0;
		value <<= 1;
		
		SRCLK_HIGH;
		NOP();
	}
	SRCLK_LOW;
	LOAD595;
}

#if 0
/***********************************************************************************************************************
* Function Name: 
* Description  : [14]==1なら施錠、[14]==0なら開錠
* Arguments    : serdata , outp[14]
* Return Value : sdata
***********************************************************************************************************************/
uint16_t	solCnt(uint16_t sdata, const uint8_t al){
	
	static	uint8_t	cnt= 0;
	
	
	if((gterminaldata & DOOROPEN)== 1)	return	sdata;		// DoorOpen中の為,一旦回避
	
	if(al== 0){
		if(cnt==0)	cnt=(gterminaldata & ROCKBIT)? 0:4;		// open指示で解錠中なら0, 1=解錠状態
		sdata = (cnt & 0x04)?	solSet(sdata,SOLBIT): solClr(sdata,SOLBIT);
		if(cnt> 0) cnt--;	
	}
	else if(al==1){
		if(cnt==0)	cnt=(gterminaldata & ROCKBIT)? 4:0;		// close指示で解錠中なら4, 1=解錠状態
		sdata = (cnt & 0x04)?	solSet(sdata,SOLBIT): solClr(sdata,SOLBIT);	//解錠状態なので1パルス
		if(cnt> 0) cnt--;	
	}
	return	sdata;
}
#endif
/***********************************************************************************************************************
* Function Name: 
* Description  : 
* Arguments    : None
* Return Value : DIPSW 4bit 反転出力
***********************************************************************************************************************/	
uint8_t		getMode(void){
	
	uint8_t olddata3= 0;
	uint8_t	olddata2= 0;
	uint8_t	olddata1= 1;
	uint8_t cnt= 0;
			
	while(TRUE){
		
		if(gloopcnt != cnt){			//CountUpは 1mS
			cnt =gloopcnt;				//0-9を繰り返す
			
			olddata3= olddata2;
			olddata2= olddata1;
			olddata1= (P2& 0x0f);
			
			if(olddata3==olddata2 && olddata2==olddata1) return ((~olddata1) &0x0f);
		}
	}
}
/***********************************************************************************************************************
* Function Name: 
* Description  : 
* Arguments    : Code
* Return Value : None
***********************************************************************************************************************/
uint8_t	enqCommY(const uint8_t *buf, struct ascflg_t *command){
	
	uint8_t		lbuf[10];
			    			
    
	if(command->y =='\0') return 0;	//* 参加許可前の返信禁止
			
	if(command->p !='P'){
		command->p ='P';

		lbuf[0] =0x02;
		lbuf[1] ='F';
		lbuf[2] ='F';
		lbuf[3] ='p';
		lbuf[4] =0x03;
		lbuf[5] =0x73;
		lbuf[6] ='\0';
		bufCopy(gtxbuf, lbuf, 6);
	}
	else if(command->i =='I'){		//* I受信済
	
       	lbuf[0] =0x04;
		bufCopy(gtxbuf, lbuf, 1);          
	}         
	else{

		lbuf[0] =0x02;
		lbuf[1] ='F';
		lbuf[2] ='F';
		lbuf[3] ='i';				//* 状態要求
		lbuf[4] =0x03;
		lbuf[5] =0x6A;
		lbuf[6] ='\0';
		bufCopy(gtxbuf, lbuf, 6);
	}                               
	return 0;
}
/***********************************************************************************************************************
* Function Name: 
* Description  : 
* Arguments    : Code
* Return Value : None
***********************************************************************************************************************/
uint8_t	stxCommY(const uint8_t *buf, struct ascflg_t *command){
	
	uint8_t		i =0;
    uint8_t		lbuf[10];
	
	
	if(buf[3] =='Y')	command->y ='Y';	//* Y電文受信後に許可される        
            
    else if(buf[3] =='V'){    			//*ASC3100もV電文のcommYはチェック無し  
			
		if(NAK == calcLrc(buf,0x03)){
			lbuf[0] =NAK;
			bufCopy(gtxbuf, lbuf, 1);	
		}
		else{
			lbuf[0] =0x02;
			lbuf[1] ='F';
			lbuf[2] ='F';
			lbuf[3] ='x';
			lbuf[4] ='0';
			lbuf[5] ='0';
			lbuf[6] =0x03;
			lbuf[7] =0x7B;
			lbuf[8] ='\0';	
			bufCopy(gtxbuf, lbuf, 8);
			command->p ='\0';
		}
	}
	else if(buf[3] =='P' /* && commY==true */){	//* P電文に Y許可が必要か？
			
		if(0x15== calcLrc(buf,0x03)){				//* '0'-'F'
			lbuf[0] =NAK;
			bufCopy(gtxbuf, lbuf, 1);
		}
		else{
			lbuf[0] =('\0'==comPression(buf))? ACK: NAK;		//* 16byte -> 8byteに圧縮
			bufCopy(gtxbuf, lbuf, 1);
		}			
	}
	else if(buf[3]=='K' && command->y=='Y'){
		
			
			
		if(NAK == calcLrc(buf,0x03)){
			lbuf[0] =NAK;
			bufCopy(gtxbuf, lbuf, 1);
		}
		else{
			
			
			lbuf[0] =kUnfold(grnum,buf);     	// rnum[RMAX] にstatusが入る 0,1,2,0,0,0,data[indexEND]					
			
			bufCopy(gtxbuf, lbuf, 1);
			if(lbuf[0] == ACK){
				for(i=0;i<16;i++) goutp[i] =0;	// outp[0]-[15] =0
				popEep(grnum);					// outdata更新
				command->i ='I';
			}
		}
    }
	return 0;
}
/***********************************************************************************************************************
* Function Name: 
* Description  : out1 - out14 --> 1-7に圧縮
* Arguments    : Code
* Return Value : None
***********************************************************************************************************************/
int8_t	comPression(const uint8_t *data){
	
	int8_t		al,bl,cl;
	uint16_t	i;
	
	
	for(i=4; i <582; i++){
			
		if(data[i] ==0x03)	return 	'\0';		//ここが正常終了
		
		al =asciiConvert('H', data[i]);
		if(al == -1) return	al;
		bl= asciiConvert('H', data[++i]);
		if(bl == -1) return	bl;
		
		al= (al <<4) +bl;						//* room 0x00 - 0x1F
		pattern.room[al][0]	=(data[++i] &0x03) +((data[++i] &0x03) <<2);		//1-2 
		pattern.room[al][1] =(data[++i] &0x03) +((data[++i] &0x03) <<2);		//3-4
		pattern.room[al][2] =(data[++i] &0x03) +((data[++i] &0x03) <<2);		//5-6
		pattern.room[al][3] =(data[++i] &0x03) +((data[++i] &0x03) <<2);		//7-8
		pattern.room[al][4] =(data[++i] &0x03) +((data[++i] &0x03) <<2);		//9-10
		pattern.room[al][5] =(data[++i] &0x03) +((data[++i] &0x03) <<2);		//11-12
		pattern.room[al][6] =(data[++i] &0x03) +((data[++i] &0x03) <<2);		//13-14
		cl =(data[++i] &0x03) +((data[++i] &0x03) <<2);						//15-16 ダミー
	}
	return	-1;		// 0x03が未検出
}
/***********************************************************************************************************************
* Function Name: K_Unfold(k電文_展開)
* Description  : data[] -> rnum[] 	ETX迄回転
* Arguments    : data[0]
* Return Value : return(0x06)
***********************************************************************************************************************/
uint8_t		kUnfold(uint8_t *dest, const uint8_t *source){

    uint16_t	i =3;				//* 0x02,0x30,0x31(端末addr)なので、3から
    uint8_t		a =0;               //* addr
    uint8_t		s =0;               //* status
    uint16_t	j;
	
	
    for( j= 0; j< ROOMMAX; j++){
        
        if(source[i] =='K'){
            a =(asciiConvert('H',source[++i]) <<4);			//* 部屋code 00 - 1F
			a |=(0x0f & asciiConvert('H',source[++i]));		//* 2byte -> HEX　例 0x02
          if(a >0x1F) return NAK;							//* RMAX 1F以上はない
            
			s =asciiConvert('H',source[++i]);	//*status -> HEX 0x00,0x01,0x02
          if(s >0x02) return NAK;				//* 0x02以上の設定は無い
            i++;
            dest[a] =s;                    		//* rmun[例0x02] <--  0とか1等の部屋status buff
        }
        if(source[i] ==0x03)	return ACK;
    }
    return	NAK;
}
/***********************************************************************************************************************
* Function Name	: 
* Description  		: 
* Arguments    		: rnum[0]
* Return Value 		: None
***********************************************************************************************************************/
void    popEep(uint8_t	*addr){
	
	uint8_t		buf[10];
	uint8_t		temp[16];
	uint16_t	i;
	
	
    for(i =0; i <ROOMMAX; i++){
        
       if(addr[i] != 0){
      
            dataSetY(buf, pattern.all,i );	//* RAM pattern[i*7]-> buff[0]-buff[6]/buff[7]-buff[14]
			convDataY(temp,buf,8);
			allOrY(goutp, temp, 16);		//* temp[16]  ||  outp[16] -> outp[16]
        }
    }
}
/***********************************************************************************************************************
* Function Name	: 
* Description  		: Buff Copy	 source[i] -> dest[i]
* Arguments    		: None
* Return Value 		: None
***********************************************************************************************************************/
uint8_t		bufCopy(uint8_t *dest, uint8_t *source, uint16_t len){
	
    uint16_t	i;
	
	if(gtxtimer ==0){
		
		gtxlen =len;
		gtxtimer =6;
    	for(i =0;i <len; i++)	dest[i] =source[i];
	}
	return 0;
}
/***********************************************************************************************************************
* Function Name	: 
* Description  		: Buff Copy	 source[i] -> dest[i]
* Arguments    		: None
* Return Value 		: None
***********************************************************************************************************************/
void    dataCopy(uint8_t *dest, uint8_t *source, uint8_t num){
	
    uint8_t	i;
	
    for(i =0;i <num; i++)	dest[i] =source[i];
}
/***********************************************************************************************************************
* Function Name	: 
* Description  		: Buff CLR		0 -> dest[i]
* Arguments    		: None
* Return Value 		: None
***********************************************************************************************************************/
void    dataSetY(uint8_t *dest, uint8_t	*source, uint16_t num){
	
    uint8_t		i;
	uint16_t	a =7*num;
	
    for(i= 0; i< 7; i++){
		dest[i] =source[a++];
	}
}
/***********************************************************************************************************************
* Function Name: * conversion data Multiplex data *
* Description  : buff[8]->temp[16]  0x03はマスク
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
void	convDataY(uint8_t *dest, uint8_t *addr, uint8_t lim){

	while(lim--){
		*dest++ = *addr & 0x03;				// ASC3100Yと逆
        *dest++ = ((*addr) >>2) & 0x03;
		*addr++;          
    }
}
/***********************************************************************************************************************
* Function Name: outp[16] or temp[16]
* Description  : 
* Arguments    : (byte *dest ,byte  *addr ,byte lim)
* Return Value : None
***********************************************************************************************************************/
void    allOrY(uint8_t *dest, uint8_t  *source, uint8_t lim){
	
	while(lim--)	*dest++ |= *source++;
		
}
/***********************************************************************************************************************
* Function Name: 
* Description  : 
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
uint16_t	yajiOut(uint16_t cunt){
	
	static 	uint16_t	lcunt= 0;
	
	if((lcunt= ++lcunt % cunt)== 0){
		
		spiOut(serOut595Y(goutp),16);
	}
	return 40;	
}
/***********************************************************************************************************************
* Function Name: 
* Description  : temp[] から unsigned short dataを作成。
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
uint16_t	serOut595Y(uint8_t *buf){
			
	static	uint16_t	serdata;
			uint16_t	mask;	
			uint8_t		i ;
	
	for(i =0, mask =1U; i <14; i++, mask <<=1){
				
		switch(buf[i]){
			case		0x00:
				serdata &= ~mask;	
				break;
			case    	0x01:
                serdata |= mask;
               	break;
           	case    	0x02:               /* BLINK */
              	serdata ^= mask;           	/* EOR */
               	break;     
           	case    	0x03:
               	serdata ^= mask;           	/* 3=EOR */
               	break;

           	default:
               	serdata &= ~mask;           /*0,1,2,3以外は0 */
       	}
    }
	return serdata;		
}
/***********************************************************************************************************************
* Function Name: 
* Description  :
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
void	testMode(uint8_t ctl){
	
	uint16_t	ax =0U;
	
	if(ctl==0)	return;
	
	while(ax !=0x0FFF){			//dipSw用
		
		ax =(uint8_t)(hc165Read(31) >>8);
		ax |=(getMode() <<8);
		spiOut(ax,16);
	}
	spiOut(CLR,16);
	
	while(TRUE){
		
		static	uint8_t		remflg =0;
		
		ax =((uint16_t)(hc165Read2(31)) &(~0xC000));// inputは14本しかない					

		if(rAMC ==(uint16_t)gremdata){
			remflg =!remflg;
			gremdata =0U;
			ax |=(remflg)? 0x4000 :0x0000;
		}
		spiOut(ax,16);
		
		(gcomflg='\0')||(gbuf[1]='\0')||(gbuf[2]='\0')||(gbuf[3]='\0');
		glifecnt =1000;
		
		P14_bit.no7 =0;
		P14_bit.no6 =1;
		
		serialPrint2(testformat,10);
		
		P14_bit.no7 =0;
		P14_bit.no6 =0;

		while(glifecnt){
			if(gcomflg=='!'){
				(gbuf[1]=='A')&&(gbuf[2]=='S')&&(gbuf[3]=='C')&&(glifecnt=50)&&(gcomflg='\0');
			}
		}	
	}	
}
/***********************************************************************************************************************
* Function Name: 
* Description  : DebugPort用
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
uint8_t		serialPrint1(uint8_t *buf, uint16_t len){

	while(g_tx1_ready_flag != FREE) __nop();
	
	g_tx1_ready_flag = BUSY;
	R_UART1_Send((uint8_t *)buf, len);
	
	while(g_tx1_ready_flag != FREE)	__nop();	
	return	0;
}
/***********************************************************************************************************************
* Function Name: 
* Description  : 
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
void	waitTimer(uint32_t cunt){

	uint32_t	i;
	
	i =glifecnt;		//要初期値 glifeCntは減算
	i -=cunt;
	
	while(i <glifecnt) __nop();
}
/***********************************************************************************************************************
* Function Name: 
* Description  : 
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
void	ledMoni(uint16_t cnt){
	
	static 	uint16_t	lcnt =0;

	if((lcnt= ++lcnt % cnt)== 0){
		
		LEDMONI = !LEDMONI;

	}
}
/***********************************************************************************************************************
* Function Name: 
* Description  : 
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
void	DebugPrint2(uint8_t *s,uint16_t n){
	
		uint8_t	buf[50];
		
		sprintf(buf,"%s%X\n" ,s ,n );

		serialPrint1(buf,strlen(buf));
}



/* End user code. Do not edit comment generated here */
