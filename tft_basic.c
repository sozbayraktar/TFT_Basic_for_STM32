#include "main.h"
#include "tft_basic.h"

//#include "bmp.h"
#include "font.h"

#include "mathTables.h"
#include "ILI9341_Defines.h"



#define SPIbuffSize 264  /* 16,24 ve 32 bit BMP dosyalarla uyumlu olması için 4 ve 3 ün katları olmasında yarar var */

uint8_t buff[SPIbuffSize];


void calculateXY(struct pixelPosition_typedef* XY,int16_t r,int16_t angle);

void TFT_writecommand(uint8_t data);
void TFT_send16bitData(uint16_t data);

void TFT_SetCursor(uint16_t Xpos, uint16_t Ypos);
void TFT_SetCursorForImage(uint16_t Xpos, uint16_t Ypos);

void printLineOnTFT(uint8_t cursorPos, uint8_t satirbasi, uint16_t satirY, char* line);

void TFT_DrawRectangle(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2,uint16_t color);
void TFT_SetWindows(uint16_t xStar, uint16_t yStar,uint16_t xEnd,uint16_t yEnd);
void TFT_WriteReg(uint8_t TFT_Reg, uint16_t TFT_RegValue);
void TFT_WriteRAM_Prepare(void);
void TFT_SetParam(void);
void TFT_writeMultipleData(uint8_t *pData, uint16_t Size);

void TFT_Fill(uint16_t sx,uint16_t sy,uint16_t ex,uint16_t ey,uint16_t color);
void TFT_ShowChar(uint16_t x,uint16_t y,uint16_t fc, uint16_t bc, uint8_t num,uint8_t size,uint8_t mode);
void copyString(char* source, char* destination, uint8_t size);

void TFT_Drawbmp16(uint16_t x,uint16_t y,const unsigned char *p);
void TFT_ShowBmp(uint16_t x,uint16_t y,uint16_t fc, uint16_t bc, const unsigned char *pBMP,uint8_t width,uint8_t height,short zoom, uint8_t mode);
void TFT_DrawBitmap(uint16_t Xpos, uint16_t Ypos, uint8_t *pbmp,uint16_t numberOfBytes, uint16_t bytesPerPixel);

void TFT_DrawArc(int16_t xC, int16_t yC, int16_t r, int16_t angleStart, int16_t angleStop);
void TFT_DrawRadius(int16_t xC, int16_t yC, int16_t r, int16_t angle, uint8_t color);
void calculateXY(struct pixelPosition_typedef* XY,int16_t r,int16_t angle);

void TFT_sendOneByte(uint8_t data);


uint16_t DeviceCode;

#ifdef SPIMODE
extern SPI_HandleTypeDef hspi1;
#endif

struct pixelPosition_typedef* pPixelPosition=&pixelPosition;



// Bu union'u 16 bitlik dataları 8 bitlik iki byte'a bölmek için kullanacağız

typedef union
{
uint16_t Word;
struct
{
uint8_t LowByte;
uint8_t HighByte;
};
} tWordByte;


//******************************************************************
//TFT_writecommand
//******************************************************************
void TFT_writecommand(uint8_t data) {
	TFT_RS_CLR;   // bazi TFTlerde "DC"
	TFT_CS_CLR;

#ifdef PARALLELMODE
	TFT_send8bitData(data);   // 8 bit Paralel TFT modu
#endif
#ifdef SPIMODE
			while(HAL_SPI_Transmit(pTft_Spi,&data,1,10000)!=HAL_OK);// SPI TFT Modu
		#endif

	TFT_CS_SET;
}


//******************************************************************
//TFT_writedata
//        8 bit modunda:   16 bitin kucuk 8 bitini B8..B15 uzerinden gonderir
//******************************************************************
void TFT_writedata(uint16_t data)
{
	TFT_RS_SET;
	TFT_CS_CLR;

#ifdef PARALLELMODE
	TFT_send8bitData(data);  // 8 bit Paralel TFT modu B8..15 uzerinden
#endif
#ifdef SPIMODE
	TFT_sendOneByte(data & 0xFF); // SPI TFT Modu
#endif

	TFT_CS_SET;
}


//******************************************************************
//TFT_writedata  16 bit modunda
//
//******************************************************************
void TFT_send16bitData(uint16_t data) {
#ifdef PARALLELMODE
	uint8_t ust,alt;
	ust = data >> 8; //üst 8 bit
	alt = data & 0xFF; // alt 8 bit
#endif

	TFT_RS_SET;
	TFT_CS_CLR;

#ifdef PARALLELMODE
	TFT_send8bitData(ust);  // once ust 8 bit
	TFT_send8bitData(alt);  // sonra da alt 8 bit
#endif

#ifdef SPIMODE
	TFT_sendOneByte(data >> 8);   // SPI TFT Modu
	TFT_sendOneByte(data & 0xFF); // SPI TFT Modu
#endif

	TFT_CS_SET;

}
//******************************************************************
//TFT_writedata  16 bit modunda
//
//******************************************************************
#ifdef PARALLELMODE
void TFT_send8bitData(uint16_t data)
{
#ifdef NUCLEO
	writeByteToAny8(data);  // Universal metod  0,35 us
#endif
#ifdef CNC_P
    writeByteToAny8(data);  // Universal metod  0,35 us
#endif
			TFT_WR_CLR;
			TFT_WR_SET;
}
#endif


//******************************************************************
//TFT_WriteReg
//******************************************************************
void TFT_WriteReg(uint8_t TFT_Reg, uint16_t TFT_RegValue)
{
	TFT_writecommand(TFT_Reg);
	TFT_writedata(TFT_RegValue);
}	   
	 
//******************************************************************
//TFT_WriteRAM_Prepare
//******************************************************************
void TFT_WriteRAM_Prepare(void)
{
	TFT_writecommand(ILI9341_RAMWR);
}	 

//******************************************************************
//TFT_DrawPoint
//******************************************************************
void TFT_DrawPoint(uint16_t x,uint16_t y)
{
	if((x<0)||(y<0)||(x>TFT_W)||(y>TFT_H)) return; //Ekran limitleri d���nda ise �izme
	TFT_SetCursor(x,y);
	TFT_send16bitData(POINT_COLOR);
}

//******************************************************************
//TFT_sendOneByte
//******************************************************************
#ifdef SPIMODE
void TFT_sendOneByte(uint8_t data) {
	while(HAL_SPI_Transmit(pTft_Spi,&data,1,10000)!=HAL_OK);
}
#endif

void TFT_Clear(uint16_t Color){
TFT_Fill(0,0,TFT_W-1,TFT_H-1,Color);
}

//******************************************************************
//      TFT_Fill
//          sx:
//        	sy:
//			ex:
//			ey:
//        	color:
//******************************************************************
void TFT_Fill(uint16_t sx,uint16_t sy,uint16_t ex,uint16_t ey,uint16_t Color)
{

	uint16_t i;

//	  uint16_t buffIndex;
	  uint16_t buffSize;
	  uint16_t pack;
	  int32_t remained;

	buffSize=SPIbuffSize;
//	buffIndex=0;
	pack=0;

	remained=(ex-sx+1)*(ey-sy+1)*2; //pixel sayisi *2 Byte

	TFT_SetWindows(sx,sy,ex,ey);

	TFT_RS_SET;
	TFT_CS_CLR;

	for(i=0;i<buffSize;i++) {
		buff[i]=Color>>8;
		i++;
		buff[i]=Color & 0xFF;
	}

	i=1;
	while(remained>0){
	if(remained>=buffSize) pack=buffSize; else pack = remained;
	remained-=pack;
	i++;
	TFT_writeMultipleData(buff,pack);
	}
//	while(HAL_SPI_Transmit(pTft_Spi,buff,640,10000)!=HAL_OK);

	TFT_CS_SET;
}

/**** END OF TFT_Fill ******/



/***********************************************************
 *                                                         *
 * TFT_writeMultipleData                                   *
 *                                                         *
 ***********************************************************/
void TFT_writeMultipleData(uint8_t *pData, uint16_t Size) {
#ifndef SPIMODE
uint16_t i,data;
#endif

#ifndef SPIMODE
/* Optimizasyon öncesi - 7200 Byte transferi 17.2 ms */
/*
		for(i=0;i<Size;i++){
			data=*(pData+i);
			TFT_send8bitData(data);
			}
*/

/* Optimizasyon adımı 2 - 17.2 ms den 14.8ms ye düştü*/
/*
for(i=0;i<Size;i++){
	data=*(pData+i);
    writeByteToAny8(data);  // BSSR Makro - Universal metod  0,35 us
		TFT_WR_CLR;
		TFT_WR_SET;
		}
*/

 /* Optimizasyon adımı 3 - TFT_WR SET/RESET HAL yerine direkt 7.6 s */

for(i=0;i<Size;i++){
	data=*(pData+i);
writeByteToAny8(data);  // Universal metod  0,7 us
TFT_WR_GPIO_Port->BSRR = (uint32_t)TFT_WR_Pin << 16u; // Reset
TFT_WR_GPIO_Port->BSRR = TFT_WR_Pin;
		}

#else
	/**  DMA MODU KULLLANILIP KULLANILMADIĞINA BAĞLI OLARAK asagidaki
	 * satirlardan istenen acilir
	 */
	/****** DMA modu *****/
/*	 while(HAL_SPI_GetState(pTft_Spi)!=HAL_SPI_STATE_READY);

	 if(HAL_SPI_Transmit_DMA(pTft_Spi, pData, Size)!=HAL_OK){
	 Error_Handler();}
*/
	/****** DMA Modunun sonu *****/

	/****** Blocking Mode  *****/
	while (HAL_SPI_GetState(pTft_Spi) != HAL_SPI_STATE_READY)
		;
	while (HAL_SPI_Transmit(pTft_Spi, pData, Size, 10000) != HAL_OK)
		;

	/****** Blocking Mode sonu ***/

#endif
}
/******  END OF TFT_writeMultipleData  ******/

//******************************************************************
//TFT_Reset
//******************************************************************
void TFT_RESET(void)
{
	TFT_RST_CLR;
	HAL_Delay(100);
	TFT_RST_SET;
	HAL_Delay(50);
}

// ******************************************************************

//TFT_init

// ******************************************************************
void TFT_Init(void)
{

#ifdef PARALLELMODE
	TFT_RD_SET;
#endif

#ifdef SPIMODE
    pTft_Spi= &hspi1; // TFT SPI1 uzerinde
#endif

	TFT_CS_CLR;

  	TFT_RESET();  // Hard reset

    HAL_Delay(5);

    // * Software reset * /
	TFT_writecommand(0x01);  //0x01  Reset

	HAL_Delay(5);

	// ************* Start Initial Sequence ********** //

  TFT_writecommand(0xEF);
  TFT_writedata(0x03);
  TFT_writedata(0x80);
  TFT_writedata(0x02);

	TFT_writecommand(0xCF);  //Power Control B
	TFT_writedata(0x00);
	TFT_writedata(0xC1);
	TFT_writedata(0X30);

	TFT_writecommand(0xED);  //PowerOn sequence control
	TFT_writedata(0x64);
	TFT_writedata(0x03);
	TFT_writedata(0X12);
	TFT_writedata(0X81);

	TFT_writecommand(0xE8);  //Driver timing control A
	TFT_writedata(0x85);
	TFT_writedata(0x10); //(?? 0x10>>0x00)
	TFT_writedata(0x7A); //(?? 0x7A>>0x78)

	TFT_writecommand(0xCB);  //Power control A
	TFT_writedata(0x39);
	TFT_writedata(0x2C);
	TFT_writedata(0x00);
	TFT_writedata(0x34);
	TFT_writedata(0x02);

	TFT_writecommand(0xF7);  //Pump ratio control
    TFT_writedata(0x20);

	TFT_writecommand(0xEA);  //Driver  Timing Control B
	TFT_writedata(0x00);
	TFT_writedata(0x00);

	TFT_writecommand(ILI9341_PWCTR1);    //0xC0 Power control 1
	TFT_writedata(0x23);   //VRH[5:0] (?? 0x1B>>0x23)

	TFT_writecommand(ILI9341_PWCTR2);    //0xC1 Power control 2
	TFT_writedata(0x10);   //SAP[2:0];BT[3:0] (?? 0x01>>0x10)

	TFT_writecommand(ILI9341_VMCTR1);    //0xC5 VCOM control - Set VCOMH voltage
	TFT_writedata(0x30); 	 //3F  (?? 0x30>>0x3E)
	TFT_writedata(0x20); 	 //3C  (?? 0x20>>0x28)

	TFT_writecommand(ILI9341_VMCTR2);    //0xC7 VCOM control2 - Set the VCOM Offset voltage
	TFT_writedata(0XB7);   // (0xB7>>0x86)

	TFT_writecommand(ILI9341_MADCTL);    // 0x36 Memory Access Control (Orientation kontrolu MADCTL)
	TFT_writedata(0x28);   // *** TFT_SetParam() i�inde tekrar deger ataniyor, buradaki etkisiz



  /*
	  B7:B6:B5:B4 0010 : Page/column selection

	  B3:B2:B1:B0 1000
	              1001 : Vertical Flip (9h)
                  1010 : Horizontal Flip (Ah)

	TFT_Orientation_Portrait_rotated TFT_WriteReg(0x36,0x18);
	TFT_Orientation_Portrait_normal  TFT_WriteReg(0x36,0xD8);
	TFT_Orientation_Landscape_horizontaFlip TFT_WriteReg(0x36,0x28);
	TFT_Orientation_Landscape_verticalFlip TFT_WriteReg(0x36,0xE8);
	TFT_Orientation_Landscape_normal TFT_WriteReg(0x36,0xA8);
	*/

	TFT_writecommand(ILI9341_PIXFMT);	   //  OK ILI9341 PIX FMT
	TFT_writedata(0x55);

	TFT_writecommand(ILI9341_FRMCTR1);    // OK  ILI9341 FRMCTRL
	TFT_writedata(0x00);
	TFT_writedata(0x1A);   //  (?? 0x1A>>0x18 )

	TFT_writecommand(ILI9341_DFUNCTR);    // OK ILI9341_DFUNCTR  Function Control
	TFT_writedata(0x0A);   // (?? 0x0A>>0x08)
	TFT_writedata(0xA2);   // (?? 0xA2>>0x82)
	TFT_writedata(0x27);

	TFT_writecommand(0xF2);    // OK 3Gamma Function Disable
	TFT_writedata(0x00);

	TFT_writecommand(ILI9341_GAMMASET);    //OK Gamma curve selected
	TFT_writedata(0x01);

	  TFT_writecommand(ILI9341_GMCTRP1);    //Set Gamma
	  TFT_writedata(0x0F);
	  TFT_writedata(0x31);
	  TFT_writedata(0x2B);
	  TFT_writedata(0x0C);
	  TFT_writedata(0x0E);
	  TFT_writedata(0x08);
	  TFT_writedata(0x4E);
	  TFT_writedata(0xF1);
	  TFT_writedata(0x37);
	  TFT_writedata(0x07);
	  TFT_writedata(0x10);
	  TFT_writedata(0x03);
	  TFT_writedata(0x0E);
	  TFT_writedata(0x09);
	  TFT_writedata(0x00);

	  TFT_writecommand(ILI9341_GMCTRN1);    //Set Gamma
	  TFT_writedata(0x00);
	  TFT_writedata(0x0E);
	  TFT_writedata(0x14);
	  TFT_writedata(0x03);
	  TFT_writedata(0x11);
	  TFT_writedata(0x07);
	  TFT_writedata(0x31);
	  TFT_writedata(0xC1);
	  TFT_writedata(0x48);
	  TFT_writedata(0x08);
	  TFT_writedata(0x0F);
	  TFT_writedata(0x0C);
	  TFT_writedata(0x31);
	  TFT_writedata(0x36);
	  TFT_writedata(0x0F);

	TFT_writecommand(ILI9341_PASET);  // 0x2B *** Set Page Address, TFT_SetParam i�inde yeniden ayarlaniyor
	TFT_writedata(0x00);
	TFT_writedata(0x00);
	TFT_writedata(0x01);
	TFT_writedata(0x3f);

	TFT_writecommand(ILI9341_CASET);  // 0x2A *** Set Column Address, TFT_SetParam() i�inde yeniden ayarlaniyor
	TFT_writedata(0x00);
	TFT_writedata(0x00);
	TFT_writedata(0x00);
	TFT_writedata(0xef);


	TFT_writecommand(ILI9341_SLPOUT); //Exit Sleep
	HAL_Delay(120);

	TFT_writecommand(ILI9341_DISPON); // on

	TFT_SetParam();
	TFT_Clear(BLUE);
    TFT_CS_SET;

#ifdef TFT_320x240
    TFT_Rotate(Landscape_normal_24TP);  // 240x320 2.4" SPI TP de b�yle
#endif

#ifdef TFT_480x320
  TFT_Rotate(Landscape_normal_35P);  // 320x480 3.5" TFT_P de böyle

#endif

    TFT_ShowString(10, 10, 32, " STM32 TFT SPI & 8B ", YELLOW, 0);
    TFT_ShowString(10, 50, 16, "         Selcuk OZBAYRAKTAR           ", YELLOW, 0);
#ifdef PARALLELMODE
    TFT_ShowString(10, 70, 16, "       ILI9341 TFT 8 BIT MODU         ", YELLOW, 0);
#else
    TFT_ShowString(10, 70, 16, "       ILI9341 TFT SPI   MODU         ", YELLOW, 0);
#endif

    TFT_ShowString(10, 90, 16, "****  TFT_BASIC R1 HAZIRAN 2020   ****", YELLOW, 0);


}
  		  
/*************************************************
*TFT_SetWindows
*************************************************/
void TFT_SetWindows(uint16_t xStar, uint16_t yStar,uint16_t xEnd,uint16_t yEnd)
{

	TFT_writecommand(ILI9341_CASET);
	TFT_writedata(xStar>>8);
	TFT_writedata(0x00FF&xStar);
	TFT_writedata(xEnd>>8);
	TFT_writedata(0x00FF&xEnd);

	TFT_writecommand(ILI9341_PASET);
	TFT_writedata(yStar>>8);
	TFT_writedata(0x00FF&yStar);
	TFT_writedata(yEnd>>8);
	TFT_writedata(0x00FF&yEnd);

	TFT_WriteRAM_Prepare();				
}   

/*************************************************
TFT_SetCursor

TFT_SetWindows gibi çalışıyor, ancak başlangıç-bitiş
koordinatları birbirine eşit olarak veriliyor.

*************************************************/
void TFT_SetCursor(uint16_t Xpos, uint16_t Ypos)
{
	TFT_writecommand(ILI9341_CASET);
	TFT_writedata(Xpos>>8);
	TFT_writedata(0x00FF&Xpos);
	TFT_writedata(Xpos>>8);        // *** bu satiri ve asagidakini ILI9381 3.5" TFT_P icin eklemem gerekti
    TFT_writedata(0x00FF&Xpos);

	
	TFT_writecommand(ILI9341_PASET);
	TFT_writedata(Ypos>>8);
	TFT_writedata(0x00FF&Ypos);
	TFT_writedata(Ypos>>8);        // *** bu satiri ve asagidakini ILI9381 3.5" TFT_P icin eklemem gerekti
    TFT_writedata(0x00FF&Ypos);

	TFT_WriteRAM_Prepare();	
} 

/*************************************************
TFT_SetCursor for image
*************************************************/

void TFT_SetCursorForImage(uint16_t Xpos, uint16_t Ypos)
{
	//	bu fonksiyon ILI9341 dekinden farkli calisiyor: TM_ILI9341_SetCursorPosition(X1,Y1,X2,Y2);
	//  writemultipledata  ile writeData farkli.

TFT_writecommand(ILI9341_CASET);
TFT_writedata(Xpos>>8);
TFT_writedata(0x00FF&Xpos);
TFT_writedata(Xpos>>8);        // *** bu satiri ve asagidakini ILI9381 3.5" TFT_P i�in eklemem gerekti
TFT_writedata(0x00FF&Xpos);

TFT_writecommand(ILI9341_PASET);
TFT_writedata(Ypos>>8);
TFT_writedata(0x00FF&Ypos);
TFT_writedata(Ypos>>8);        // *** bu satiri ve asagidakini ILI9381 3.5" TFT_P i�in eklemem gerekti
TFT_writedata(0x00FF&Ypos);

TFT_writecommand(ILI9341_RAMWR);

	}


void TFT_SetParam(void)
{ 
  /*
	  B7:B6:B5:B4 0010 : Page/column selection 
	
	  B3:B2:B1:B0 1000
	              1001 : Vertical Flip (0x09)
                  1010 : Horizontal Flip (0x0A)
	
	TFT_Orientation_Portrait_rotated        TFT_WriteReg(0x36,0x18);
	TFT_Orientation_Portrait_normal         TFT_WriteReg(0x36,0xD8);
	TFT_Orientation_Landscape_horizontaFlip TFT_WriteReg(0x36,0x28);
	TFT_Orientation_Landscape_verticalFlip  TFT_WriteReg(0x36,0xE8);
	TFT_Orientation_Landscape_normal        TFT_WriteReg(0x36,0xA8);
	*/
	
TFT_Rotate(Landscape_normal_24TP);

}

//******************************************************************
//           TFT_DrawLine
//          x1,y1:
//        	x2,y2:
//****************************************************************** 
void TFT_DrawLine(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color)
{
	uint16_t t;
	POINT_COLOR=color;


	int xerr=0,yerr=0,delta_x,delta_y,distance; 
	int incx,incy,uRow,uCol; 

	delta_x=x2-x1;
	delta_y=y2-y1; 
	uRow=x1; 
	uCol=y1; 
	if(delta_x>0)incx=1; 
	else if(delta_x==0)incx=0; 
	else {incx=-1;delta_x=-delta_x;} 
	if(delta_y>0)incy=1; 
	else if(delta_y==0)incy=0;
	else{incy=-1;delta_y=-delta_y;} 
	if( delta_x>delta_y)distance=delta_x; 
	else distance=delta_y; 
	for(t=0;t<=distance+1;t++ )
	{  
    TFT_DrawPoint(uRow,uCol);
		xerr+=delta_x ; 
		yerr+=delta_y ; 
		if(xerr>distance) 
		{ 
			xerr-=distance; 
			uRow+=incx; 
		} 
		if(yerr>distance) 
		{ 
			yerr-=distance; 
			uCol+=incy; 
		} 
	}  
} 



//******************************************************************
//                        TFT_DrawArc
//         (x1,y1),(x2,y2):
//******************************************************************  
void TFT_DrawArc(int16_t xC, int16_t yC, int16_t r, int16_t angleStart, int16_t angleStop)
{uint16_t angle;
	struct pixelPosition_typedef newPixel;

POINT_COLOR=YELLOW;	
for(angle=angleStart;angle<=angleStop;angle++){
/*
	itoa(angle,buffer,10);
	TFT_ShowString(10,220,16,buffer,0);
*/	
	calculateXY(&newPixel,r,angle);
	/*
	itoa(xC+newPixel.x,buffer,10);
	TFT_ShowString(50,220,16,buffer,0);
	itoa(yC+newPixel.y,buffer,10);
	TFT_ShowString(100,220,16,buffer,0);
	*/
	TFT_DrawPoint(xC+newPixel.x,yC+newPixel.y);
}
}  


//******************************************************************
//                        TFT_DrawRadius
//         
//******************************************************************  
void TFT_DrawRadius(int16_t xC, int16_t yC, int16_t r, int16_t angle, uint8_t color)
{

	struct pixelPosition_typedef newPixel;
	
	calculateXY(&newPixel,r,angle);
	/*
	itoa(xC+newPixel.x,buffer,10);
	TFT_ShowString(50,220,16,buffer,0);
	itoa(yC+newPixel.y,buffer,10);
	TFT_ShowString(100,220,16,buffer,0);
	*/
	TFT_DrawLine(xC,yC,xC+newPixel.x,yC+newPixel.y,color);
}
 
//*********************************************************************
//                        calculateXY                                 *
// Verilen yaricap ve acidan hareketle cember uzerindeki noktanin     *
// koordinatlarini hesaplayarak XY(x,y) parametresi icine yerlestirir.*
// cember merkezini (0,0) kabul eder, yani x,y koordinatlari pozitif  *
// ve negatif degerler alabilir.                                      *
// Aci 0..360 arasinda tam sayi olmalidir. Hesaplama mathTables.h     *
// icindeki sinus, cosinus tablolari kullanilarak yapilir. tablolarda *
// 0..45 dereceye kadar acilarin cos ve sin degerleri oldugundan      *
// cember 4x2 yani 8 bolgeye bolunerek hesap yapilmaktadir.           *
// boylece 0..360 derecelik tablolar kullanmaktan kurtuluyoruz.       *
//                                                                    *
// Cos ve Sin degerleri tablolarda x100 olarak 8 bit integerdir.      *
// Dolayisi ile sonuclar %1 dogruluktadir.                            *
//*********************************************************************  
void calculateXY(struct pixelPosition_typedef* XY,int16_t r,int16_t aci)
{
	int16_t A0,A1,angle;
	uint8_t half;

// 0..45 derece cos ve sin degerleri ile yetinebilmek icin
// ceyrek bolgeler halinde hesaplama yapiyoruz.

// once 360 ve daha buyuk acilari 0..359 bolgesine getirelim
	angle=aci/360;
	angle=aci-angle*360;

A0=0;
A1=45;
for(half=1;half<=2;half++){
	
if((angle>=A0)&&(angle<A1)){  // 0<=angle<45  half_2 de 180<=angle<225
	                         XY->x=r*cosineTable[angle-A0]/100;
	                         XY->y=r*sineTable[angle-A0]/100;
													break;
                           }
A0=A0+45;
A1=A1+45;													 
if((angle>=A0)&&(angle<A1)){  // 45<=angle<90  half_2 de 225<=angle<270
	                         XY->x=r*sineTable[A1-angle]/100;
	                         XY->y=r*cosineTable[A1-angle]/100;
													break;
													}
A0=A0+45;
A1=A1+45;
if((angle>=A0)&&(angle<A1)){  // 90<=angle<135  half_2 de 270<=angle<315
	                         XY->x=r*sineTable[angle-A0]/100;
	                         XY->y=r*cosineTable[angle-A0]/100;
													break;
                           }
A0=A0+45;
A1=A1+45;													 
if((angle>=A0)&&(angle<A1)){  // 135>=angle<180 half_2 de 315<=angle<360
	                         XY->x=r*cosineTable[A1-angle]/100;
	                         XY->y=r*sineTable[A1-angle]/100;
													break;
													}
A0=A0+45;
A1=A1+45;													
}

  if((angle>90)&&(angle<270)) XY->x=-XY->x;	// x in negatif oldugu iki �eyrek									   
  if(angle>180) XY->y=-XY->y; // y nin negatif oldugu iki �eyrek
}


//******************************************************************
//                        TFT_DrawRectangle
//         (x1,y1),(x2,y2):
//******************************************************************  
void TFT_DrawRectangle(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2,uint16_t color)
{
	TFT_DrawLine(x1,y1,x2,y1,color);
	TFT_DrawLine(x1,y1,x1,y2,color);
	TFT_DrawLine(x1,y2,x2,y2,color);
	TFT_DrawLine(x2,y1,x2,y2,color);
}  

 
//***********************************************************************
//                        drawCircle_8_points                           *
//   Bresenham algoritmasi ile hesaplanan bir set x,y koordinatindan    *
//   yararlanarak cemberin 45 er derecelik 8 bolgesinde 8 nokta cizer.  *
//      (xc,yc) : Merkez koordinatlari                                  *
// 		(x,y)   : mutlak degerleri ayni olan x,y degerlerine sahip      *
//                8 cember noktasini cizer. Boylece tek hesaplamada 8   *
//                nokta cizilmis olur.                                  *
//                                                                      *
//***********************************************************************  
void drawCircle_8_points(int xc, int yc, int x, int y)
	{
	TFT_DrawPoint(xc + y, yc - x);  // 0..45
	TFT_DrawPoint(xc + x, yc - y);  //45..90
	TFT_DrawPoint(xc - x, yc - y);  //90..135		
	TFT_DrawPoint(xc - y, yc - x);  //135..180
	TFT_DrawPoint(xc - y, yc + x);  //180..225
	TFT_DrawPoint(xc - x, yc + y);  //225..270
	TFT_DrawPoint(xc + x, yc + y);  //270..315
	TFT_DrawPoint(xc + y, yc + x);  //315..360	
}


//***********************************************************************************
//                        TFT_circle                                                *
// Bresenham algoritmasi kullanarak 0-45 derece araligindaki x,y degerlerini        *
// hesapliyoruz.Bunlari drawCircle_8_points fonksiyonuna gondererek cemberin        *
// 8 simetrik bolgesindeki 8 noktayi cizdiriyoruz. Burada "d" karar parametresi    *
//                                                                                  *
//      (xc,yc) : cember merkez koordinatlari                                       *
//      c       : renk (WHITE, YELLOW vb.)                                                 *
//		r       : yari cap (piksel olarak)                                                 *
//		fill    : dolu/bos cember secimi                                               *
//***********************************************************************************  
void TFT_circle(int xc, int yc, uint16_t c, int r, int fill) {
	int x = 0, y = r, yi, d;

	d = 3 - 2 * r;
	if (fill) {    //dolu cember

		while (x <= y) {
			for (yi = x; yi <= y; yi++)  // merkezden disa dogru tek cizgi
				drawCircle_8_points(xc, yc, x, yi);

			if (d < 0) {
				d = d + 4 * x + 6;
			} else {
				d = d + 4 * (x - y) + 10;
				y--;
			}
			x++;
		}
	} else {        // bos cember
		while (x <= y) {
			drawCircle_8_points(xc, yc, x, y);
			if (d < 0) {
				d = d + 4 * x + 6;
			} else {
				d = d + 4 * (x - y) + 10;
				y--;
			}
			x++;
		}
	}
}

//******************************************************************
//                          TFT_ShowBmpDump
//      (x,y): Resmin ekran uzerindeki pozisyonu
//          fc:  ForeColor
//			bc:  BackColor
//			bmpID: Goruntulenecek BMP nin bmpFile daki baslangic satiri
//			width: genislik
//          height: yukseklik
//			mode: 0, ;1
//******************************************************************
void TFT_ShowBmp(uint16_t x, uint16_t y, uint16_t fc, uint16_t bc,
		const unsigned char *pBMP, uint8_t width, uint8_t height, short carpan,
		uint8_t mode) {
	int i;
	uint8_t temp;
	uint8_t numberOfChars, column;
	short reDrawPix, reDrawRow;

	uint16_t buffIndex;
	uint16_t buffSize;

	buffSize = SPIbuffSize;
	buffIndex = 0;

	numberOfChars = height * width / 8;

	TFT_SetWindows(x, y, x + width * carpan - 1, y + height * carpan - 1); //BMP nin cizilecegi alani sec
	TFT_RS_SET;
	TFT_CS_CLR;

	for (i = 0; i < numberOfChars; i++) {
		for (reDrawRow = 0; reDrawRow < carpan; reDrawRow++) { // cizimi buyutmek icin ayni satiri tekrar kullaniyoruz
			temp = *(pBMP + i);
			for (column = 0; column < 8; column++) //karakterin her biti bir pixel
					{
				for (reDrawPix = 0; reDrawPix < carpan; reDrawPix++) { //cizimi buyutmek icin ayni pixeli tekrar kullaniyoruz
					if (temp & 0x01) { // raster uzerindeki bit "1" ise front color, yoksa back color ile ciz
						buff[buffIndex] = fc >> 8;
						buff[buffIndex + 1] = fc & 0xFF;
					} else if (!mode) { //Mode "0" da bos pixellerde arka plani siliyoruz
						buff[buffIndex] = bc >> 8;
						buff[buffIndex + 1] = bc & 0xFF;
					}
					buffIndex += 2;
					if (buffIndex >= buffSize) { //buff dolunca TFT ye gonderiyoruz
						TFT_writeMultipleData(buff, buffIndex);
						buffIndex = 0;
					}
				} // End loop yatay pixel tekrari
				temp = temp >> 1;            // sonraki bite gec
			}
		} // End loop satir tekrari
	}
  if((buffIndex>0)&&(buffIndex<buffSize)) {  // buffer dolmadigi için yukarıda gitmemiş,
		TFT_writeMultipleData(buff, buffIndex); // tam dolu olmayan bufferi gonderiyoruz
}
	TFT_CS_SET;

	TFT_SetWindows(0, 0, TFT_W - 1, TFT_H - 1);
}
/**** END OF TFTShowBMP  ***/



//******************************************************************
//                          TFT_ShowChar
//      (x,y):
//      fc:  ForeColor
//			bc:  BackColor
//			num: Goruntulenecek karakter
//			size: Font Size
//			mode: 0, ;1
//******************************************************************
void TFT_ShowChar(uint16_t x, uint16_t y, uint16_t fc, uint16_t bc, uint8_t num,
		uint8_t size, uint8_t mode) {
	uint8_t temp;
	uint8_t pos, pos2, t;


	uint16_t buffIndex;
	uint16_t buffSize;

	buffSize = SPIbuffSize;
	buffIndex = 0;

	num = num - ' '; //karakterin Font dizini icindeki indeksini hesapliyoruz

	TFT_SetWindows(x, y, x + size / 2 - 1, y + size - 1); //Karakterin dizilecegi alani sec (genislik=fontSize/2)

	TFT_RS_SET;
	TFT_CS_CLR;

	pos2 = 0;

	for (pos = 0; pos < size; pos++) // Font Boyutu (yani yuksekligi)ni satir satir tara
			{
		if (((size == 32) || (size == 24)) && (pos != (pos / 2) * 2))
			pos2++; //iki kat buyuklukte olan fontlarda
					//her iki satirda bir satir ilerleyek yeni piksel oku
		if (size == 12)
			temp = asc2_1206[num][pos]; // Font ve satir secimi (12 ve 16 fontlar ve bunlarin iki katlarini kullaniyoruz)
		else if (size == 16)
			temp = asc2_1608[num][pos];
		else if (size == 24)
			temp = asc2_1206[num][pos2];  //24 lik font
		else
			temp = asc2_1608[num][pos2];  //32 lik font

		for (t = 0; t < size / 2; t++)  //Font'un bir satirini tara
				{
			if (temp & 0x01) { // raster uzerindeki bit "1" ise front color, yoksa back color ile ciz
				buff[buffIndex] = fc >> 8;
				buff[buffIndex + 1] = fc & 0xFF;
			} else if (!mode) { //Mode "0" da bos pixellerde arka plani siliyoruz
				buff[buffIndex] = bc >> 8;
				buff[buffIndex + 1] = bc & 0xFF;
			}
			buffIndex += 2; // Her pixelde 2 byte var.
			if ((buffIndex >= buffSize) || (pos == (size - 1))) {
//    		HAL_GPIO_TogglePin(OSC_Port,OSC_Pin);
				TFT_writeMultipleData(buff, buffIndex);
//    		while(HAL_SPI_Transmit(pTft_Spi,buff,buffIndex,10000)!=HAL_OK);
//    		HAL_GPIO_TogglePin(OSC_Port,OSC_Pin);
				buffIndex = 0;
			}
			if ((size == 12) || (size == 16))
				temp >>= 1;
			else if (t != (t / 2) * 2)
				temp >>= 1;	//2x Buyuk fontlarda her iki kolonda bir yeni pixel oku
		}
	}

	TFT_CS_SET;

	TFT_SetWindows(0, 0, TFT_W - 1, TFT_H - 1);

}
/**** END OF TFT_ShowChar  ****/

//******************************************************************
//                          TFT_ShowStringFast
//       TFT_ShowString fonksiyonunun hızlı versiyonu.
//       TFT_ShowChar fonksiyonunu kullanmaz,
//       karakter karakter değil, satırın tamamını bir
//       pencere halinde, buffere koyarak TFT ye gönderir.
//
//**** Sadece 12 ve 16 pix fontları destekler. (asc2_1206 ve asc2_1608) ****
//******************************************************************
void TFT_ShowStringFast(uint16_t x, uint16_t y, uint8_t fontSize, uint8_t* pLineToShow,
		 uint16_t fc, uint16_t bc, uint8_t mode) {
	uint8_t temp;
	uint8_t pos,t;
	uint8_t num;
	uint8_t charIndex;
	uint8_t stringLength;  // Karakter olarak satir uzunlugu
    uint8_t fontWidth = fontSize/2;
    uint8_t fcH,fcL;
    uint8_t bcH,bcL;

	uint16_t buffIndex;
	uint16_t buffSize;
	uint16_t lengthOfLine;  // Pixel olarak satir uzunluğu

	uint8_t fontEksiBir = fontSize-1;

	buffSize = SPIbuffSize;

	unsigned char* p;


tWordByte forecolor;
tWordByte backcolor;

forecolor.Word = fc;
backcolor.Word = bc;
/* fc ve bc nin alt-üst byte'larını yer değiştireceğiz,
 * TFT ye gönderilecek buffer'a hazır olması için
 */
temp=forecolor.HighByte;
forecolor.HighByte=forecolor.LowByte;
forecolor.LowByte=temp;

temp=backcolor.HighByte;
backcolor.HighByte=backcolor.LowByte;
backcolor.LowByte=temp;

    stringLength=strlen(pLineToShow);
	lengthOfLine=stringLength*fontWidth;// piksel olarak satır uzunluğu

	TFT_SetWindows(x, y, x + lengthOfLine - 1, y + fontSize - 1); //Textin dizilecegi alani sec (genislik=fontSize/2)

	TFT_RS_SET;
	TFT_CS_CLR;

	buffIndex = 0;

	for (pos = 0; pos < fontSize; pos++) // Font Boyutu (yani yuksekligini) satir satir tara
			{
		p = pLineToShow;
	    charIndex = 1;

	     while((*p<='~')&&(*p>=' ')) { // Basilamayan (non-printable char) ise yatay taramaya son ver

			num=*p - ' '; //karakterin Font dizini icindeki indeksini hesapliyoruz

		if (fontSize == 12)
			temp = asc2_1206[num][pos]; // Font ve satir secimi (12 ve 16 fontlar ve bunlarin iki katlarini kullaniyoruz)
		else if (fontSize == 16)
			temp = asc2_1608[num][pos];
		else  // Geçerli iki fonttan birisi değil ise varsayılan font 1206 olan.
 			temp = asc2_1206[num][pos];

		for (t = 0; t < fontWidth; t++)  //Font'un bir satirini tara (tek karakterin tek satırı 7.5 us)
				{
			// Aşağıdaki if/else 2 uS

			if (temp & 0x01) { // raster uzerindeki bit "1" ise front_color, yoksa back_color ile ciz
			    *(uint16_t *) (buff + buffIndex) = (uint16_t)forecolor.Word;
/*				*(buff+buffIndex) = forecolor.HighByte;
				*(buff+buffIndex+1) = forecolor.LowByte;
*/			} else if (!mode) { //Mode "0" da bos pixellerde arka plani siliyoruz
	            *(uint16_t *) (buff + buffIndex) = (uint16_t)backcolor.Word;
/*			    *(buff+buffIndex) = backcolor.HighByte;
				*(buff+buffIndex+1) = backcolor.LowByte;
*/			}
			buffIndex += 2; // Her pixelde 2 byte var.

			if ((buffIndex >= buffSize) || ((charIndex >= stringLength)&&(pos==fontEksiBir)))
			{  // Buffer dolmuş ya da string'in sonuna gelinmiş
//				HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_2);
				TFT_writeMultipleData(buff, buffIndex);
//				HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_2);
				buffIndex = 0;
			}
			temp >>= 1;
		}  // Font satiri bitti
		p++; //sonraki harfe geç
		charIndex++;
			} // end of while
	}  // Yeni satir... Bu çevrimde yeni satira geçmeden önce bir sonraki karaktere devam et

	TFT_CS_SET;

	TFT_SetWindows(0, 0, TFT_W - 1, TFT_H - 1);

}
/**** END OF TFT_ShowLine  ****/

//******************************************************************
//
//******************************************************************  	  
void TFT_ShowString(uint16_t x, uint16_t y, uint8_t size, unsigned char *p, uint16_t  pencolor, uint8_t mode)
{
	POINT_COLOR=pencolor;
	BACK_COLOR=BLUE;

    while((*p<='~')&&(*p>=' '))  // Basilamayan (non-printable char) ise goruntulemeye son ver
    {   
		if(x>(TFT_W-1)||y>(TFT_H-1))return; //Ekrana sigmiyorsa sonlandir
			     
        TFT_ShowChar(x,y,POINT_COLOR,BACK_COLOR,*p,size,mode);
        x+=size/2;  //Font genisligi kadar ilerle
        p++; // siradaki karaktere gec
    }  
} 


//******************************************************************
//
//******************************************************************
void TFT_PrintOnLine(uint8_t lineNo, uint8_t size, char *p, uint8_t mode)
{
	TFT_ShowString(0, lineNo*size, size, p, POINT_COLOR, mode);
}

//******************************************************************
//
//******************************************************************  
uint32_t mypow(uint8_t m,uint8_t n)
{
	uint32_t result=1;	 
	while(n--)result*=m;    
	return result;
}

//******************************************************************
//
//******************************************************************  			 
void TFT_ShowNum(uint16_t x,uint16_t y,uint32_t num,uint8_t len,uint8_t size)
{         	
	uint8_t t,temp;
	uint8_t enshow=0;						   
	for(t=0;t<len;t++)
	{
		temp=(num/mypow(10,len-t-1))%10;
		if(enshow==0&&t<(len-1))
		{
			if(temp==0)
			{
				TFT_ShowChar(x+(size/2)*t,y,POINT_COLOR,BACK_COLOR,' ',size,0);
				continue;
			}else enshow=1; 
		 	 
		}
	 	TFT_ShowChar(x+(size/2)*t,y,POINT_COLOR,BACK_COLOR,temp+'0',size,0); 
	}
} 


/* *************         lcdOnTFT  ***************************************
*        By Selcuk OZBAYRAKTAR      Mar 2019
*
*   display(uint8_t cursor, char* string) fonksiyonunu 4x20 LCD yi
*   simule ederek, herbir satiri iki satirda goruntulemek suretiyle
*   daha iri puntolarla 8x10 formatinda TFT ekran uzerinde calistirir.
*   LCD nin bulunmadiği durumlarda kullanilmak uzere uygundur.
*   Ekran uzerinde sol ust kose koordinatlari degistirilerek istenen yere
*   yerlestirilebilir.
*
 * cursorPos 0x80 / 0x89 arasinda ise 1. satir
 *           0x8A / 0x93 arasinda ise 2. satir
 *           0xC0 / 0xC9 arasinda ise 3. satir
 *           0xCA / 0xD3 arasinda ise 4. satir
 *           0x94 / 0x9D arasinda ise 5. satir
 *           0x9E / 0xA8 arasinda ise 6. satir
 *           0xD4 / 0xDD arasinda ise 7. satir
 *           0xDE / 0xE8 arasinda ise 8. satir
 *
**************************************************************************/
void lcdOnTFT_8x10(uint8_t cursorPos, char* pString)
{
#define displayWidth 10
#define satir_H 18

uint16_t satirY,satirbasi;
uint16_t topBorderY;
int8_t temp;

char line[displayWidth+1];  //satir yazdirmada kullanacagimiz buffer

//font_W=8;
//solUstX=TFT_W-displayWidth*font_W;
topBorderY=TFT_H/5;
//sagAltX=solUstX+displayWidth*font_W;
//sagAltY=solUstY+8*satir_H;

//TFT_DrawRectangle(solUstX, solUstY, sagAltX, sagAltY,WHITE);
satirY=topBorderY;
satirbasi=0x80;

if((cursorPos>=0x80)&&(cursorPos<0x8A)) { satirY=topBorderY;
satirbasi=0x80;
}
if((cursorPos>=0x8A)&&(cursorPos<0x94)) {satirY=topBorderY+satir_H;
satirbasi=0x8A;
}

if((cursorPos>=0xC0)&&(cursorPos<0xCA)) {satirY=topBorderY+2*satir_H;
satirbasi=0xC0;
}
if((cursorPos>=0xCA)&&(cursorPos<0xD4)) {satirY=topBorderY+3*satir_H;
satirbasi=0xCA;
}
if((cursorPos>=0x94)&&(cursorPos<0x9E)) {satirY=topBorderY+4*satir_H;
satirbasi=0x94;
}
if((cursorPos>=0x9E)&&(cursorPos<0xA9)) {satirY=topBorderY+5*satir_H;
satirbasi=0x9C;
}
if((cursorPos>=0xD4)&&(cursorPos<0xDE)) {satirY=topBorderY+6*satir_H;
satirbasi=0xD4;
}
if((cursorPos>=0xDE)&&(cursorPos<0xE9)) {satirY=topBorderY+7*satir_H;
satirbasi=0xDE;
}

// ortak islemler

copyString(pString,line,10);
line[displayWidth]=0x00;
printLineOnTFT(cursorPos,satirbasi,satirY,line);

// satir uzunlugunu konrol edelim, 2. satira tasiyor mu

temp=satirbasi+displayWidth-(strlen(pString)+cursorPos);
if(temp<0)
{ // 2.satira tasiyor satiri ikiye bolerek tasan kisim ile 2.satiri olusturalim
copyString(pString-temp,line,10);// Temp negatif, burada mutlak degerini ekliyoruz
line[displayWidth]=0x00;
// ikinci satiri yaz
cursorPos=satirbasi;  // bolunmus satirlarda 2.satirin ilk karakteri satir basinda oluyor
satirY+=satir_H;

printLineOnTFT(cursorPos,satirbasi,satirY,line);
}

}


//******************************************************************
//  TFT_Drawbmp16
//******************************************************************  
void TFT_Drawbmp16(uint16_t x,uint16_t y,const unsigned char *p)
{
  	int i; 
	unsigned char picH,picL; 
	TFT_SetWindows(x,y,x+40-1,y+40-1);
    for(i=0;i<40*40;i++)
	{	
	 	picL=*(p+i*2);
		picH=*(p+i*2+1);				
		TFT_send16bitData(picH<<8|picL);
	}	
	TFT_SetWindows(0,0,TFT_W-1,TFT_H-1);

}

//******************************************************************
//  TFT_DrawBitmap
//******************************************************************

void TFT_DrawBitmap(uint16_t Xpos, uint16_t Ypos, uint8_t *pBmp,
		uint16_t numberOfBytes, uint16_t bytesPerPixel) {

	uint32_t stop;
	uint16_t i, j;
	uint16_t red, green, blue;

	tWordByte data;

	stop = numberOfBytes - bytesPerPixel;

	j = 0;

	if (bytesPerPixel == 2)
		pBmp++;  // 16 bit piksel modunda iken bu otelemeye gerek oluyor

	for (i = 0; i <= stop; i = i + bytesPerPixel) {

		if (bytesPerPixel == 2) { /* 16 bit 5/6/5 RGB icin  */
			data.Word = *(pBmp++) << 8;
			data.Word |= *(pBmp++);
		}

		if (bytesPerPixel == 3) { /* Micosoft BMP 24 bit -> 16 bit adaptasyonu icin  */

			blue = *(pBmp++);
			green = *(pBmp++);
			red = *(pBmp++);

			/* 24 bit RGB formattan 16 bit 565 RGB Formata çevirme işlemi  */

			data.Word = (red << 8) & 0xF800; // Kirmizinin en buyuk 5 bitini aliyoruz
			data.Word |= (green << 3) & 0x07E0;  // yesilin en buyuk 6 biti
			data.Word |= (blue >> 3) & 0x001F;   // mavinin en buyuk 5 biti
		}

		if (bytesPerPixel == 4) { /* Micosoft BMP 32 bit -> 16 bit adaptasyonu icin  */
			blue = *(pBmp++);
			green = *(pBmp++);
			red = *(pBmp++);
			pBmp++;  // Alpha Byte'i (transparency) atliyoruz.

			/* 24 bit RGB formattan 16 bit 565 RGB Formata çevirme işlemi  */

			data.Word = (red << 8) & 0xF800; // Kirmizinin en buyuk 5 bitini aliyoruz
			data.Word |= (green << 3) & 0x07E0;  // yesilin en buyuk 6 biti
			data.Word |= (blue >> 3) & 0x001F;   // mavinin en buyuk 5 biti
		}

		/*   Bu yöntem piksellerin teker teker gönderilmesi ile calisiyor */
		/*
		 buff[0] = *(pBmp++);
		 buff[1] = *(pBmp++);
		 TFT_writeMultipleData(buff, 2);
		 */

		/* Bu yöntem DMA ve pikselleri once buffere yerlestirerek toplu data transferi ile calisiyor */

		if (j < SPIbuffSize) {
			buff[j++] = data.HighByte;
			buff[j++] = data.LowByte;
		} else { /* buffer dolmuş, yeni gelen pikselleri buffere koymadan once
		 dolan bufferi TFT ye gonder*/
			TFT_writeMultipleData(buff, j);
			j = 0;
			buff[j++] = data.HighByte;
			buff[j++] = data.LowByte;
		}
	}
	if (j > 0)
		TFT_writeMultipleData(buff, j);

}



//******************************************************************
//  TFT_Rotate
//******************************************************************

void TFT_Rotate(TFT_Orientation_t orientation) {
//	spi_writecommand(ILI9341_MADCTL);//ILI9341_MAC);
	if (orientation == Portrait_rotated) {
	TFT_WriteReg(0x36,0x18);
	} else if (orientation == Portrait_normal) {
	TFT_WriteReg(0x36,0xD8);
	} else if (orientation == Landscape_horizontalFlip) {
	TFT_WriteReg(0x36,0x28);
	} else if (orientation == Landscape_verticalFlip) {
	TFT_WriteReg(0x36,0xE8);
	} else if (orientation == Landscape_verticalFlip_24TP) {
	TFT_WriteReg(0x36,0xA8);
	} else if (orientation == Landscape_normal_35P) {
		TFT_WriteReg(0x36,0x28);
	} else if (orientation == Landscape_normal_24TP) {
	TFT_WriteReg(0x36,0x6C);
	}


	if (orientation == Portrait_normal ||orientation == Portrait_rotated) {
		_width=TFT_W;
		_height=TFT_H;
//		ILI9341_Opts.orientation = TM_ILI9341_Portrait;
	} else {
		_width=TFT_H;
		_height=TFT_W;
//		ILI9341_Opts.orientation = TM_ILI9341_Landscape;
	}
}

//******************************************************************
//TFT_GPIOInit
//******************************************************************
void TFT_GPIO_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;

/*  bunlar main icinde RCC Configuration ve GPIO configuration taraf1ndan ayarlaniyor zaten
 	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC|RCC_APB2Periph_GPIOB|RCC_APB2Periph_AFIO, ENABLE);
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable , ENABLE);
*/

#ifdef PARALLELMODE

//  TFT Kontrol pinleri

	GPIO_InitStruct.Pin = TFT_RS_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(TFT_RS_GPIO_Port, &GPIO_InitStruct); //GPIOA
	HAL_GPIO_WritePin(TFT_RS_GPIO_Port,TFT_RS_Pin, GPIO_PIN_SET);

	GPIO_InitStruct.Pin = TFT_CS_Pin;
	HAL_GPIO_Init(TFT_CS_GPIO_Port, &GPIO_InitStruct); //GPIOA
	HAL_GPIO_WritePin(TFT_CS_GPIO_Port,TFT_CS_Pin, GPIO_PIN_SET);

	GPIO_InitStruct.Pin = TFT_RD_Pin;
	HAL_GPIO_Init(TFT_RD_GPIO_Port, &GPIO_InitStruct); //GPIOA
	HAL_GPIO_WritePin(TFT_RD_GPIO_Port,TFT_RD_Pin, GPIO_PIN_SET);

	GPIO_InitStruct.Pin = TFT_WR_Pin;
	HAL_GPIO_Init(TFT_WR_GPIO_Port, &GPIO_InitStruct); //GPIOA
	HAL_GPIO_WritePin(TFT_WR_GPIO_Port,TFT_WR_Pin, GPIO_PIN_SET);

	GPIO_InitStruct.Pin = TFT_RST_Pin;
	HAL_GPIO_Init(TFT_RST_GPIO_Port, &GPIO_InitStruct); //GPIOB
	HAL_GPIO_WritePin(TFT_RST_GPIO_Port,TFT_RST_Pin, GPIO_PIN_SET);

	// PA11>>TFT_SWITCH (Calibration button)

		GPIO_InitStruct.Pin = TFT_SWITCH_Pin;
		GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
		GPIO_InitStruct.Pull = GPIO_PULLUP;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
		HAL_GPIO_Init(TFT_SWITCH_GPIO_Port, &GPIO_InitStruct);


	//  TFT Data pinleri

		GPIO_InitStruct.Pin = TFT_D0_Pin;
		GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
		HAL_GPIO_Init(TFT_D0_GPIO_Port, &GPIO_InitStruct);
		HAL_GPIO_WritePin(TFT_D0_GPIO_Port,TFT_D0_Pin, GPIO_PIN_SET);

		GPIO_InitStruct.Pin = TFT_D1_Pin;
		HAL_GPIO_Init(TFT_D1_GPIO_Port, &GPIO_InitStruct);
		HAL_GPIO_WritePin(TFT_D1_GPIO_Port,TFT_D1_Pin,GPIO_PIN_SET);

		GPIO_InitStruct.Pin = TFT_D2_Pin;
		HAL_GPIO_Init(TFT_D2_GPIO_Port, &GPIO_InitStruct);
		HAL_GPIO_WritePin(TFT_D2_GPIO_Port,TFT_D2_Pin,GPIO_PIN_SET);

		GPIO_InitStruct.Pin = TFT_D3_Pin;
		HAL_GPIO_Init(TFT_D3_GPIO_Port, &GPIO_InitStruct);
		HAL_GPIO_WritePin(TFT_D3_GPIO_Port,TFT_D3_Pin,GPIO_PIN_SET);

		GPIO_InitStruct.Pin = TFT_D4_Pin;
		HAL_GPIO_Init(TFT_D4_GPIO_Port, &GPIO_InitStruct);
		HAL_GPIO_WritePin(TFT_D4_GPIO_Port,TFT_D4_Pin,GPIO_PIN_SET);

		GPIO_InitStruct.Pin = TFT_D5_Pin;
		HAL_GPIO_Init(TFT_D5_GPIO_Port, &GPIO_InitStruct);
		HAL_GPIO_WritePin(TFT_D5_GPIO_Port,TFT_D5_Pin,GPIO_PIN_SET);

		GPIO_InitStruct.Pin = TFT_D6_Pin;
		HAL_GPIO_Init(TFT_D6_GPIO_Port, &GPIO_InitStruct);
		HAL_GPIO_WritePin(TFT_D6_GPIO_Port,TFT_D6_Pin,GPIO_PIN_SET);

		GPIO_InitStruct.Pin = TFT_D7_Pin;
		HAL_GPIO_Init(TFT_D7_GPIO_Port, &GPIO_InitStruct);
		HAL_GPIO_WritePin(TFT_D7_GPIO_Port,TFT_D7_Pin,GPIO_PIN_SET);


#endif
/*
#ifdef SPIMODE

	  // Configure GPIO pin Output Level
	  HAL_GPIO_WritePin(TFT_DC_GPIO_Port, TFT_DC_Pin, GPIO_PIN_RESET);
	  HAL_GPIO_WritePin(TFT_CS_GPIO_Port, TFT_CS_Pin, GPIO_PIN_RESET);
	  HAL_GPIO_WritePin(TFT_RST_GPIO_Port, TFT_RST_Pin, GPIO_PIN_RESET);

	  // Configure GPIO pins : TFT_CS_Pin
	  GPIO_InitStruct.Pin = TFT_CS_Pin;
	  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	  GPIO_InitStruct.Pull = GPIO_NOPULL;
	  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	  HAL_GPIO_Init(TFT_CS_GPIO_Port, &GPIO_InitStruct);

	  // *Configure GPIO pins : TFT_RST_Pin
	  GPIO_InitStruct.Pin = TFT_RST_Pin;
	  HAL_GPIO_Init(TFT_RST_GPIO_Port, &GPIO_InitStruct);

	  HAL_GPIO_WritePin(TFT_CS_GPIO_Port,TFT_CS_Pin, GPIO_PIN_SET);

	  // Configure GPIO pins : TFT_DC_Pin
	  GPIO_InitStruct.Pin = TFT_DC_Pin;
	  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	  GPIO_InitStruct.Pull = GPIO_NOPULL;
	  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	  HAL_GPIO_Init(TFT_DC_GPIO_Port, &GPIO_InitStruct);

#endif
*/

}

/* *************      copyString  ***************************************
*        By Selcuk OZBAYRAKTAR      March 2019
*
*
**************************************************************************/
void copyString(char* source, char* destination, uint8_t size)
{uint8_t i;
for(i=0;i<size;i++)
{*(destination+i)=*(source+i);}
}


/* *************      printLineOnTFT  ***************************************
*        By Selcuk OZBAYRAKTAR      March 2019
*
* TFT ekran uzerine bir satir yazar
**************************************************************************/
void printLineOnTFT(uint8_t cursorPos, uint8_t satirbasi,uint16_t satirY, char* line)
{
#define displayWidth 10

uint16_t cursor;
uint8_t font_W=8;
uint16_t indentX=TFT_W-displayWidth*font_W;

cursor=cursorPos-satirbasi;  // satir basindan karakter sayisi olarak ne kadar ileride
cursor=cursor*font_W;  // Pixel'e cevirelim
cursor=indentX+cursor; // TFT satir basina ekleyerek mutlak pixel pozisyonunu hesaplayalim

TFT_ShowString(cursor,satirY,16,line,WHITE,0);
}



/****************************************************************************************
 displayFloat
*****************************************************************************************/
void TFT_displayFloat(float Number, uint16_t xPos, uint16_t yPos,
		uint8_t fontSize) {
	int percentage;
	float number;
	char buffer[3];
	uint16_t cursorPosition;
	char negative;
	uint8_t offset;
	uint8_t font_W;

	font_W = fontSize / 2;
	negative = 0;

	number = Number;

	if (number < 0) {
		negative = 1;
		number = -number;
	}

	cursorPosition = xPos;

	if (negative == 1)
		cursorPosition = cursorPosition + font_W;

	TFT_ShowStringFast(cursorPosition, yPos, fontSize, "    ", YELLOW,BLUE, 0);

	itoa((int) number, buffer, 10);

	TFT_ShowStringFast(cursorPosition, yPos, fontSize, buffer, YELLOW,BLUE, 0);

	if (number >= 100)
		offset = 4;
	else if (number >= 10)
		offset = 3;
	else if (number >= 0)
		offset = 2;

	TFT_ShowStringFast(cursorPosition + (offset - 1) * font_W, yPos, fontSize, ".0",
			YELLOW,BLUE, 0);

	percentage = 100 * (number - (int) number);
	itoa(percentage, buffer, 10);
	if (percentage < 10)
		TFT_ShowStringFast(cursorPosition + (1 + offset) * font_W, yPos, fontSize,
				buffer, YELLOW,BLUE, 0);
	else
		TFT_ShowStringFast(cursorPosition + (offset) * font_W, yPos, fontSize,
				buffer, YELLOW,BLUE, 0);

	if (negative == 1) {
		TFT_ShowStringFast(cursorPosition - font_W, yPos, fontSize, "-", YELLOW,BLUE, 0);
	}
}


/* *******  readNumericInput   ****************************************
*        By Selcuk OZBAYRAKTAR      July 2017
*
*   non-numeric bir tusa basilana kadar, girilen numeric digitleri
*   entryArray[] dizisine yerlestirir. ilk girilen en buyuk digit [0] indeksinde
*   olacak sekilde.
*
*   Non n�merik bir tusa basildiginda giris islemini sonlandirir ve
*   tuslanmis olan rakami integer degerine �evirerek geri g�nderir.
*
*   Digit sayisi 4 ile sinirlandirilmistir. 4 digite ulasildiginda girisi sonlandirir,
*   girilmis olan rakami hesaplar ve geri gonderir.
*
*   Tek bir numerik digit bile girilmemis ise 0xFFFF degerini geri gonderir.
***********************************************************************/
float readNumericInput(struct mtrx_typedef* pTFT_keypad,uint16_t cursorX,uint16_t cursorY,uint8_t fontSize, uint8_t* entryArray){
	uint8_t temp;
	char completed=0;
	volatile float value=0;
    volatile int8_t i=0;
	uint8_t fontWidth;

	struct tBox_typedef* pButton;

	pButton=&button;

	fontWidth=fontSize/2;

	while(completed!=4)  // GO tu�una bas�lana kadar burada kal (Exit a�a��daki �evrim i�inde sonland�r�yor zaten)
	{
	TFT_ShowString(cursorX,cursorY, fontSize,">>      ", YELLOW,0);

	button.width=TFT_H/4-1;
	button.height=button.width;
	button.x0=(TFT_H/4)*3;
	button.y0=(TFT_H/4)*3;
	button.gap=2;
	button.errMargin=0;
	button.labelIndex=256;

	drawButtonAndLabel(pButton);

completed=0;
while (completed==0){
temp=0;
while(temp==0)temp=getButton(&source);
clickButton(pTFT_keypad,temp);

switch(temp)
{
	case '*': {//EXIT BASILMIS ("*")
		        TFT_Clear(BLUE);
		        return 0xFFFF; }
	case 'D': {//"GO" MOVE TO ENTERED COORDINATES ("D")
              completed=4;
          	button.labelIndex=736;
          	drawButtonAndLabel(pButton);

	          break;}

	case 'A': completed=1; i=0; break; //"A" numerik giris sirasinda girisi sonlandir,
	case 'B': completed=2; i=0; break; //"B"
	case 'C': completed=3; i=0; break; //"C"
	default :{    //sadece 0..9 ondalik digitleri ve noktayi buraya gonderiyoruz
			entryArray[i]=temp;
			entryArray[i+1]=0x00;
//			ItoA(entryArray[i],buffer,10);
			TFT_ShowString(cursorX+i*fontWidth,cursorY, fontSize,entryArray+i, YELLOW,0);
//    display(0x89+i,buffer);
           i++;
		Delay_1ms(200); //avoid repeated touches
		break;
			}
				} //end Switch
			} // end while completed loop
	}

	if(i==0) value=0xFFFF;

	if(i>0) // more than 1 digit entered, sayiya �evirelim
		value=strtod(entryArray,NULL);
	return value;
}

