#ifndef __TFT_BASIC_H
#define __TFT_BASIC_H


#include "inttypes.h"
#include "stm32f1xx_hal.h"

int16_t  _width, _height; // Display w/h as modified by current rotation

#define USE_HORIZONTAL  	1
#define TFT_USE8BIT_MODEL   1
#define TFT_INVERTED

#ifdef SPIMODE
SPI_HandleTypeDef* pTft_Spi;
#endif

#define TFT_Type_1  // Touch panel kalibrasyonunda hedef isaretinin yerini
					// ekran tipine gore alt/ust, sag/sol aksetmek gerekiyor.
					// terslik cikarsa bu tanimi kapatiyoruz.

//#define useEeprom // Touch panel kullanildiginda kalibrasyon parametrelerini
					// 24C32 .. Eeprom da saklamak istersek aciyoruz. Kit uzerinde
					// I2C arayuzlu 24Cxx EEPROM olmali.

/*
 * ASAGIDAKİLER PROJECT PROPERTIES ICINDE TANIMLANIYOR. NASIL YAPILACAGINI
 * GORMEK ICIN YUKARIDAKI ACIKLAMAYA BAK
 *
 */


#ifdef TFT_320x240
	#if USE_HORIZONTAL==1
		#define TFT_W 320
		#define TFT_H 240
		#else
		#define TFT_W 240
		#define TFT_H 320
	#endif
#endif

#ifdef TFT_480x320
	#if USE_HORIZONTAL==1
		#define TFT_W 480
		#define TFT_H 320
		#else
		#define TFT_W 320
		#define TFT_H 480
	#endif
#endif

/* TFT Paralel modda iken
 *
 *  SD SPI1 uzerinde   -- Minikit RBT
 *  TFT_P de PA1>>TFT_P_RS, PA0>>TFT_P_CS, PA12>>TFT_P_RD, PA15>>TFT_P_WR
 *
 * EĞER TOUCH PANEL KULLANILIYORSA :
 *
 * TFT_CS pini TPX_M ile ortak. TP okuma Modunda Analog input olarak ADC00 girisi oluyor
 * TFT_RS pini TPY_M ile ortak. TP okuma Modunda Analog input olarak ADC01 girisi oluyor
 * TP modunda TFT_P nin D0 pini TPX, D1 pini de TPY olarak kullaniliyor
 *
 * TFT_RS pini bazi TFT lerde "DC" olarak adlandiriliyor
 */



#ifdef PARALLELMODE
/*   PARALEL TFT DE SD KART SPI1 UZERINDE    */
/* SD_CS pini cubeMX ile main.h içinde tanımlanıyor CNC de PC12 olarak	  */

		#define SD_SCK_Pin 			SPI1_SCK_Pin
		#define SD_SCK_GPIO_Port 	SPI1_SCK_GPIO_Port
		#define SD_MOSI_Pin 		SPI1_MOSI_Pin
		#define SD_MOSI_GPIO_Port 	SPI1_MOSI_GPIO_Port
		#define SD_MISO_Pin 		SPI1_MISO_Pin
		#define SD_MISO_GPIO_Port 	SPI1_MISO_GPIO_Port
#endif


#ifdef SPIMODE

/* SD_CS pini cubeMX ile main.h içinde tanımlanıyor MINIKIT_S de PB12 olarak */

		#define SD_SCK_Pin 			SPI2_SCK_Pin
		#define SD_SCK_GPIO_Port 	SPI2_SCK_GPIO_Port
		#define SD_MOSI_Pin			SPI2_MOSI_Pin
		#define SD_MOSI_GPIO_Port 	SPI2_MOSI_GPIO_Port
		#define SD_MISO_Pin 		SPI2_MISO_Pin
		#define SD_MISO_GPIO_Port 	SPI2_MISO_GPIO_Port
#endif

/*   AŞAĞIDAKİLER cubeMX ile tanımlanıyor, o nedenle buradan kaldırdım */

/* CNC_P (MINIKIT)TFT KONTROL PINLERI *
 *                                    *
 * Bu pinlerin tanımlamaları CubeMX   *
 * ile, main.h içinde yapılmaktadır.  *
 *                                    *
 * TFT_RST        PB0                 *
 * TFT_RD         PA12                *
 * TFT_WR         PA15                *
 * TFT_RS         PA1                 *
 * TFT_CS         PA0                 *
 **************************************/

/* CNC_S (MINIKIT)TFT KONTROL PINLERI *
 *                                    *
 * Bu pinlerin tanımlamaları CubeMX   *
 * ile, main.h içinde yapılmaktadır.  *
 *                                    *
 * TFT_RST        PB0                 *
 * TFT_RS(DC)     PA4                 *
 * TFT_CS         PB8                 *
 **************************************/

/** NUCLEO DA TFT KONTROL PINLERI  ****
 *                                    *
 * Bu pinlerin tanımlamaları CubeMX   *
 * ile, main.h içinde yapılmaktadır.  *
 *                                    *
 * TFT_RST        PC1                 *
 * TFT_RD         PA0                 *
 * TFT_WR         PA1                 *
 * TFT_RS         PA4                 *
 * TFT_CS         PB0                 *
 **************************************/

/*


#ifndef NUCLEO
	#define TFT_RST_GPIO_Port	GPIOB
	#define TFT_RST_Pin	        GPIO_PIN_0
	#define TFT_RST             TFT_RST_GPIO_Port,TFT_RST_Pin
#endif


#ifdef SPIMODE	// TFT SPI1 uzerinde, SD SPI2 de
// SD SPI2 uzerinde   -- Minikit RBT

#ifdef STM32_MINI_S
#define TFT_CS_Pin   	GPIO_PIN_8
#define TFT_CS_GPIO_Port  	GPIOB
#define TFT_DC_Pin   		GPIO_PIN_4   // BAZI TFT LERDE BUNUN ADI RS
#define TFT_DC_GPIO_Port  	GPIOA        // BAZI TFTLERDE BUNUN ADI RS

#define TFT_RS_Pin   		GPIO_PIN_4   // BAZI TFT LERDE BUNUN ADI DC
#define TFT_RS_GPIO_Port  	GPIOA        // BAZI TFTLERDE BUNUN ADI DC
#endif

#ifdef STM32_MAPPLE
#define TFT_CS_Pin   	GPIO_PIN_5
#define TFT_CS_GPIO_Port  	GPIOB
#endif

#ifdef STM32_PICO
#define TFT_CS_Pin   	GPIO_PIN_5
#define TFT_CS_GPIO_Port  	GPIOB
#endif


 #define TFT_CLK_Pin   		SPI1_CLK_Pin
 #define TFT_CLK_GPIO_Port 	SPI1_CLK_GPIO_Port

 #define TFT_MOSI_Pin   		SPI1_MOSI_pin
 #define TFT_MOSI_GPIO_Port  SPI1_MOSI_GPIO_Port

#endif
*/

//----------------- TFT KONTROL KOMUTLARI         ----------------

	#define	TFT_CS_SET  		HAL_GPIO_WritePin(TFT_CS_GPIO_Port, TFT_CS_Pin, GPIO_PIN_SET)
	#define	TFT_CS_CLR  		HAL_GPIO_WritePin(TFT_CS_GPIO_Port, TFT_CS_Pin, GPIO_PIN_RESET)

	#define	TFT_RS_SET			HAL_GPIO_WritePin(TFT_RS_GPIO_Port, TFT_RS_Pin, GPIO_PIN_SET)	// Bazi TFT lerde bunun ad� "DC"
	#define	TFT_RS_CLR			HAL_GPIO_WritePin(TFT_RS_GPIO_Port, TFT_RS_Pin, GPIO_PIN_RESET)

	#define	TFT_RD_SET			HAL_GPIO_WritePin(TFT_RD_GPIO_Port, TFT_RD_Pin, GPIO_PIN_SET)
	#define	TFT_RD_CLR			HAL_GPIO_WritePin(TFT_RD_GPIO_Port, TFT_RD_Pin, GPIO_PIN_RESET)

	#define	TFT_WR_SET			HAL_GPIO_WritePin(TFT_WR_GPIO_Port, TFT_WR_Pin, GPIO_PIN_SET)  // Bu pin sadece TFT_P de var
	#define	TFT_WR_CLR			HAL_GPIO_WritePin(TFT_WR_GPIO_Port, TFT_WR_Pin, GPIO_PIN_RESET)

	#define	TFT_RST_SET			HAL_GPIO_WritePin(TFT_RST_GPIO_Port, TFT_RST_Pin, GPIO_PIN_SET)
	#define	TFT_RST_CLR			HAL_GPIO_WritePin(TFT_RST_GPIO_Port, TFT_RST_Pin, GPIO_PIN_RESET)




typedef enum {
	Portrait_normal,
	Portrait_rotated,
	Landscape_horizontalFlip,
	Landscape_verticalFlip,
	Landscape_verticalFlip_24TP,
	Landscape_normal_35P,
	Landscape_normal_24TP,
} TFT_Orientation_t;


struct pixelPosition_typedef
{
	int16_t x;
	int16_t y;
};

struct pixelPosition_typedef pixelPosition;

uint16_t POINT_COLOR, BACK_COLOR;

uint8_t source; // kullanici girislerinin kaynagi - mekanik keypanel=1, touchpanel=2 ...

//////////////////////////////////////////////////////////////////////////////////

#define TFT_SWITCH_GPIO_Port	GPIOA
#define TFT_SWITCH_Pin	GPIO_PIN_11
#define TFT_SWITCH      TFT_SWITCH_GPIO_Port,TFT_SWITCH_Pin

#define L2R_U2D  0
#define L2R_D2U  1
#define R2L_U2D  2
#define R2L_D2U  3

#define U2D_L2R  4
#define U2D_R2L  5
#define D2U_L2R  6
#define D2U_R2L  7

#define DFT_SCAN_DIR  L2R_U2D

#define WHITE       0xFFFF
#define BLACK      	0x0000	  
#define BLUE       	0x001F  
#define BRED        0XF81F
#define GRED 	    0XFFE0
#define GBLUE	    0X07FF
#define RED         0xF800
#define MAGENTA     0xF81F
#define GREEN       0x07E0
#define CYAN        0x7FFF
#define YELLOW      0xFFE0
#define BROWN 		0XBC40
#define BRRED 		0XFC07


#define LIGHTGRAY       0XEF5B  // 11101 111010 11011  (normal)
#define LGRAY 			0XC618  // 11000 110000 11000  (darker)
#define LGRAY2 	 		0XC618  // 10100 101000 10100  (a bit more darker)
#define DARKGRAY  		0X8430  // 10000 100001 10000 (darkest)

#define DARKBLUE      	 0X01CF 
#define LIGHTBLUE      	 0X7D7C  
#define GRAYBLUE       	 0X5458
#define LIGHTGREEN     	0X841F


#define LGRAYBLUE      	0XA651
#define LBBLUE          0X2B12

#endif


/***************************************************************************
 *                                                                         *
 * paralel_DATAOUT( fonksiyonumuz PB8..PB15 pinleri üzerinden çalışıyordu. *
 * Buna alternatif olarak bu makro daha esnek.                             *
 *                                                                         *
 * ÖNEMLİ: TFT pinlerinin bağlı olduğu GPIO lara göre her seferinde        *
 * aşağıda adaptasyon yapmak gerekiyor                                     *
 *                                                                         *
 *                                                                         *
 * Aşağıdaki iki yöntemi Controllers Tech You Tube sitesinden aldım.       *
 *                                                                         *
 ***************************************************************************/

/****** bir bytelık data'yı rasgele seçilmiş 8 pine yazmak için **********
 *
 * Once bu 8 pinin sıfırlanması gerekiyor. Bunun için pinlerin bulunduğu
 * portun BSSR ının üst 16 bit'inde bu pinlere karşı düşenlere "1" yazılıyor.
 * Aşağıdaki şablonda A ve B portlarında seçilmiş pinler olduğu varsayımı ile
 * ikişer grup kod var.
 *
 * Örnek:B portunun 3,4,8,9 pinleri için : GPIOB->BSSR= 0b0000001100011000<<16
 *
 * Pinlere data yazmak için de BSSR ın alt 16 bitinde, bu pinlere karşı düşenlere
 * '1' yazılır.
 *
 * Örnek: data byte in D4 biti PB7 ye, D6 biti PB2 ye yazılacak ise :
 *
 * GPIOB->BSSR = (data&(1<<4)) << 3  yani önce D4 bitini seçip, bunu sola 3
 * kaydırınca 4+3=7 pozisyonuna getirmiş oluyoruz.
 * GPIOB->BSSR = (data&(1<<6)) >> 4  bu şekilde data'nın D6 bitini alarak 4 sağa
 * yani 6-4 = 2 pozisyonuna yazmış oluyoruz.
 *
 * Aşağıdaki örnekde :
 *
 * D0  :  PB0
 * D1  :  PB1
 * D2  :  PA15
 * D3  :  PB3
 * D4  :  PB4
 * D5  :  PB5
 * D6  :  PB6
 * D7  :  PA5
 *
 * #define writeByteToAny8(data) {\
	GPIOA->BSRR = 0b1000000000100000 <<16; \
	GPIOB->BSSR = 0b0000000111101011 <<16; \
	GPIOA->BSSR = ((data&(1<<7))>>2) \
	            | ((data&(1<<2))<<13);\
	GPIOB->BSSR = ((data&(1<<0))<<0) \
                | ((data&(1<<1))<<0) \
                | ((data&(1<<3))<<0) \
				| ((data&(1<<4))<<0) \
				| ((data&(1<<5))<<0) \
				| ((data§(1<<6))<<0);}
 *
 *
 * Bizim minikitlerde TFT data pinleri PB8..PB15 arasinda.
 *
 */

#ifdef MINIKIT
#define writeByteToAny8(d) {\
	GPIOB->BSRR = 0b1111111100000000 <<16; \
	GPIOB->BSRR = ((d & (1<<0)) << 8) \
                | ((d & (1<<1)) << 8) \
				| ((d & (1<<2)) << 8) \
                | ((d & (1<<3)) << 8) \
				| ((d & (1<<4)) << 8) \
				| ((d & (1<<5)) << 8) \
				| ((d & (1<<6)) << 8) \
				| ((d & (1<<7)) << 8);}
#endif

#ifdef CNC_P
#define writeByteToAny8(d) {\
	GPIOB->BSRR = 0b1111111100000000 <<16; \
	GPIOB->BSRR = ((d & (1<<0)) << 8) \
                | ((d & (1<<1)) << 8) \
				| ((d & (1<<2)) << 8) \
                | ((d & (1<<3)) << 8) \
				| ((d & (1<<4)) << 8) \
				| ((d & (1<<5)) << 8) \
				| ((d & (1<<6)) << 8) \
				| ((d & (1<<7)) << 8);}
#endif

#ifdef CNC_P
#define writeByteToPort(d) {\
	GPIOB->BSRR |= 0xFF000000; \
	GPIOB->BSRR = (d<< 8);}
#endif

/* Nucleo için TFT DATA Pinleri:
*
* D0  :  PA9
* D1  :  PC7
* D2  :  PA10
* D3  :  PB3
* D4  :  PB5
* D5  :  PB4
* D6  :  PB10
* D7  :  PA8
*/

#ifdef NUCLEO
#define writeByteToAny8(d) {\
	GPIOA->BSRR = 0b0000011100000000 <<16; \
	GPIOB->BSRR = 0b0000010000111000 <<16; \
	GPIOC->BSRR = 0b0000000010000000 <<16; \
	GPIOA->BSRR = ((d & (1<<0)) << 9) \
                | ((d & (1<<2)) << 8) \
				| ((d & (1<<7)) << 1); \
    GPIOB->BSRR = ((d & (1<<3)) << 0) \
				| ((d & (1<<4)) << 1) \
				| ((d & (1<<5)) >> 1) \
				| ((d & (1<<6)) << 4); \
    GPIOC->BSRR = ((d & (1<<1)) << 6);}
#endif

/*****  Rasgele seçilmiş pinlerden 8 bit datayı bir byte içine okumak  *****
 *
 *
 * Pinlerden data okumak için bunların portlarının IDR leri okunur.
 *
 * Örnek: data byte in D4 biti PB7 den, D6 biti PB2 den okunacak ise :
 *
 * data |= GPIOB->IDR & (1<<7) >> 3 Bu şekilde önce PB7 yi okuyup D4 e yerleştiriyoruz 7-3=4
 *
 * data |= GPIOB->IDR § (1<<2)  <<4 bu şekilde PB2 yi okuyup D6 ya yerleştriyoruz 2+4=6
 *
 * Yukarıdaki yazma fonksiyonundaki örnek ile :
 *
 *
 *#define readAny8ToByte() ( \
		  ((GPIOA->IDR & (1<<5))<<2) \
		| ((GPIOA->IDR & (1<<15))>>13) \
		| ((GPIOB->IDR & (1<<0))>>0) \
	    | ((GPIOB->IDR & (1<<1))>>0) \
	    | ((GPIOB->IDR & (1<<3))>>0) \
	    | ((GPIOB->IDR & (1<<4))>>0) \
	    | ((GPIOB->IDR & (1<<5))>>0) \
	    | ((GPIOB->IDR & (1<<6))>>0))
 *
 *  Minikitlerde TFT data pinleri PB8..PB15 e bağlı olduğundan :
 */

#define readAny8ToByte() ( \
		  ((GPIOB->IDR & (1<<2))>>0) \
		| ((GPIOB->IDR & (1<<7))>>0) \
		| ((GPIOB->IDR & (1<<0))>>0) \
	    | ((GPIOB->IDR & (1<<1))>>0) \
	    | ((GPIOB->IDR & (1<<3))>>0) \
	    | ((GPIOB->IDR & (1<<4))>>0) \
	    | ((GPIOB->IDR & (1<<5))>>0) \
	    | ((GPIOB->IDR & (1<<6))>>0) \
					)


struct mtrx_typedef{
short matrixNo;
uint8_t numberOfColumns;
uint8_t numberOfRows;
uint16_t leftUpCornerX;
uint16_t leftUpCornerY;
uint16_t xSize;
uint16_t ySize;
short gap;
short labelOffset;
short errMargin;
};

 struct tBox_typedef{ //  Butonlar icin kullandigimiz textBox nesnesi
   uint16_t x0; //sol ust kose X
   uint16_t y0; //sol ust kose Y
   uint16_t width;
   uint16_t height;
   uint16_t labelIndex; // Buton uzerine konacak BMP sembolunun bmp file icindeki indeksi
   uint8_t pressed;
   uint8_t gap;
   uint8_t errMargin;
 };

 struct tBox_typedef button;

 struct tBox_typedef* pButton;

 struct point {
 	  float x;
 	  float y;
 	  float z;
 	  char name1;
 	  char name2;
 	  char name3;
 	};

struct mtrx_typedef TFT_keypad;

typedef char dosyaAdi_t[20];
dosyaAdi_t dropDownList[10];
dosyaAdi_t* pDropDownList;




void TFT_GPIO_Init(void); //tft.h
void TFT_Init(void); //tft.h
void TFT_Rotate(TFT_Orientation_t orientation);// tft.h
void lcdOnTFT_8x10(uint8_t cursorPos, char* pString);

void TFT_displayFloat(float Number, uint16_t xPos, uint16_t yPos, uint8_t fontSize);
void TFT_ShowNum(uint16_t x,uint16_t y,uint32_t num,uint8_t len,uint8_t size);
void TFT_Clear(uint16_t Color); //tft.h
void TFT_DrawLine(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color);
void TFT_DrawPoint(uint16_t x,uint16_t y);
void TFT_circle(int xc, int yc,uint16_t c,int r, int fill);
void TFT_DrawRectangle(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2,uint16_t color);
void TFT_DrawFillRectangle(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2);
void TFT_ShowString(uint16_t x, uint16_t y, uint8_t size, unsigned char *p, uint16_t  pencolor, uint8_t mode);
void TFT_PrintOnLine(uint8_t lineNo, uint8_t size, char *p, uint8_t mode);

