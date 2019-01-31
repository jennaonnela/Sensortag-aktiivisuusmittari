/***************************************
 *
 * Jenna Onnela, 2549569
 * Riina Annunen, 2553801
 *
 ***************************************/

#include <math.h>
//Otetaan SensorTagin kirjastoja
#include <string.h>
#include <stdio.h>
#include <inttypes.h>

#include <stdint.h>
#include "assert.h"

/* XDCtools Header files */
#include <xdc/std.h>
#include <xdc/runtime/System.h>
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Clock.h>

/* TI-RTOS Header files */
#include <ti/drivers/I2C.h>
#include <ti/drivers/PIN.h>
#include <ti/drivers/pin/PINCC26XX.h>
#include <ti/drivers/i2c/I2CCC26XX.h>
#include <ti/mw/display/Display.h>
#include <ti/mw/display/DisplayExt.h>

/* Board Header files */
#include "Board.h"

/* jtkj Header files */
#include "wireless/comm_lib.h"
#include "sensors/bmp280.h"
#include "sensors/mpu9250.h"

//Muuttujia kiihtyvyys- ja gyrosensoreille
float 	ax,
		ay,
		az,
		gx,
		gy,
		gz,
		max,
		min;

//Ilmanpaine ja l‰mpˆtila
double pres,
		temp;

//liikutaanko ylˆs vai alas. 1 = ylˆs 2 = alas
int liikkumissuunta;

//Datojen keskiarvoja yms. Laskettu testidatoista
float hissika = -0.968129,
		paikallaanka = -1.013304,
		dataedellisetaz[5],
		dataedellisetay[5],
		dataedellisetax[5],
		dataedellisetgz[5],
		dataka,
		datakh,
		datagz,
		portaatka = -0.985460 ,
		portaatkh = 0.20795242059498,
	    hissikh = 	0.041018087050789,
	    paikallaankh = 0.020218367387056;
//Lista ilmaanpaineelle, josta voidaan katsoa sitten menn‰‰nkˆ ylˆs vai alas
double pressure[2];

//Display handle
Display_Handle hDisplay;



/**********************************************
* 		GRAFIIKKA
* 		ALKAA
*
***********************************************/


//Portaat bittimappi
const uint8_t portaat_1[8] = {
	0x3, 0x3, 0x3, 0x3, 0x3, 0x3, 0x3F, 0x3F
};
const uint8_t portaat_2[8] = {
	0xFC, 0xFC, 0, 0, 0, 0, 0, 0
};
const uint8_t portaat_3[8] = {
	0, 0, 0xF, 0xF, 0xC, 0xC, 0xC, 0xC
};
const uint8_t portaat_4[8] = {
	0x30, 0x30, 0xF0, 0xF0, 0, 0, 0, 0
};
const uint8_t portaat_5[8] = {
	0, 0, 0, 0, 0x3F, 0x3F, 0x30, 0x30
};

//Hissi bittimappi
const uint8_t hissi_1[32] = {
	0xFF, 0xFF, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0,
	0xC1, 0xC1, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0,
	0xC1, 0xC1, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0,
	0xFF, 0xFF
};

const uint8_t hissi_2[32] = {
	0xFF, 0xFF, 0, 0, 0, 0x18, 0x3C, 0x7E, 0xFF, 0xDB,
	0x99, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x99,
	0xDB, 0xFF, 0x7E, 0x3C, 0x18, 0, 0, 0, 0, 0,
	0xFF, 0xFF
};

const uint8_t hissi_3[32] = {
	0xFF, 0xFF, 0x3, 0x3, 0x3, 0x3, 0x3, 0x3, 0x3, 0x3,
	0x83, 0x83, 0x3, 0x3, 0x3, 0x3, 0x3, 0x3, 0x3, 0x3,
	0x83, 0x83, 0x3, 0x3, 0x3, 0x3, 0x3, 0x3, 0x3, 0x3,
	0xFF, 0xFF
};

// logon vasen ylaosa
const uint8_t otit_vas1[5] = {
	0xAA, 0xEA, 0x3A, 0xE, 0x3
};
const uint8_t otit_vas2[18] = {
	0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0x6A, 0x2A, 0x3A, 0x1A, 0xA, 0xE, 0x6, 0x6, 0x2, 0x1, 0x1
};

// logon oikea alaosa
const uint8_t otit_oik1[18] = {
	0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x57, 0x56, 0x56, 0x56, 0x54, 0x5C, 0x58, 0x70, 0x60, 0x40, 0xC0, 0x80
};

const uint8_t otit_oik2[6] = {
	0x55, 0x57, 0x5C, 0x70, 0xC0, 0x80
};

//Nuoli ylos bittimappi

const uint8_t nuoliy [12] = {
	0x18, 0x3C, 0x7E, 0xFF, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18,
	0x18, 0x18,
};

//Nuoli alas bittimappi

const uint8_t nuolia [12] = {
	0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0xFF, 0x7E,
	0x3C, 0x18
};

//Funktio nuolen piirtoon
Void drawNuoliYlos(){

	// Mustavalkoinen kuva: v‰rit musta ja valkoinen
	uint32_t imgPalette[] = {0, 0xFFFFFF};

	// Kuvan m‰‰rittelyt
	tImage image_NuolY = {
		.BPP = IMAGE_FMT_1BPP_UNCOMP,
		.NumColors = 2,
		.XSize = 1,
		.YSize = 12,
		.pPalette = imgPalette,
		.pPixel = nuoliy
	};

	if (hDisplay) {

		// Grafiikkaa varten tarvitsemme lis‰‰ RTOS:n muuttujia
		tContext *pContext = DisplayExt_getGrlibContext(hDisplay);

		if (pContext) {

			GrImageDraw(pContext, &image_NuolY, 6, 80);
			GrFlush(pContext);

		} // if
	} // if

}

//Funktio nuolen piirtoon
Void drawNuoliAlas(){

	// Mustavalkoinen kuva: v‰rit musta ja valkoinen
	uint32_t imgPalette[] = {0, 0xFFFFFF};

	// Kuvan m‰‰rittelyt
	tImage image_NuolA = {
		.BPP = IMAGE_FMT_1BPP_UNCOMP,
		.NumColors = 2,
		.XSize = 1,
		.YSize = 12,
		.pPalette = imgPalette,
		.pPixel = nuolia
	};

	if (hDisplay) {

		// Grafiikkaa varten tarvitsemme lis‰‰ RTOS:n muuttujia
		tContext *pContext = DisplayExt_getGrlibContext(hDisplay);

		if (pContext) {

			GrImageDraw(pContext, &image_NuolA, 6, 80);
			GrFlush(pContext);

		} // if
	} // if

}

/* Funktio menun valintalaatikon piirtoon
 * Saa parametreina laatikon vasemaan ylakulman x ja y koordinaatit
 */
Void drawSelect(uint8_t x, uint8_t y){

	// suorakulmion maarittely
	tRectangle select = {
		.sXMin = x,
		.sYMin = y,
		.sXMax = 75,
		.sYMax = y + 17,
	};

	if (hDisplay) {
		// Grafiikkaa varten tarvitsemme lis‰‰ RTOS:n muuttujia
		tContext *pContext = DisplayExt_getGrlibContext(hDisplay);
		GrContextForegroundSet(pContext, 0xFFFFFF);

		if (pContext) {

			// Piirretaan suorakulmio
			GrRectDraw(pContext, &select);
		    GrFlush(pContext);

		} // if
	} // if

}

/*
 * Funktio menun valintalaatikon poistoon
 * Saa parametreina poistettavan alueen vasemman
 * ylakulman x ja y koordinaatit
 */
Void deleteSelect(uint8_t x, uint8_t y){

	tRectangle select = {
		.sXMin = x,
		.sYMin = y,
		.sXMax = 75,
		.sYMax = y + 17,
	};

	if (hDisplay) {
		// Grafiikkaa varten tarvitsemme lis‰‰ RTOS:n muuttujia
		tContext *pContext = DisplayExt_getGrlibContext(hDisplay);
		GrContextForegroundSet(pContext, 0);

		if (pContext) {

			GrRectDraw(pContext, &select);
		    GrFlush(pContext);

		 } // if
	} // if
}

Void drawYmpyra(uint8_t r){

	if (hDisplay) {
		// Grafiikkaa varten tarvitsemme lis‰‰ RTOS:n muuttujia
		tContext *pContext = DisplayExt_getGrlibContext(hDisplay);
		GrContextForegroundSet(pContext, 0xFFFFFF);

		if (pContext) {

			// Piirretaan ympyra
			GrCircleFill(pContext, 48, 48, r );
		    GrFlush(pContext);

		} // if
	} // if

}

Void drawYmpyraM(uint8_t r){

	if (hDisplay) {
		// Grafiikkaa varten tarvitsemme lis‰‰ RTOS:n muuttujia
		tContext *pContext = DisplayExt_getGrlibContext(hDisplay);
		GrContextForegroundSet(pContext, 0);

		if (pContext) {

			// Piirretaan ympyra
			GrCircleDraw(pContext, 48, 48, r );
		    GrFlush(pContext);

		} // if
	} // if

}


//Funktio portaiden piirtoon
Void drawPortaat(){

	// Mustavalkoinen kuva: v‰rit musta ja valkoinen
	uint32_t imgPalette[] = {0, 0xFFFFFF};

	// Kuvan m‰‰rittelyt
	tImage image_port1 = {
		.BPP = IMAGE_FMT_1BPP_UNCOMP,
		.NumColors = 2,
		.XSize = 1,
		.YSize = 8,
		.pPalette = imgPalette,
		.pPixel = portaat_1
	};
	tImage image_port2 = {
		.BPP = IMAGE_FMT_1BPP_UNCOMP,
		.NumColors = 2,
		.XSize = 1,
		.YSize = 8,
		.pPalette = imgPalette,
		.pPixel = portaat_2
	};
	tImage image_port3 = {
		.BPP = IMAGE_FMT_1BPP_UNCOMP,
		.NumColors = 2,
		.XSize = 1,
		.YSize = 8,
		.pPalette = imgPalette,
		.pPixel = portaat_3
	};
	tImage image_port4 = {
		.BPP = IMAGE_FMT_1BPP_UNCOMP,
		.NumColors = 2,
		.XSize = 1,
		.YSize = 8,
		.pPalette = imgPalette,
		.pPixel = portaat_4
	};
	tImage image_port5 = {
		.BPP = IMAGE_FMT_1BPP_UNCOMP,
		.NumColors = 2,
		.XSize = 1,
		.YSize = 8,
		.pPalette = imgPalette,
		.pPixel = portaat_5
	};


	if (hDisplay) {

		// Grafiikkaa varten tarvitsemme lis‰‰ RTOS:n muuttujia
		tContext *pContext = DisplayExt_getGrlibContext(hDisplay);

		if (pContext) {

			GrImageDraw(pContext, &image_port1, 32, 48);
			GrImageDraw(pContext, &image_port2, 40, 48);
			GrImageDraw(pContext, &image_port3, 40, 40);
			GrImageDraw(pContext, &image_port4, 48, 40);
			GrImageDraw(pContext, &image_port5, 48, 32);
			GrFlush(pContext);

		} // if
	} // if

}

//Funktio hissin piirtoon
Void drawHissi(){

	// Mustavalkoinen kuva: v‰rit musta ja valkoinen
	uint32_t imgPalette[] = {0, 0xFFFFFF};

	// Kuvan m‰‰rittelyt
	tImage image_his1 = {
		.BPP = IMAGE_FMT_1BPP_UNCOMP,
		.NumColors = 2,
		.XSize = 1,
		.YSize = 32,
		.pPalette = imgPalette,
		.pPixel = hissi_1
	};
	tImage image_his2 = {
		.BPP = IMAGE_FMT_1BPP_UNCOMP,
		.NumColors = 2,
		.XSize = 1,
		.YSize = 32,
		.pPalette = imgPalette,
		.pPixel = hissi_2
	};
	tImage image_his3 = {
		.BPP = IMAGE_FMT_1BPP_UNCOMP,
		.NumColors = 2,
		.XSize = 1,
		.YSize = 32,
		.pPalette = imgPalette,
		.pPixel = hissi_3
	};


	if (hDisplay) {

		// Grafiikkaa varten tarvitsemme lis‰‰ RTOS:n muuttujia
		tContext *pContext = DisplayExt_getGrlibContext(hDisplay);

		if (pContext) {

			GrImageDraw(pContext, &image_his1, 32, 32);
			GrImageDraw(pContext, &image_his2, 40, 32);
			GrImageDraw(pContext, &image_his3, 48, 32);
			GrFlush(pContext);

		} // if
	} // if

}

Void draw_otitsuorakaide(uint8_t x, uint8_t y){

	// suorakulmion maarittely
	tRectangle select = {
		.sXMin = 30,
		.sYMin = y,
		.sXMax = 66,
		.sYMax = y + 3,
	};

	if (hDisplay) {
		// Grafiikkaa varten tarvitsemme lis√§√§ RTOS:n muuttujia
		tContext *pContext = DisplayExt_getGrlibContext(hDisplay);
		GrContextForegroundSet(pContext, 0xFFFFFF);

		if (pContext) {

			// Piirretaan suorakulmio
			GrRectFill(pContext, &select);
		    GrFlush(pContext);

		} // if
	} // if

}


//Funktio OTiTin logon pystyviivam piirtoon

Void draw_otitpystyviiva(uint8_t x, uint8_t y){

	// pystyviivan maarittely
	tRectangle select = {
		.sXMin = 47,
		.sYMin = 20,
		.sXMax = 49,
		.sYMax = 77,
	};

	if (hDisplay) {
		// Grafiikkaa varten tarvitsemme lis√§√§ RTOS:n muuttujia
		tContext *pContext = DisplayExt_getGrlibContext(hDisplay);
		GrContextForegroundSet(pContext, 0xFFFFFF);

		if (pContext) {

			// Piirretaan suorakulmio
			GrRectFill(pContext, &select);
		    GrFlush(pContext);

		} // if
	} // if

}

//funktio OTiTin logon piirtoon
Void drawotitvasen(){


	// Mustavalkoinen kuva: v√§rit musta ja valkoinen
	uint32_t imgPalette[] = {0, 0xFFFFFF};

	// Kuvan m√§√§rittelyt
	tImage image_vas1 = {
		.BPP = IMAGE_FMT_1BPP_UNCOMP,
		.NumColors = 2,
		.XSize = 1,
		.YSize = 5,
		.pPalette = imgPalette,
		.pPixel = otit_vas1
	};

	tImage image_vas2 = {
		.BPP = IMAGE_FMT_1BPP_UNCOMP,
		.NumColors = 2,
		.XSize = 1,
		.YSize = 18,
		.pPalette = imgPalette,
		.pPixel = otit_vas2
	};

	if (hDisplay) {

		// Grafiikkaa varten tarvitsemme lis√§√§ RTOS:n muuttujia
		tContext *pContext = DisplayExt_getGrlibContext(hDisplay);

		if (pContext) {

			GrImageDraw(pContext, &image_vas1, 32, 24);
			GrImageDraw(pContext, &image_vas2, 40, 24);
			GrFlush(pContext);

		} // if
	} // if

}

Void drawotitoikea(){

	// Mustavalkoinen kuva: v√§rit musta ja valkoinen
	uint32_t imgPalette[] = {0, 0xFFFFFF};

	// Kuvan m√§√§rittelyt
	tImage image_oik1 = {
		.BPP = IMAGE_FMT_1BPP_UNCOMP,
		.NumColors = 2,
		.XSize = 1,
		.YSize = 18,
		.pPalette = imgPalette,
		.pPixel = otit_oik1
	};

	tImage image_oik2 = {
		.BPP = IMAGE_FMT_1BPP_UNCOMP,
		.NumColors = 2,
		.XSize = 1,
		.YSize = 6,
		.pPalette = imgPalette,
		.pPixel = otit_oik2
	};

	if (hDisplay) {

		// Grafiikkaa varten tarvitsemme lis√§√§ RTOS:n muuttujia
		tContext *pContext = DisplayExt_getGrlibContext(hDisplay);

		if (pContext) {

			GrImageDraw(pContext, &image_oik1, 54, 53);
			GrImageDraw(pContext, &image_oik2, 62, 53);
			GrFlush(pContext);

		} // if
	} // if

}

/**********************************************
* 			GRAFIIKKA
* 			LOPPUU
*
***********************************************/


//Listaan otetaan 10 viimeisint‰ liikumistapaa
char historia[10];

//Liikkumistapa 1 = ei liikuta, 2 = ollaan portaissa, 3 = hississ‰
uint8_t liikkumistapa = 1;

//N‰ytˆn tila, 1 = menu, 2 = liikutaan 3  = historia 4 = made by
uint8_t nayttotila = 1;

// Napin painalluksen tila
uint8_t button_press = 0;

//Liikeanturille jotai
static const I2CCC26XX_I2CPinCfg i2cMPUCfg = {
    .pinSDA = Board_I2C0_SDA1,
    .pinSCL = Board_I2C0_SCL1
};


#define STACKSIZE 2048

Char sensorTaskStack[STACKSIZE];
Char menuTaskStack[STACKSIZE];
Char commTaskStack[STACKSIZE];


// Konfiguraatio liikeanturille
static PIN_Handle hMpuPin;
//static PIN_State MpuPinState;
static PIN_Config MpuPinConfig[] = {
    Board_MPU_POWER  | PIN_GPIO_OUTPUT_EN | PIN_GPIO_HIGH | PIN_PUSHPULL | PIN_DRVSTR_MAX,
    PIN_TERMINATE
};

//Laitetaan button1 virtan‰pp‰imeksi
static PIN_Handle hPowerButton;
static PIN_State sPowerButton;

PIN_Config cPowerButton[] = {
    Board_BUTTON1 | PIN_INPUT_EN | PIN_PULLUP | PIN_IRQ_NEGEDGE,
    PIN_TERMINATE
};
PIN_Config cPowerWake[] = {
    Board_BUTTON1 | PIN_INPUT_EN | PIN_PULLUP | PINCC26XX_WAKEUP_NEGEDGE,
    PIN_TERMINATE
};

//Ledille configgi
static PIN_Handle hLed;
static PIN_State sLed;
PIN_Config cLed[] = {
    Board_LED0 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
    PIN_TERMINATE
};

//Button0 valintan‰pp‰imeksi
static PIN_Handle hButton0;
static PIN_State sButton0;
PIN_Config cButton0[] = {
    Board_BUTTON0 | PIN_INPUT_EN | PIN_PULLUP | PIN_IRQ_NEGEDGE,
    PIN_TERMINATE
};

//Handle virtapainikkeelle
Void powerButtonFxn(PIN_Handle handle, PIN_Id pinId) {

    Display_clear(hDisplay);
    Display_close(hDisplay);
    Task_sleep(100000 / Clock_tickPeriod);

	PIN_close(hPowerButton);

    PINCC26XX_setWakeup(cPowerWake);
	Power_shutdown(NULL,0);
}


//T‰ll‰ funktiolla lis‰t‰‰n historia listaan viimeisimm‰t kulkutavat (paitsi ne jos on oltu paikallaan), funktio s‰ilytt‰‰ 10 viimeisint‰
Void lisaahistoriaan(int tapa){
	//jos pituus on 10 pit‰‰ poistaa vanhin tieto ja siirret‰‰n muut
	//lasketaan pituus
	int pituus = 0;
	int i;
	for(i=0;i<10;i++){
		if(historia[i] != 0){
			pituus++;
		}
	}
	if(pituus == 10){
		int i;
		for(i=1;i<9;i++){
			historia[i] = historia[i-1];
		}
		//sitten laitetaan uusin tieto viimeisimm‰ksi
		historia[9] = tapa;
	}
	else{
		historia[pituus] = tapa;

	}
}

//maximin ja minimin laskemiselle funktio
Void maxmin(float maxmindata[]){
	char abc[32];
	sprintf(abc, "%f", maxmindata[0] );
	int x;
	float vertailumax = maxmindata[0];
	float vertailumin = maxmindata[0];

	for(x=0; x<5; x++){
		if(maxmindata[x] >= vertailumax){
			max = maxmindata[x];
			vertailumax = maxmindata[x];
		}
		if(maxmindata[x] <= vertailumin){
			min = maxmindata[x];
			vertailumin = maxmindata[x];
		}

	} //for

} //Void

//Keskihajonnan laskemiselle funktio
float keskihajonta(float khdata[]){
	int x;
		float summa = 0;
		float keskiarvo = 0;
		float keskihajonta = 0;
		for(x=0; x < 5; x++){
				summa = summa + khdata[x];
			}

		keskiarvo = summa/5;
		for(x=0; x < 5; x++){
			keskihajonta = keskihajonta +  pow(khdata[x] - keskiarvo, 2);
		}
		keskihajonta = sqrt(keskihajonta/5);
		return keskihajonta;

}
//Keskiarvon laskemiselle funktio
 float laskekeskiarvo(float data[]){
	 int i;
	 float summa = 0;
	 for(i=0;i<5;i++){
		 summa = summa + data[i];
	 }
	 return summa/5;

}
//Funktio, jolla verrataan sensorTagista tulevaa dataa portaissa, paikallaan ja hississ‰ otettuun dataan
Void tunnistusalgoritmi(float max, float min, float axkeskiarvo, float azkh, float paineenvaihtelu){
	if(paineenvaihtelu < 10){
		//Ei olla liikuttu tarpeeksi ylˆs tai alas joten laitetaan liikkumistapa 1:ksi ja poistutaan funktiosta
		liikkumistapa = 1;
		return;

	}
	float azvertailu = 0.1;
		 //Jos az keskihajonta on pienempi kuin 0.1 voidaan olettaa, ett‰ k‰ytt‰j‰ ei ole portaissa
		if(azkh < azvertailu){
			//Sensorin Az:n keskihajonta on l‰hemp‰n‰ hissin az-keskihajontaa
			if(fabs(hissikh-azkh) < fabs(paikallaankh-azkh)){
					//Lis‰t‰‰n historiaan ett‰ on oltu hississ‰
					lisaahistoriaan(3);
					//Vaihdetaan liikkumistapa hissiin
					liikkumistapa = 3;
					return;
				}
			//Sensorin az:n keskihajonta on l‰hemp‰n‰ paikallaanolemisen az-keskihajontaa
			else if(fabs(paikallaankh-azkh) < fabs(hissikh-azkh)){
				//Lis‰t‰‰n historiaan ett‰ on om
				//Vaihdetaan liikkumistapa ett‰ ollaan paikallaan, eli ei olla hississ‰ tai portaissa
				liikkumistapa = 1;
				return;
			}
			else{
					//System_printf("liikkumistapaa ei tunnistettu ja azkh pienempi kuin 0.1" );
					liikkumistapa = 1;
					return;
				}
		}
		else{
			//Sensorin az:n keskihajonta on l‰hemp‰n‰ portaiden az-keskihajontaa
			if(fabs(portaatkh-azkh)< fabs(hissikh-azkh) && fabs(portaatkh-azkh)<fabs(paikallaankh-azkh)){
					//Lis‰t‰‰n historiaan ett‰ on oltu portaissa
					lisaahistoriaan(2);
					//Vaihdetaan liikkumistapa portaisiin
					liikkumistapa = 2;
					return;
				}
			else{
				//liikkumistapaa ei tunnistettu ja az keskihajonta on suurempi kuin 0.1
				liikkumistapa = 1;
				return;

			}

		}
}
//Sensoreille taski
Void sensorTask(UArg arg0, UArg arg1){

		// *******************************
	    //
	    // USE TWO DIFFERENT I2C INTERFACES
	    //
	    // *******************************
		I2C_Handle i2c; // INTERFACE FOR OTHER SENSORS
		I2C_Params i2cParams;
		I2C_Handle i2cMPU; // INTERFACE FOR MPU9250 SENSOR
		I2C_Params i2cMPUParams;

		char str[80];

	    I2C_Params_init(&i2cParams);
	    i2cParams.bitRate = I2C_400kHz;

	    I2C_Params_init(&i2cMPUParams);
	    i2cMPUParams.bitRate = I2C_400kHz;
	    i2cMPUParams.custom = (uintptr_t)&i2cMPUCfg;

	    // *******************************
	    //
	    // MPU OPEN I2C
	    //
	    // *******************************
	    i2cMPU = I2C_open(Board_I2C, &i2cMPUParams);
	    if (i2cMPU == NULL) {
	        System_abort("Error Initializing I2CMPU\n");
	    }

	    // *******************************
	    //
	    // MPU POWER ON
	    //
	    // *******************************fR, Board_MPU_POWER_ON);

	    // WAIT 100MS FOR THE SENSOR TO POWER UP
		Task_sleep(100000 / Clock_tickPeriod);
	    System_printf("MPU9250: Power ON\n");
	    System_flush();

	    // *******************************
	    //
	    // MPU9250 SETUP AND CALIBRATION
	    //
	    // *******************************
		System_printf("MPU9250: Setup and calibration...\n");
		System_flush();

		mpu9250_setup(&i2cMPU);

		System_printf("MPU9250: Setup and calibration OK\n");
		System_flush();

	    // *******************************
	    //
	    // MPU CLOSE I2C
	    //
	    // *******************************
	    I2C_close(i2cMPU);

	    // *******************************
	    //
	    // OTHER SENSOR OPEN I2C
	    //
	    // *******************************
	    i2c = I2C_open(Board_I2C, &i2cParams);
	    if (i2c == NULL) {
	        System_abort("Error Initializing I2C\n");
	    }

	    // BMP280 SETUP
	    bmp280_setup(&i2c);

	    // *******************************
	    //
	    // OTHER SENSOR CLOSE I2C
	    //
	    // *******************************
	    I2C_close(i2c);

	    // LOOP FOREVER
		while (1) {

		    // *******************************
		    //
		    // OTHER SENSORS OPEN I2C
		    //
		    // *******************************
		    i2c = I2C_open(Board_I2C, &i2cParams);
		    if (i2c == NULL) {
		        System_abort("Error Initializing I2C\n");
		    }

		    // *******************************
		    //
		    // BMP280 ASK DATA
		    //
		    // *******************************/

		    bmp280_get_data(&i2c, &pres, &temp);
		    pressure[0] = pres;
	    	System_flush();

		    // *******************************
		    //
		    // OTHER SENSORS CLOSE I2C
		    //
		    // *******************************
		    I2C_close(i2c);

		    // *******************************
		    //
		    // MPU OPEN I2C
		    //
		    // *******************************
		    i2cMPU = I2C_open(Board_I2C, &i2cMPUParams);
		    if (i2cMPU == NULL) {
		        System_abort("Error Initializing I2CMPU\n");
		    }

		    // *******************************
		    //
		    // MPU ASK DATA
			//
	        //    Accelerometer values: ax,ay,az
		 	//    Gyroscope values: gx,gy,gz
			//
		    // *******************************
		    int i;
		    	for(i=0;i<5;i++){
		    		mpu9250_get_data(&i2cMPU, &ax, &ay, &az, &gx, &gy, &gz);
		    		dataedellisetaz[i] = az;
		    		dataedellisetay[i] = ay;
		    		dataedellisetax[i] = ax;
		    		Task_sleep(400000 / Clock_tickPeriod);

		    }

			System_flush();

		    // *******************************
		    //
		    // MPU CLOSE I2C
		    //
		    // *******************************
		    I2C_close(i2cMPU);

		    // WAIT 100MS


		    // *******************************
		    //
		    // OTHER SENSORS OPEN I2C
		    //
		    // *******************************
		    i2c = I2C_open(Board_I2C, &i2cParams);
		    if (i2c == NULL) {
		        System_abort("Error Initializing I2C\n");
		    }

		    // *******************************
		    //
		    // BMP280 ASK DATA
		    //
		    // *******************************/

		    bmp280_get_data(&i2c, &pres, &temp);
		    pressure[1] = pres;
	    	System_flush();

		    // *******************************
		    //
		    // OTHER SENSORS CLOSE I2C
		    //
		    // *******************************
		    I2C_close(i2c);

			maxmin(dataedellisetaz);
			float axkeskiarvo = laskekeskiarvo(dataedellisetax);
			float azkh = keskihajonta(dataedellisetaz);

			//menn‰‰n alasp‰in tai ylˆsp‰in
			if(pressure[0] < pressure[1]){
				//Alasp‰in
				liikkumissuunta = 2;

			}else{
				//ylˆsp‰in
				liikkumissuunta = 1;

			}
			//Kutsutaan vaan t‰t‰ funktiota jos korkeus on vaihtunut
			float painevaihtelu = fabs(pressure[1]-pressure[0]);
			tunnistusalgoritmi(max,min, axkeskiarvo,azkh, painevaihtelu);


			Task_sleep(1000000 / Clock_tickPeriod);

		}

		// MPU9250 POWER OFF
		//     Because of loop forever, code never goes here
	    PIN_setOutputValue(hMpuPin,Board_MPU_POWER, Board_MPU_POWER_OFF);

}




Void buttonFxn(){
	button_press = 1;
}

// Taskifunktio, sis‰lt‰‰ tilakoneen
Void menuTask(UArg arg0, UArg arg1) {

	uint8_t position = 1,
	r = 1;

   // Alustetaan n‰yttˆ nyt taskissa
	Display_Params displayParams;
	displayParams.lineClearMode = DISPLAY_CLEAR_BOTH;
	Display_Params_init(&displayParams);

   // N‰yttˆ k‰yttˆˆn ohjelmassa
    hDisplay = Display_open(Display_Type_LCD, &displayParams);
        if (hDisplay == NULL) {
            System_abort("Error initializing Display\n");
        }
		
				Display_clear(hDisplay);
				for(r = 1; r < 70; r++ ){
				drawYmpyra(r);
				Task_sleep(60000 / Clock_tickPeriod);
				}

				Display_clear(hDisplay);
				

   while(1){
			char payload[16];
		   switch(nayttotila){
		   	   //ollaan menussa eli n‰ytet‰‰n valintakohdat "mittaus" ja "historiatiedot", sek‰ "made by"
		   	   case 1:

		   		Display_clear(hDisplay);

		   		Display_print0(hDisplay, 3, 3, "Aloita");
				Display_print0(hDisplay, 6, 3, "Historia");
				Display_print0(hDisplay, 9, 3, "Made by");

				drawSelect(15, 15);

				while (1) {

					// valinta Aloita
					if(ay < -0.2 & position == 2){
						drawSelect(15, 15);
						deleteSelect(15, 40);
						deleteSelect(15, 64);
						position = 1;

						Task_sleep(350000 / Clock_tickPeriod);
					}

					// valinta Historia
					if((ay > 0.2 & position == 1) || (ay < -0.2 & position == 3)){
						drawSelect(15, 40);
						deleteSelect(15, 15);
						deleteSelect(15, 64);
						position = 2;


						Task_sleep(350000 / Clock_tickPeriod);
					}

					// valinta Made by
					if(ay > 0.2 & position == 2){
						drawSelect(15, 64);
						deleteSelect(15, 40);
						deleteSelect(15, 15);
						position = 3;

						Task_sleep(350000 / Clock_tickPeriod);
					}

					// katsotaan onko valintanappia painettu
					if(button_press != 0){
						switch(position){
							// Aloita
							case 1:
								nayttotila = 2;
								System_flush();

								break;

							// Historia
							case 2:
								nayttotila = 3;
								System_flush();

								break;
							//Made by
							case 3:
								nayttotila = 4;
								System_flush();

								break;

							default:
							break;
					}

					// Suljetaan menu jos mittaus tai historia tai made by valittu
						if(button_press == 1 && position == 1 || button_press == 1 && position == 2 || button_press == 1 && position == 3){
							System_printf("Suljetaan menu");
							System_flush();
							button_press = 0;
							break;
						} // if

					} // if

					Task_sleep(100000 / Clock_tickPeriod);

				} // while
		break;


   //liikutaan eli n‰ytet‰‰n joko portaat tai hissi. Ollaan valittu datan ker‰‰minen
	   case 2:
		   Display_clear(hDisplay);
		   Display_print0(hDisplay, 5, 3, "Tervetuloa mittaamaan dataa!");
		   while(1){
			   char nayttodata[16];
			   switch(liikkumistapa){

			   //Ei liikuta yl‰/alasuunnassa
					case 1:
						   sprintf(nayttodata,"az: %f", az);
						   Display_clear(hDisplay);
						   Display_print0(hDisplay, 1, 1, "Odotetaan");
						   Display_print0(hDisplay, 2, 1, "Tunnistusta...");
						   Display_print0(hDisplay, 4, 1, nayttodata) ;
						   Display_print0(hDisplay, 5, 1, "Ilmanpaine") ;
						   sprintf(nayttodata,"%f", pres);
						   Display_print0(hDisplay, 6, 1, nayttodata);
						   sprintf(nayttodata,"%f", temp);
						   Display_print0(hDisplay, 8, 1, "lampotila noin:");
						   Display_print0(hDisplay, 9, 1,  nayttodata);


						   Task_sleep(1000000/Clock_tickPeriod);
						   break;

			   //Portaat
					case 2:
						  //Vilkutetaan ledi‰ kannustukseksi
						  PIN_setOutputValue( hLed, Board_LED0, !PIN_getOutputValue( Board_LED0 ) );

						  //L‰hetet‰‰n l‰hell‰ oleville laitteille ett‰ ollaan portaissa

							sprintf(payload, "Portaissa! %x", 0x0155);
							Send6LoWPAN(0xFFFF, payload, strlen(payload));

							Display_clear(hDisplay);
						   drawPortaat();
						   //Kerrotaan menn‰‰nkˆ ylˆs vai alas
						   if(liikkumissuunta == 1){

							   Display_print0(hDisplay, 1, 1, " Going up!");
							   drawNuoliYlos();
						   }
						   else if(liikkumissuunta == 2){

							   Display_print0(hDisplay, 1, 1, " Going down!");
							   drawNuoliAlas();
						   }
						   //Display_print0(hDisplay, 1, 1, "Jeejee portaat!");
						   Task_sleep(5* 1000000/Clock_tickPeriod);
						   //Sammutetaan ledi
						   PIN_setOutputValue( hLed, Board_LED0, !PIN_getOutputValue( Board_LED0 ) );
						   break;
				//Hissi
					case 3:
						 //L‰hetet‰‰n l‰helle oleville laitteille ett‰ olaan hississ‰
						sprintf(payload, "Hissi! %x", 0x0155);
						Send6LoWPAN(0xFFFF, payload, strlen(payload));

						   Display_clear(hDisplay);
						   Display_print0(hDisplay, 1, 1, "Hyi hyi hissi!");
						   drawHissi();
						   //Kerrotaan menn‰‰nkˆ ylˆs vai alas
						   if(liikkumissuunta == 1){
							   Display_print0(hDisplay, 1, 1, " Going up!");
							   drawNuoliYlos();
							}
						   else if(liikkumissuunta == 2){
							   Display_print0(hDisplay, 1, 1, " Going down!");
							   drawNuoliAlas();
						   }
						   Task_sleep(5*1000000/Clock_tickPeriod);
						   break;

					default: break;
			   } //switch
			   
			   // Katsotaan onko nappia painettu, jos on siirrytaan takaisin menuun
			   if(button_press == 1){
				   nayttotila = 1;
				   button_press = 0;
					uint8_t r = 70;

					Display_clear(hDisplay);

					drawYmpyra(70);

					for(r = 70; r > 1; r-- ){
						Task_sleep(60000 / Clock_tickPeriod);
						drawYmpyraM(r);
					}
				   Task_sleep(5* 1000000/Clock_tickPeriod);
				   break; // suljetaan while
			   } // if


		   } //while
		   break;

	   //Katsellaan historiatietoja:
	   case 3:
		   Display_clear(hDisplay);
		   Display_print0(hDisplay, 1, 1, "HISTORIA");

		   int i;
		   int tapa;
		   for(i  = 0;i < 10; i++){

			  System_printf("ollan loopissa %d", i);
			   tapa = historia[i];
			   if(tapa == 2){
				   //Kirjotetaan n‰ytˆlle: portaat
				   Display_print0(hDisplay, i+3, 1, "Portaat!");
			   }
			   if(tapa == 3){
				   //Kirjoitetaan n‰ytˆlle: hissi
				   Display_print0(hDisplay, i+3, 1, "Hissi!");
			   }

		   }

			 while(1){
			   // Katsotaan onko nappia painettu, jos on siirrytaan takaisin menuun
			   if(button_press == 1){
				   nayttotila = 1;
				   button_press = 0;
					uint8_t r = 70;

					Display_clear(hDisplay);

					drawYmpyra(70);

					for(r = 70; r > 1; r-- ){
						Task_sleep(60000 / Clock_tickPeriod);
						drawYmpyraM(r);
					}
					break;
			   }

			   } //while
			break;


			//Made by
			case 4:

		   //piirret‰‰n OTiTin logo
			Display_clear(hDisplay);
			Display_print0(hDisplay, 1, 1, "Jenna Onnela");
			Display_print0(hDisplay, 10, 1, "Riina Annunen");
			draw_otitsuorakaide(30,20);
			drawotitoikea();
			draw_otitsuorakaide(30, 49);
			draw_otitpystyviiva(47,20);
			drawotitvasen();

			 while(1){
			   // Katsotaan onko nappia painettu, jos on siirrytaan takaisin menuun
			   if(button_press == 1){
				   nayttotila = 1;
				   button_press = 0;
					uint8_t r = 70;

					Display_clear(hDisplay);

					drawYmpyra(70);

					for(r = 70; r > 1; r-- ){
						Task_sleep(60000 / Clock_tickPeriod);
						drawYmpyraM(r);
					}
					break;
			   }

			   } //while
			 //Task_sleep(1000000/Clock_tickPeriod);
			break;

	   	   default: break;


   }
}

}

//Viestien vastaanotto
Void commTask(UArg arg0, UArg arg1) {

	// Radio to receive mode
	uint16_t senderAddr;
	char payload[16];
	int32_t result = StartReceive6LoWPAN();
	if(result != true) {
		System_abort("Wireless receive mode failed");
	}

    while (1) {

        // DO __NOT__ PUT YOUR SEND MESSAGE FUNCTION CALL HERE!!
    	if (GetRXFlag()){
    		memset(payload,0,16);
    		Receive6LoWPAN(&senderAddr, payload,16);
    		Display_print0(hDisplay, 9, 2, payload);
    		//Annetaan viestin n‰ky‰ v‰h‰n aikaa ennenkuin poistetaan se n‰ytˆlt‰
    		Task_sleep(100000 / Clock_tickPeriod);
    		Display_clear(hDisplay);
    		System_flush();
    	}

    	// NOTE THAT COMMUNICATION WHILE LOOP DOES NOT NEED Task_sleep
    	// It has lower priority than main loop (set by course staff)

    }
}


int main(void){
	//Sensori
	Task_Params sParams;
	Task_Handle sHandle;
	//Menu
	Task_Params mParams;
	Task_Handle mHandle;
	//Kommunikointi muiden laitteiden kanssa
	Task_Params commTaskParams;
	Task_Handle hCommTask;

	//Laitteen alustus
		Board_initGeneral();
		Board_initI2C();

	//Ledit
	hLed = PIN_open(&sLed, cLed);
		if(!hLed) {
			System_abort("Error initializing LED pin\n");
		}

	Task_Params_init(&mParams);
	mParams.stackSize = STACKSIZE;
	mParams.stack = &menuTaskStack;
	mParams.priority = 3;


	mHandle = Task_create((Task_FuncPtr)menuTask, &mParams, NULL);
	//Jos menutaskin luominen ei onnistu
	if (mHandle == NULL) {
		      System_abort("Task create failed");
		   }
	Task_Params_init(&sParams);
	sParams.stackSize = STACKSIZE;
	sParams.stack = &sensorTaskStack;
	sParams.priority=2;
	//otetaan az datat portaista, hissist‰ ja paikoillaan olemisesta

	sHandle = Task_create((Task_FuncPtr)sensorTask, &sParams, NULL);
	//Jos sensoriTaskin luominen ei onnistu
	   if (sHandle == NULL) {
	      System_abort("Task create failed");
	   }

	//Kerrotaan ohjelmoijalle, etta alustus onnistui
	System_printf("Hello world! \n");
	 hPowerButton = PIN_open(&sPowerButton, cPowerButton);
	 	if(!hPowerButton) {
	 		System_abort("Error initializing power button shut pins\n");
	 	}

	 	if (PIN_registerIntCb(hPowerButton, &powerButtonFxn) != 0) {
	 		System_abort("Error registering power button callback function");
	 	}

	     // JTKJ: INITIALIZE BUTTON0 HERE
	     hButton0 = PIN_open(&sButton0, cButton0);
	     if (PIN_registerIntCb( hButton0, &buttonFxn ) != 0 ) {
	         System_abort("Error registering button callback function");
	     }

	 /* JTKJ: Init Communication Task */
		 Init6LoWPAN();

		 Task_Params_init(&commTaskParams);
		 commTaskParams.stackSize = STACKSIZE;
		 commTaskParams.stack = &commTaskStack;
		 commTaskParams.priority=1;

		 hCommTask = Task_create(commTask, &commTaskParams, NULL);
		 if (hCommTask == NULL) {
			System_abort("Task create failed!");
		 }

	BIOS_start();

	return(0);
}
