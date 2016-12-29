/*
******************************************************************************
File:     main.c
Info:     Generated by Atollic TrueSTUDIO(R) 7.0.0   2016-12-19

The MIT License (MIT)
Copyright (c) 2009-2016 Atollic AB

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
******************************************************************************
*/

/* Includes */
#include "main.h"

/* Private macro */
/* Private variables */
/* Private function prototypes */
/* Private functions */

/**
**===========================================================================
**
**  Abstract: main program
**
**===========================================================================
*/
int main(void)
{
//#define MC		// defined = build MC / commented = build Antrieb
#ifdef MC		// MC

	uint32_t wert=0;
	uint8_t state=0;
	CAN1_TX_FRAME_t myTXFrame; // Puffer f�r TX-Daten
	CAN1_RX_FRAME_t myRXFrame; // Puffer f�r RX-Daten

	//SystemInit(); // Quarz Einstellungen aktivieren

	// Init der LEDs
	UB_Led_Init();

	// Init vom CAN-1 (an PB8+PB9)
	UB_CAN1_Init();

	while(1)
	{
		// state machine
		switch(state) {
			// CMD states
			case 0:
				myTXFrame.can_id = 0x100;	// 0x100 COBID for CMDs
				myTXFrame.anz_bytes = 1;	// length
				myTXFrame.data[0] = 0x1;	// 0x1 = vorw�rts fahren
				state++;
				break;
			case 5:
				myTXFrame.can_id = 0x100;	// 0x100 COBID for CMDs
				myTXFrame.anz_bytes = 1;	// length
				myTXFrame.data[0] = 0x2;	// 0x2 = im uhrzeigersin drehen
				state++;
				break;
			// Value to CMD states
			case 2:
				myTXFrame.can_id = 0x110;		// 0x110 COBID for Values of CMDs
				myTXFrame.anz_bytes = 4;		// length
				wert = 10000;					// Distanz in mm
				myTXFrame.data[0] = wert;		// first byte
				myTXFrame.data[1] = wert >> 8;	// second byte
				myTXFrame.data[2] = wert >> 16;	// third byte
				myTXFrame.data[3] = wert >> 24;	// fourth byte
				state++;
				break;
			case 7:
				myTXFrame.can_id = 0x110;		// 0x110 COBID for Values of CMDs
				myTXFrame.anz_bytes = 4;		// length
				wert = 1800;					// Grad plus eine kommastelle
				myTXFrame.data[0] = wert;		// first byte
				myTXFrame.data[1] = wert >> 8;	// second byte
				myTXFrame.data[2] = wert >> 16;	// third byte
				myTXFrame.data[3] = wert >> 24;	// fourth byte
				state++;
				break;
			// TX states
			case 1:
			case 3:
			case 6:
			case 8:
				UB_CAN1_send_std_data(myTXFrame);
				state++;
				break;
			default:
				break;
		}
		// receive
		if(UB_CAN1_receive(&myRXFrame)==CAN1_RX_READY) {
			// Received Monitor
			if(myRXFrame.can_id==0x300) {
				// Save Monitorvalue...
			}
			// Received CMD finished
			if(myRXFrame.can_id==0x310) {
				// change state
				switch(myRXFrame.data[0]){
					case 1:
						state=5;
						break;
					case 2:
						state=0;
						break;
				}
			}
		}
	}
#else		// Antrieb

	uint32_t delay=0;
	uint32_t wert=0;
	uint32_t soll=0;
	uint32_t ist=0;
	uint8_t state=0;
	uint8_t cmd=0;
	uint8_t running=0;
	CAN1_TX_FRAME_t myTXFrame; // Puffer f�r TX-Daten
	CAN1_RX_FRAME_t myRXFrame; // Puffer f�r RX-Daten

	//SystemInit(); // Quarz Einstellungen aktivieren

	// Init der LEDs
	UB_Led_Init();

	// Init vom CAN-1 (an PB8+PB9)
	UB_CAN1_Init();

	while(1)
	{
		// state machine
		if(running==2) {
			switch(cmd) {
				case 1:
					switch(state) {
						case 0:
							soll = wert;
							state++;
							break;
						case 1:
							if(ist<soll) {
								if(delay++%2500000==0){
									// send monitoring value
									myTXFrame.can_id = 0x300;		// 0x300 COBID for monitoring
									myTXFrame.anz_bytes = 4;		// length
									myTXFrame.data[0] = ist;		// first byte
									myTXFrame.data[1] = ist >> 8;	// second byte
									myTXFrame.data[2] = ist >> 16;	// third byte
									myTXFrame.data[3] = ist >> 24;	// fourth byte
									state++;
								}
								if(delay%7500==0) {
									ist++;
								}
							}
							else{
								// send cmd finished
								myTXFrame.can_id = 0x310;	// 0x310 COBID for CMDs finished
								myTXFrame.anz_bytes = 1;	// length
								myTXFrame.data[0] = cmd;	// cmd
								state++;
							}
							break;
						case 2:
							UB_CAN1_send_std_data(myTXFrame);
							if(ist<soll) {
								state=1;
								delay++;
							}
							else {
								state=0;
								cmd=0;
								running=0;
								delay=0;
							}
							break;
					}
					break;
				case 2:
					switch(state) {
						case 0:
							soll = wert;
							state++;
							break;
						case 1:
							if(ist<soll) {
								if(delay++%2500000==0){
									// send monitoring value
									myTXFrame.can_id = 0x300;		// 0x300 COBID for monitoring
									myTXFrame.anz_bytes = 4;		// length
									myTXFrame.data[0] = ist;		// first byte
									myTXFrame.data[1] = ist >> 8;	// second byte
									myTXFrame.data[2] = ist >> 16;	// third byte
									myTXFrame.data[3] = ist >> 24;	// fourth byte
									state++;
								}
								if(delay%5000==0) {
									ist++;
								}
							}
							else{
								// send cmd finished
								myTXFrame.can_id = 0x310;	// 0x310 COBID for CMDs finished
								myTXFrame.anz_bytes = 1;	// length
								myTXFrame.data[0] = cmd;	// cmd
								state++;
							}
							break;
						case 2:
							if(ist<soll) {
								state=1;
								delay++;
							}
							else {
								state=0;
								cmd=0;
								running=0;
								delay=0;
							}
							UB_CAN1_send_std_data(myTXFrame);
							break;
					}
					break;
			}
		}
		// receive
		if(UB_CAN1_receive(&myRXFrame)==CAN1_RX_READY) {
			// Received CMD
			if(myRXFrame.can_id==0x100) {
				if(running==0){
					running=1;
					state=0;
					delay=0;
					soll=0;
					ist=0;
					cmd = myRXFrame.data[0];
				}
			}
			// Received value to CMD
			if(myRXFrame.can_id==0x110) {
				if(running==1){
					running=2;
					state=0;
					delay=0;
					soll=0;
					ist=0;
					wert = myRXFrame.data[0] | myRXFrame.data[1] << 8 | myRXFrame.data[2] << 16 | myRXFrame.data[3] << 24;
				}
			}
		}
	}
#endif
}


/*
 * Callback used by stm32f4_discovery_audio_codec.c.
 * Refer to stm32f4_discovery_audio_codec.h for more info.
 */
void EVAL_AUDIO_TransferComplete_CallBack(uint32_t pBuffer, uint32_t Size){
  /* TODO, implement your code here */
  return;
}

/*
 * Callback used by stm324xg_eval_audio_codec.c.
 * Refer to stm324xg_eval_audio_codec.h for more info.
 */
uint16_t EVAL_AUDIO_GetSampleCallBack(void){
  /* TODO, implement your code here */
  return -1;
}
