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
	uint32_t delay=0;
	uint8_t wert=0;
	CAN1_TX_FRAME_t myTXFrame; // Puffer f�r TX-Daten
	CAN1_RX_FRAME_t myRXFrame; // Puffer f�r RX-Daten

	//SystemInit(); // Quarz Einstellungen aktivieren

	// Init der LEDs
	UB_Led_Init();

	// Init vom CAN-1 (an PB8+PB9)
	UB_CAN1_Init();

	while(1)
	{
		//---------------------------
		// senden
		//---------------------------
		delay++;
		if(delay>5000000) {
			// zyklisch ein CAN-Telegramm senden
			delay=0;
			UB_Led_Toggle(LED_GREEN);
			// telegramm erstellen
			myTXFrame.can_id=0x123; // ID ist egal
			myTXFrame.anz_bytes=1;  // ein Datenbyte wird gesendet
			myTXFrame.data[0]=wert; // Datenwert
			// Frame senden
			UB_CAN1_send_std_data(myTXFrame);
			// Datenwert wird nach jedem senden incrementiert
			wert++;
		}

		//---------------------------
		// empfangen
		//---------------------------
		if(UB_CAN1_receive(&myRXFrame)==CAN1_RX_READY) {
			// es wurde etwas empfangen
			UB_Led_Toggle(LED_BLUE);
			// Datenbyte auswerten (nur als Demo)
			if((myRXFrame.data[0]&0x02)!=0) {
				// beim empfangenen Datenwert ist das Bit1 gesetzt
				UB_Led_Toggle(LED_RED);
			}
		}
	}
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
