 // freedeck arduino code for flashing to atmega32u4 based arduinos
// and compatible Copyright (C) 2020 Kilian Gosewisch
//
// This program is free software: you can redistribute it and/or
// modify it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License,
// or (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
// See the GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public
// License along with this program. If not, see
// <https://www.gnu.org/licenses/>.

#include <HID-Project.h>

#include "./settings.h"
#include "./src/FreeDeck.h"
#include "./src/FreeDeckSerialAPI.h"
#include <Encoder.h>

//Initialise deej Faders
//--------------------------------------------------------
const int NUM_SLIDERS = 4;
const int analogInputs[NUM_SLIDERS] = {A0, A1, A2, A3};
int analogSliderValues[NUM_SLIDERS];
int StopFaderSerial = 0;
//--------------------------------------------------------

//Initialise Rotary Encoder
//--------------------------------------------------------
Encoder RotEnc(3, 2);
//int EncStepPerClick;
int EncSW = 4;
int PageCounter = 0;
long EncPos = -999;
long EncPosNew; 
int EncButtonState;
int StatusLED = 5;
//--------------------------------------------------------


void setup() {
//Setup Encoderswitch & Status-LED
//------------------------------------ 
pinMode(EncSW, INPUT_PULLUP);
pinMode(StatusLED, OUTPUT);

//Setup Freedeck
//------------------------------------ 
	Serial.begin(9600);
	Serial.setTimeout(100);
	delay(BOOT_DELAY);
	Keyboard.begin();
	Consumer.begin();
	pinMode(BUTTON_PIN, INPUT_PULLUP);
	pinMode(S0_PIN, OUTPUT);
#if BD_COUNT > 2
	pinMode(S1_PIN, OUTPUT);
#endif
#if BD_COUNT > 4
	pinMode(S2_PIN, OUTPUT);
#endif
#if BD_COUNT > 8
	pinMode(S3_PIN, OUTPUT);
#endif
	initAllDisplays();
	delay(100);
	initSdCard();
	postSetup();
}

void handleSerial() {
	if (Serial.available() > 0) {
    StopFaderSerial = 1;
		unsigned long read = readSerialBinary();
		if (read == 0x3) {
			handleAPI();
		}
		while (Serial.available()) {
			Serial.read();
		}
	}
}

void loop() {
//Encoder loop
//----------------------------------------------------------------------
EncPosNew = RotEnc.read();
EncButtonState = digitalRead(EncSW);
if(EncPosNew > ((pageCount-1)*EncStepPerClick+1) || EncButtonState == 0){
   RotEnc.write(0);
 }
if(EncPosNew < 0){
  RotEnc.write(((pageCount-1)*EncStepPerClick+1));
}
if(EncPosNew != EncPos){
  //Serial.println(EncPosNew);
  PageCounter = EncPosNew / EncStepPerClick;
  //Serial.println(PageCounter);
  //Serial.println( );
  EncPos = EncPosNew;
  loadPage(PageCounter);
  
  }
//Freedeck loop
//-----------------------------------------------------------------------

  handleSerial();
	for (uint8_t buttonIndex = 0; buttonIndex < BD_COUNT; buttonIndex++) {
		checkButtonState(buttonIndex);
	}
	if (TIMEOUT_TIME > 0) checkTimeOut();

 
//deej loop
//-----------------------------------------------------------------------
  if (StopFaderSerial == 0){
     digitalWrite(StatusLED, LOW);
     updateSliderValues();
     sendSliderValues();
   }
  if (StopFaderSerial == 1){
     digitalWrite(StatusLED, HIGH);
  }
  if(EncButtonState == 0){
    StopFaderSerial = 0;
  }
}
//deej Fader Functions
//----------------------------------------------------------
void updateSliderValues() {
  for (int i = 0; i < NUM_SLIDERS; i++) {
     analogSliderValues[i] = analogRead(analogInputs[i]);
  }
}
void sendSliderValues() {
  String builtString = String("");

  for (int i = 0; i < NUM_SLIDERS; i++) {
    builtString += String((int)analogSliderValues[i]);

    if (i < NUM_SLIDERS - 1) {
      builtString += String("|");
    }
  }
  
  Serial.println(builtString);
}
