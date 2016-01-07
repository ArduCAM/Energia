
#include "pitches.h"
#include <Wire.h>
#include <tmp006.h>
//#include "OPT3001.h"
#define USE_USCI_B1 
#define USING_MSP430F5529_LAUNCHPAD
//#define USING_TIVA_C_LAUNCHPAD




// Core library for code-sense
#if defined(WIRING) // Wiring specific
#include "Wiring.h"
#elif defined(MAPLE_IDE) // Maple specific
#include "WProgram.h"
#elif defined(MPIDE) // chipKIT specific
#include "WProgram.h"
#elif defined(DIGISPARK) // Digispark specific
#include "Arduino.h"
#elif defined(ENERGIA) // LaunchPad MSP430, Stellaris and Tiva, Experimeter Board FR5739 specific
#include "Energia.h"
#elif defined(CORE_TEENSY) // Teensy specific
#include "WProgram.h"
#elif defined(ARDUINO) && (ARDUINO >= 100) // Arduino 1.0 and 1.5 specific
#include "Arduino.h"
#elif defined(ARDUINO) && (ARDUINO < 100) // Arduino 23 specific
#include "WProgram.h"
#else // error
#error Platform not defined
#endif

// Include application, user and local libraries
#include "SPI.h"

// Screen selection
#define HX8353E // HX8353E K35 HI32 W32 ILI9225B HY28A ST7735 PicasoSPE PicasoSGC

#include "Screen_HX8353E.h"
Screen_HX8353E myScreen;


// Define variables and constants
//uint32_t chrono;


///
/// @brief      protocolSquare
/// @details    measure time to draw a square with side=pixels
/// @param      pixels number of pixels of one side
///
void protocolSquare(uint16_t pixels)
{
    if ((pixels < myScreen.screenSizeX()) && (pixels < myScreen.screenSizeY()) && (pixels > 48)) {
        
        myScreen.setOrientation(0);
        myScreen.setPenSolid(false);
        
        uint16_t x100 = (myScreen.screenSizeX()-pixels)/2;
        uint16_t y100 = (myScreen.screenSizeY()-pixels)/2;
        
        myScreen.dRectangle(x100-1, y100-1, pixels+2, pixels+2, whiteColour);
        myScreen.setPenSolid(true);
        uint32_t chrono = millis();
        myScreen.dRectangle(x100, y100, pixels, pixels, grayColour);
        chrono = millis() - chrono;
        
        myScreen.setFontSize(myScreen.fontMax());
        myScreen.gText(x100 +2, y100 +2, "(" + i32toa(pixels) + ")");
        myScreen.gText(x100 +2, y100 +pixels -myScreen.fontSizeY() -2, i32toa(chrono) + " ms");
        
        Serial.print("Square(");
        Serial.print(pixels, DEC);
        Serial.print(")\t");
        Serial.println(chrono, DEC);
    }
}

///
/// @brief      protocolCopyPaste
/// @details    measure time to copy-paste a 64x64 area
/// @param      orientation default=1
///
void protocolCopyPaste(uint8_t orientation = 1)
{
    uint32_t chrono;
    myScreen.clear();
    myScreen.setOrientation(orientation);
    
    // Image
    chrono = millis();
    for (uint8_t i=0; i<64; i++) {
        for (uint8_t j=0; j<64; j++) {
            myScreen.point(i, j, myScreen.calculateColour(4*i, 4*j, 254-2*i-2*j));
        }
    }
    
    myScreen.setPenSolid(false);
    myScreen.rectangle(1, 1, 62, 62, blackColour);
    myScreen.line(0,   0, 63, 63, whiteColour);
    myScreen.line(32,  0, 63, 63, whiteColour);
    myScreen.line(0,  32, 63, 63, whiteColour);
    myScreen.dRectangle(0, 0, 64, 64, whiteColour);
    chrono = millis() - chrono;
    
    myScreen.setFontSize(0);
    myScreen.gText(0, 66, "0: Original");
    myScreen.setFontSize(1);
    myScreen.gText(0, 76, i32toa(chrono) + " ms");
    
    Serial.print("Original(=");
    Serial.print(orientation, DEC);
    Serial.print(")\t");
    Serial.println(chrono, DEC);
    
    // Method 1
    chrono = millis();
    for (uint16_t i=0; i<64; i++) {
        for (uint16_t j=0; j<64; j++) {
            myScreen.point(myScreen.screenSizeX()/2+i, j, myScreen.readPixel(i, j));
        }
    }
    chrono = millis() - chrono;
    myScreen.setFontSize(0);
    myScreen.gText(myScreen.screenSizeX()/2, 66, "1. point readPixel");
    myScreen.setFontSize(1);
    myScreen.gText(myScreen.screenSizeX()/2, 76, i32toa(chrono) + " ms");
    
    Serial.print("point(readPixel())\t");
    Serial.println(chrono, DEC);
    
    // Method 2
    chrono = millis();
    myScreen.copyPaste(0, 0, 0, myScreen.screenSizeY()/2, 64, 64);
    chrono = millis() - chrono;
    
    myScreen.setFontSize(0);
    myScreen.gText(0, myScreen.screenSizeY()/2 +66, "2. copyPaste");
    myScreen.setFontSize(1);
    myScreen.gText(0, myScreen.screenSizeY()/2 +76, i32toa(chrono) + " ms");
    
    Serial.print("copyPaste()\t");
    Serial.println(chrono, DEC);
    
    // Method 3
    if (myScreen.isStorage()) {
        uint32_t address;
        
        chrono = millis();
        address = 100;
        myScreen.copyArea(0, 0, 64, 64, address);
        address = 100;
        myScreen.pasteArea(myScreen.screenSizeX()/2, myScreen.screenSizeY()/2, 64, 64, address);
        chrono = millis() - chrono;
        
        myScreen.setFontSize(0);
        myScreen.gText(myScreen.screenSizeX()/2, myScreen.screenSizeY()/2 +66, "3. copy-paste SRAM");
        myScreen.setFontSize(1);
        myScreen.gText(myScreen.screenSizeX()/2, myScreen.screenSizeY()/2 +76, i32toa(chrono) + " ms");
        
        Serial.print("copyArea()+pasteArea() SRAM\t");
        Serial.println(chrono, DEC);
    }
}


///
/// @brief      protocolText
/// @details    measure time to draw text in 3 fonts, 4 orientations, 10x
///
void protocolText()
{
    uint32_t chrono1, chrono2;
    uint16_t colour;
    uint8_t k = 0;

    // Serial.print("fast gText... ");
    myScreen.clear(grayColour);
    myScreen.setFontSolid(true);
    chrono1 = millis();
    for (uint8_t j=0; j<10; j++) {
        for (uint8_t i=0; i<4; i++) {
            if (k==1) colour = redColour;
            else if (k==2) colour = yellowColour;
            else if (k==3) colour = greenColour;
            else if (k==4) colour = cyanColour;
            else if (k==5) colour = blueColour;
            else  colour = violetColour;
            k++;
            k %= 7;
            
            myScreen.setPenSolid(false);
            myScreen.dRectangle(0, 0, myScreen.screenSizeX(), myScreen.screenSizeY(), colour);
            
            myScreen.setOrientation(i);
            myScreen.setFontSize(0);
            myScreen.gText(4, 4, "font 0 on " + String(i), colour);
            myScreen.setFontSize(1);
            myScreen.gText(4, 14, "font 1 on " + String(i), colour);
            myScreen.setFontSize(2);
            myScreen.gText(4, 34, "font 2 on " + String(i), colour);
        }
    }
    chrono1 = millis()-chrono1;
    Serial.print("10xFontSolid(true)\t");
    Serial.println(chrono1, DEC);
    
    // Serial.print("slow gText... ");
    myScreen.clear(grayColour);
    myScreen.setFontSolid(false);
    chrono2 = millis();
    
    for (uint8_t j=0; j<10; j++) {
        for (uint8_t i=0; i<4; i++) {
            if (k==1) colour = redColour;
            else if (k==2) colour = yellowColour;
            else if (k==3) colour = greenColour;
            else if (k==4) colour = cyanColour;
            else if (k==5) colour = blueColour;
            else  colour = violetColour;
            k++;
            k %= 7;
            
            myScreen.setPenSolid(false);
            myScreen.dRectangle(0, 0, myScreen.screenSizeX(), myScreen.screenSizeY(), colour);
            
            myScreen.setOrientation(i);
            myScreen.setFontSize(0);
            myScreen.gText(4, 4, "font 0 on " + String(i), colour);
            myScreen.setFontSize(1);
            myScreen.gText(4, 14, "font 1 on " + String(i), colour);
            myScreen.setFontSize(2);
            myScreen.gText(4, 34, "font 2 on " + String(i), colour);
        }
    }
    
    chrono2 = millis()-chrono2;
    Serial.print("10xFontSolid(false)\t");
    Serial.println(chrono2, DEC);
    
    Serial.print("Ratio%\t");
    Serial.println((uint32_t)((uint64_t)(chrono1*100)/chrono2), DEC);
    
}



const int JOY_X = 2;
const int JOY_Y = 26;
const int SEL = 5;

//MIC
const int MIC = 6;

//ACCELEROMETER
const int ACC_X = 23;
const int ACC_Y = 24;
const int ACC_Z = 25;


//SWITCHES
const int SW1 = 33;
const int SW2 = 32;

//BUZZER
const int BUZZ = 40;



//RGB LED
const int RGB_RED = 39;
const int RGB_GRN = 38;
const int RGB_BLU = 37;

//TMP006
float tempReading = 0;

// notes in the melody:
#define NOTE_C4_1 260

// OPT3001
//opt3001 opFILE0  Ôý    p0 8  à                ô           `           H      Z²ê=®-Ñ € Ê­·Ð:«úÑZ²ê=®-Ñ                    Ê           ´p	    0   x          Z     ç    =Z²ê=®-ÑZ²ê=®-ÑZ²ê=®-ÑZ²ê=®-Ñ                        G A T O R H ~ 1 . I N O       0   €          h     ç    =Z²ê=®-ÑZ²ê=®-ÑZ²ê=®-ÑZ²ê=®-Ñ                        G a t o r H o l e B u z z e r . i n o €   H                         @                          1€€   ÿÿÿÿ‚yG                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                              FILE0  `9K    09 8  ø                õ           `           H      ½í=®-Ñ½í=®-Ñæ¸R®-Ñ½í=®-Ñ                    É          °!–	    0   p          R     Ñ    ½í=®-Ñ½í=®-Ñ½í=®-Ñ½í=®-Ñ                       J o y s t i c k       @   (                VpÂ¡™åœ¸ˆãäÜ¼   À                $ I 3 0 0                         ö   W2p Z     õ   09½í=®-Ñ € Ê­·Ð_œ­úÑ½í=®-Ñ       1
              J o y s t i c k . i n o                     ÿÿÿÿ‚yG                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                      FILE0      W2 8  `                ö           `           H      ½í=®-Ñ € Ê­·Ð_œ­úÑ½í=®-Ñ                    Ê          ¸¶p	    0   x          Z     õ   09½í=®-Ñ½í=®-Ñ½í=®-Ñ½í=®-Ñ                        J o y s t i c k . i n o       €   H                         @              1
      1
      1€   ÿÿÿÿ‚yG                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                              FILE0  )‰K    ay 8  ø                ÷           `           H      ½í=®-Ñ½í=®-Ñ®•èR®-Ñ½í=®-Ñ                    É          èK–	    0   p          R     Ñ    ½í=®-Ñ½í=®-Ñ½í=®-Ñ½í=®-Ñ                       J O Y S T I ~ 1       0   €          d     Ñ    ½í=®-Ñ½í=®-Ñ½í=®-Ñ½í=®-Ñ                       J o y s t i c k D r a w C o l o r     @   (                WpÂ¡™åœ¸ˆãäÜ¼   @              $ I 3 0 0                       ø   ) € l    ÷   ay½í=®-Ñ € Ê­·Ð:«úÑ½í=®-Ñ       T              J o y s t i c k D r a w C o l o r . i n o     ø   ) p Z     ÷   ay½í=®-Ñ € Ê­·Ð:«úÑ½í=®-Ñ       T              J O Y S T I ~ 1 . I N O                     ÿÿÿÿ‚yG                                                                                                                                                                                                                                                                       FILE0  3    )  8  è                ø           `           H      ½í=®-Ñ € Ê­·Ð:«úÑ½í=®-Ñ                    Ê          Ð¹p	    0   x          Z     ÷   ay½í=®-Ñ½í=®-Ñ½í=®-Ñ½í=®-Ñ                        J O Y S T I ~ 1 . I N O       0   ˆ          l     ÷   ay½í=®-Ñ½í=®-Ñ½í=®-Ñ½í=®-Ñ                        J o y s t i c k D r a w C o l o r . i n o     €   H                         @              T      T      1‚€   ÿÿÿÿ‚yG                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                      FILE0  ±¯K    
  8  è                ù  
        `           H      wï=®-Ñwï=®-ÑNES®-Ñwï=®-Ñ                    É          °`–	    0   p          R     Ñ    wï=®-Ñwï=®-Ñwï=®-Ñwï=®-Ñ                       J O Y S T I ~ 2       0   x          ^     Ñ    wï=®-Ñwï=®-Ñwï=®-Ñwï=®-Ñ                       J o y s t i c k S e r i a l   @   (                XpÂ¡™åœ¸ˆãäÜ¼   8             $ I 3 0 0                       ú    bx f     ù    wï=®-Ñ € Ê­·Ð:«úÑwï=®-Ñ       W              J o y s t i c k S e r i a l . i n o   ú    bp Z     ù   
 wï=®-Ñ € Ê­·Ð:«úÑwï=®-Ñ       W              J O Y S T I ~ 1 . I N O                     ÿÿÿÿ‚yG                                                                                                                                                                                                                                                                                       FILE0  š     b 8  à                ú           `           H      wï=®-Ñ € Ê­·Ð:«úÑwï=®-Ñ                    Ê          À¼p	    0   x          Z     ù   
 wï=®-Ñwï=®-Ñwï=®-Ñwï=®-Ñ                        J O Y S T I ~ 1 . I N O       0   €          f     ù   
 wï=®-Ñwï=®-Ñwï=®-Ñwï=®-Ñ                        J o y s t i c k S e r i a l . i n o   €   H                         @              W      W      1ƒ€   ÿÿÿÿ‚yG                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                              FILE0  íL    ri 8  è                û  ri       `           H      wï=®-Ñ€Ùñ=®-ÑUWJS®-Ñ€Ùñ=®-Ñ                    É          xŠ–	    0   p          R     Ñ    wï=®-Ñwï=®-Ñwï=®-Ñwï=®-Ñ                       K O N A M I ~ 1       0   x          Z     Ñ    wï=®-Ñwï=®-Ñwï=®-Ñwï=®-Ñ                       K o n a m i C o n t r a       @   (                YpÂ¡™åœ¸ˆãäÜ¼   8             $ I 3 0 0                       ü   **x b     û    €Ùñ=®-Ñ € Ê­·Ð_œ­úÑ€Ùñ=®-Ñ        ?              K o n a m i C o n t r a . i n o       ü   **p Z     û   ri€Ùñ=®-Ñ € Ê­·Ð_œ­úÑ€Ùñ=®-Ñ