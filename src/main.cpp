
extern "C"
{
#include <NeoPixelBus.h>
#include <util/delay.h>   // _delay_ms()
#include "etl_profile.h"  // this together with <ArduinoSTL> lib are necessary to use etl containers:
                          // https://www.etlcpp.com/setup.html
#include "vector.h"       // LEDSeqs
#include "avr/pgmspace.h" // PROGMEM  (program space) to store table inside flash ROM instead of SRAM
                          // https://www.nongnu.org/avr-libc/user-manual/pgmspace.html
}

typedef RowMajorAlternatingLayout panelLayout;

const uint8_t WIDTH = 16;
const uint8_t HEIGHT = 16;
const uint8_t PIXEL_COUNT = WIDTH * HEIGHT;
const uint8_t PIN_NUM = 2;

NeoTopology<panelLayout> panelTopology(WIDTH, HEIGHT);
NeoPixelBus<NeoGrbFeature, Neo800KbpsMethod> strip(PIXEL_COUNT, PIN_NUM);

struct PixelCoord
{
    uint8_t x = 0;
    uint8_t y = 0;

    PixelCoord(uint8_t t_a, uint8_t t_b) : x(t_a), y(t_b) {}
};

namespace LightSeqCordinates
{
const PixelCoord Seq88[] = {{1, 1}, {1, 2}, {1, 3}, {2, 3}, {3, 3}, {3, 2}, {3, 1}, {4, 1}, {5, 1}, {5, 2}, {5, 3}, {4, 3}, {2, 1}, {1, 4}, {1, 5}, {1, 6}, {2, 6}, {3, 6}, {3, 5}, {3, 4}, {4, 4}, {5, 4}, {5, 5}, {5, 6}, {4, 6}, {2, 4}};
const size_t size88 = sizeof(etl::array_size(Seq88));
const etl::vector<PixelCoord, size88> LEDSeq88(etl::begin(Seq88), etl::end(Seq88));
} // namespace LightSeqCordinates

void iterateSeq(const etl::ivector<PixelCoord> &t_LEDSeqs)
{
    etl::ivector<PixelCoord>::const_iterator itr;
    itr = t_LEDSeqs.begin();

    // grow:
    while (itr != t_LEDSeqs.end())
    {
        emitPixel(itr);
        _delay_ms(500);
    }

    // del:
}

// https://www.arduino.cc/en/Reference/PortManipulation
// DDR: determines I or O
// PORT: determines HIGH or LOW
// PIN: reads the state of INPUT pins set to input with pinMode(). (Read only)
int main(void)
{
    //-------------------
    //------ Setup ------
    //-------------------

    // handled by API
    // // make the LED pin an output for PORTB5 (pin 13)
    // DDRB = 1 << PORTB5;

    strip.Begin();
    strip.ClearTo(black);
    strip.Show();

    while (1)
    {
        iteratSeq(delays1);

        // toggle the LED
        PORTB ^= 1 << PORTB5;
    }

    return 0;
}

// -------------------------------------------------------------------------------------------------
// -------------------------------------------------------------------------------------------------
// // NeoPixelBufferCylon
// // This example will move a Cylon Red Eye back and forth across the
// // the full collection of pixels on the strip.
// //
// // This will demonstrate the use of the NeoVerticalSpriteSheet
// //

// #include <NeoPixelBus.h>
// #include <NeoPixelAnimator.h>

// // The actual image is contained in the data structure in one of the Cylon*.h files
// // You will need to use the one that has the same color feature as your NeoPixelBus
// // There are two provided, but you can create your own easily enough using
// // free versions of Paint.Net and the plugin
// #include "CylonGrb.h"
// typedef NeoGrbFeature MyPixelColorFeature;

// // #include "CylonGrbw.h"
// // typedef NeoGrbwFeature MyPixelColorFeature;

// const uint16_t PixelCount = 16; // the sample images are meant for 16 pixels
// const uint16_t PixelPin = 2;
// const uint16_t AnimCount = 1; // we only need one

// NeoPixelBus<MyPixelColorFeature, Neo800KbpsMethod> strip(PixelCount, PixelPin);
// // for esp8266 omit the pin
// //NeoPixelBus<MyPixelColorFeature, Neo800KbpsMethod> strip(PixelCount);
// NeoPixelAnimator animations(AnimCount); // NeoPixel animation management object

// // sprite sheet stored in progmem using the same pixel feature as the NeoPixelBus
// NeoVerticalSpriteSheet<NeoBufferProgmemMethod<MyPixelColorFeature>> spriteSheet(
//     myImageWidth,  // image width and sprite width since its vertical sprite sheet
//     myImageHeight, // image height
//     1,             // sprite is only one pixel high
//     myImage);

// uint16_t indexSprite;

// void LoopAnimUpdate(const AnimationParam &param)
// {
//     // wait for this animation to complete,
//     // we are using it as a timer of sorts
//     if (param.state == AnimationState_Completed)
//     {
//         // done, time to restart this position tracking animation/timer
//         animations.RestartAnimation(param.index);

//         // draw the next frame in the sprite
//         spriteSheet.Blt(strip, 0, indexSprite);
//         indexSprite = (indexSprite + 1) % myImageHeight; // increment and wrap
//     }
// }

// void setup()
// {
//     strip.Begin();
//     strip.Show();

//     indexSprite = 0;

//     // we use the index 0 animation to time how often we rotate all the pixels
//     animations.StartAnimation(0, 60, LoopAnimUpdate);
// }

// void loop()
// {
//     // this is all that is needed to keep it running
//     // and avoiding using delay() is always a good thing for
//     // any timing related routines
//     animations.UpdateAnimations();
//     strip.Show();
// }

// // -------------------------------------------------------------------------------------------------
// // NeoPixelTest
// // This example will cycle between showing four pixels as Red, Green, Blue, White
// // and then showing those pixels as Black.
// //
// // Included but commented out are examples of configuring a NeoPixelBus for
// // different color order including an extra white channel, different data speeds, and
// // for Esp8266 different methods to send the data.
// // NOTE: You will need to make sure to pick the one for your platform
// //
// //
// // There is serial output of the current state so you can confirm and follow along
// //

// #include <NeoPixelBus.h>

// const uint16_t PixelCount = 256; // this example assumes 4 pixels, making it smaller will cause a failure
// const uint8_t PixelPin = 2;      // make sure to set this to the correct pin, ignored for Esp8266

// #define colorSaturation 128

// // three element pixels, in different order and speeds
// NeoPixelBus<NeoGrbFeature, Neo800KbpsMethod> strip(PixelCount, PixelPin);
// //NeoPixelBus<NeoRgbFeature, Neo400KbpsMethod> strip(PixelCount, PixelPin);

// // For Esp8266, the Pin is omitted and it uses GPIO3 due to DMA hardware use.
// // There are other Esp8266 alternative methods that provide more pin options, but also have
// // other side effects.
// //NeoPixelBus<NeoGrbFeature, Neo800KbpsMethod> strip(PixelCount);
// //
// // NeoEsp8266Uart800KbpsMethod uses GPI02 instead

// // You can also use one of these for Esp8266,
// // each having their own restrictions
// //
// // These two are the same as above as the DMA method is the default
// // NOTE: These will ignore the PIN and use GPI03 pin
// //NeoPixelBus<NeoGrbFeature, NeoEsp8266Dma800KbpsMethod> strip(PixelCount, PixelPin);
// //NeoPixelBus<NeoRgbFeature, NeoEsp8266Dma400KbpsMethod> strip(PixelCount, PixelPin);

// // Uart method is good for the Esp-01 or other pin restricted modules
// // NOTE: These will ignore the PIN and use GPI02 pin
// //NeoPixelBus<NeoGrbFeature, NeoEsp8266Uart800KbpsMethod> strip(PixelCount, PixelPin);
// //NeoPixelBus<NeoRgbFeature, NeoEsp8266Uart400KbpsMethod> strip(PixelCount, PixelPin);

// // The bitbang method is really only good if you are not using WiFi features of the ESP
// // It works with all but pin 16
// //NeoPixelBus<NeoGrbFeature, NeoEsp8266BitBang800KbpsMethod> strip(PixelCount, PixelPin);
// //NeoPixelBus<NeoRgbFeature, NeoEsp8266BitBang400KbpsMethod> strip(PixelCount, PixelPin);

// // four element pixels, RGBW
// //NeoPixelBus<NeoRgbwFeature, Neo800KbpsMethod> strip(PixelCount, PixelPin);

// RgbColor red(colorSaturation, 0, 0);
// RgbColor green(0, colorSaturation, 0);
// RgbColor blue(0, 0, colorSaturation);
// RgbColor white(colorSaturation);
// RgbColor black(0);

// HslColor hslRed(red);
// HslColor hslGreen(green);
// HslColor hslBlue(blue);
// HslColor hslWhite(white);
// HslColor hslBlack(black);

// void setup()
// {
//     Serial.begin(115200);
//     while (!Serial)
//         ; // wait for serial attach

//     Serial.println();
//     Serial.println("Initializing...");
//     Serial.flush();

//     // this resets all the neopixels to an off state
//     strip.Begin();
//     strip.Show();

//     Serial.println();
//     Serial.println("Running...");
// }

// void loop()
// {
//     delay(5000);

//     Serial.println("Colors R, G, B, W...");

//     // set the colors,
//     // if they don't match in order, you need to use NeoGrbFeature feature
//     strip.SetPixelColor(0, red);
//     strip.SetPixelColor(1, green);
//     strip.SetPixelColor(2, blue);
//     strip.SetPixelColor(3, white);
//     // the following line demonstrates rgbw color support
//     // if the NeoPixels are rgbw types the following line will compile
//     // if the NeoPixels are anything else, the following line will give an error
//     //strip.SetPixelColor(3, RgbwColor(colorSaturation));
//     strip.Show();

//     delay(5000);

//     Serial.println("Off ...");

//     // turn off the pixels
//     strip.SetPixelColor(0, black);
//     strip.SetPixelColor(1, black);
//     strip.SetPixelColor(2, black);
//     strip.SetPixelColor(3, black);
//     strip.Show();

//     delay(5000);

//     Serial.println("HSL Colors R, G, B, W...");

//     // set the colors,
//     // if they don't match in order, you may need to use NeoGrbFeature feature
//     strip.SetPixelColor(0, hslRed);
//     strip.SetPixelColor(1, hslGreen);
//     strip.SetPixelColor(2, hslBlue);
//     strip.SetPixelColor(3, hslWhite);
//     strip.Show();

//     delay(5000);

//     Serial.println("Off again...");

//     // turn off the pixels
//     strip.SetPixelColor(0, hslBlack);
//     strip.SetPixelColor(1, hslBlack);
//     strip.SetPixelColor(2, hslBlack);
//     strip.SetPixelColor(3, hslBlack);
//     strip.Show();
// }
