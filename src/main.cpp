#include <NeoPixelBus.h>
#include <NeoPixelAnimator.h>

#include "heart-animation.h"
// #include "tst5.h"


typedef RowMajorAlternatingLayout MyPanelLayout;
typedef NeoGrbFeature MyPanelColourType;

const uint8_t PanelWidth = 16; // 8 pixel x 8 pixel matrix of leds
const uint8_t PanelHeight = 16;
const uint16_t PixelCount = PanelWidth * PanelHeight;
const uint8_t PixelPin = 2;   // make sure to set this to the correct pin, ignored for Esp8266
const uint16_t AnimCount = 1; // we only need one

const uint16_t left = 0;
const uint16_t right = PanelWidth - 1;
const uint16_t top = 0;
const uint16_t bottom = PanelHeight - 1;


NeoTopology<MyPanelLayout> topo(PanelWidth, PanelHeight);
NeoPixelBus<MyPanelColourType, Neo800KbpsMethod> strip(PixelCount, PixelPin);
NeoPixelAnimator animations(AnimCount); // NeoPixel animation management object

RgbColor red(128, 0, 0);
RgbColor green(0, 128, 0);
RgbColor blue(0, 0, 128);
RgbColor white(128);
// if using NeoRgbwFeature above, use this white instead to use
// the correct white element of the LED
//RgbwColor white(128);
RgbColor black(0);

// sprite sheet stored in progmem using the same pixel feature as the NeoPixelBus
NeoVerticalSpriteSheet<NeoBufferProgmemMethod<MyPanelColourType>> spriteSheet(
    myImageWidth,      // image width and sprite width since its vertical sprite sheet
    myImageHeight,     // image height
    16, // sprite height
    // myImageHeight/3,
    myImage);

uint16_t indexSprite;

// call back for sprite sheet methods
uint16_t MyLayoutMap(int16_t x, int16_t y)
{
    return topo.MapProbe(x, y);
}

void LoopAnimUpdate(const AnimationParam &param)
{
    // wait for this animation to complete,
    // we are using it as a timer of sorts
    if (param.state == AnimationState_Completed)
    {
        // done, time to restart this position tracking animation/timer
        animations.RestartAnimation(param.index);

        // draw the next frame in the sprite
        spriteSheet.Blt(strip, 0, 0, indexSprite, MyLayoutMap);
        indexSprite = (indexSprite + 1) % (myImageHeight/16); // increment and wrap
    }
}

void setup()
{
    strip.Begin();
    strip.Show();

    indexSprite = 0;

    // we use the index 0 animation to time how often we rotate all the pixels
    animations.StartAnimation(0, 100, LoopAnimUpdate); // time is in ms
}

void loop()
{
    // this is all that is needed to keep it running
    // and avoiding using delay() is always a good thing for
    // any timing related routines
    animations.UpdateAnimations();
    strip.Show();
    if (indexSprite == 0 || indexSprite == 21 || indexSprite == 26)
        delay(1000);


}