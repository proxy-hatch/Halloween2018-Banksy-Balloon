# Halloween2018 - Banksy Balloon

Code in this project controls a WS2812B Neopixel Matrix.

For a one-time thing like Halloween, I chose [platformio](https://docs.platformio.org/en/latest/) as the framework to control my Arduino Uno.

While I do appreciate the [library support](https://platformio.org/lib/show/547/NeoPixelBus?utm_source=platformio&utm_medium=piohome) that comes with it, I later discovered this "Open Source" framework charges for even the most basic features like stepping through code with avr-gdb. Looks like its back to Atmel Studio next time!

The project cycles through three animations controlled by a 16*16 spritesheet with too many frames to clean up manually. With some photoshop the frames were formed, but going through the library author's [Paint.Net plugin](http://forums.getpaint.net/index.php?/topic/107921-arduino-neopixel-sketch-exporter/) to generate the frame in ```.c```.


### It's Halloween!
![](https://i.imgur.com/egnYgwB.jpg)
