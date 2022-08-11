# sudo pip3 install rpi_ws281x
import time
from rpi_ws281x import *

NUM_LEDS = 30
LED_PIN = 18
LED_FREQ = 800000
LED_DMA = 10
LED_BRIGHTNESS = 255  # Set to 0 for darkest and 255 for brightest
LED_INVERT = False    # True to invert the signal (when using NPN transistor level shift)
LED_CHANNEL = 0

WHITE_LIGHT = Color(255,255,255)

led_strip = Adafruit_NeoPixel(NUM_LEDS, LED_PIN, LED_FREQ, LED_DMA, LED_INVERT, LED_BRIGHTNESS, LED_CHANNEL)
led_strip.begin()

# Set the led to a white light
for led in range(0, NUM_LEDS):
    led_strip.setPixelColor(led, WHITE_LIGHT)

led_strip.show()