#include <hwrev2_rgbled.hpp>

const RGBColor hwrev2_RGBLED::RED      = {255, 0, 0};
const RGBColor hwrev2_RGBLED::GREEN    = {0, 255, 0};
const RGBColor hwrev2_RGBLED::BLUE     = {0, 0, 255};
const RGBColor hwrev2_RGBLED::AMBER    = {255, 255, 0};
const RGBColor hwrev2_RGBLED::CYAN    = {0, 255, 255};
const RGBColor hwrev2_RGBLED::BLACK    = {0, 0, 0};

hwrev2_RGBLED::hwrev2_RGBLED(VehicleConfig cfg){

  _config = cfg;

}

void hwrev2_RGBLED::init(ILogger *logger){

  _logger = logger;
  _led = new Adafruit_NeoPixel(1, _config.pinConfig.rgbLed, NEO_RGB + NEO_KHZ800);
  _led->begin();

  _logger->sendMessage("hwrev2_RGBLED::innit", _logger->INFO, "Initialised NeoPixel");

}

void hwrev2_RGBLED::setStaticColor(RGBColor col){

  _led->setPixelColor(0, _led->Color(col.red, col.green, col.blue));
  _led->show();
  _logger->sendMessage("hwrev2_RGBLED::setStaticColor", _logger->INFO, "Set color " + String(col.red) + ", " + String(col.green) + ", " + String(col.blue));

}

void hwrev2_RGBLED::limitBrightness(uint8_t lim){

  _led->setBrightness(lim);
  _logger->sendMessage("hwrev2_RGBLED::limitBrightness", _logger->INFO, "Limiting brightness to " + String(lim) + "/255");
  
}