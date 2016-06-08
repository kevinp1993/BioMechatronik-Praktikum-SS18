#include <ch.hpp>
#include <hal.h>

#include <amiro/bus/spi/HWSPIDriver.hpp>
#include <amiro/leds/tlc5947.hpp>

using namespace chibios_rt;
using namespace amiro;

TLC5947::TLC5947(HWSPIDriver *spi, ioportid_t blankPort, int blankPad)
    : BaseStaticThread<192>(),
      spi(spi),
      blankPort(blankPort),
      blankPad(blankPad),
      brightness(0x00u) {
  for (int i = 0; i < 8; i++)
    this->colors[i] = Color::BLACK;
}

TLC5947::
~TLC5947() {

}

void TLC5947::disable() {
  palWritePad(this->blankPort, this->blankPad, 1);
}

void TLC5947::enable() {
  palWritePad(this->blankPort, this->blankPad, 0);
}

void TLC5947::setBrightness(int brightness) {
  if (brightness < 0)
    this->brightness = 0;
  else if (brightness > 100)
    this->brightness = 100;
  else
    this->brightness = brightness;
}

void TLC5947::setColor(int index, Color color) {
  this->colors[index] = color;
}

void TLC5947::update() {
  this->signalEvents(static_cast<eventmask_t>(1));
}

msg_t TLC5947::main(void) {
  uint8_t buffer[36];

  this->setName("Tlc5947");

  while (!this->shouldTerminate()) {
    int brightness = this->brightness;

    for (int i = 0, j = 0; i < 8; i += 2) {
      Color color1 = this->colors[i];
      Color color2 = this->colors[i + 1];

      int values[6];
      values[0] = caluclateBlueGrayscale(color1, brightness);
      values[1] = caluclateGreenGrayscale(color1, brightness);
      values[2] = caluclateRedGrayscale(color1, brightness);
      values[3] = caluclateBlueGrayscale(color2, brightness);
      values[4] = caluclateGreenGrayscale(color2, brightness);
      values[5] = caluclateRedGrayscale(color2, brightness);

      for (int k = 0; k < 6; k += 2) {
        int value1 = values[k];
        int value2 = values[k + 1];

        buffer[j++] = static_cast<uint8_t>((value1 >> 4) & 0xff);
        buffer[j++] = static_cast<uint8_t>((value1 << 4) | ((value2 >> 8) & 0xff));
        buffer[j++] = static_cast<uint8_t>(value2 & 0xff);
      }
    }
    this->spi->write(buffer, 36);

    this->waitAnyEvent(ALL_EVENTS);
  }

  return true;
}

int TLC5947::caluclateRedGrayscale(Color color, int brightness) const {
  return color.getRed() * brightness * 16 / 100;
}

int TLC5947::caluclateGreenGrayscale(Color color, int brightness) const {
  return color.getGreen() * brightness * 16 / 100;
}

int TLC5947::caluclateBlueGrayscale(Color color, int brightness) const {
  return color.getBlue() * brightness * (16 * 3) / (100 * 4);
}
