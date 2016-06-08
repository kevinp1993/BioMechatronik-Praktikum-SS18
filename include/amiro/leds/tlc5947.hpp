#ifndef AMIRO_TLC5947_H_
#define AMIRO_TLC5947_H_

#include <amiro/Color.h>

namespace amiro {

  class HWSPIDriver;

  class TLC5947 : public chibios_rt::BaseStaticThread<192> {
  public:
    TLC5947(HWSPIDriver *spi, ioportid_t blankPort, int blankPad);
    virtual ~TLC5947();
    void disable();
    void enable();
    void setBrightness(int brightness);
    void setColor(int index, Color color);
    void update();

  protected:
    virtual msg_t main(void);

  private:
    int caluclateRedGrayscale(Color color, int brightness) const;
    int caluclateGreenGrayscale(Color color, int brightness) const;
    int caluclateBlueGrayscale(Color color, int brightness) const;

    HWSPIDriver *spi;
    ioportid_t blankPort;
    int blankPad;
    Color colors[8];
    int brightness;
  };

}

#endif /* AMIRO_TLC5947_H_ */
