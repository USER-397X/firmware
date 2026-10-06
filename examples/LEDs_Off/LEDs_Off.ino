// # LEDs Off
//
// There is no physical power switch on the Qbead, so this sketch exists
// purely to turn every LED off. It keeps clearing the display every loop
// (rather than just once) so that an occasional corrupted frame -- which
// can happen since BLE is still running in the background -- is instantly
// overwritten rather than left stuck on screen.

#include <Qbead.h>

Qbead::Qbead bead;

void setup() {
  bead.begin();
}

void loop() {
  bead.clear();
  bead.show();
}
