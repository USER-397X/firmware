// # Measurement Probability (Sphere Fill)
//
// This is a variant of Measurement_Probability.ino that makes the same
// Born-rule probability far easier to see. Instead of modulating the
// brightness of two single LEDs at the poles -- a subtle effect that is
// hard to make out, especially on video -- the *entire* sphere is filled
// with a purple/yellow split whose boundary latitude tracks the probability.
//
// A fully purple sphere means a Z-measurement would certainly yield |0>.
// A fully yellow sphere means a Z-measurement would certainly yield |1>.
// An exact half-purple / half-yellow split (boundary at the equator) means
// an equal superposition. In between, the purple/yellow boundary slides up
// or down the sphere as you tilt the device and the probability changes.
//
// As in Measurement_Probability.ino, the moving blue point marks the
// device's current orientation (the state being considered). It is drawn
// on top of the fill, so you can see both the state and the probability
// gauge at the same time.

// First, let's include the Qbead library and set up a few useful data structures.
#include <Qbead.h>

Qbead::Qbead bead;

// The north pole, used as the reference for the Born-rule P(|0>) calculation.
BlochVector north_pole(0, 0);

// Prepare some colors for the visualization.
uint32_t blue = color(0, 0, 255);     // the moving point on the Bloch sphere
uint32_t purple = color(255, 0, 255); // the |0> end of the probability gauge
uint32_t yellow = color(255, 255, 0); // the |1> end of the probability gauge

// ## Setup
//
// The setup function is called once when the Qbead is powered on.
void setup() {
  bead.begin();
  bead.setBrightness(25);
  // Test the pixels by flashing a colorful pattern to make sure they are working.
  bead.testPixels();
}

// ## Event loop
//
// The loop function is called repeatedly until the Qbead is powered off.
// It reads the IMU, fills the whole sphere as a probability gauge, and
// then draws the moving point on top of it.
void loop() {
  // Read the IMU to get the current gravity direction.
  bead.readIMU(false);

  // The point currently under consideration is wherever the device is tilted to.
  BlochVector current_state(bead.t_acc, bead.p_acc);

  // ### Compute the Born-rule probability of measuring |0>.
  //
  // |<north_pole|point>| is given by innerProductAbs(); squaring it gives
  // the probability, exactly as in Measurement_Probability.ino.
  float amp_north = innerProductAbs(current_state, north_pole);
  float p_north = amp_north * amp_north;

  // Clear the display.
  bead.clear();

  // ### Fill the whole sphere as a probability gauge.
  //
  // The sphere has (nsections + 1) latitude rings, from the north pole
  // (ring 0) down to the south pole (ring nsections). `fill` is how many
  // of those rings, counted from the north, should be purple; the one ring
  // that straddles the boundary gets a blended color so the transition is
  // smooth as the probability changes.
  float fill = p_north * (bead.nsections + 1);
  for (int ring = 0; ring <= bead.nsections; ring++) {
    float purple_fraction = constrain(fill - ring, 0.0f, 1.0f);
    uint32_t ring_color = addColor(scaleColor(purple_fraction, purple), scaleColor(1.0f - purple_fraction, yellow));

    if (ring == 0 || ring == bead.nsections) {
      // The poles are a single physical LED shared by every leg.
      bead.setLegPixelColor(0, ring, ring_color);
    } else {
      for (int leg = 0; leg < bead.nlegs; leg++) {
        bead.setLegPixelColor(leg, ring, ring_color);
      }
    }
  }

  // ### Draw the moving point on top of the fill.
  //
  // Drawn last, so it overwrites the fill color at its own position and
  // stays visible against either a green or a red background.
  bead.setBloch_deg_smooth(current_state, blue);

  // Show the result.
  bead.show();

  Serial.print(millis());
  Serial.print("  | theta: ");
  Serial.print(current_state.theta);
  Serial.print(" | P(|0>): ");
  Serial.print(p_north);
  Serial.print(" | fill: ");
  Serial.println(fill);
}
