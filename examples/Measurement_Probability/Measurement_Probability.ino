// # Measurement Probability
//
// This example visualizes the Born-rule probability of a projective
// measurement in the Z basis, i.e. of the two poles of the Bloch sphere.
//
// As you tilt the Qbead around, the direction of gravity (as read by the
// IMU) is drawn as a moving blue point on the surface of the Bloch sphere --
// this is the quantum state we are considering.
//
// The two poles of the Z-axis are always the *same* two physical LEDs,
// regardless of how the point moves (they do not rotate with phi). As the
// point moves, each pole grows brighter in green in proportion to the
// probability that a Z-measurement of the point's state would collapse to
// that pole, i.e. |<pole|point>|^2, per the Born rule. When the point sits
// exactly on a pole, that pole lights up fully and the other pole goes
// completely dark, since a measurement then has probability 1 of landing on
// the aligned pole.
//
// Unlike Tap_to_Measure.ino, this example never actually performs a
// collapse -- it is a continuous, hands-on demonstration of how the
// probabilities themselves evolve as you tilt the device, so there is no
// tap handling here.

// First, let's include the Qbead library and set up a few useful data structures.
#include <Qbead.h>

Qbead::Qbead bead;

// The two poles of the Z-axis, fixed in space: north is theta=0 (|0>), south is theta=180 (|1>).
BlochVector north_pole(0, 0);
BlochVector south_pole(180, 0);

// Prepare some colors for the visualization.
uint32_t blue = color(0, 0, 255);   // the moving point on the Bloch sphere
uint32_t green = color(0, 255, 0);  // the poles, brightness = measurement probability

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
// It reads the IMU, draws the point, and then computes and draws the
// Born-rule probabilities on the two fixed poles.
void loop() {
  // Read the IMU to get the current gravity direction.
  bead.readIMU(false);

  // The point currently under consideration is wherever the device is tilted to.
  BlochVector current_state(bead.t_acc, bead.p_acc);

  // Clear the display.
  bead.clear();

  // ### Draw the moving point.
  //
  // We draw the point *before* the poles below, so that whenever the point
  // happens to land exactly on a physical pole pixel, the pole's green is
  // drawn on top and takes precedence over the point's blue.
  bead.setBloch_deg_smooth(current_state, blue);

  // ### Compute and draw the Born-rule measurement probabilities on the poles.
  //
  // |<pole|point>| is given by innerProductAbs(); the Born-rule probability
  // of collapsing to that pole upon a Z-measurement is the square of that.
  float amp_north = innerProductAbs(current_state, north_pole);
  float p_north = amp_north * amp_north;
  // p_south is the exact complement of p_north (cos^2(t/2) + sin^2(t/2) = 1),
  // so we get it for free without a second, more expensive trig calculation.
  float p_south = 1.0f - p_north;

  // The poles are two fixed physical LEDs (independent of phi/leg), addressed
  // the same way setBloch_deg itself addresses them internally.
  bead.setLegPixelColor(0, 0, scaleColor(p_north, green));
  bead.setLegPixelColor(0, bead.nsections, scaleColor(p_south, green));

  // Show the result.
  bead.show();

  Serial.print(millis());
  Serial.print("  | theta: ");
  Serial.print(current_state.theta);
  Serial.print(" | P(north): ");
  Serial.print(p_north);
  Serial.print(" | P(south): ");
  Serial.println(p_south);
}
