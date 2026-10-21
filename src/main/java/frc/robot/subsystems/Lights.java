package frc.robot.subsystems;

import static frc.robot.Constants.LightsConstants.PWM_PORT;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * REV Blinkin LED controller on PWM 0.
 *
 * <p>The Blinkin takes a PWM value that selects a pattern, so "colour" here is a pattern code from
 * {@link frc.robot.Constants.LightsConstants}, not a brightness. One setter is enough — the named
 * colour helpers in the Worlds code were just wrappers around this call.
 */
public class Lights extends SubsystemBase {
  private final Spark leds = new Spark(PWM_PORT);

  public void setColor(double blinkinPatternCode) {
    leds.set(blinkinPatternCode);
  }
}
