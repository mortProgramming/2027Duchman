package frc.robot.commands;

import static frc.robot.Constants.LightsConstants.BLUE;
import static frc.robot.Constants.LightsConstants.DARK_BLUE;
import static frc.robot.Constants.LightsConstants.HEIGHT_DARK_BLUE;
import static frc.robot.Constants.LightsConstants.HEIGHT_WHITE;
import static frc.robot.Constants.LightsConstants.HEIGHT_YELLOW;
import static frc.robot.Constants.LightsConstants.RED;
import static frc.robot.Constants.LightsConstants.WHITE;
import static frc.robot.Constants.LightsConstants.YELLOW;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Lights;

/**
 * Default command for the LEDs: shows elevator height as a colour so the drive team can read the
 * carriage position from across the field.
 *
 * <p>Thresholds were magic numbers inside the Worlds command; they now live in LightsConstants.
 */
public class TeleopLights extends Command {
  private final Lights lights;
  private final Elevator elevator;

  public TeleopLights(Lights lights, Elevator elevator) {
    this.lights = lights;
    this.elevator = elevator;
    addRequirements(lights);
  }

  @Override
  public void execute() {
    double height = elevator.getElevatorPositionInches();

    if (height > HEIGHT_YELLOW) {
      lights.setColor(YELLOW);
    } else if (height > HEIGHT_DARK_BLUE) {
      lights.setColor(DARK_BLUE);
    } else if (height > HEIGHT_WHITE) {
      lights.setColor(WHITE);
    } else {
      lights.setColor(RED);
    }
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    lights.setColor(BLUE);
  }
}
