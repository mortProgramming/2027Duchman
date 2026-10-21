package frc.robot.commands;

import static frc.robot.Constants.AlgaeRollerConstants.INTAKE_SPEED;
import static frc.robot.Constants.AlgaeRollerConstants.OUTTAKE_SPEED;
import static frc.robot.Constants.AlgaeRollerConstants.REST_SPEED;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.AlgaeRoller;

/**
 * Runs the algae roller at a fixed percent while held.
 *
 * <p>On release it falls back to REST_SPEED, a light negative hold rather than zero, so a captured
 * game piece stays gripped. Matches Worlds.
 */
public class VelocityAlgaeRoller extends Command {
  private final AlgaeRoller algaeRoller;
  private final double speed;

  public VelocityAlgaeRoller(AlgaeRoller algaeRoller, double speed) {
    this.algaeRoller = algaeRoller;
    this.speed = speed;
    addRequirements(algaeRoller);
  }

  @Override
  public void execute() {
    algaeRoller.setRollerSpeed(speed);
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    algaeRoller.setRollerSpeed(REST_SPEED);
  }

  public static Command intake(AlgaeRoller algaeRoller) {
    return new VelocityAlgaeRoller(algaeRoller, INTAKE_SPEED);
  }

  public static Command outtake(AlgaeRoller algaeRoller) {
    return new VelocityAlgaeRoller(algaeRoller, OUTTAKE_SPEED);
  }

  public static Command rest(AlgaeRoller algaeRoller) {
    return new VelocityAlgaeRoller(algaeRoller, REST_SPEED);
  }
}
