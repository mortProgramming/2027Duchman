package frc.robot.commands;

import static frc.robot.Constants.TikiTorchRollerConstants.INTAKE_SPEED;
import static frc.robot.Constants.TikiTorchRollerConstants.L1_SPEED;
import static frc.robot.Constants.TikiTorchRollerConstants.OUTTAKE_SPEED;
import static frc.robot.Constants.TikiTorchRollerConstants.REST_SPEED;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.TikiTorchRoller;

/**
 * Runs the coral roller at a fixed percent while held, falling back to a light hold on release.
 */
public class VelocityTikiTorchRoller extends Command {
  private final TikiTorchRoller tikiTorchRoller;
  private final double speed;

  public VelocityTikiTorchRoller(TikiTorchRoller tikiTorchRoller, double speed) {
    this.tikiTorchRoller = tikiTorchRoller;
    this.speed = speed;
    addRequirements(tikiTorchRoller);
  }

  @Override
  public void execute() {
    tikiTorchRoller.setRollerSpeed(speed);
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    tikiTorchRoller.setRollerSpeed(REST_SPEED);
  }

  public static Command intake(TikiTorchRoller tikiTorchRoller) {
    return new VelocityTikiTorchRoller(tikiTorchRoller, INTAKE_SPEED);
  }

  public static Command outtake(TikiTorchRoller tikiTorchRoller) {
    return new VelocityTikiTorchRoller(tikiTorchRoller, OUTTAKE_SPEED);
  }

  /** Slower score for L1 so the coral is placed rather than launched. */
  public static Command l1(TikiTorchRoller tikiTorchRoller) {
    return new VelocityTikiTorchRoller(tikiTorchRoller, L1_SPEED);
  }

  public static Command rest(TikiTorchRoller tikiTorchRoller) {
    return new VelocityTikiTorchRoller(tikiTorchRoller, REST_SPEED);
  }
}
