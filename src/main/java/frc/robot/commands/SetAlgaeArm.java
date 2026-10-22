package frc.robot.commands;

import static frc.robot.Constants.AlgaeArmConstants.BARGE_SCORE;
import static frc.robot.Constants.AlgaeArmConstants.FLOOR_INTAKE;
import static frc.robot.Constants.AlgaeArmConstants.MAX_DEGREES;
import static frc.robot.Constants.AlgaeArmConstants.MIN_DEGREES;
import static frc.robot.Constants.AlgaeArmConstants.POP_INTAKE;
import static frc.robot.Constants.AlgaeArmConstants.REEF_INTAKE;
import static frc.robot.Constants.AlgaeArmConstants.REST;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.AlgaeArm;

/**
 * Drives the algae arm to an angle in degrees and holds it.
 *
 * <p>Unlike Elevate, the PID output is NOT negated here. The arm subsystem negates internally in
 * setArmPercent, so negating again would drive the arm the wrong way.
 */
public class SetAlgaeArm extends Command {
  private final AlgaeArm algaeArm;
  private final double setpoint;

  public SetAlgaeArm(AlgaeArm algaeArm, double setpoint) {
    this.algaeArm = algaeArm;
    // Clamped to the travel limits so a bad setpoint cannot send the arm into a hard stop.
    // Clamping the TARGET rather than the motor output is deliberate: it can never trap the arm,
    // whereas an output limit with the sign guessed wrong could.
    this.setpoint = MathUtil.clamp(setpoint, MIN_DEGREES, MAX_DEGREES);
    addRequirements(algaeArm);
  }

  @Override
  public void initialize() {
    algaeArm.getController().reset(algaeArm.getDegrees());
  }

  @Override
  public void execute() {
    algaeArm.setArmPercent(algaeArm.getController().calculate(algaeArm.getDegrees(), setpoint));
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    algaeArm.setArmPercent(0);
  }

  public static Command rest(AlgaeArm algaeArm) {
    return new SetAlgaeArm(algaeArm, REST);
  }

  public static Command reefIntake(AlgaeArm algaeArm) {
    return new SetAlgaeArm(algaeArm, REEF_INTAKE);
  }

  public static Command floor(AlgaeArm algaeArm) {
    return new SetAlgaeArm(algaeArm, FLOOR_INTAKE);
  }

  public static Command pop(AlgaeArm algaeArm) {
    return new SetAlgaeArm(algaeArm, POP_INTAKE);
  }

  public static Command barge(AlgaeArm algaeArm) {
    return new SetAlgaeArm(algaeArm, BARGE_SCORE);
  }
}
