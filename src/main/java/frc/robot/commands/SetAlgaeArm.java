package frc.robot.commands;

import static frc.robot.Constants.AlgaeArmConstants.BARGE_SCORE;
import static frc.robot.Constants.AlgaeArmConstants.FLOOR_INTAKE;
import static frc.robot.Constants.AlgaeArmConstants.MAX_DEGREES;
import static frc.robot.Constants.AlgaeArmConstants.MIN_DEGREES;
import static frc.robot.Constants.AlgaeArmConstants.POP_INTAKE;
import static frc.robot.Constants.AlgaeArmConstants.REEF_INTAKE;
import static frc.robot.Constants.AlgaeArmConstants.REST;

import java.util.Set;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

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
    this(algaeArm, setpoint, true);
  }

  private SetAlgaeArm(AlgaeArm algaeArm, double setpoint, boolean clampToTravel) {
    this.algaeArm = algaeArm;
    // Clamped to the travel limits so a bad setpoint cannot send the arm into a hard stop.
    // Clamping the TARGET rather than the motor output is deliberate: it can never trap the arm,
    // whereas an output limit with the sign guessed wrong could.
    this.setpoint =
        clampToTravel ? MathUtil.clamp(setpoint, MIN_DEGREES, MAX_DEGREES) : setpoint;
    addRequirements(algaeArm);
  }

  /**
   * Holds the arm at whatever angle it is at when this starts. The arm's default command, so it is
   * never left unpowered after a jog or a cancelled pose (Nov 2026 review). Not clamped: holding
   * where the arm already is must never drive it somewhere else.
   */
  public static Command holdCurrentAngle(AlgaeArm algaeArm) {
    return Commands.defer(
        () -> new SetAlgaeArm(algaeArm, algaeArm.getDegrees(), false), Set.of(algaeArm));
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
