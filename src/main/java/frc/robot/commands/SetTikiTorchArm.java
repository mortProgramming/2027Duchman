package frc.robot.commands;

import static frc.robot.Constants.TikiTorchArmConstants.ALGAE_CLEAR;
import static frc.robot.Constants.TikiTorchArmConstants.INTAKE;
import static frc.robot.Constants.TikiTorchArmConstants.L1_SCORE;
import static frc.robot.Constants.TikiTorchArmConstants.MAX_DEGREES;
import static frc.robot.Constants.TikiTorchArmConstants.MIN_DEGREES;
import static frc.robot.Constants.TikiTorchArmConstants.L2_SCORE;
import static frc.robot.Constants.TikiTorchArmConstants.L3_SCORE;
import static frc.robot.Constants.TikiTorchArmConstants.L4_SCORE;
import static frc.robot.Constants.TikiTorchArmConstants.REST;

import java.util.Set;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import frc.robot.subsystems.TikiTorchArm;

/** Drives the coral arm to an angle in degrees and holds it. */
public class SetTikiTorchArm extends Command {
  private final TikiTorchArm tikiTorchArm;
  private final double setpoint;

  public SetTikiTorchArm(TikiTorchArm tikiTorchArm, double setpoint) {
    this(tikiTorchArm, setpoint, true);
  }

  private SetTikiTorchArm(TikiTorchArm tikiTorchArm, double setpoint, boolean clampToTravel) {
    this.tikiTorchArm = tikiTorchArm;
    // Clamped to the travel limits. See the note in SetAlgaeArm on why the target is clamped
    // rather than the motor output.
    this.setpoint =
        clampToTravel ? MathUtil.clamp(setpoint, MIN_DEGREES, MAX_DEGREES) : setpoint;
    addRequirements(tikiTorchArm);
  }

  /**
   * Holds the arm at whatever angle it is at when this starts. The arm's default command, so it is
   * never left unpowered after a jog or a cancelled pose (Nov 2026 review). Not clamped: holding
   * where the arm already is must never drive it somewhere else.
   */
  public static Command holdCurrentAngle(TikiTorchArm tikiTorchArm) {
    return Commands.defer(
        () -> new SetTikiTorchArm(tikiTorchArm, tikiTorchArm.getDegrees(), false),
        Set.of(tikiTorchArm));
  }

  @Override
  public void initialize() {
    tikiTorchArm.getController().reset(tikiTorchArm.getDegrees());
  }

  @Override
  public void execute() {
    tikiTorchArm.setArmPercent(
        -tikiTorchArm.getController().calculate(tikiTorchArm.getDegrees(), setpoint));
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    tikiTorchArm.setArmPercent(0);
  }

  /** Clearance pose. Every composite endeffector move starts here so the arms do not collide. */
  public static Command algaeClear(TikiTorchArm tikiTorchArm) {
    return new SetTikiTorchArm(tikiTorchArm, ALGAE_CLEAR);
  }

  public static Command rest(TikiTorchArm tikiTorchArm) {
    return new SetTikiTorchArm(tikiTorchArm, REST);
  }

  public static Command intake(TikiTorchArm tikiTorchArm) {
    return new SetTikiTorchArm(tikiTorchArm, INTAKE);
  }

  public static Command l1(TikiTorchArm tikiTorchArm) {
    return new SetTikiTorchArm(tikiTorchArm, L1_SCORE);
  }

  public static Command l2(TikiTorchArm tikiTorchArm) {
    return new SetTikiTorchArm(tikiTorchArm, L2_SCORE);
  }

  public static Command l3(TikiTorchArm tikiTorchArm) {
    return new SetTikiTorchArm(tikiTorchArm, L3_SCORE);
  }

  public static Command l4(TikiTorchArm tikiTorchArm) {
    return new SetTikiTorchArm(tikiTorchArm, L4_SCORE);
  }
}
