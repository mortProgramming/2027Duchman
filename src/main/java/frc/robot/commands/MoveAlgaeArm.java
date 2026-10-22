package frc.robot.commands;

import static frc.robot.Constants.OperatorConstants.DRIVE_DEADBAND;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.AlgaeArm;

/**
 * Manual open-loop jog for the algae arm.
 *
 * <p>Bypasses the PID and the feedforward entirely and drives the motor directly, which is what
 * you want during bring-up: it tells you which way the arm physically moves for a positive
 * command, independent of whether the closed loop is wired correctly.
 *
 * <p>Nothing stops this at the travel limits. {@link SetAlgaeArm} clamps its target, but a manual
 * jog is the operator driving the arm by hand — keep a finger near disable.
 */
public class MoveAlgaeArm extends Command {
  private final AlgaeArm algaeArm;
  private final DoubleSupplier speed;

  /** Analog jog, e.g. from a stick axis. */
  public MoveAlgaeArm(AlgaeArm algaeArm, DoubleSupplier speed) {
    this.algaeArm = algaeArm;
    this.speed = speed;
    addRequirements(algaeArm);
  }

  /** Fixed-speed jog, for a button binding. */
  public MoveAlgaeArm(AlgaeArm algaeArm, double speed) {
    this(algaeArm, () -> speed);
  }

  @Override
  public void execute() {
    algaeArm.setArmSpeed(MathUtil.applyDeadband(speed.getAsDouble(), DRIVE_DEADBAND));
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    algaeArm.setArmSpeed(0);
  }
}
