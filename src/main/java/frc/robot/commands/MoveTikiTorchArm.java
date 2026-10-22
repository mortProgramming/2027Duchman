package frc.robot.commands;

import static frc.robot.Constants.OperatorConstants.DRIVE_DEADBAND;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.TikiTorchArm;

/**
 * Manual open-loop jog for the coral arm. Renamed from SpeedTikiTorch so both arms share one
 * naming pattern.
 *
 * <p>Bypasses the PID and the feedforward and drives the motor directly. Note this arm uses
 * {@code set()} rather than {@code setVoltage()}, so the same number here gives a slightly
 * different result than on the algae arm as the battery sags.
 *
 * <p>Nothing stops this at the travel limits — keep a finger near disable.
 */
public class MoveTikiTorchArm extends Command {
  private final TikiTorchArm tikiTorchArm;
  private final DoubleSupplier speed;

  /** Analog jog, e.g. from a stick axis. */
  public MoveTikiTorchArm(TikiTorchArm tikiTorchArm, DoubleSupplier speed) {
    this.tikiTorchArm = tikiTorchArm;
    this.speed = speed;
    addRequirements(tikiTorchArm);
  }

  /** Fixed-speed jog, for a button binding. */
  public MoveTikiTorchArm(TikiTorchArm tikiTorchArm, double speed) {
    this(tikiTorchArm, () -> speed);
  }

  @Override
  public void execute() {
    tikiTorchArm.setArmSpeed(MathUtil.applyDeadband(speed.getAsDouble(), DRIVE_DEADBAND));
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    tikiTorchArm.setArmSpeed(0);
  }
}
