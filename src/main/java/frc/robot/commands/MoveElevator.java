package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.Elevator;

/**
 * Manual open-loop jog for the elevator.
 *
 * <p>Bypasses the PID and drives the motor directly, so it tells you which way the carriage moves
 * for a positive command and whether the height reading tracks — independent of whether the closed
 * loop works.
 *
 * <p>Note the speed still goes through {@code setElevatorMotorPercent}, which adds the standing
 * POS_KG bias. That is intentional: the bias is this mechanism's gravity compensation, and jogging
 * without it would behave differently from every other elevator command.
 *
 * <p>The Worlds equivalent also called the PID controller and discarded the result; that dead call
 * is not reproduced here.
 *
 * <p><b>Nothing stops this at either end of travel.</b> The limit switches re-zero the height
 * tracking but do not cut the output. Keep a finger near disable.
 */
public class MoveElevator extends Command {
  private final Elevator elevator;
  private final DoubleSupplier speed;

  /** Analog jog, e.g. from a stick axis. */
  public MoveElevator(Elevator elevator, DoubleSupplier speed) {
    this.elevator = elevator;
    this.speed = speed;
    addRequirements(elevator);
  }

  /** Fixed-speed jog, for a button binding. */
  public MoveElevator(Elevator elevator, double speed) {
    this(elevator, () -> speed);
  }

  @Override
  public void execute() {
    elevator.setElevatorMotorPercent(speed.getAsDouble());
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    elevator.setElevatorMotorPercent(0);
  }
}
