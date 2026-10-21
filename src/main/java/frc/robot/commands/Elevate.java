package frc.robot.commands;

import static frc.robot.Constants.ElevatorConstants.BARGE_HEIGHT;
import static frc.robot.Constants.ElevatorConstants.HIGH_ALGAE_HEIGHT;
import static frc.robot.Constants.ElevatorConstants.INTAKE_HEIGHT;
import static frc.robot.Constants.ElevatorConstants.L1_HEIGHT;
import static frc.robot.Constants.ElevatorConstants.L2_HEIGHT;
import static frc.robot.Constants.ElevatorConstants.L3_HEIGHT;
import static frc.robot.Constants.ElevatorConstants.L4_HEIGHT;
import static frc.robot.Constants.ElevatorConstants.LOWER_LIMIT_SWITCH_HEIGHT;
import static frc.robot.Constants.ElevatorConstants.LOW_ALGAE_HEIGHT;
import static frc.robot.Constants.ElevatorConstants.POS_CONSTRAINTS;
import static frc.robot.Constants.ElevatorConstants.POS_TELEOP_CONSTRAINTS;
import static frc.robot.Constants.ElevatorConstants.REST_HEIGHT;

import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.Elevator;

/**
 * Drives the elevator to a height in inches and holds it there.
 *
 * <p>Never finishes on its own, so it keeps holding the carriage up while scoring until something
 * interrupts it. The PID output is negated on the way in. That sign is load-bearing and matches
 * the Worlds code.
 */
public class Elevate extends Command {
  private final Elevator elevator;
  private final double targetPosition;
  private final double maxVelocity;
  private final double maxAcceleration;

  public Elevate(Elevator elevator, double targetPosition) {
    this(elevator, targetPosition, POS_CONSTRAINTS.maxVelocity, POS_CONSTRAINTS.maxAcceleration);
  }

  public Elevate(Elevator elevator, double targetPosition, double maxVelocity) {
    this(elevator, targetPosition, maxVelocity, POS_CONSTRAINTS.maxAcceleration);
  }

  public Elevate(
      Elevator elevator, double targetPosition, double maxVelocity, double maxAcceleration) {
    this.elevator = elevator;
    this.targetPosition = targetPosition;
    this.maxVelocity = maxVelocity;
    this.maxAcceleration = maxAcceleration;
    addRequirements(elevator);
  }

  @Override
  public void initialize() {
    elevator.getController().reset(elevator.getElevatorPositionInches());
    elevator.getController().setConstraints(new Constraints(maxVelocity, maxAcceleration));
  }

  @Override
  public void execute() {
    elevator.setElevatorMotorPercent(
        -elevator.getController().calculate(elevator.getElevatorPositionInches(), targetPosition));
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    elevator.setElevatorMotorPercent(0);
  }

  public static Command rest(Elevator elevator) {
    return new Elevate(elevator, REST_HEIGHT);
  }

  public static Command l1(Elevator elevator) {
    return new Elevate(elevator, L1_HEIGHT);
  }

  public static Command l2(Elevator elevator) {
    return new Elevate(elevator, L2_HEIGHT);
  }

  public static Command l3(Elevator elevator) {
    return new Elevate(elevator, L3_HEIGHT);
  }

  public static Command l4(Elevator elevator) {
    return new Elevate(elevator, L4_HEIGHT);
  }

  public static Command teleopL4(Elevator elevator) {
    return new Elevate(
        elevator,
        L4_HEIGHT,
        POS_TELEOP_CONSTRAINTS.maxVelocity,
        POS_TELEOP_CONSTRAINTS.maxAcceleration);
  }

  public static Command intake(Elevator elevator) {
    return new Elevate(elevator, INTAKE_HEIGHT);
  }

  public static Command lowAlgae(Elevator elevator) {
    return new Elevate(elevator, LOW_ALGAE_HEIGHT);
  }

  public static Command highAlgae(Elevator elevator) {
    return new Elevate(elevator, HIGH_ALGAE_HEIGHT);
  }

  public static Command barge(Elevator elevator) {
    return new Elevate(elevator, BARGE_HEIGHT);
  }

  public static Command zero(Elevator elevator) {
    return new Elevate(elevator, LOWER_LIMIT_SWITCH_HEIGHT);
  }
}
