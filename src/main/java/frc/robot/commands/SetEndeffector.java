package frc.robot.commands;

import static frc.robot.Constants.AlgaeArmConstants.BARGE_SCORE;
import static frc.robot.Constants.AlgaeArmConstants.FLOOR_INTAKE;
import static frc.robot.Constants.AlgaeArmConstants.REEF_INTAKE;
import static frc.robot.Constants.ElevatorConstants.BARGE_HEIGHT;
import static frc.robot.Constants.ElevatorConstants.FLOOR_HEIGHT;
import static frc.robot.Constants.ElevatorConstants.HIGH_ALGAE_HEIGHT;
import static frc.robot.Constants.ElevatorConstants.INTAKE_HEIGHT;
import static frc.robot.Constants.ElevatorConstants.L1_HEIGHT;
import static frc.robot.Constants.ElevatorConstants.L2_HEIGHT;
import static frc.robot.Constants.ElevatorConstants.L3_HEIGHT;
import static frc.robot.Constants.ElevatorConstants.L4_HEIGHT;
import static frc.robot.Constants.ElevatorConstants.LOWER_LIMIT_SWITCH_HEIGHT;
import static frc.robot.Constants.ElevatorConstants.LOW_ALGAE_HEIGHT;
import static frc.robot.Constants.ElevatorConstants.POS_TELEOP_CONSTRAINTS;
import static frc.robot.Constants.ElevatorConstants.REST_HEIGHT;
import static frc.robot.Constants.TikiTorchArmConstants.ALGAE_CLEAR;
import static frc.robot.Constants.TikiTorchArmConstants.INTAKE;
import static frc.robot.Constants.TikiTorchArmConstants.L1_SCORE;
import static frc.robot.Constants.TikiTorchArmConstants.L2_SCORE;
import static frc.robot.Constants.TikiTorchArmConstants.L3_SCORE;
import static frc.robot.Constants.TikiTorchArmConstants.L4_SCORE;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.subsystems.AlgaeArm;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.TikiTorchArm;

/**
 * Moves the whole end effector to a pose: elevator height plus both arm angles.
 *
 * <p>The ordering matters and is copied exactly from the Worlds code. Both arms are swept to a
 * clearance pose BEFORE the elevator moves, held there for 0.5 s, and held again for 0.75 s while
 * the elevator is travelling. Only then do the arms go to their real setpoints. That is what stops
 * the arms from colliding with the superstructure as the carriage passes. Do not flatten these
 * stages into one parallel group.
 */
public class SetEndeffector extends SequentialCommandGroup {

  public SetEndeffector(
      Elevator elevator,
      TikiTorchArm tikiTorchArm,
      AlgaeArm algaeArm,
      double elevatorPos,
      double tikiArmPos,
      double algaeArmPos) {
    this(
        elevator,
        tikiTorchArm,
        algaeArm,
        elevatorPos,
        tikiArmPos,
        algaeArmPos,
        Double.NaN,
        Double.NaN);
  }

  /**
   * Same sequence, with an explicit elevator velocity and acceleration. Pass NaN for either to use
   * the default profile.
   */
  public SetEndeffector(
      Elevator elevator,
      TikiTorchArm tikiTorchArm,
      AlgaeArm algaeArm,
      double elevatorPos,
      double tikiArmPos,
      double algaeArmPos,
      double elevatorSpeed,
      double elevatorAcceleration) {

    Command elevate =
        Double.isNaN(elevatorSpeed)
            ? new Elevate(elevator, elevatorPos)
            : new Elevate(elevator, elevatorPos, elevatorSpeed, elevatorAcceleration);

    addCommands(
        new ParallelCommandGroup(
                SetTikiTorchArm.algaeClear(tikiTorchArm), SetAlgaeArm.rest(algaeArm))
            .withTimeout(0.5),
        new ParallelCommandGroup(
            elevate,
            new SequentialCommandGroup(
                new ParallelCommandGroup(
                        SetTikiTorchArm.algaeClear(tikiTorchArm), SetAlgaeArm.rest(algaeArm))
                    .withTimeout(0.75),
                new ParallelCommandGroup(
                    new SetAlgaeArm(algaeArm, algaeArmPos),
                    new SetTikiTorchArm(tikiTorchArm, tikiArmPos)))));
  }

  public static Command rest(Elevator e, TikiTorchArm t, AlgaeArm a) {
    return new SetEndeffector(
        e, t, a, REST_HEIGHT, ALGAE_CLEAR, frc.robot.Constants.AlgaeArmConstants.REST);
  }

  public static Command l1(Elevator e, TikiTorchArm t, AlgaeArm a) {
    return new SetEndeffector(
        e, t, a, L1_HEIGHT, L1_SCORE, frc.robot.Constants.AlgaeArmConstants.REST);
  }

  public static Command l2(Elevator e, TikiTorchArm t, AlgaeArm a) {
    return new SetEndeffector(
        e, t, a, L2_HEIGHT, L2_SCORE, frc.robot.Constants.AlgaeArmConstants.REST);
  }

  public static Command l3(Elevator e, TikiTorchArm t, AlgaeArm a) {
    return new SetEndeffector(
        e, t, a, L3_HEIGHT, L3_SCORE, frc.robot.Constants.AlgaeArmConstants.REST);
  }

  /** L4 on the faster teleop profile. This is what the operator D-pad up runs. */
  public static Command teleopL4(Elevator e, TikiTorchArm t, AlgaeArm a) {
    return new SetEndeffector(
        e,
        t,
        a,
        L4_HEIGHT,
        L4_SCORE,
        frc.robot.Constants.AlgaeArmConstants.REST,
        POS_TELEOP_CONSTRAINTS.maxVelocity,
        POS_TELEOP_CONSTRAINTS.maxAcceleration);
  }

  public static Command lowAlgae(Elevator e, TikiTorchArm t, AlgaeArm a) {
    return new SetEndeffector(e, t, a, LOW_ALGAE_HEIGHT, ALGAE_CLEAR, REEF_INTAKE);
  }

  public static Command highAlgae(Elevator e, TikiTorchArm t, AlgaeArm a) {
    return new SetEndeffector(e, t, a, HIGH_ALGAE_HEIGHT, ALGAE_CLEAR, REEF_INTAKE);
  }

  public static Command barge(Elevator e, TikiTorchArm t, AlgaeArm a) {
    return new SetEndeffector(e, t, a, BARGE_HEIGHT, ALGAE_CLEAR, BARGE_SCORE);
  }

  public static Command floor(Elevator e, TikiTorchArm t, AlgaeArm a) {
    return new SetEndeffector(e, t, a, FLOOR_HEIGHT, ALGAE_CLEAR, FLOOR_INTAKE);
  }

  /**
   * Coral intake. Drives the carriage all the way down first so the lower limit switch re-zeroes
   * the rotation count, then comes up to the intake height. The 1 s stage is what gives the
   * carriage time to reach the switch.
   */
  public static Command intake(Elevator e, TikiTorchArm t, AlgaeArm a) {
    return new SequentialCommandGroup(
        new Elevate(e, LOWER_LIMIT_SWITCH_HEIGHT).withTimeout(1.0),
        new SetEndeffector(
            e, t, a, INTAKE_HEIGHT, INTAKE, frc.robot.Constants.AlgaeArmConstants.REST));
  }
}
