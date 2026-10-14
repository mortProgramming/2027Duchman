package frc.robot.commands;

import static frc.robot.Constants.DrivetrainConstants.MAX_ANGULAR_RAD_PER_SEC;
import static frc.robot.Constants.DrivetrainConstants.MAX_SPEED_MPS;
import static frc.robot.Constants.OperatorConstants.DRIVE_DEADBAND;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.CommandSwerveDrivetrain;

/**
 * Default teleop driving command: joystick input -> field-relative (or robot-relative) swerve.
 *
 * <p>Same shape as EVE's TeleopSwerve — suppliers in, deadband and scaling applied here — but the
 * output goes to a CTRE {@link SwerveRequest} instead of a hand-rolled kinematics call, because
 * this robot's drivetrain is the Tuner X generated one.
 *
 * <p>Deadband is applied here and nowhere else. {@code SwerveRequest} can apply its own deadband
 * via {@code withDeadband()}; these requests deliberately do not, so there is one place to change
 * the feel of the controls.
 *
 * <p>Drive output is {@code OpenLoopVoltage}: the driver commands voltage directly rather than a
 * closed-loop velocity. That keeps driving honest while the drive gear ratio is still in dispute
 * (see the TODO in Constants) — an open-loop stick command does not depend on the ratio being
 * right, whereas a velocity setpoint does.
 */
public class TeleopSwerve extends Command {
  private final CommandSwerveDrivetrain drivetrain;
  private final DoubleSupplier translationX;
  private final DoubleSupplier translationY;
  private final DoubleSupplier rotation;
  private final BooleanSupplier robotRelative;

  private final SwerveRequest.FieldCentric fieldCentric =
      new SwerveRequest.FieldCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage);
  private final SwerveRequest.RobotCentric robotCentric =
      new SwerveRequest.RobotCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage);
  private final SwerveRequest.Idle idle = new SwerveRequest.Idle();

  public TeleopSwerve(
      CommandSwerveDrivetrain drivetrain,
      DoubleSupplier translationX,
      DoubleSupplier translationY,
      DoubleSupplier rotation,
      BooleanSupplier robotRelative) {
    this.drivetrain = drivetrain;
    this.translationX = translationX;
    this.translationY = translationY;
    this.rotation = rotation;
    this.robotRelative = robotRelative;
    addRequirements(drivetrain);
  }

  @Override
  public void execute() {
    // WPILib convention: X is forward, Y is to the left.
    double vx = MathUtil.applyDeadband(translationX.getAsDouble(), DRIVE_DEADBAND) * MAX_SPEED_MPS;
    double vy = MathUtil.applyDeadband(translationY.getAsDouble(), DRIVE_DEADBAND) * MAX_SPEED_MPS;
    double omega =
        MathUtil.applyDeadband(rotation.getAsDouble(), DRIVE_DEADBAND) * MAX_ANGULAR_RAD_PER_SEC;

    if (robotRelative.getAsBoolean()) {
      drivetrain.setControl(
          robotCentric.withVelocityX(vx).withVelocityY(vy).withRotationalRate(omega));
    } else {
      drivetrain.setControl(
          fieldCentric.withVelocityX(vx).withVelocityY(vy).withRotationalRate(omega));
    }
  }

  @Override
  public void end(boolean interrupted) {
    drivetrain.setControl(idle);
  }
}
