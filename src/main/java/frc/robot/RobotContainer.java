package frc.robot;

import static frc.robot.Constants.DrivetrainConstants.AUTO_CREEP_SPEED_MPS;
import static frc.robot.Constants.DrivetrainConstants.AUTO_DRIVE_SECONDS;
import static frc.robot.Constants.DrivetrainConstants.MAX_SPEED_MPS;
import static frc.robot.Constants.OperatorConstants.DRIVER_CONTROLLER_PORT;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;

import frc.robot.commands.TeleopSwerve;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

/**
 * Binds hardware to commands. This is a drivetrain-only build — Dutchman's elevator, algae arm,
 * tiki torch, climber, lights and vision have not been ported over from the 2025 project yet. When
 * they are, instantiate them here the same way as {@link #drivetrain} and add their bindings in
 * {@link #configureBindings()}.
 *
 * <p><b>The driver pad is a PlayStation controller, so this uses {@link CommandPS4Controller}.</b>
 * This matters more than it looks. Read through CommandXboxController, a PlayStation pad reports
 * axis 4 as the R2 trigger, and PlayStation triggers rest at -1 rather than 0 — so an untouched
 * controller commanded full rotation continuously and the robot drove itself. Button indices shift
 * too: Xbox back/start are 7/8, which on a PlayStation pad are the L2/R2 buttons. PS4 and PS5 pads
 * share identical axis and button indices (only button 9's name differs, share vs create), so
 * CommandPS5Controller is a drop-in rename if that reads better for a DualSense.
 *
 * <p>If the driver pad is ever swapped back to an Xbox controller, change the class here AND the
 * button methods below — L1/R1/cross/circle are PlayStation names.
 */
public class RobotContainer {
  private final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

  private final CommandPS4Controller driverController =
      new CommandPS4Controller(DRIVER_CONTROLLER_PORT);

  private final Telemetry logger = new Telemetry(MAX_SPEED_MPS);
  private final SendableChooser<Command> autoChooser = new SendableChooser<>();

  /* Diagnostic requests — held, not latched. */
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
  private final SwerveRequest.Idle idle = new SwerveRequest.Idle();

  public RobotContainer() {
    // L1 held = robot-relative driving; released = field-relative.
    drivetrain.setDefaultCommand(
        new TeleopSwerve(
            drivetrain,
            () -> -driverController.getLeftY(),
            () -> -driverController.getLeftX(),
            () -> -driverController.getRightX(),
            driverController.L1()));

    configureBindings();

    // EVERY option here must hold the drivetrain for the WHOLE autonomous period. An auto command
    // that finishes early (an InstantCommand, or a sequence with no trailing hold) releases the
    // drivetrain while the robot is still enabled, and CommandScheduler immediately re-schedules
    // the default command — TeleopSwerve — which reads the live sticks. A hand resting on a
    // joystick would then drive the robot during auto. That is why "Do Nothing" idles the
    // drivetrain forever instead of being an InstantCommand.
    autoChooser.setDefaultOption("Do Nothing", drivetrain.applyRequest(() -> idle));
    autoChooser.addOption("Drive Forward 5s", driveForwardAuto());
    SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  private void configureBindings() {
    // R1 re-zeroes the field-centric heading.
    driverController.R1().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

    driverController.cross().whileTrue(drivetrain.applyRequest(() -> brake));
    driverController
        .circle()
        .whileTrue(
            drivetrain.applyRequest(
                () ->
                    point.withModuleDirection(
                        new Rotation2d(
                            -driverController.getLeftY(), -driverController.getLeftX()))));

    // Hold the configured neutral mode while disabled instead of coasting freely.
    RobotModeTriggers.disabled()
        .whileTrue(drivetrain.applyRequest(() -> idle).ignoringDisable(true));

    // NO SysId bindings on the driver controller, deliberately. They drive the robot under their
    // own power, and their two-button combos landed on L2/R2 once the pad was a PlayStation one.
    // The reason they were here — measuring the drive gear ratio — is settled now that we have
    // Jaden's measurements. If characterization is needed again, bind it on the operator
    // controller, not this one.

    drivetrain.registerTelemetry(logger::telemeterize);
  }

  /** Faces away from the alliance wall, creeps forward, then holds the drivetrain for the rest of auto. */
  private Command driveForwardAuto() {
    SwerveRequest.RobotCentric forward = new SwerveRequest.RobotCentric();
    return Commands.sequence(
        drivetrain.runOnce(() -> drivetrain.seedFieldCentric(Rotation2d.kZero)),
        drivetrain
            .applyRequest(
                () -> forward.withVelocityX(AUTO_CREEP_SPEED_MPS).withVelocityY(0).withRotationalRate(0))
            .withTimeout(AUTO_DRIVE_SECONDS),
        drivetrain.applyRequest(() -> idle));
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
