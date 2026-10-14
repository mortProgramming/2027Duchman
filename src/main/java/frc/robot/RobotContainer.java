package frc.robot;

import static frc.robot.Constants.DrivetrainConstants.MAX_SPEED_MPS;
import static frc.robot.Constants.OperatorConstants.DRIVER_CONTROLLER_PORT;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.commands.TeleopSwerve;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

/**
 * Binds hardware to commands. This is a drivetrain-only build — Dutchman's elevator, algae arm,
 * tiki torch, climber, lights and vision have not been ported over from the 2025 project yet. When
 * they are, instantiate them here the same way as {@link #drivetrain} and add their bindings in
 * {@link #configureBindings()}.
 *
 * <p>The drivetrain is built by {@link TunerConstants#createDrivetrain()} rather than constructed
 * directly, because Tuner X owns the per-module constants and generates that factory alongside
 * them.
 *
 * <p>Auto chooser uses {@code SendableChooser} here, not EVE's {@code Selectable}: Selectable is
 * the WPILib 2027 alpha replacement and does not exist on 2026, which is what this roboRIO runs.
 */
public class RobotContainer {
  public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

  private final CommandXboxController driverController =
      new CommandXboxController(DRIVER_CONTROLLER_PORT);

  private final Telemetry logger = new Telemetry(MAX_SPEED_MPS);
  private final SendableChooser<Command> autoChooser = new SendableChooser<>();

  /* Diagnostic requests — held, not latched. */
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
  private final SwerveRequest.Idle idle = new SwerveRequest.Idle();

  public RobotContainer() {
    drivetrain.setDefaultCommand(
        new TeleopSwerve(
            drivetrain,
            () -> -driverController.getLeftY(),
            () -> -driverController.getLeftX(),
            () -> -driverController.getRightX(),
            driverController.leftBumper()));

    configureBindings();

    autoChooser.setDefaultOption("Do Nothing", new InstantCommand());
    autoChooser.addOption("Drive Forward 5s", driveForwardAuto());
    SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  private void configureBindings() {
    // Hold to drive robot-relative; released, driving is field-relative. Consumed by TeleopSwerve.
    // Right bumper rather than EVE's menu() button re-zeroes the heading, because back/start are
    // taken by the SysId combos below and a bare start() binding would fire alongside them.
    driverController.rightBumper().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

    driverController.a().whileTrue(drivetrain.applyRequest(() -> brake));
    driverController
        .b()
        .whileTrue(
            drivetrain.applyRequest(
                () ->
                    point.withModuleDirection(
                        new Rotation2d(
                            -driverController.getLeftY(), -driverController.getLeftX()))));

    // Hold the configured neutral mode while disabled instead of coasting freely.
    RobotModeTriggers.disabled()
        .whileTrue(drivetrain.applyRequest(() -> idle).ignoringDisable(true));

    // SysId characterization, two-button combos so they cannot be hit by accident. Worth running:
    // a translation sweep measures this robot's real drive ratio, which is the open question in
    // Constants. Each routine should be run exactly once per log.
    driverController.back().and(driverController.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
    driverController.back().and(driverController.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
    driverController.start().and(driverController.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
    driverController.start().and(driverController.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

    drivetrain.registerTelemetry(logger::telemeterize);
  }

  /** Faces away from the alliance wall, creeps forward for 5 seconds, then holds. */
  private Command driveForwardAuto() {
    SwerveRequest.RobotCentric forward = new SwerveRequest.RobotCentric();
    return Commands.sequence(
        drivetrain.runOnce(() -> drivetrain.seedFieldCentric(Rotation2d.kZero)),
        drivetrain
            .applyRequest(() -> forward.withVelocityX(0.5).withVelocityY(0).withRotationalRate(0))
            .withTimeout(5.0),
        drivetrain.applyRequest(() -> idle));
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
