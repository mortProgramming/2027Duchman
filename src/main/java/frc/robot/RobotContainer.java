package frc.robot;

import static frc.robot.Constants.DrivetrainConstants.AUTO_CREEP_SPEED_MPS;
import static frc.robot.Constants.DrivetrainConstants.AUTO_DRIVE_SECONDS;
import static frc.robot.Constants.DrivetrainConstants.MAX_SPEED_MPS;
import static frc.robot.Constants.OperatorConstants.DRIVER_CONTROLLER_PORT;
import static frc.robot.Constants.OperatorConstants.OPERATOR_CONTROLLER_PORT;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;

import frc.robot.commands.Climb;
import frc.robot.commands.SetEndeffector;
import frc.robot.commands.TeleopLights;
import frc.robot.commands.TeleopSwerve;
import frc.robot.commands.VelocityAlgaeRoller;
import frc.robot.commands.VelocityTikiTorchRoller;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.AlgaeArm;
import frc.robot.subsystems.AlgaeRoller;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Lights;
import frc.robot.subsystems.TikiTorchArm;
import frc.robot.subsystems.TikiTorchRoller;

/**
 * Binds hardware to commands.
 *
 * <p>Two controllers, two different classes, because they are two different physical pads. The
 * driver uses a PlayStation controller on port 0, so it needs CommandPS4Controller: read through
 * CommandXboxController, a PlayStation pad reports axis 4 as the R2 trigger, which rests at -1 and
 * drove the robot with nobody touching it. The operator uses an Xbox pad on port 1. Do not unify
 * these classes without swapping the hardware to match.
 *
 * <p>Subsystems are constructed here and passed into commands, rather than reached through
 * singletons the way the 2025 code did.
 */
public class RobotContainer {
  private final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

  private final Elevator elevator = new Elevator();
  private final AlgaeArm algaeArm = new AlgaeArm();
  private final AlgaeRoller algaeRoller = new AlgaeRoller();
  private final TikiTorchArm tikiTorchArm = new TikiTorchArm();
  private final TikiTorchRoller tikiTorchRoller = new TikiTorchRoller();
  private final Climber climber = new Climber();
  private final Lights lights = new Lights();

  private final CommandPS4Controller driverController =
      new CommandPS4Controller(DRIVER_CONTROLLER_PORT);
  private final CommandXboxController operatorController =
      new CommandXboxController(OPERATOR_CONTROLLER_PORT);

  private final Telemetry logger = new Telemetry(MAX_SPEED_MPS);
  private final SendableChooser<Command> autoChooser = new SendableChooser<>();

  /* Diagnostic requests, held rather than latched. */
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

    // Rollers idle to a light hold rather than zero, so a held game piece does not slip. The 2025
    // code did this with whileFalse(rest()) on every binding; a default command is the same
    // behaviour with one place to change it.
    algaeRoller.setDefaultCommand(VelocityAlgaeRoller.rest(algaeRoller));
    tikiTorchRoller.setDefaultCommand(VelocityTikiTorchRoller.rest(tikiTorchRoller));
    lights.setDefaultCommand(new TeleopLights(lights, elevator));

    configureDriverBindings();
    configureOperatorBindings();

    // EVERY option here must hold the drivetrain for the WHOLE autonomous period. An auto command
    // that finishes early releases the drivetrain while the robot is still enabled, and
    // CommandScheduler immediately re-schedules the default command, TeleopSwerve, which reads the
    // live sticks.
    autoChooser.setDefaultOption("Do Nothing", drivetrain.applyRequest(() -> idle));
    autoChooser.addOption("Drive Forward 5s", driveForwardAuto());
    SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  private void configureDriverBindings() {
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

    // Deep climb stays on the driver pad, as at Worlds. The person watching the cage runs it.
    driverController.povRight().onTrue(Climb.up(climber));
    driverController.povLeft().onTrue(Climb.down(climber));

    // Hold the configured neutral mode while disabled instead of coasting freely.
    RobotModeTriggers.disabled()
        .whileTrue(drivetrain.applyRequest(() -> idle).ignoringDisable(true));

    drivetrain.registerTelemetry(logger::telemeterize);
  }

  /** Operator layout preserved from the Worlds code, so nobody has to relearn it. */
  private void configureOperatorBindings() {
    // Rollers. Triggers and bumpers, held.
    operatorController.rightTrigger(0.25).whileTrue(VelocityTikiTorchRoller.intake(tikiTorchRoller));
    operatorController.rightBumper().whileTrue(VelocityTikiTorchRoller.outtake(tikiTorchRoller));
    operatorController.leftTrigger(0.25).whileTrue(VelocityAlgaeRoller.outtake(algaeRoller));
    operatorController.leftBumper().whileTrue(VelocityAlgaeRoller.intake(algaeRoller));

    // Coral scoring poses on the D-pad.
    operatorController.povUp().onTrue(SetEndeffector.teleopL4(elevator, tikiTorchArm, algaeArm));
    operatorController.povDown().onTrue(SetEndeffector.l3(elevator, tikiTorchArm, algaeArm));
    operatorController.povLeft().onTrue(SetEndeffector.l2(elevator, tikiTorchArm, algaeArm));
    operatorController.povRight().onTrue(SetEndeffector.rest(elevator, tikiTorchArm, algaeArm));

    // Face buttons and menu buttons.
    operatorController.x().onTrue(SetEndeffector.l1(elevator, tikiTorchArm, algaeArm));
    operatorController.a().onTrue(SetEndeffector.floor(elevator, tikiTorchArm, algaeArm));
    operatorController.b().onTrue(SetEndeffector.intake(elevator, tikiTorchArm, algaeArm));
    operatorController.y().onTrue(SetEndeffector.barge(elevator, tikiTorchArm, algaeArm));
    operatorController.back().onTrue(SetEndeffector.lowAlgae(elevator, tikiTorchArm, algaeArm));
    operatorController.start().onTrue(SetEndeffector.highAlgae(elevator, tikiTorchArm, algaeArm));
  }

  /** Faces away from the wall, creeps forward, then holds the drivetrain for the rest of auto. */
  private Command driveForwardAuto() {
    SwerveRequest.RobotCentric forward = new SwerveRequest.RobotCentric();
    return Commands.sequence(
        drivetrain.runOnce(() -> drivetrain.seedFieldCentric(Rotation2d.kZero)),
        drivetrain
            .applyRequest(
                () ->
                    forward
                        .withVelocityX(AUTO_CREEP_SPEED_MPS)
                        .withVelocityY(0)
                        .withRotationalRate(0))
            .withTimeout(AUTO_DRIVE_SECONDS),
        drivetrain.applyRequest(() -> idle));
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
