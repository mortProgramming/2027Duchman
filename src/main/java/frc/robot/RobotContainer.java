package frc.robot;

import static frc.robot.Constants.DrivetrainConstants.AUTO_CREEP_SPEED_MPS;
import static frc.robot.Constants.DrivetrainConstants.AUTO_DRIVE_SECONDS;
import static frc.robot.Constants.DrivetrainConstants.MAX_SPEED_MPS;
import static frc.robot.Constants.ElevatorConstants.JOG_SPEED;
import static frc.robot.Constants.OperatorConstants.DRIVER_CONTROLLER_PORT;
import static frc.robot.Constants.OperatorConstants.DRIVE_DEADBAND;
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
import edu.wpi.first.wpilibj2.command.button.Trigger;

import frc.robot.commands.MoveTikiTorchArm;
import frc.robot.commands.Climb;
import frc.robot.commands.MoveAlgaeArm;
import frc.robot.commands.MoveElevator;
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
import frc.robot.subsystems.Vision;

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
  private final Vision vision = new Vision(drivetrain);

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

  /**
   * COMPETITION LAYOUT. Matches the Worlds operator layout so nobody has to relearn it, with the
   * manual jogs kept on controls the competition layout leaves free.
   *
   * <p>Bring-up note: the per-subsystem closed-loop test bindings that used to sit on the face
   * buttons are gone, because the face buttons are scoring poses again. Every mechanism was
   * verified individually before this layout went back on. To test one subsystem in isolation
   * again, jog it from the sticks or the stick-press buttons rather than un-binding the poses.
   */
  private void configureOperatorBindings() {
    // ---- Scoring poses. Each moves elevator + both arms through the staged clearance sequence ----
    operatorController.povUp().onTrue(SetEndeffector.teleopL4(elevator, tikiTorchArm, algaeArm));
    operatorController.povDown().onTrue(SetEndeffector.l3(elevator, tikiTorchArm, algaeArm));
    operatorController.povLeft().onTrue(SetEndeffector.l2(elevator, tikiTorchArm, algaeArm));
    operatorController.povRight().onTrue(SetEndeffector.rest(elevator, tikiTorchArm, algaeArm));

    operatorController.x().onTrue(SetEndeffector.l1(elevator, tikiTorchArm, algaeArm));
    operatorController.a().onTrue(SetEndeffector.floor(elevator, tikiTorchArm, algaeArm));
    operatorController.b().onTrue(SetEndeffector.intake(elevator, tikiTorchArm, algaeArm));
    operatorController.y().onTrue(SetEndeffector.barge(elevator, tikiTorchArm, algaeArm));
    operatorController.back().onTrue(SetEndeffector.lowAlgae(elevator, tikiTorchArm, algaeArm));
    operatorController.start().onTrue(SetEndeffector.highAlgae(elevator, tikiTorchArm, algaeArm));

    // ---- Rollers. Triggers and bumpers, held ----
    operatorController.rightTrigger(0.25).whileTrue(VelocityTikiTorchRoller.intake(tikiTorchRoller));
    operatorController.rightBumper().whileTrue(VelocityTikiTorchRoller.outtake(tikiTorchRoller));
    operatorController.leftTrigger(0.25).whileTrue(VelocityAlgaeRoller.outtake(algaeRoller));
    operatorController.leftBumper().whileTrue(VelocityAlgaeRoller.intake(algaeRoller));

    // ---- Manual jogs, kept for recovery ----
    // These are what gets a mispositioned mechanism back without a redeploy, so they stay bound in
    // competition. They sit on controls the Worlds layout never used, so nothing is displaced.
    // All three interrupt whatever pose is holding, which is the point.
    new Trigger(() -> Math.abs(operatorController.getLeftY()) > DRIVE_DEADBAND)
        .whileTrue(new MoveAlgaeArm(algaeArm, () -> -operatorController.getLeftY()));
    new Trigger(() -> Math.abs(operatorController.getRightY()) > DRIVE_DEADBAND)
        .whileTrue(new MoveTikiTorchArm(tikiTorchArm, () -> -operatorController.getRightY()));

    // Elevator jog on the stick-press buttons, since the D-pad is scoring poses again. Negative
    // raises the carriage, confirmed on the robot.
    operatorController.rightStick().whileTrue(new MoveElevator(elevator, -JOG_SPEED));
    operatorController.leftStick().whileTrue(new MoveElevator(elevator, JOG_SPEED));
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
