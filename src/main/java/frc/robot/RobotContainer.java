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

import frc.robot.commands.SetTikiTorchArm;
import frc.robot.commands.MoveTikiTorchArm;
import frc.robot.commands.Climb;
import frc.robot.commands.Elevate;
import frc.robot.commands.MoveAlgaeArm;
import frc.robot.commands.MoveElevator;
import frc.robot.commands.SetAlgaeArm;
import frc.robot.commands.SetTikiTorchArm;
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

  /**
   * BRING-UP LAYOUT. One subsystem per control, nothing composite.
   *
   * <p>The SetEndeffector poses are deliberately unbound for now. They move three subsystems at
   * once through a staged sequence, which is exactly what you do not want while working out
   * whether an individual mechanism is wired and tuned correctly — a failure anywhere looks like a
   * failure everywhere. SetEndeffector itself is untouched and still in the tree; restoring the
   * competition layout is just re-adding the D-pad and face-button bindings.
   *
   * <p>Each mechanism gets both halves of the test: an open-loop jog to prove direction and
   * wiring, and a closed-loop setpoint to prove the PID and the encoder together. Run the jog
   * first — if the jog moves the wrong way, the setpoint will run away.
   */
  private void configureOperatorBindings() {
    // ---- Open loop: direction and wiring ----
    // Arms on the sticks, analog so you can ease toward a hard stop. Bound as deflection triggers
    // rather than default commands so a jog always takes over, whatever else is holding the arm.
    new Trigger(() -> Math.abs(operatorController.getLeftY()) > DRIVE_DEADBAND)
        .whileTrue(new MoveAlgaeArm(algaeArm, () -> -operatorController.getLeftY()));
    new Trigger(() -> Math.abs(operatorController.getRightY()) > DRIVE_DEADBAND)
        .whileTrue(new MoveTikiTorchArm(tikiTorchArm, () -> -operatorController.getRightY()));

    // Elevator on the D-pad, fixed speed. No travel protection — watch the carriage.
    //
    // Negative raises the carriage on this mechanism, confirmed on the robot. The sign lives here
    // rather than inside MoveElevator or the subsystem on purpose: the closed-loop path through
    // Elevate is already correct, so inverting any deeper would break what works.
    operatorController.povUp().whileTrue(new MoveElevator(elevator, -JOG_SPEED));
    operatorController.povDown().whileTrue(new MoveElevator(elevator, JOG_SPEED));

    // Rollers. Triggers and bumpers, held.
    operatorController.rightTrigger(0.25).whileTrue(VelocityTikiTorchRoller.intake(tikiTorchRoller));
    operatorController.rightBumper().whileTrue(VelocityTikiTorchRoller.outtake(tikiTorchRoller));
    operatorController.leftTrigger(0.25).whileTrue(VelocityAlgaeRoller.outtake(algaeRoller));
    operatorController.leftBumper().whileTrue(VelocityAlgaeRoller.intake(algaeRoller));

    // ---- Closed loop: one subsystem at a time ----
    // Each of these drives exactly one mechanism to one setpoint and holds it, so an overshoot or
    // a runaway is attributable to that mechanism alone.
    operatorController.a().whileTrue(Elevate.rest(elevator));
    operatorController.b().whileTrue(Elevate.l2(elevator));
    operatorController.x().whileTrue(SetAlgaeArm.rest(algaeArm));
    operatorController.y().whileTrue(SetTikiTorchArm.algaeClear(tikiTorchArm));

    // whileTrue, not onTrue, on purpose: releasing the button drops the mechanism rather than
    // leaving it holding a setpoint you may not want it chasing.
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
