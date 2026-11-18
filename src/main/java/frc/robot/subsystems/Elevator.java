package frc.robot.subsystems;

import static frc.robot.Constants.ElevatorConstants.ELEVATOR_OFFSET;
import static frc.robot.Constants.ElevatorConstants.ENCODER_DIO;
import static frc.robot.Constants.ElevatorConstants.LOWER_LIMIT_SWITCH_DIO;
import static frc.robot.Constants.ElevatorConstants.LOWER_LIMIT_SWITCH_HEIGHT;
import static frc.robot.Constants.ElevatorConstants.MAXIMUM_INCH_CHANGE;
import static frc.robot.Constants.ElevatorConstants.MOTOR_ID;
import static frc.robot.Constants.ElevatorConstants.POS_CONSTRAINTS;
import static frc.robot.Constants.ElevatorConstants.POS_KD;
import static frc.robot.Constants.ElevatorConstants.POS_KG;
import static frc.robot.Constants.ElevatorConstants.POS_KI;
import static frc.robot.Constants.ElevatorConstants.POS_KP;
import static frc.robot.Constants.ElevatorConstants.ROTATIONS_TO_INCHES;
import static frc.robot.Constants.ElevatorConstants.UPPER_LIMIT_SWITCH_DIO;
import static frc.robot.Constants.ElevatorConstants.UPPER_LIMIT_SWITCH_HEIGHT;
import static frc.robot.Constants.ROBOT_VOLTAGE;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Three-stage elevator: a Vortex on CAN 11, a REV through-bore absolute encoder on DIO 0, and
 * limit switches at both ends.
 *
 * <p>The through-bore is absolute over one turn but the elevator travels several turns, so height
 * is tracked by counting rotations: {@link #calculateElevatorPosition()} watches for a jump larger
 * than MAXIMUM_INCH_CHANGE in a single cycle and adjusts the rotation count when the encoder
 * wraps. Hitting either limit switch re-seeds that count, which is what keeps it honest across a
 * match.
 *
 * <p>Ported verbatim from the Worlds code. Two oddities are deliberately preserved rather than
 * cleaned up, because the mechanism was tuned around them:
 *
 * <ul>
 *   <li>POS_KG is added as a raw percent-output bias in {@link #setElevatorMotorPercent}. The
 *       Worlds code also built an ElevatorFeedforward and then never called it; that dead object
 *       is not recreated here. Do not "restore" it — the bias is already doing that job, in
 *       different units.
 *   <li>UPPER_LIMIT_SWITCH_HEIGHT is negative while every setpoint is positive, and the negation
 *       cancels against one inside {@link #setElevatorPosition}. It works. Re-deriving the signs
 *       to make them read consistently would change behaviour on a working robot.
 * </ul>
 */
public class Elevator extends SubsystemBase {
  private final SparkFlex motor = new SparkFlex(MOTOR_ID, MotorType.kBrushless);
  private final DutyCycleEncoder encoder = new DutyCycleEncoder(ENCODER_DIO);
  private final DigitalInput lowerLimitSwitch = new DigitalInput(LOWER_LIMIT_SWITCH_DIO);
  private final DigitalInput upperLimitSwitch = new DigitalInput(UPPER_LIMIT_SWITCH_DIO);

  private final ProfiledPIDController controller =
      new ProfiledPIDController(POS_KP, POS_KI, POS_KD, POS_CONSTRAINTS);

  private double motorSpeed = 0;
  private double elevatorPosition = 0;
  private double rotationsCompleted = ELEVATOR_OFFSET / ROTATIONS_TO_INCHES;

  public Elevator() {
    // Matches the 2025 boot-time reset. See the note in AlgaeArm.
    SparkFlexConfig config = new SparkFlexConfig();
    config.closedLoop.pid(0, 0, 0);
    motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  @Override
  public void periodic() {
    // The limit switches stop the motor driving further into either end (Nov 2026 review: before,
    // they only re-zeroed the height, and barge's 71.5 in target sits above the 69.4 in upper
    // switch, so the carriage kept pushing into the top). Negative output raises this carriage.
    double output = motorSpeed;
    if (getAtUpperLimitSwitch()) {
      output = Math.max(output, POS_KG); // up no harder than the standing gravity bias
    }
    if (getAtLowerLimitSwitch()) {
      output = Math.min(output, 0); // never drive down into the bottom
    }
    motor.setVoltage(output * ROBOT_VOLTAGE);

    elevatorPosition = calculateElevatorPosition();

    fixWithLimitSwitch();

    SmartDashboard.putNumber("Elevator/Height", getElevatorPositionInches());
    SmartDashboard.putBoolean("Elevator/LowerLimit", getAtLowerLimitSwitch());
    SmartDashboard.putBoolean("Elevator/UpperLimit", getAtUpperLimitSwitch());
  }

  /** Applies a percent output plus the standing POS_KG bias. See the class note on units. */
  public void setElevatorMotorPercent(double motorSpeed) {
    this.motorSpeed = motorSpeed + POS_KG;
  }

  /** Re-seeds the rotation count so the current encoder reading means {@code newPoseInches}. */
  public void setElevatorPosition(double newPoseInches) {
    rotationsCompleted -= (getElevatorPositionInches() + newPoseInches) / ROTATIONS_TO_INCHES;

    elevatorPosition =
        (getAbsoluteEncoderPositionRotations() + rotationsCompleted) * ROTATIONS_TO_INCHES;
  }

  /** Either limit switch is ground truth: hitting one corrects accumulated rotation drift. */
  public void fixWithLimitSwitch() {
    if (getAtLowerLimitSwitch()) {
      setElevatorPosition(LOWER_LIMIT_SWITCH_HEIGHT);
    }

    if (getAtUpperLimitSwitch()) {
      setElevatorPosition(UPPER_LIMIT_SWITCH_HEIGHT);
    }
  }

  /** Tracks encoder wrap-around so a multi-turn elevator reads continuously. */
  public double calculateElevatorPosition() {
    double inchesFound =
        (getAbsoluteEncoderPositionRotations() + rotationsCompleted) * ROTATIONS_TO_INCHES;

    if ((inchesFound - elevatorPosition) > MAXIMUM_INCH_CHANGE) {
      rotationsCompleted -= 1;
    }

    if ((elevatorPosition - inchesFound) > MAXIMUM_INCH_CHANGE) {
      rotationsCompleted += 1;
    }

    return (getAbsoluteEncoderPositionRotations() + rotationsCompleted) * ROTATIONS_TO_INCHES;
  }

  public double getElevatorPositionInches() {
    return elevatorPosition;
  }

  public double getAbsoluteEncoderPositionRotations() {
    return 1 - encoder.get();
  }

  /** Both switches are wired active-low, so the raw read is inverted. */
  public boolean getAtLowerLimitSwitch() {
    return !lowerLimitSwitch.get();
  }

  public boolean getAtUpperLimitSwitch() {
    return !upperLimitSwitch.get();
  }

  public ProfiledPIDController getController() {
    return controller;
  }
}
