package frc.robot.subsystems;

import static frc.robot.Constants.AlgaeArmConstants.ARM_ROT_CONSTRAINTS;
import static frc.robot.Constants.AlgaeArmConstants.MOTOR_ID;
import static frc.robot.Constants.AlgaeArmConstants.OFFSET;
import static frc.robot.Constants.AlgaeArmConstants.ROT_KA;
import static frc.robot.Constants.AlgaeArmConstants.ROT_KD;
import static frc.robot.Constants.AlgaeArmConstants.ROT_KG;
import static frc.robot.Constants.AlgaeArmConstants.ROT_KI;
import static frc.robot.Constants.AlgaeArmConstants.ROT_KP;
import static frc.robot.Constants.AlgaeArmConstants.ROT_KS;
import static frc.robot.Constants.AlgaeArmConstants.ROT_KV;
import static frc.robot.Constants.AlgaeArmConstants.WRAP_LOWER;
import static frc.robot.Constants.AlgaeArmConstants.WRAP_UPPER;
import static frc.robot.Constants.ROBOT_VOLTAGE;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Algae arm: one NEO 550 with a duty-cycle absolute encoder plugged into the SparkMax data port.
 *
 * <p>The encoder is mounted 1:1 on the pivot, so rotations map straight to degrees with no gear
 * ratio. The closed loop runs on the RIO, not the SparkMax: a command calls
 * {@link #getController()}, computes against {@link #getDegrees()}, and feeds the result into
 * {@link #setArmPercent}. The PID output is in "percent per degree of error", which is why KP is
 * so small.
 *
 * <p>Ported verbatim from the Worlds code, including two things worth knowing. The sign flip in
 * setArmPercent is load-bearing. And WRAP_UPPER/WRAP_LOWER are not soft limits despite reading
 * like them — they define an unreachable band used to unwrap the absolute encoder, so this arm has
 * no travel protection in software. It did not at Worlds either.
 */
public class AlgaeArm extends SubsystemBase {
  private final SparkMax motor = new SparkMax(MOTOR_ID, MotorType.kBrushless);

  private final ProfiledPIDController controller =
      new ProfiledPIDController(ROT_KP, ROT_KI, ROT_KD, ARM_ROT_CONSTRAINTS);

  /** All four gains are zero today, so this contributes nothing. Wired up for when they are not. */
  private final ArmFeedforward feedforward = new ArmFeedforward(ROT_KS, ROT_KG, ROT_KV, ROT_KA);

  private double armSpeed = 0;

  public AlgaeArm() {
    // The 2025 code reset every SPARK to safe defaults on each boot, inside its motor wrapper.
    // Without it the controller keeps whatever is burned in flash, including the absolute encoder
    // inverted flag and conversion factor, so readings differ from what the Worlds code saw.
    SparkMaxConfig config = new SparkMaxConfig();
    config.closedLoop.pid(0, 0, 0);
    motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  @Override
  public void periodic() {
    motor.setVoltage(armSpeed * ROBOT_VOLTAGE);

    SmartDashboard.putNumber("AlgaeArm/Degrees", getDegrees());
    SmartDashboard.putNumber("AlgaeArm/VelocityDegPerSec", getVelocityDegrees());
  }

  /**
   * Commands the arm. The argument is negated, and the feedforward subtracted, exactly as at
   * Worlds. Note the feedforward is handed position in radians but velocity in degrees/sec — a
   * unit mismatch that is harmless only because KV is zero. Fix the units before tuning KV.
   */
  public void setArmPercent(double percent) {
    this.armSpeed =
        -percent - feedforward.calculate(Math.toRadians(getDegrees()), getVelocityDegrees());
  }

  /** Arm angle in degrees, with the absolute encoder unwrapped through the dead band. */
  public double getDegrees() {
    double degrees = getEncoderRotations() * 360 + OFFSET;

    if (degrees < WRAP_UPPER && degrees > WRAP_LOWER) {
      degrees += 360;
    }

    return degrees;
  }

  public double getEncoderRotations() {
    return 1 - motor.getAbsoluteEncoder().getPosition();
  }

  public double getVelocityDegrees() {
    return motor.getAbsoluteEncoder().getVelocity() * 360;
  }

  public ProfiledPIDController getController() {
    return controller;
  }
}
