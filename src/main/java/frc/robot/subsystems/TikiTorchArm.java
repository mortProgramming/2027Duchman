package frc.robot.subsystems;

import static frc.robot.Constants.TikiTorchArmConstants.ARM_ROT_CONSTRAINTS;
import static frc.robot.Constants.TikiTorchArmConstants.MOTOR_ID;
import static frc.robot.Constants.TikiTorchArmConstants.OFFSET;
import static frc.robot.Constants.TikiTorchArmConstants.ROT_KA;
import static frc.robot.Constants.TikiTorchArmConstants.ROT_KD;
import static frc.robot.Constants.TikiTorchArmConstants.ROT_KG;
import static frc.robot.Constants.TikiTorchArmConstants.ROT_KI;
import static frc.robot.Constants.TikiTorchArmConstants.ROT_KP;
import static frc.robot.Constants.TikiTorchArmConstants.ROT_KS;
import static frc.robot.Constants.TikiTorchArmConstants.ROT_KV;
import static frc.robot.Constants.TikiTorchArmConstants.WRAP_LOWER;
import static frc.robot.Constants.TikiTorchArmConstants.WRAP_UPPER;

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
 * Tiki torch arm, the coral end effector: one NEO 550 with an absolute encoder on the data port.
 *
 * <p>Same RIO-side closed loop as the algae arm, with two differences preserved from Worlds. It
 * drives the motor with {@code set()} (raw duty cycle) rather than {@code setVoltage()}, so its
 * output is NOT battery-compensated. And its angle formula negates before adding the offset, which
 * is why its setpoints run the opposite direction from the algae arm's.
 *
 * <p>ROT_KG is the only non-zero arm feedforward gain in the project — negative, so gravity is
 * assisting rather than opposing over the working range.
 */
public class TikiTorchArm extends SubsystemBase {
  private final SparkMax motor = new SparkMax(MOTOR_ID, MotorType.kBrushless);

  private final ProfiledPIDController controller =
      new ProfiledPIDController(ROT_KP, ROT_KI, ROT_KD, ARM_ROT_CONSTRAINTS);

  private final ArmFeedforward feedforward = new ArmFeedforward(ROT_KS, ROT_KG, ROT_KV, ROT_KA);

  private double armSpeed = 0;

  public TikiTorchArm() {
    // The 2025 code reset every SPARK to safe defaults on each boot, inside its motor wrapper.
    // Without it the controller keeps whatever is burned in flash, including the absolute encoder
    // inverted flag and conversion factor, so readings differ from what the Worlds code saw.
    SparkMaxConfig config = new SparkMaxConfig();
    config.closedLoop.pid(0, 0, 0);
    motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  @Override
  public void periodic() {
    // set(), not setVoltage() — matches Worlds. Not battery-compensated.
    motor.set(armSpeed);

    SmartDashboard.putNumber("TikiTorchArm/Degrees", getDegrees());
    SmartDashboard.putNumber("TikiTorchArm/VelocityDegPerSec", getVelocityDegrees());
  }
  public void setArmSpeed(double speed) {
    this.armSpeed = speed;
  }

  public void setArmPercent(double percent) {
    this.armSpeed =
        -percent - feedforward.calculate(Math.toRadians(getDegrees()), getVelocityDegrees());
  }

  /** Arm angle in degrees. Note the negation before the offset — opposite sense to the algae arm. */
  public double getDegrees() {
    double degrees = getEncoderRotations() * 360;

    if (degrees < WRAP_UPPER && degrees > WRAP_LOWER) {
      degrees += 360;
    }

    return -degrees + OFFSET;
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
