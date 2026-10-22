package frc.robot.subsystems;

import static frc.robot.Constants.ROBOT_VOLTAGE;
import static frc.robot.Constants.TikiTorchRollerConstants.MOTOR_ID;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Coral intake/scoring roller: a single NEO 550. Open loop.
 *
 * <p>Like the algae roller, "rest" is a light negative hold rather than zero so a held coral does
 * not slip out.
 */
public class TikiTorchRoller extends SubsystemBase {
  private final SparkMax motor = new SparkMax(MOTOR_ID, MotorType.kBrushless);

  private double rollerSpeed = 0;

  public TikiTorchRoller() {
    // The 2025 code reset every SPARK to safe defaults on each boot, inside its motor wrapper.
    // Without it the controller keeps whatever is burned in flash, including the absolute encoder
    // inverted flag and conversion factor, so readings differ from what the Worlds code saw.
    SparkMaxConfig config = new SparkMaxConfig();
    config.closedLoop.pid(0, 0, 0);
    motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  @Override
  public void periodic() {
    motor.setVoltage(rollerSpeed * ROBOT_VOLTAGE);
  }

  public void setRollerSpeed(double rollerSpeed) {
    this.rollerSpeed = rollerSpeed;
  }

  public double getRollerSpeed() {
    return rollerSpeed;
  }
}
