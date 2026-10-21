package frc.robot.subsystems;

import static frc.robot.Constants.AlgaeRollerConstants.MOTOR_ID;
import static frc.robot.Constants.ROBOT_VOLTAGE;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Algae intake roller: a single NEO 550. Open loop.
 *
 * <p>Speed is held in a field and applied every cycle rather than set directly, matching the Worlds
 * behaviour: commands set a speed and the subsystem keeps applying it until told otherwise. Note
 * that "rest" is a light negative hold, not zero — it keeps a game piece gripped.
 */
public class AlgaeRoller extends SubsystemBase {
  private final SparkMax motor = new SparkMax(MOTOR_ID, MotorType.kBrushless);

  private double rollerSpeed = 0;

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
