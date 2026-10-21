package frc.robot.subsystems;

import static frc.robot.Constants.ClimberConstants.PCM_CAN_ID;
import static frc.robot.Constants.ClimberConstants.SOLENOID_DOWN;
import static frc.robot.Constants.ClimberConstants.SOLENOID_UP;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Deep-climb latch: a double solenoid that grabs the cage, plus the compressor.
 *
 * <p>Pneumatic, not a motor — which is why it has no CAN motor ID. The piston state is held in a
 * field and re-applied every cycle, exactly as at Worlds.
 *
 * <p>The Worlds source carries a "module type might be wrong?" note against CTREPCM. It climbed on
 * CTREPCM, so that is what this uses — but if the pistons never fire, the module type and the PCM
 * CAN ID (32, whose comment in the old code contradicted its value) are the first things to check.
 */
public class Climber extends SubsystemBase {
  private final DoubleSolenoid piston =
      new DoubleSolenoid(PCM_CAN_ID, PneumaticsModuleType.CTREPCM, SOLENOID_UP, SOLENOID_DOWN);
  private final Compressor compressor = new Compressor(PCM_CAN_ID, PneumaticsModuleType.CTREPCM);

  /** true = extended/latched, false = retracted. */
  private boolean climberUp = false;

  public Climber() {
    piston.set(DoubleSolenoid.Value.kReverse);
  }

  @Override
  public void periodic() {
    piston.set(climberUp ? DoubleSolenoid.Value.kForward : DoubleSolenoid.Value.kReverse);
  }

  public void setClimberPosition(boolean climberUp) {
    this.climberUp = climberUp;
  }

  public boolean getPosition() {
    return climberUp;
  }

  public double getPressure() {
    return compressor.getPressure();
  }
}
