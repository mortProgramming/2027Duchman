package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;

public class SpeedTikiTorch extends Command {
  private final double speed;
  private final frc.robot.subsystems.TikiTorchArm tikiTorchArm;

  public SpeedTikiTorch(frc.robot.subsystems.TikiTorchArm tikiTorchArm, double speed) {
    this.tikiTorchArm = tikiTorchArm;
    this.speed = speed;
    addRequirements(tikiTorchArm);
  }

  @Override
  public void initialize() {

  }
  public void execute() {
    tikiTorchArm.setArmSpeed(speed);
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    tikiTorchArm.setArmSpeed(0);
  }
    
}
