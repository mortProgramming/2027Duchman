package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
public class MoveAlgaeArm extends Command {
  private final double speed;
  private final frc.robot.subsystems.AlgaeArm algaeArm;

  public MoveAlgaeArm(frc.robot.subsystems.AlgaeArm algaeArm, double speed) {
    this.algaeArm = algaeArm;
    this.speed = speed;
    addRequirements(algaeArm);
  }

  public void initialize() {

  }
  public void execute() {
    algaeArm.setArmSpeed(speed);
  }

  public boolean isFinished() {
    return false;
  }

  public void end(boolean interrupted) {
    algaeArm.setArmSpeed(0);
  }
    
}
