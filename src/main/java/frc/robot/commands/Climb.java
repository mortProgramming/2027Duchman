package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.Climber;

/**
 * Extends or retracts the deep-climb latch.
 *
 * <p>Finishes immediately — the climber subsystem holds the commanded piston state from then on,
 * so this does not need to stay scheduled. Matches Worlds.
 */
public class Climb extends Command {
  private final Climber climber;
  private final boolean climberUp;

  public Climb(Climber climber, boolean climberUp) {
    this.climber = climber;
    this.climberUp = climberUp;
    addRequirements(climber);
  }

  @Override
  public void initialize() {
    climber.setClimberPosition(climberUp);
  }

  @Override
  public boolean isFinished() {
    return true;
  }

  public static Command up(Climber climber) {
    return new Climb(climber, true);
  }

  public static Command down(Climber climber) {
    return new Climb(climber, false);
  }
}
