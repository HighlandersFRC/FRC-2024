package frc.robot.commands.defaults;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber;

public class ClimberDefault extends Command {
  Climber climber;

  public ClimberDefault(Climber climber) {
    this.climber = climber;
    addRequirements(climber);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    this.climber.setElevatorPercents(0);
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
