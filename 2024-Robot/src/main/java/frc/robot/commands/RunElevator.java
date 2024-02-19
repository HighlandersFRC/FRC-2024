package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;

public class RunClimber extends Command {
  Climber climber;
  double position;

  public RunClimber(Climber climber, double position) {
    this.climber = climber;
    this.position = position;
    addRequirements(climber);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    this.climber.setElevatorPercents(position);
  }

  @Override
  public void end(boolean interrupted) {
    this.climber.setElevatorPercents(0.0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
