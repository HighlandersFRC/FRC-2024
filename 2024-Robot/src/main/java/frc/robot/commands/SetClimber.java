package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Climber;

public class SetClimber extends Command {
  private Climber climber;
  private double positionMeters;
  private double servoDegrees;

  public SetClimber(Climber climber, double positionMeters, double servoDegrees) {
    this.climber = climber;
    this.positionMeters = positionMeters;
    this.servoDegrees = servoDegrees;
    addRequirements(this.climber);
  }

  public SetClimber(Climber climber, Constants.SetPoints.ElevatorPosition position, double servoDegrees) {
    this.climber = climber;
    this.positionMeters = position.meters;
    this.servoDegrees = servoDegrees;
    addRequirements(this.climber);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    this.climber.setElevatorPositionMeters(this.positionMeters);
    this.climber.setTrapRollerPercent(0);
    this.climber.setTrapServoDegrees(this.servoDegrees);
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
