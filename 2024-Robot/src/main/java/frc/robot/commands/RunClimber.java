package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Climber;

public class RunClimber extends Command {
  private Climber climber;
  private double percent;
  private double servoDegrees;

  public RunClimber(Climber climber, double percent, double servoDegrees) {
    this.climber = climber;
    this.percent = percent;
    this.servoDegrees = servoDegrees;
    addRequirements(this.climber);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    this.climber.setElevatorPercent(this.percent);
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
