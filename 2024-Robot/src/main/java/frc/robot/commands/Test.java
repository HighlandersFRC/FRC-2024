package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber;

public class Test extends Command {
  private Climber climber;
  private double param1;
  private boolean param2;

  public Test(Climber climber, double param1, boolean param2) {
    this.climber = climber;
    this.param1 = param1;
    this.param2 = param2;
    addRequirements(this.climber);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    // System.out.println("Running Test");
    if (param2){
      this.climber.setCarriageRotationDegrees(param1);
    } else {
      this.climber.setCarriageRotationPercent(param1);
    }
    
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
