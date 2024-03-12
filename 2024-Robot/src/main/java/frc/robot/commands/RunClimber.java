package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Intake;

public class RunClimber extends Command {
  private Climber climber;
  private double current;
  private double maxPercent;

  private boolean hitLimit;
  private int numTimesHitLimit;

  public RunClimber(Climber climber, double current, double maxPercent) {
    this.climber = climber;
    this.current = current;
    this.maxPercent = maxPercent;
    addRequirements(this.climber);
  }

  @Override
  public void initialize() {
    this.hitLimit = false;
    this.numTimesHitLimit = 0;
  }

  @Override
  public void execute() {
    this.climber.setTrapRollerPercent(0);
    this.climber.setCarriageRotation(Constants.SetPoints.CarriageRotation.kFEED);

    if (Math.abs(this.climber.getElevatorVelocityMPS()) < 0.01){
      this.numTimesHitLimit ++;
    }

    if (this.numTimesHitLimit > 2){
      this.hitLimit = true;
      this.numTimesHitLimit = 0;
    }
      
    if (this.hitLimit){
      this.climber.setElevatorTorque(0, 0);
    } else {
      this.climber.setElevatorTorque(this.current, this.maxPercent);
    }
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
