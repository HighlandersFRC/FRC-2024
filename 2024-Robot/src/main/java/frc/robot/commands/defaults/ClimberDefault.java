package frc.robot.commands.defaults;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber;

public class ClimberDefault extends Command {
  Climber climber;
  boolean isZeroed = false;
  int numTimesHitBottom = 0;

  public ClimberDefault(Climber climber) {
    this.climber = climber;
    addRequirements(climber);
  }

  @Override
  public void initialize() {
    this.isZeroed = false;
    this.numTimesHitBottom = 0;
  }

  @Override
  public void execute() {
    this.climber.setTrapRollerPercent(0);

    if (Math.abs(this.climber.getElevatorVelocityMPS()) < 0.01){
      this.numTimesHitBottom ++;
    }

    if (this.numTimesHitBottom > 2){
      this.isZeroed = true;
      this.numTimesHitBottom = 0;
    }

    if (Math.abs(this.climber.getElevatorPositionMeters()) > 0.05){
      this.isZeroed = false;
    }

    if (this.isZeroed){
      this.climber.setElevatorTorque(0, 0.1);
    } else {
      this.climber.setElevatorTorque(-15, 0.3);
    }
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}