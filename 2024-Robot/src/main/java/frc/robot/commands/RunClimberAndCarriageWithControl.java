package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.OI;
import frc.robot.subsystems.Climber;

public class RunClimberAndCarriageWithControl extends Command {
  private Climber climber;
  private double degrees;
  private double trapRollerCurrent;
  private double trapRollerMaxPercent;
  private double elevatorCurrent;
  private double elevatorMaxPercent;

  private boolean hitLimit;
  private int numTimesHitLimit;

  public RunClimberAndCarriageWithControl(Climber climber, double degrees, double trapRollerCurrent, double trapRollerMaxPercent, double elevatorCurrent, double elevatorMaxPercent) {
    this.climber = climber;
    this.degrees = degrees;
    this.trapRollerCurrent = trapRollerCurrent;
    this.trapRollerMaxPercent = trapRollerMaxPercent;
    this.elevatorCurrent = elevatorCurrent;
    this.elevatorMaxPercent = elevatorMaxPercent;
    addRequirements(this.climber);
  }

  public RunClimberAndCarriageWithControl(Climber climber, Constants.SetPoints.CarriageRotation carriageRotation, double trapRollerCurrent, double trapRollerMaxPercent, double elevatorCurrent, double elevatorMaxPercent) {
    this.climber = climber;
    this.degrees = carriageRotation.degrees;
    this.trapRollerCurrent = trapRollerCurrent;
    this.trapRollerMaxPercent = trapRollerMaxPercent;
    this.elevatorCurrent = elevatorCurrent;
    this.elevatorMaxPercent = elevatorMaxPercent;
    addRequirements(this.climber);
  }

  @Override
  public void initialize() {
    this.hitLimit = false;
    this.numTimesHitLimit = 0;
  }

  @Override
  public void execute() {
    this.climber.setCarriageRotationDegrees(this.degrees);
    if (OI.getDriverRB()){
      this.climber.setTrapRollerTorque(20, 0.5);
    } else if (OI.getDriverLB()){
      this.climber.setTrapRollerTorque(-20, 0.1);
    } else {
      this.climber.setTrapRollerTorque(this.trapRollerCurrent, this.trapRollerMaxPercent);
    }

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
      this.climber.setElevatorTorque(this.elevatorCurrent, this.elevatorMaxPercent);
    }
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
