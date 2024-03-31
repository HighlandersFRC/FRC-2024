package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Climber;

public class SetClimberWithoutIntake extends Command {
  private Climber climber;
  private double positionMeters;
  private double carriageDegrees;
  private boolean intakeIsClear = false;

  public SetClimberWithoutIntake(Climber climber, double positionMeters, double carriageDegrees) {
    this.climber = climber;
    this.positionMeters = positionMeters;
    this.carriageDegrees = carriageDegrees;
    addRequirements(this.climber);
  }

  public SetClimberWithoutIntake(Climber climber, Constants.SetPoints.ElevatorPosition position, double carriageDegrees) {
    this.climber = climber;
    this.positionMeters = position.meters;
    this.carriageDegrees = carriageDegrees;
    addRequirements(this.climber);
  }

  @Override
  public void initialize() {
    this.intakeIsClear = false;
  }

  @Override
  public void execute() {
    // System.out.println("amp preset");
    this.climber.setCarriageRotationDegrees(carriageDegrees);
    if (positionMeters < this.climber.getElevatorPositionMeters() + 0.02 && positionMeters > this.climber.getElevatorPositionMeters() - 0.02){
      this.climber.setElevatorTorque(0.0, 0.0);
    } else {
      if (positionMeters > this.climber.getElevatorPositionMeters()){
        this.climber.setElevatorTorque(20, 1.0);
      } else {
        this.climber.setElevatorTorque(-20, 1.0);
      }
    }
  }

  @Override
  public void end(boolean interrupted) {
    this.climber.setElevatorTorque(0.0, 0.0);
  }

  @Override
  public boolean isFinished() {
    if (this.positionMeters < this.climber.getElevatorPositionMeters() + 0.02 && this.positionMeters > this.climber.getElevatorPositionMeters() - 0.02){
      return true;
    } else {
      return false;
    }
  }
}