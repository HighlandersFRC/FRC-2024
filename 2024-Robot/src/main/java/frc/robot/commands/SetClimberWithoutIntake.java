package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Intake;

public class SetClimberWithoutIntake extends Command {
  private Climber climber;
  private Intake intake;
  private double positionMeters;
  private double intakeDegrees;
  private boolean intakeIsClear = false;

  public SetClimberWithoutIntake(Climber climber, Intake intake, double positionMeters, Constants.SetPoints.IntakePosition intakePosition) {
    this.climber = climber;
    this.intake = intake;
    this.intakeDegrees = intakePosition.degrees;
    this.positionMeters = positionMeters;
    addRequirements(this.climber);
  }

  public SetClimberWithoutIntake(Climber climber, Intake intake, Constants.SetPoints.ElevatorPosition position, Constants.SetPoints.IntakePosition intakePosition) {
    this.climber = climber;
    this.intake = intake;
    this.intake = intake;
    this.intakeDegrees = intakePosition.degrees;
    this.positionMeters = position.meters;
    addRequirements(this.climber);
  }

  @Override
  public void initialize() {
    this.intakeIsClear = false;
  }

  @Override
  public void execute() {
    this.climber.setTrapRollerPercent(0);
    // this.intake.setAngle(intakeDegrees);

  //   if (this.intake.getAngleRotations() < -0.1){
  //     this.intakeIsClear = true;
  //   }

  //   if (this.intakeIsClear){
  //     this.climber.setElevatorPositionMeters(this.positionMeters);
  //   } else {
  //     this.climber.setElevatorTorque(-5, 0.1);
  //   }
    this.climber.setElevatorPositionMeters(positionMeters);
  }

  @Override
  public void end(boolean interrupted) {
    this.climber.setElevatorTorque(0.0, 0.0);
  }

  @Override
  public boolean isFinished() {
    if (positionMeters == climber.getElevatorPositionMeters()){
      return true;
    } else {
      return false;
    }
  }
}
