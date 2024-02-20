package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Intake;

public class SetClimber extends Command {
  private Climber climber;
  private Intake intake;
  private double positionMeters;
  private boolean intakeIsClear = false;

  public SetClimber(Climber climber, Intake intake, double positionMeters) {
    this.climber = climber;
    this.intake = intake;
    this.positionMeters = positionMeters;
    addRequirements(this.climber);
  }

  public SetClimber(Climber climber, Intake intake, Constants.SetPoints.ElevatorPosition position) {
    this.climber = climber;
    this.intake = intake;
    this.intake = intake;
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
    this.intake.setAngle(Constants.SetPoints.IntakePosition.kDOWN);

    if (this.intake.getAngleRotations() < -0.1){
      this.intakeIsClear = true;
    }

    if (this.intakeIsClear){
      this.climber.setElevatorPositionMeters(this.positionMeters);
    } else {
      this.climber.setElevatorTorque(-5, 0.1);
    }
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
