package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;

public class ExtendClimber extends Command {
  private Climber climber;
  private Intake intake;
  private Feeder feeder;

  private boolean intakeIsClear;

  public ExtendClimber(Climber climber, Intake intake, Feeder feeder) {
    this.climber = climber;
    this.intake = intake;
    this.feeder = feeder;
  }

  @Override
  public void initialize() {
    this.intakeIsClear = false;
  }

  @Override
  public void execute() {
    this.climber.setTrapRollerPercent(0);
    this.climber.setCarriageRotation(Constants.SetPoints.CarriageRotation.kDOWN);
    this.intake.setAngle(Constants.SetPoints.IntakePosition.kDOWN);

    if (this.intake.getAngleRotations() < -0.1){
      this.intakeIsClear = true;
    }

    if (this.intakeIsClear){
      this.climber.setElevatorPosition(Constants.SetPoints.ElevatorPosition.kFIRST_EXTEND);
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
