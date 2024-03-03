package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Intake;

public class RunClimber extends Command {
  private Climber climber;
  private Intake intake;
  private double current;
  private double maxPercent;
  private boolean intakeIsClear = false;

  public RunClimber(Climber climber, Intake intake, double current, double maxPercent) {
    this.climber = climber;
    this.intake = intake;
    this.current = current;
    this.maxPercent = maxPercent;
    addRequirements(this.climber);
  }

  @Override
  public void initialize() {
    this.intakeIsClear = false;
  }

  @Override
  public void execute() {
    this.climber.setTrapRollerPercent(0);
    // this.intake.setAngle(Constants.SetPoints.IntakePosition.kDOWN);

    // if (this.intake.getAngleRotations() < -0.1){
    //   this.intakeIsClear = true;
    // }

    // if (this.intakeIsClear){
      this.climber.setElevatorTorque(this.current, this.maxPercent);
    // } else {
      // this.climber.setElevatorTorque(-5, 0.1);
    // }
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
