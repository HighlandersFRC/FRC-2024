package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Intake;

public class RunIntake extends Command {
  Intake intake;
  double RPM;

  public RunIntake(Intake intake, double RPM) {
    this.intake = intake;
    this.RPM = RPM;
    addRequirements(this.intake);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    this.intake.setIntake(Constants.SetPoints.IntakePosition.kDOWN, this.RPM);
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
