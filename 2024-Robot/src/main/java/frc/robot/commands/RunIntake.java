package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Intake;

public class RunIntake extends Command {
  Intake intake;
  double angle;
  double RPM;

  public RunIntake(Intake intake, double RPM) {
    this.intake = intake;
    this.angle = Constants.SetPoints.INTAKE_DOWN_ANGLE;
    this.RPM = RPM;
    addRequirements(this.intake);
  }

  public RunIntake(Intake intake, double angle, double RPM){
    this.intake = intake;
    this.angle = angle;
    this.RPM = RPM;
    addRequirements(this.intake);
  }

  public RunIntake(Intake intake, Constants.SetPoints.IntakePosition setpoint, double RPM){
    this.intake = intake;
    this.angle = setpoint.angle;
    this.RPM = RPM;
    addRequirements(this.intake);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    this.intake.setIntakePercent(RPM);
    System.out.println("INTAKE " + this.angle);
  }

  @Override
  public void end(boolean interrupted) {
    this.intake.setIntakePercent(0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
