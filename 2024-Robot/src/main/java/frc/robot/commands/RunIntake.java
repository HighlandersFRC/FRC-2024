package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Intake;

public class RunIntake extends Command {
  Intake intake;
  double degrees;
  double RPM;

  public RunIntake(Intake intake, double RPM) {
    this.intake = intake;
    this.degrees = Constants.SetPoints.INTAKE_DOWN_ANGLE_DEG;
    this.RPM = RPM;
    addRequirements(this.intake);
  }

  public RunIntake(Intake intake, double degrees, double RPM){
    this.intake = intake;
    this.degrees = degrees;
    this.RPM = RPM;
    addRequirements(this.intake);
  }

  public RunIntake(Intake intake, Constants.SetPoints.IntakePosition setpoint, double RPM){
    this.intake = intake;
    this.degrees = setpoint.degrees;
    this.RPM = RPM;
    addRequirements(this.intake);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    // this.intake.set(this.degrees, this.RPM);
    this.intake.setRollers(RPM);
  }

  @Override
  public void end(boolean interrupted) {
    this.intake.setRollerPercent(0.0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
