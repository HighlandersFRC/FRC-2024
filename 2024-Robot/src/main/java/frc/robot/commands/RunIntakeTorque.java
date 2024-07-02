package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

public class RunIntakeTorque extends Command {
  Intake intake;
  double angleCurrent;
  double angleMaxPercent;
  double RPM;

  public RunIntakeTorque(Intake intake, double angleCurrent, double angleMaxPercent, double RPM){
    this.intake = intake;
    this.angleCurrent = angleCurrent;
    this.angleMaxPercent = angleMaxPercent;
    this.RPM = RPM;
    addRequirements(this.intake);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    this.intake.setAngleTorqueCurrent(this.angleCurrent, this.angleMaxPercent);
    this.intake.setRollers(this.RPM);
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
