package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;

public class RunIntakeAndFeeder extends Command {
  Intake intake;
  Feeder feeder;
  double degrees;
  double intakeRPM;
  double feederRPM;

  public RunIntakeAndFeeder(Intake intake, Feeder feeder, double intakeRPM, double feederRPM) {
    this.intake = intake;
    this.feeder = feeder;
    this.degrees = Constants.SetPoints.INTAKE_DOWN_ANGLE_DEG;
    this.intakeRPM = intakeRPM;
    this.feederRPM = feederRPM;
    addRequirements(this.intake, this.feeder);
  }

  public RunIntakeAndFeeder(Intake intake, Feeder feeder, double degrees, double intakeRPM, double feederRPM){
    this.intake = intake;
    this.feeder = feeder;
    this.degrees = degrees;
    this.intakeRPM = intakeRPM;
    this.feederRPM = feederRPM;
    addRequirements(this.intake, this.feeder);
  }

  public RunIntakeAndFeeder(Intake intake, Feeder feeder, Constants.SetPoints.IntakePosition setpoint, double intakeRPM, double feederRPM){
    this.intake = intake;
    this.feeder = feeder;
    this.degrees = setpoint.degrees;
    this.intakeRPM = intakeRPM;
    this.feederRPM = feederRPM;
    addRequirements(this.intake, this.feeder);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    this.intake.set(this.degrees, this.intakeRPM);
    this.feeder.set(this.feederRPM);
  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
