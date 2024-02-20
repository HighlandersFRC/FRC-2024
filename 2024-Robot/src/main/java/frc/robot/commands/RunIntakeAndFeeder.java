package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;

public class RunIntakeAndFeeder extends Command {
  Intake intake;
  Feeder feeder;
  Climber climber;
  double degrees;
  double intakeRPM;
  double feederRPM;
  double carriageRPM;

  public RunIntakeAndFeeder(Intake intake, Feeder feeder, Climber climber, double intakeRPM, double feederRPM, double carriageRPM) {
    this.intake = intake;
    this.feeder = feeder;
    this.climber = climber;
    this.degrees = Constants.SetPoints.INTAKE_DOWN_ANGLE_DEG;
    this.intakeRPM = intakeRPM;
    this.feederRPM = feederRPM;
    this.carriageRPM = carriageRPM;
    addRequirements(this.intake, this.feeder);
  }

  public RunIntakeAndFeeder(Intake intake, Feeder feeder, Climber climber, double degrees, double intakeRPM, double feederRPM, double carriageRPM){
    this.intake = intake;
    this.feeder = feeder;
    this.climber = climber;
    this.degrees = degrees;
    this.intakeRPM = intakeRPM;
    this.feederRPM = feederRPM;
    this.carriageRPM = carriageRPM;
    addRequirements(this.intake, this.feeder);
  }

  public RunIntakeAndFeeder(Intake intake, Feeder feeder, Climber climber, Constants.SetPoints.IntakePosition setpoint, double intakeRPM, double feederRPM, double carriageRPM){
    this.intake = intake;
    this.feeder = feeder;
    this.climber = climber;
    this.degrees = setpoint.degrees;
    this.intakeRPM = intakeRPM;
    this.feederRPM = feederRPM;
    this.carriageRPM = carriageRPM;
    addRequirements(this.intake, this.feeder);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    this.intake.set(this.degrees, this.intakeRPM);
    this.feeder.set(this.feederRPM);
    this.climber.setTrapRollerPercent(carriageRPM);
  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
