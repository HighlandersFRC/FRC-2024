package frc.robot.commands;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.OI;
import frc.robot.sensors.TOF;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Lights;

public class SmartIntake extends Command {
  private Intake intake;
  private Feeder feeder;
  private Lights lights;
  private TOF tof;
  private double intakeDegrees;
  private double intakeRPM;
  private double feederRPM;

  private boolean rumbleControllers = true;

  public SmartIntake(Intake intake, Feeder feeder, Lights lights, TOF tof, Constants.SetPoints.IntakePosition intakePosition, double intakeRPM, double feederRPM) {
    this.intake = intake;
    this.feeder = feeder;
    this.lights = lights;
    this.tof = tof;
    this.intakeDegrees = intakePosition.degrees;
    this.intakeRPM = intakeRPM;
    this.feederRPM = feederRPM;
    addRequirements(this.intake);
  }

  public SmartIntake(Intake intake, Feeder feeder, Lights lights, TOF tof, Constants.SetPoints.IntakePosition intakePosition, double intakeRPM, double feederRPM, boolean rumble) {
    this.intake = intake;
    this.feeder = feeder;
    this.lights = lights;
    this.tof = tof;
    this.intakeDegrees = intakePosition.degrees;
    this.intakeRPM = intakeRPM;
    this.feederRPM = feederRPM;
    this.rumbleControllers = rumble;
    addRequirements(this.intake);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    this.intake.set(this.intakeDegrees, this.intakeRPM);
    if (this.tof.getFeederDistMillimeters() <= Constants.SetPoints.FEEDER_TOF_THRESHOLD_MM){
      this.feeder.set(0);
      if (this.rumbleControllers){
        OI.driverController.setRumble(RumbleType.kBothRumble, 0.7);
        OI.operatorController.setRumble(RumbleType.kBothRumble, 0.7);
      }
    } else {
      this.feeder.set(this.feederRPM);
    }
  }

  @Override
  public void end(boolean interrupted) {
    this.feeder.set(0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
