package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.OI;
import frc.robot.sensors.TOF;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Lights;

public class SmartIntake extends Command {
  private Intake intake;
  private Feeder feeder;
  private Climber climber;
  private Lights lights;
  private TOF tof;
  private double intakeDegrees;
  private double intakeRPM;
  private double feederRPM;

  private boolean haveNote = false;
  private boolean haveIntakedNote = false;
  private double haveNoteTime = 0;

  private boolean rumbleControllers = true;

  public SmartIntake(Intake intake, Feeder feeder, Climber climber, Lights lights, TOF tof, Constants.SetPoints.IntakePosition intakePosition, double intakeRPM, double feederRPM) {
    this.intake = intake;
    this.feeder = feeder;
    this.climber = climber;
    this.lights = lights;
    this.tof = tof;
    this.intakeDegrees = intakePosition.degrees;
    this.intakeRPM = intakeRPM;
    this.feederRPM = feederRPM;
    addRequirements(this.intake, this.feeder, this.climber);
  }

  public SmartIntake(Intake intake, Feeder feeder, Climber climber, Lights lights, TOF tof, Constants.SetPoints.IntakePosition intakePosition, double intakeRPM, double feederRPM, boolean rumble) {
    this.intake = intake;
    this.feeder = feeder;
    this.lights = lights;
    this.climber = climber;
    this.tof = tof;
    this.intakeDegrees = intakePosition.degrees;
    this.intakeRPM = intakeRPM;
    this.feederRPM = feederRPM;
    this.rumbleControllers = rumble;
    addRequirements(this.intake, this.feeder, this.climber);
  }

  @Override
  public void initialize() {
    this.haveNote = false;
    if (this.tof.getFeederDistMillimeters() <= Constants.SetPoints.FEEDER_TOF_THRESHOLD_MM){
      this.haveNote = true;
    }
    this.haveIntakedNote = false;
  }

  @Override
  public void execute() {
    this.intake.set(this.intakeDegrees, this.intakeRPM);
    
    if (this.tof.getFeederDistMillimeters() <= Constants.SetPoints.FEEDER_TOF_THRESHOLD_MM){
      if (!this.haveNote){
        this.haveNoteTime = Timer.getFPGATimestamp();
      }
      this.haveNote = true;
    }

    if (this.tof.getIntakeDistMillimeters() <= Constants.SetPoints.INTAKE_TOF_THRESHOLD_MM && this.rumbleControllers){
      this.haveIntakedNote = true;  
    }

    if (this.haveIntakedNote){
      OI.driverController.setRumble(RumbleType.kBothRumble, 0.5);
      OI.operatorController.setRumble(RumbleType.kBothRumble, 0.5);
    } else {
      OI.driverController.setRumble(RumbleType.kBothRumble, 0);
      OI.operatorController.setRumble(RumbleType.kBothRumble, 0);
    }

    if (this.haveNote){
      this.feeder.setPercent(0);
      this.climber.setTrapRollerTorque(-5, 0.1);
      this.climber.setCarriageRotation(Constants.SetPoints.CarriageRotation.kDOWN);
    } else {
      this.feeder.set(this.feederRPM);
      this.climber.setTrapRollerPercent(0.6);
      this.climber.setCarriageRotation(Constants.SetPoints.CarriageRotation.kFEED);
    }
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
