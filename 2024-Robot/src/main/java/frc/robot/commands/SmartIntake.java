package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.OI;
import frc.robot.sensors.TOF;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Lights;
import frc.robot.subsystems.Peripherals;
import frc.robot.tools.controlloops.PID;
import frc.robot.tools.math.Vector;

public class SmartIntake extends Command {
  private Intake intake;
  private Feeder feeder;
  private Climber climber;
  private Drive drive;
  private Peripherals peripherals;
  private Lights lights;
  private TOF tof;
  private double intakeDegrees;
  private double intakeRPM;
  private double feederRPM;

  private boolean haveNote = false;
  private boolean haveIntakedNote = false;
  private double haveNoteTime = 0;

  private boolean rumbleControllers = true;

  private PID pid;
  private double kP = 3;
  private double kI = 0;
  private double kD = 0;

  public SmartIntake(Intake intake, Feeder feeder, Climber climber, Drive drive, Peripherals peripherals, Lights lights, TOF tof, Constants.SetPoints.IntakePosition intakePosition, double intakeRPM, double feederRPM) {
    this.intake = intake;
    this.feeder = feeder;
    this.climber = climber;
    this.drive = drive;
    this.peripherals = peripherals;
    this.lights = lights;
    this.tof = tof;
    this.intakeDegrees = intakePosition.degrees;
    this.intakeRPM = intakeRPM;
    this.feederRPM = feederRPM;
    addRequirements(this.intake, this.feeder, this.climber, this.drive, this.peripherals);
  }

  public SmartIntake(Intake intake, Feeder feeder, Climber climber, Drive drive, Peripherals peripherals, Lights lights, TOF tof, Constants.SetPoints.IntakePosition intakePosition, double intakeRPM, double feederRPM, boolean rumble) {
    this.intake = intake;
    this.feeder = feeder;
    this.lights = lights;
    this.climber = climber;
    this.drive = drive;
    this.peripherals = peripherals;
    this.tof = tof;
    this.intakeDegrees = intakePosition.degrees;
    this.intakeRPM = intakeRPM;
    this.feederRPM = feederRPM;
    this.rumbleControllers = rumble;
    addRequirements(this.intake, this.feeder, this.climber, this.drive, this.peripherals);
  }

  @Override
  public void initialize() {
    this.haveNote = false;
    if (this.tof.getFeederDistMillimeters() <= Constants.SetPoints.FEEDER_TOF_THRESHOLD_MM){
      this.haveNote = true;
    }
    this.haveIntakedNote = false;

    pid = new PID(kP, kI, kD);
    pid.setSetPoint(0);
    pid.setMinOutput(-4);
    pid.setMaxOutput(4);
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

    // if (this.haveNote && Timer.getFPGATimestamp() - this.haveNoteTime < 0.15){
    //   this.feeder.setPercent(-0.1);
    //   this.climber.setTrapRollerPercent(0);
    // } else if (this.haveNote){
    //   this.feeder.setPercent(0);
    //   this.climber.setTrapRollerPercent(0);
    // } else {
    //   this.feeder.set(this.feederRPM);
    //   this.climber.setTrapRollerPercent(0.7);
    // }

    if (this.haveNote){
      this.feeder.setPercent(0);
      this.climber.setTrapRollerPercent(0);
    } else {
      this.feeder.set(this.feederRPM);
      // this.climber.setTrapRollerTorque(30, 0.7);
      this.climber.setTrapRollerPercent(0.7);
    }

    double angleToPiece = peripherals.getBackCamTargetTx();
    pid.updatePID(angleToPiece);
    double result = -pid.getResult();

    drive.autoRobotCentricDrive(new Vector(-3, 0), result);
  }

  @Override
  public void end(boolean interrupted) {
    drive.autoRobotCentricDrive(new Vector(0, 0), 0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
