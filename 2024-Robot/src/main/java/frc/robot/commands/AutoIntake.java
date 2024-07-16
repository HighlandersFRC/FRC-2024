package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.OI;
import frc.robot.sensors.Proximity;
import frc.robot.sensors.TOF;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Lights;

public class AutoIntake extends Command {
  private Intake intake;
  private Feeder feeder;
  private Climber climber;
  private Lights lights;
  private TOF tof;
  private Proximity proximity;
  private double intakeDegrees;
  private double intakeRPM;
  private double feederRPM;

  private boolean haveNote;
  private double haveNoteTime = 0;
  private double timeout = 60;
  private double initTime;
  private boolean haveCarriageNote;
  private double timeToCenterNote = 0.1;
  private double haveCarriageNoteTime = 0;
  private boolean buzzControllers;

  private boolean noteInIntake;
  private int numTimeNoteInIntake = 0;
  private boolean moveUp5Inches = false;

  public AutoIntake(Intake intake, Feeder feeder, Climber climber, Lights lights, TOF tof, Proximity proximity, Constants.SetPoints.IntakePosition intakePosition, double intakeRPM, double feederRPM, boolean buzzControllers, boolean moveUp5Inches) {
    this.intake = intake;
    this.feeder = feeder;
    this.climber = climber;
    this.lights = lights;
    this.tof = tof;
    this.proximity = proximity;
    this.intakeDegrees = intakePosition.degrees;
    this.intakeRPM = intakeRPM;
    this.feederRPM = feederRPM;
    this.buzzControllers = buzzControllers;
    this.moveUp5Inches = moveUp5Inches;
    addRequirements(this.intake, this.feeder, this.climber);
  }

  public AutoIntake(Intake intake, Feeder feeder, Climber climber, Lights lights, TOF tof, Proximity proximity, Constants.SetPoints.IntakePosition intakePosition, double intakeRPM, double feederRPM, double timeout, boolean buzzControllers, boolean moveUp5Inches) {
    this.intake = intake;
    this.feeder = feeder;
    this.climber = climber;
    this.lights = lights;
    this.tof = tof;
    this.proximity = proximity;
    this.intakeDegrees = intakePosition.degrees;
    this.intakeRPM = intakeRPM;
    this.feederRPM = feederRPM;
    this.timeout = timeout;
    this.buzzControllers = buzzControllers;
    this.moveUp5Inches = moveUp5Inches;
    addRequirements(this.intake, this.feeder, this.climber);
  }

  @Override
  public void initialize() {
    this.haveNote = false;
    this.haveCarriageNote = false;
    this.noteInIntake = false;
    this.initTime = Timer.getFPGATimestamp();
    this.numTimeNoteInIntake = 0;
    lights.clearAnimations();
    lights.setCommandRunning(true);
    lights.setStrobePurple();
    // climber.intakeRunning = true;
  }

  @Override
  public void execute() {
    // System.out.println("Auto Intake");
    // System.out.println("Note in intake: " + this.noteInIntake);
    // System.out.println("Intake Dist: " + this.tof.getIntakeDistMillimeters());

    // if (this.climber.getElevatorLimitSwitch()){
    //   this.climber.setElevatorTorque(-30, 0.7);
    // } else {
    //   this.climber.setElevatorTorque(0, 0);
    // }
    

    if (this.proximity.getShooterProximity()){
      if (!this.haveNote){
        this.haveNoteTime = Timer.getFPGATimestamp();
        lights.clearAnimations();
        lights.setStrobeGreen();
      }
      this.haveNote = true;
    }

    if (this.proximity.getCarriageProximity()){
      if (!this.haveCarriageNote){
        this.haveCarriageNoteTime = Timer.getFPGATimestamp();
      }
      this.haveCarriageNote = true;
    }

    if (this.tof.getIntakeDistMillimeters() <= Constants.SetPoints.INTAKE_TOF_THRESHOLD_MM){
      this.noteInIntake = true;
      this.numTimeNoteInIntake++;
    }

    // if (this.numTimeNoteInIntake >= 8 && this.tof.isIntakeTOFConnected() && this.moveUp5Inches){
    //   // System.out.println("1");
    //   this.intake.set(Constants.SetPoints.IntakePosition.kDOWN.degrees + 50, this.intakeRPM);
    // } else {
      // System.out.println("2");
      this.intake.set(this.intakeDegrees, this.intakeRPM);
    // }

    if (this.buzzControllers){
      if (noteInIntake){
        OI.driverController.setRumble(RumbleType.kBothRumble, 1.0);
        OI.operatorController.setRumble(RumbleType.kBothRumble, 1.0);
      }
    }

    // System.out.println("carriage: " + this.proximity.getCarriageProximity());
    // System.out.println("shooter: " + this.proximity.getShooterProximity());
    // System.out.println("feeder: " + this.proximity.getFeederProximity());
    // System.out.println("have note: " + this.haveNote);

    if (!this.proximity.getCarriageProximity() && !this.proximity.getShooterProximity() && this.proximity.getFeederProximity()){
      // System.out.println("1");
      this.feeder.setPercent(0);
      this.climber.setTrapRollerTorque(15, 0.1);
      this.climber.setCarriageRotationDegrees(Constants.SetPoints.CarriageRotation.kDOWN.degrees);
      // OI.driverController.setRumble(RumbleType.kBothRumble, 0.6);
      // OI.operatorController.setRumble(RumbleType.kBothRumble, 0.6);
    } else if (this.haveNote && !this.proximity.getShooterProximity()){
      // System.out.println("2");
      this.feeder.setPercent(0);
      this.climber.setTrapRollerTorque(15, 0.1);
      this.climber.setCarriageRotation(Constants.SetPoints.CarriageRotation.kDOWN);
    } else if (this.haveNote){
      // System.out.println("3");
      this.feeder.set(110);
      this.climber.setTrapRollerTorque(15, 0.1);
      this.climber.setCarriageRotationDegrees(Constants.SetPoints.CarriageRotation.kFEED.degrees - 5);
      // OI.driverController.setRumble(RumbleType.kBothRumble, 0.6);
      // OI.operatorController.setRumble(RumbleType.kBothRumble, 0.6);
    } else {
      // System.out.println("4");
      this.feeder.set(this.feederRPM);
      this.climber.setTrapRollerTorque(30, 0.50);
      this.climber.setCarriageRotationDegrees(Constants.SetPoints.CarriageRotation.kFEED.degrees - 5);
    }
  }

  @Override
  public void end(boolean interrupted) {
    // this.intake.setAngle(Constants.SetPoints.IntakePosition.kUP);
    this.feeder.setPercent(0);
    this.climber.setTrapRollerPercent(0);
    lights.clearAnimations();
    lights.setCommandRunning(false);
    // climber.intakeRunning = false;
  }

  @Override
  public boolean isFinished() {
    if (OI.getOperatorLB() && Timer.getFPGATimestamp() - haveCarriageNoteTime > timeToCenterNote && this.haveCarriageNote){
      lights.blinkGreen(2);
      return true;
    } else if (/*!this.proximity.getCarriageProximity() &&*/ !this.proximity.getShooterProximity() && this.proximity.getFeederProximity()){
      lights.blinkGreen(2);
      return true;
    } else if (Timer.getFPGATimestamp() - this.initTime >= this.timeout) {
      lights.clearAnimations();
      return true;
    } else {
      return false;
    }
  }
}