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
  private double timeout = 10;
  private double initTime;
  private boolean buzzControllers;

  public AutoIntake(Intake intake, Feeder feeder, Climber climber, Lights lights, TOF tof, Proximity proximity, Constants.SetPoints.IntakePosition intakePosition, double intakeRPM, double feederRPM, boolean buzzControllers) {
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
    addRequirements(this.intake, this.feeder, this.climber);
  }

  public AutoIntake(Intake intake, Feeder feeder, Climber climber, Lights lights, TOF tof, Proximity proximity, Constants.SetPoints.IntakePosition intakePosition, double intakeRPM, double feederRPM, double timeout, boolean buzzControllers) {
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
    addRequirements(this.intake, this.feeder, this.climber);
  }

  @Override
  public void initialize() {
    this.haveNote = false;
    this.initTime = Timer.getFPGATimestamp();
    lights.clearAnimations();
    lights.setCommandRunning(true);
    lights.setStrobePurple();
    // climber.intakeRunning = true;
  }

  @Override
  public void execute() {
    // System.out.println("Auto Intake");
    this.intake.set(this.intakeDegrees, this.intakeRPM);
    if (this.proximity.getShooterProximity()){
      if (!this.haveNote){
        this.haveNoteTime = Timer.getFPGATimestamp();
        lights.clearAnimations();
        lights.setStrobeGreen();
      }
      this.haveNote = true;
    }

    if (this.buzzControllers){
      if (this.tof.getIntakeDistMillimeters() <= Constants.SetPoints.INTAKE_TOF_THRESHOLD_MM){
        OI.driverController.setRumble(RumbleType.kBothRumble, 0.6);
        OI.operatorController.setRumble(RumbleType.kBothRumble, 0.6);
      }
    }

    // System.out.println("carriage: " + this.proximity.getCarriageProximity());
    // System.out.println("shooter: " + this.proximity.getShooterProximity());
    // System.out.println("feeder: " + this.proximity.getFeederProximity());
    // System.out.println("have note: " + this.haveNote);

    if (!this.proximity.getCarriageProximity() && !this.proximity.getShooterProximity() && this.proximity.getFeederProximity()){
      // System.out.println("runs");
      this.feeder.setPercent(0);
      this.climber.setTrapRollerTorque(15, 0.1);
      this.climber.setCarriageRotationDegrees(Constants.SetPoints.CarriageRotation.kDOWN.degrees);
      // OI.driverController.setRumble(RumbleType.kBothRumble, 0.6);
      // OI.operatorController.setRumble(RumbleType.kBothRumble, 0.6);
    } else if (this.haveNote && !this.proximity.getShooterProximity()){
      // System.out.println("shooter sees");
      this.feeder.setPercent(0);
      this.climber.setTrapRollerTorque(15, 0.1);
      this.climber.setCarriageRotation(Constants.SetPoints.CarriageRotation.kDOWN);
    } else if (this.haveNote){
      // System.out.println("has note");
      this.feeder.set(110);
      this.climber.setTrapRollerTorque(15, 0.1);
      this.climber.setCarriageRotationDegrees(Constants.SetPoints.CarriageRotation.kFEED.degrees - 5);
      // OI.driverController.setRumble(RumbleType.kBothRumble, 0.6);
      // OI.operatorController.setRumble(RumbleType.kBothRumble, 0.6);
    } else {
      // System.out.println("else");
      this.feeder.set(this.feederRPM);
      this.climber.setTrapRollerTorque(30, 0.50);
      this.climber.setCarriageRotationDegrees(Constants.SetPoints.CarriageRotation.kFEED.degrees - 5);
    }
  }

  @Override
  public void end(boolean interrupted) {
    this.feeder.setPercent(0);
    this.climber.setTrapRollerPercent(0);
    lights.clearAnimations();
    lights.setCommandRunning(false);
    // climber.intakeRunning = false;
  }

  @Override
  public boolean isFinished() {
    if (!this.proximity.getCarriageProximity() && !this.proximity.getShooterProximity() && this.proximity.getFeederProximity()){
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
