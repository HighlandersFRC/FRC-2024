package frc.robot.commands;

import java.util.ArrayList;

import org.json.JSONArray;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.sensors.TOF;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Peripherals;
import frc.robot.subsystems.Shooter;
import frc.robot.tools.controlloops.PID;
import frc.robot.tools.math.Vector;

public class ShootWhilePathingAndIntaking extends Command {
  private Drive drive;
  private Intake intake;
  private Feeder feeder;
  private Shooter shooter;
  private Peripherals peripherals;
  private TOF tof;

  private double afterPathTimeout = 2;

  //Pathing stuff
  private JSONArray path;

  private double initTime;
  private double currentTime;
  private double previousTime;

  private double odometryX = 0;
  private double odometryY = 0;
  private double odometryTheta = 0;

  //Shooting stuff
  private PID turnPID;
  private double turnP = 0.04;
  private double turnI = 0.0;
  private double turnD = 0.06;

  private double speakerElevationDegrees;
  private double speakerAngleDegrees;
  private double targetPigeonAngleDegrees;

  private double[] shooterValues;
  private double shooterDegrees;
  private double shooterRPM;
  private double feederRPM;

  private boolean hasShot;
  private boolean hasReachedSetPoint;

  private double shooterDegreesAllowedError = 2;
  private double shooterRPMAllowedError = 100;
  private double driveAngleAllowedError = 2;

  private double shootDelay;

  //Intaking stuff
  private double intakeRPM;

  private boolean haveIntakeNote;

  public ShootWhilePathingAndIntaking(Drive drive, Intake intake, Feeder feeder, Shooter shooter, Peripherals peripherals, TOF tof, JSONArray pathPoints, double intakeRPM, double feederRPM, double shootDelay) {
    this.drive = drive;
    this.intake = intake;
    this.feeder = feeder;
    this.shooter = shooter;
    this.peripherals = peripherals;
    this.tof = tof;
    this.path = pathPoints;
    this.intakeRPM = intakeRPM;
    this.feederRPM = feederRPM;
    this.shootDelay = shootDelay;
    addRequirements(this.drive, this.feeder, this.shooter);
  }

  @Override
  public void initialize() {
    this.initTime = Timer.getFPGATimestamp();
    this.hasShot = false;
    this.hasReachedSetPoint = false;
    this.turnPID = new PID(this.turnP, this.turnI, this.turnD);
    this.turnPID.setMinOutput(-3);
    this.turnPID.setMaxOutput(3);

    this.haveIntakeNote = false;
  }

  @Override
  public void execute() {
    this.currentTime = Timer.getFPGATimestamp() - this.initTime;

    //Theta PID control
    double pigeonAngleDegrees = this.peripherals.getPigeonAngle();
    this.speakerElevationDegrees = this.peripherals.getFrontCamTargetTy();
    this.speakerAngleDegrees = this.peripherals.getFrontCamTargetTx();
    this.targetPigeonAngleDegrees = pigeonAngleDegrees - this.speakerAngleDegrees;
    // System.out.println("Speaker Angle: " + this.speakerAngleDegrees);
    // System.out.println("Target Angle: " + this.targetPigeonAngleDegrees);
    this.turnPID.setSetPoint(this.targetPigeonAngleDegrees);
    this.turnPID.updatePID(pigeonAngleDegrees);
    // System.out.println("RPM: " + this.shooter.getFlywheelRPM());
    // System.out.println("RPM Err: " + Math.abs(this.shooter.getFlywheelRPM() - this.shooterRPM));
    // System.out.println("Angle: " + this.shooter.getAngleDegrees());
    // System.out.println("Angle Err: " + Math.abs(this.shooter.getAngleDegrees() - this.shooterDegrees));
    // System.out.println("Pigeon Angle: " + pigeonAngleDegrees);
    // System.out.println("Pigeon Angle Err: " + Math.abs(pigeonAngleDegrees - targetPigeonAngleDegrees));

    double turnResult = -this.turnPID.getResult();

    ArrayList<Integer> tagIDs = this.peripherals.getFrontCamIDs();
    boolean canSeeSpeakerTag = false;
    for (int id : tagIDs){
      if (id == 7 || id == 4){
        canSeeSpeakerTag = true;
      }
    }
    if (this.speakerAngleDegrees > 90 || this.speakerElevationDegrees > 90){
      canSeeSpeakerTag = false;
    }

    //Translational pathing
    this.drive.updateOdometryFusedArray();
    this.odometryX = this.drive.getFusedOdometryX();
    this.odometryY = this.drive.getFusedOdometryY();
    this.odometryTheta = this.drive.getFusedOdometryTheta();

    double[] desiredVelocityArray = this.drive.pidController(this.odometryX, this.odometryY, this.odometryTheta, this.currentTime, this.path);
    Vector velocityVector = new Vector();
    velocityVector.setI(desiredVelocityArray[0]);
    velocityVector.setJ(desiredVelocityArray[1]);
    // velocityVector.setI(0);
    // velocityVector.setJ(0);
    if (canSeeSpeakerTag){
      this.drive.autoDrive(velocityVector, turnResult);
    } else {
      this.drive.autoDrive(velocityVector, 0);
    }

    //Shooting and indexing
    if (canSeeSpeakerTag){
      this.shooterValues = Constants.SetPoints.getShooterValues(this.speakerElevationDegrees);
      this.shooterDegrees = shooterValues[0];
      this.shooterRPM = shooterValues[1];
    }

    this.shooter.set(this.shooterDegrees, this.shooterRPM);

    if (Timer.getFPGATimestamp() - this.initTime >= this.shootDelay && Math.abs(this.shooter.getAngleDegrees() - this.shooterDegrees) <= this.shooterDegreesAllowedError && Math.abs(this.shooter.getFlywheelRPM() - this.shooterRPM) <= this.shooterRPMAllowedError && Math.abs(pigeonAngleDegrees - this.targetPigeonAngleDegrees) <= this.driveAngleAllowedError){
      this.hasReachedSetPoint = true;
    }

    //indexing logic
    if (this.hasShot){
      if (this.tof.getFeederDistMillimeters() <= Constants.SetPoints.FEEDER_TOF_THRESHOLD_MM){
        this.feeder.set(0);
        this.haveIntakeNote = true;
      } else {
        this.feeder.set(this.feederRPM);
      }
    } else {
      if (this.hasReachedSetPoint){
        this.feeder.set(this.feederRPM);
      } else {
        this.feeder.set(0);
      }
    }

    if (this.tof.getFeederDistMillimeters() >= Constants.SetPoints.FEEDER_TOF_THRESHOLD_MM && !this.hasShot){
      this.hasShot = true;
    }

    //Intaking
    this.intake.set(Constants.SetPoints.IntakePosition.kDOWN, this.intakeRPM);

    this.previousTime = this.currentTime;
  }

  @Override
  public void end(boolean interrupted) {
    this.drive.autoDrive(new Vector(0, 0), 0);
    this.feeder.set(0);
  }

  @Override
  public boolean isFinished() {
    if (this.currentTime >= path.getJSONArray(path.length() - 1).getDouble(0) && this.hasShot && this.haveIntakeNote){
      return true;
    } else if (Timer.getFPGATimestamp() - this.initTime > path.getJSONArray(path.length() - 1).getDouble(0) + this.afterPathTimeout){
      return true;
    } else {
      return false;
    }
  }
}
