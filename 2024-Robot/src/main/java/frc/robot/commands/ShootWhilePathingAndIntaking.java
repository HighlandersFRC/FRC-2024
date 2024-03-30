package frc.robot.commands;

import java.util.ArrayList;

import org.json.JSONArray;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.sensors.Proximity;
// import frc.robot.sensors.Proximity;
import frc.robot.subsystems.Drive;
// import frc.robot.subsystems.Feeder;
// import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Peripherals;
// import frc.robot.subsystems.Shooter;
import frc.robot.tools.controlloops.PID;
import frc.robot.tools.math.Vector;

public class ShootWhilePathingAndIntaking extends Command {
  private Drive drive;
  // private Intake intake;
  // private Feeder feeder;
  // private Shooter shooter;
  private Peripherals peripherals;
  private Proximity proximity;

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
  private double turnP = 0.045;
  private double turnI = 0;
  private double turnD = 0.07;

  private double speakerElevationDegrees;
  private double speakerAngleDegrees;

  private double[] shooterValues;
  private double shooterDegrees;
  private double shooterRPM;
  private double feederRPM;

  private boolean hasShot;
  private boolean hasReachedSetPoint;

  private double shooterDegreesAllowedError = 2;
  private double shooterRPMAllowedError = 500;
  private double driveAngleAllowedError = 4;

  private double shooterLookAheadTime = 0.0;

  private double shootDelay;
  private double shootTimeout = 0.5;

  //Intaking stuff
  private double intakeRPM;

  private boolean haveIntakeNote;

  private boolean pickupNote;
  private double desiredThetaChange = 0;

  public ShootWhilePathingAndIntaking(Drive drive, Peripherals peripherals, Proximity proximity, JSONArray pathPoints, double intakeRPM, double feederRPM, double shootDelay, boolean pickupNote) {
    this.drive = drive;
    this.peripherals = peripherals;
    this.proximity = proximity;
    this.path = pathPoints;
    this.intakeRPM = intakeRPM;
    this.feederRPM = feederRPM;
    this.shootDelay = shootDelay;
    this.pickupNote = pickupNote;
    addRequirements(this.drive);
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

    if (canSeeSpeakerTag){
      this.speakerElevationDegrees = this.peripherals.getFrontCamTargetTy();
      this.speakerAngleDegrees = this.peripherals.getFrontCamTargetTx();
      this.shooterValues = Constants.SetPoints.getShooterValuesFromAngle(speakerElevationDegrees);
      this.shooterDegrees = shooterValues[0];
      this.shooterRPM = shooterValues[1];
    }

    //velocity compensation
    double[] currentSetpoints = Constants.SetPoints.getVelocityAdjustedSetpoint(pigeonAngleDegrees, this.speakerAngleDegrees, this.shooterDegrees, this.shooterRPM, this.drive.getRobotVelocityVector());
    double targetCurrentPigeonAngleDegrees = currentSetpoints[0];
    double targetCurrentShooterDegrees = currentSetpoints[1];
    double targetCurrentShooterRPM = currentSetpoints[2];

    double futureTime = this.currentTime + this.shooterLookAheadTime;
    JSONArray futurePathPoint = this.drive.getPathPoint(this.path, futureTime);
    double futurePathAngleDegrees = Math.toDegrees(futurePathPoint.getDouble(3));

    double speakerX = Constants.Vision.TAG_POSES[4][0];
    double futurePathPointX = futurePathPoint.getDouble(1);
    double speakerY = Constants.Vision.TAG_POSES[4][1];
    double futurePathPointY = futurePathPoint.getDouble(2);

    double futureSpeakerAngleDegrees = Math.atan2(Math.abs(speakerY - futurePathPointY), Math.abs(speakerX - futurePathPointX));

    if (this.drive.getFieldSide() == "blue"){
      futureSpeakerAngleDegrees *= -1;
    }

    double futureSpeakerDist = Constants.getDistance(speakerX, speakerY, futurePathPointX, futurePathPointY);
    double[] futureSetpoints = Constants.SetPoints.getShooterValuesFromDistance(futureSpeakerDist);
    Vector futureVelocityVector = new Vector();
    futureVelocityVector.setI(futurePathPoint.getDouble(4));
    futureVelocityVector.setJ(futurePathPoint.getDouble(5));

    double[] futureCorrectedSetpoints = Constants.SetPoints.getVelocityAdjustedSetpoint(futurePathAngleDegrees, futureSpeakerAngleDegrees, futureSetpoints[0], futureSetpoints[1], futureVelocityVector);
    double targetFuturePigeonAngleDegrees = futureCorrectedSetpoints[0];
    double targetFutureShooterDegrees = futureCorrectedSetpoints[1];
    double targetFutureShooterRPM = futureCorrectedSetpoints[2];

    this.turnPID.setSetPoint(targetCurrentPigeonAngleDegrees);
    this.turnPID.updatePID(pigeonAngleDegrees);

    // System.out.println("RPM: " + this.shooter.getFlywheelRPM());
    // System.out.println("Targ. RPM: " + targetShooterRPM);
    // System.out.println("RPM Err: " + Math.abs(this.shooter.getFlywheelRPM() - targetShooterRPM));
    // System.out.println("Elev: " + this.shooter.getAngleDegrees());
    // System.out.println("Targ. Elev: " + targetShooterDegrees);
    // System.out.println("Elev Err: " + Math.abs(this.shooter.getAngleDegrees() - targetShooterDegrees));
    System.out.println("Pigeon Angle: " + pigeonAngleDegrees);
    System.out.println("Targ. Pigeon Angle: " + targetCurrentPigeonAngleDegrees);
    System.out.println("Pigeon Angle Err: " + Math.abs(pigeonAngleDegrees - targetCurrentPigeonAngleDegrees));
    System.out.println("<================>");

    double turnResult = -this.turnPID.getResult();

    //Translational pathing
    this.drive.updateOdometryFusedArray();
    this.odometryX = this.drive.getFusedOdometryX();
    this.odometryY = this.drive.getFusedOdometryY();
    this.odometryTheta = this.drive.getFusedOdometryTheta();

    double[] desiredVelocityArray = this.drive.pidController(this.odometryX, this.odometryY, this.odometryTheta, this.currentTime, this.path, this.pickupNote, 0);
    Vector velocityVector = new Vector();
    velocityVector.setI(desiredVelocityArray[0]);
    velocityVector.setJ(desiredVelocityArray[1]);
    desiredThetaChange = desiredVelocityArray[2]; 
    // velocityVector.setI(0);
    // velocityVector.setJ(0);
    if (canSeeSpeakerTag && proximity.getFeederProximity()){
      System.out.println("1");
      this.drive.autoDrive(velocityVector, turnResult);
    } else {
      System.out.println("2");
      this.drive.autoDrive(velocityVector, desiredThetaChange);
    }

    // this.shooter.set(targetFutureShooterDegrees, targetFutureShooterRPM);

    // if (Timer.getFPGATimestamp() - this.initTime >= this.shootDelay && Math.abs(this.shooter.getAngleDegrees() - targetCurrentShooterDegrees) <= this.shooterDegreesAllowedError && Math.abs(this.shooter.getFlywheelRPM() - targetCurrentShooterRPM) <= this.shooterRPMAllowedError && Math.abs(pigeonAngleDegrees - targetCurrentPigeonAngleDegrees) <= this.driveAngleAllowedError){
    //   this.hasReachedSetPoint = true;
    // }

    // if (Timer.getFPGATimestamp() - this.initTime - this.shootDelay >= this.shootTimeout){
    //   this.hasReachedSetPoint = true;
    //   System.out.println("\nSHOOTER TIMEOUT\n");
    // }

    // //indexing logic
    // if (this.hasShot){
    //   if (this.proximity.getFeederProximity()){
    //     this.feeder.set(0);
    //     this.haveIntakeNote = true;
    //   } else {
    //     this.feeder.set(this.feederRPM);
    //   }
    // } else {
    //   if (this.hasReachedSetPoint){
    //     this.feeder.set(this.feederRPM);
    //   } else {
    //     this.feeder.set(0);
    //   }
    // }

    // if (this.tof.getFeederDistMillimeters() >= Constants.SetPoints.FEEDER_TOF_THRESHOLD_MM && !this.hasShot){
    //   this.hasShot = true;
    // }

    //Intaking
    // this.intake.set(Constants.SetPoints.IntakePosition.kDOWN, this.intakeRPM);

    this.previousTime = this.currentTime;
  }

  @Override
  public void end(boolean interrupted) {
    this.drive.autoDrive(new Vector(0, 0), 0);
    // this.feeder.set(0);
  }

  @Override
  public boolean isFinished() {
    if (this.currentTime > path.getJSONArray(path.length() - 1).getDouble(0)){
      return true;
    } else if (Timer.getFPGATimestamp() - this.initTime > path.getJSONArray(path.length() - 1).getDouble(0) + this.afterPathTimeout){
      return true;
    } else {
      return false;
    }
  }
}
