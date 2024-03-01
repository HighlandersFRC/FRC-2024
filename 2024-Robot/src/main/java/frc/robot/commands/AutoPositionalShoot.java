package frc.robot.commands;

import org.json.JSONArray;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.sensors.TOF;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Peripherals;
import frc.robot.subsystems.Shooter;
import frc.robot.tools.controlloops.PID;

public class AutoPositionalShoot extends Command {
  private Drive drive;
  private Shooter shooter;
  private Feeder feeder;
  private Peripherals peripherals;
  private TOF tof;

  private PID turnPID;
  private double kP = 0.04;
  private double kI = 0.0;
  private double kD = 0.06;

  private boolean haveGoodPosition;
  private boolean haveShot;
  private boolean haveReachedSetpoint;
  private int numPositionMeasurements;
  private boolean haveShotParams;

  private double[] position = new double[2];
  private double shooterDegrees;
  private double shooterRPM;
  private double targetPigeonAngleDegrees;

  private double startTime;

  public AutoPositionalShoot(Drive drive, Shooter shooter, Feeder feeder, Peripherals peripherals, TOF tof) {
    this.drive = drive;
    this.shooter = shooter;
    this.feeder = feeder;
    this.peripherals = peripherals;
    this.tof = tof;
    addRequirements(this.drive, this.shooter, this.feeder);
  }

  @Override
  public void initialize() {
    this.turnPID = new PID(this.kP, this.kI, this.kD);
    this.turnPID.setMaxOutput(3);
    this.turnPID.setMinOutput(-3);
    this.haveGoodPosition = false;
    this.numPositionMeasurements = 0;
    this.haveShot = false;
    this.haveReachedSetpoint = false;
    this.haveShotParams = false;
    this.startTime = Timer.getFPGATimestamp();
  }

  @Override
  public void execute() {
    if (this.numPositionMeasurements >= 5){
      this.haveGoodPosition = true;
    }

    if (!this.haveGoodPosition){
      JSONArray frontCamBotPose = this.peripherals.getRawFrontCamBasedPosition();
      JSONArray leftCamBotPose = this.peripherals.getRawLeftCamBasedPosition();
      JSONArray rightCamBotPose = this.peripherals.getRawRightCamBasedPosition();
      double frontCamTagDist = this.peripherals.getFrontHorizontalDistToTag();
      double leftCamTagDist = this.peripherals.getLeftHorizontalDistToTag();
      double rightCamTagDist = this.peripherals.getRightHorizontalDistToTag();
      boolean haveFrontCamTrack = frontCamBotPose.length() == 7;
      boolean haveLeftCamTrack = leftCamBotPose.length() == 7;
      boolean haveRightCamTrack = rightCamBotPose.length() == 7;

      if (haveFrontCamTrack && frontCamTagDist <= leftCamTagDist && frontCamTagDist <= rightCamTagDist){
        if (this.numPositionMeasurements == 0){
          this.position[0] = frontCamBotPose.getDouble(0);
          this.position[1] = frontCamBotPose.getDouble(1);
        } else {
          this.position[0] = (this.position[0] + frontCamBotPose.getDouble(0)) / 2;
          this.position[1] = (this.position[1] + frontCamBotPose.getDouble(1)) / 2;
        }
        this.numPositionMeasurements ++;
      } else if (haveLeftCamTrack && leftCamTagDist <= frontCamTagDist && leftCamTagDist <= rightCamTagDist){
        if (this.numPositionMeasurements == 0){
          this.position[0] = leftCamBotPose.getDouble(0);
          this.position[1] = leftCamBotPose.getDouble(1);
        } else {
          this.position[0] = (this.position[0] + leftCamBotPose.getDouble(0)) / 2;
          this.position[1] = (this.position[1] + leftCamBotPose.getDouble(1)) / 2;
        }
        this.numPositionMeasurements ++;
      } else if (haveRightCamTrack && rightCamTagDist <= frontCamTagDist && rightCamTagDist <= leftCamTagDist){
        if (this.numPositionMeasurements == 0){
          this.position[0] = rightCamBotPose.getDouble(0);
          this.position[1] = rightCamBotPose.getDouble(1);
        } else {
          this.position[0] = (this.position[0] + rightCamBotPose.getDouble(0)) / 2;
          this.position[1] = (this.position[1] + rightCamBotPose.getDouble(1)) / 2;
        }
        this.numPositionMeasurements ++;
      }
    } else {
      if (!this.haveShotParams){
        int speakerTagID;
        if (this.drive.getFieldSide() == "blue"){
          speakerTagID = 7;
        } else {
          speakerTagID = 4;
        }
        double distToSpeaker = Constants.getDistance(this.position[0], this.position[1], Constants.Vision.TAG_POSES[speakerTagID - 1][0], Constants.Vision.TAG_POSES[speakerTagID - 1][1]);
        double[] shooterParams = Constants.SetPoints.getShooterValuesFromDistance(distToSpeaker);
        this.shooterDegrees = shooterParams[0];
        this.shooterRPM = shooterParams[1];
        
      }
    }
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
