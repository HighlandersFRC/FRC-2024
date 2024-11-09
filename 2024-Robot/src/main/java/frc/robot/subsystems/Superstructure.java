package frc.robot.subsystems;

import java.util.ArrayList;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.SetPoints.IntakePosition;
import frc.robot.sensors.Proximity;
import frc.robot.sensors.TOF;
import frc.robot.tools.controlloops.PID;

public class Superstructure extends SubsystemBase {
  private Drive drive;
  private Intake intake;
  private Shooter shooter;
  private Feeder feeder;
  private Lights lights;
  private Peripherals peripherals;
  private Climber climber;

  public enum SuperState {
    FEEDING,
    CLEAN_UP,
    SHOOT_SPEAKER,
    AMP,
    CLIMB,
    PRESET_SHOTS,
    CYCLING,
    INTAKE,
  }
  
  private SuperState wantedSuperState = SuperState.CYCLING;
  private SuperState currentSuperState = SuperState.CYCLING;
  private SuperState previousSuperState;

  public Superstructure(Drive drive, Intake intake, Shooter shooter, Feeder feeder, Lights lights, Peripherals peripherals, Climber climber) {
    this.drive = drive;
    this.intake = intake;
    this.shooter = shooter;
    this.feeder = feeder;
    this.lights = lights;
    this.peripherals = peripherals;
    this.climber = climber;
  }

  public void setWantedState(SuperState wantedState) {
    this.wantedSuperState = wantedState;
  }

  public double[] getAimingParameters() {
    double x = 0;
    double y = 0;
    double targetAngle = 0;
    double angleX = 0;
    double angleY = 0;
    double[] shooterValues = new double[3];
    double shooterDegrees = 0;
    double shooterRPM = 0;
    double shooterDegreesAllowedError = 0;
  
    double distToSpeakerMeters;
    double angleToSpeakerDegrees;
    if (drive.getFieldSide() == "red"){
      // System.out.println("-----------red------");
      x = Constants.Physical.FIELD_LENGTH;
      angleX = x - (Constants.Physical.SPEAKER_DEPTH / 2);
      angleY = Constants.Physical.SPEAKER_Y - 0.2;
    } else {
    // System.out.println("-----------blue------");
      x = Constants.Physical.SPEAKER_X;
      angleX = x + (Constants.Physical.SPEAKER_DEPTH / 2);
      angleY = Constants.Physical.SPEAKER_Y + 0.2;
    }

    if (drive.getMT2OdometryY() < 2.0){
      angleY -= 0.06;
    }


    double pigeonAngleDegrees =  peripherals.getPigeonAngle();

    distToSpeakerMeters = Constants.getDistance(x, Constants.Physical.SPEAKER_Y, drive.getMT2OdometryX(), drive.getMT2OdometryY());
    Logger.recordOutput("DistToSpeakerMeters", distToSpeakerMeters);
    angleToSpeakerDegrees = Constants.getAngleToPoint(angleX, angleY, drive.getMT2OdometryX(), drive.getMT2OdometryY());
    shooterValues = Constants.SetPoints.getShooterValuesFromDistance( distToSpeakerMeters, false);
    shooterDegrees =  shooterValues[0];
    shooterRPM =  shooterValues[1];
    shooterDegreesAllowedError =  shooterValues[2];

    if (drive.getFieldSide() == "red"){
      targetAngle = angleToSpeakerDegrees + 180;
    } else {
      targetAngle = angleToSpeakerDegrees;
    }

    if (DriverStation.isAutonomousEnabled() && drive.getFieldSide() == "red"){
      pigeonAngleDegrees = 180 + pigeonAngleDegrees;
    }
    double[] aimingParameters = new double[]{
      shooterDegrees,
      shooterRPM,
      shooterDegreesAllowedError,
      targetAngle,
      pigeonAngleDegrees
    };

    return aimingParameters;
  }

  private void applyStates(){
    switch (currentSuperState) {
      case FEEDING:
        // Feeding state

        break;
      case CLEAN_UP:
        // Clean up state
        break;
      case SHOOT_SPEAKER:
        // Shoot speaker state
        break;
      case AMP:
        // Amp state
        break;
      case CLIMB:
        // Climb state
        break;
      case PRESET_SHOTS:
        // Preset shots state
        break;
      case CYCLING:
        // Cycling state
        break;
      case INTAKE:
        // Intake state
        break;
      default:

        break;
    }
  }

  public void feedingState(){
    
  }

  public void shootingState(){
    
  }

  @Override
  public void periodic() {

  }
}
