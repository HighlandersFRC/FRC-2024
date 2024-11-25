package frc.robot.subsystems;

import java.util.ArrayList;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.SetPoints.IntakePosition;
import frc.robot.sensors.Proximity;
import frc.robot.sensors.TOF;
import frc.robot.tools.controlloops.PID;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Climber.ClimberState;
import frc.robot.subsystems.Drive.DriveState;
import frc.robot.subsystems.Feeder.FeederState;
import frc.robot.subsystems.Intake.IntakeState;
import frc.robot.subsystems.Shooter.ShooterState;

public class Superstructure extends SubsystemBase {
  private Drive drive;
  private Intake intake;
  private Shooter shooter;
  private Feeder feeder;
  private Lights lights;
  private Peripherals peripherals;
  private Climber climber;
  private Proximity proximity;

  public enum SuperState {
    FEEDING,
    CLEAN_UP,
    SHOOT_SPEAKER,
    PREPARING_SPEAKER_SHOT,
    AMP,
    CLIMBER_UP,
    CLIMBER_DOWN,
    INDEX_TO_TRAP,
    INDEX_TO_AMP,
    TRAP,
    PRESET_SHOTS,
    CYCLING,
    INTAKE,
    OUTAKING,
  }

  private SuperState wantedSuperState = SuperState.CYCLING;
  private SuperState currentSuperState = SuperState.CYCLING;
  private SuperState previousSuperState;

  public Superstructure(Drive drive, Intake intake, Shooter shooter, Feeder feeder, Lights lights,
      Peripherals peripherals, Climber climber, Proximity proximity) {
    this.drive = drive;
    this.intake = intake;
    this.shooter = shooter;
    this.feeder = feeder;
    this.lights = lights;
    this.peripherals = peripherals;
    this.climber = climber;
    this.proximity = proximity;
  }

  public void setWantedState(SuperState wantedState) {
    this.wantedSuperState = wantedState;
  }

  public Command setWantedSuperStateCommand(SuperState wantedSuperState) {
    return new InstantCommand(() -> setWantedState(wantedSuperState));
  }

  public double[] getAimingParameters() {
    double x = 0;
    double angleX = 0;
    double angleY = 0;
    double[] shooterValues = new double[3];
    double shooterDegrees = 0;
    double shooterRPM = 0;
    double shooterDegreesAllowedError = 0;

    double distToSpeakerMeters;
    double angleToSpeakerDegrees;
    if (drive.getFieldSide() == "red") {
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

    if (drive.getMT2OdometryY() < 2.0) {
      angleY -= 0.06;
    }

    double pigeonAngleDegrees = peripherals.getPigeonAngle();

    distToSpeakerMeters = Constants.getDistance(x, Constants.Physical.SPEAKER_Y, drive.getMT2OdometryX(),
        drive.getMT2OdometryY());
    Logger.recordOutput("DistToSpeakerMeters", distToSpeakerMeters);
    angleToSpeakerDegrees = Constants.getAngleToPoint(angleX, angleY, drive.getMT2OdometryX(), drive.getMT2OdometryY());
    shooterValues = Constants.SetPoints.getShooterValuesFromDistance(distToSpeakerMeters, false);
    shooterDegrees = shooterValues[0];
    shooterRPM = shooterValues[1];
    shooterDegreesAllowedError = shooterValues[2];

    double[] aimingParameters = new double[] {
        shooterDegrees,
        shooterRPM,
        shooterDegreesAllowedError,
        angleToSpeakerDegrees,
        pigeonAngleDegrees
    };

    return aimingParameters;
  }

  private void applyStates() {
    switch (currentSuperState) {
      case FEEDING:
        // Feeding state

        break;
      case CLEAN_UP:
        // Clean up state
        break;
      case PREPARING_SPEAKER_SHOT:
        prepareShot(getAimingParameters());
        break;
      case SHOOT_SPEAKER:
        // Shoot speaker state
        shootSpeaker(getAimingParameters());
        break;
      case AMP:
        // Amp state
        ampState();
        break;
      case CLIMBER_UP:
        climberUpState();
        // Climb state
        break;
      case CLIMBER_DOWN:
        climberDownState();
        // Climb state
        break;
      case PRESET_SHOTS:
        // Preset shots state
        break;
      case CYCLING:
        handleCyclingState();
        // Cycling state
        break;
      case INTAKE:
        // Intake state
        intakingState();
        break;
      case OUTAKING:
        // Outake state
        outakingState();
        break;
      default:
        handleCyclingState();
        break;
    }
  }

  private SuperState handleStateTransitions() {
    previousSuperState = currentSuperState;
    switch (wantedSuperState) {
      case FEEDING:
        // Feeding state
        currentSuperState = SuperState.FEEDING;
        break;
      case CLEAN_UP:
        // Clean up state
        break;
      case SHOOT_SPEAKER:
        // Shoot speaker state
        currentSuperState = areSystemsReadyForTeleopShot()
            ? SuperState.SHOOT_SPEAKER
            : SuperState.PREPARING_SPEAKER_SHOT;
        break;
      case AMP:
        // Amp state
        currentSuperState = SuperState.AMP;
        break;
      case CLIMBER_DOWN:
        currentSuperState = SuperState.CLIMBER_DOWN;
        // Climb state
        break;
      case CLIMBER_UP:
        currentSuperState = SuperState.CLIMBER_UP;
        // Climb state
        break;
      case PRESET_SHOTS:
        // Preset shots state
        break;
      case CYCLING:
        // Cycling state
        currentSuperState = SuperState.CYCLING;
        break;
      case INTAKE:
        // Intake state
        currentSuperState = SuperState.INTAKE;
        break;
      case OUTAKING:
        // Intake state
        currentSuperState = SuperState.OUTAKING;
        break;
      default:
        currentSuperState = SuperState.CYCLING;
        break;
    }
    return currentSuperState;
  }

  public void climberDownState() {
    climber.setWantedState(ClimberState.CLIMBER_DOWN);
  }

  public void climberUpState() {
    climber.setWantedState(ClimberState.CLIMBER_UP);
  }

  private boolean areSystemsReadyForTeleopShot() {
    boolean isReady = shooter.shooterAtSetpoint()
        && drive.atSetpoint();
    System.out.println("is ready: " + isReady);
    return isReady;
  }

  public boolean isNoteIndexed() {
    return false;
  }

  public void feedingState() {

  }

  public void ampState() {
    feeder.setWantedState(FeederState.INDEX_TO_AMP);
    climber.setWantedState(ClimberState.AMP);
    intake.setWantedState(IntakeState.INDEX_TO_AMP);
  }

  public void outakingState() {
    intake.setWantedState(IntakeState.REJECT);
    feeder.setWantedState(FeederState.REJECT);
    climber.setWantedState(ClimberState.REJECT);
  }

  public void intakingState() {
    intake.setWantedState(IntakeState.COLLECT);
    feeder.setWantedState(FeederState.COLLECT);
    climber.setWantedState(ClimberState.COLLECT);
  }

  public void handleCyclingState() {
    drive.setWantedState(DriveState.DEFAULT);
    intake.setWantedState(IntakeState.OFF);
    feeder.setWantedState(FeederState.IDLE);
    shooter.setWantedState(ShooterState.OFF);
    climber.setWantedState(ClimberState.DEFAULT);
  }

  public void prepareShot(double[] aimingParameters) {
    shooter.setWantedState(ShooterState.SHOOT, aimingParameters[0], aimingParameters[1], aimingParameters[2]);
    drive.setWantedState(DriveState.SHOOT, aimingParameters[3]);
  }

  public void shootSpeaker(double[] aimingParameters) {
    shooter.setWantedState(ShooterState.SHOOT, aimingParameters[0], aimingParameters[1], aimingParameters[2]);
    drive.setWantedState(DriveState.SHOOT, aimingParameters[3]);
    feeder.setWantedState(FeederState.EJECT);
  }

  public void indexToAmp() {

  }

  public void indexToTrap() {
  }

  @Override
  public void periodic() {
    currentSuperState = handleStateTransitions();
    applyStates();
  }
}
