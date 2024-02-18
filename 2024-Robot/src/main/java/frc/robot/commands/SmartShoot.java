package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.sensors.TOF;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Lights;
import frc.robot.subsystems.Peripherals;
import frc.robot.subsystems.Shooter;

public class SmartShoot extends Command {
  private Shooter shooter;
  private Feeder feeder;
  private Peripherals peripherals;
  private Lights lights;
  private TOF tof;
  private double shooterDegrees;
  private double shooterRPM;
  private double feederRPM;

  private double startTime;
  private double timeout = 15;

  private double shotTime = 0;
  private boolean hasShot;
  private double shotPauseTime = 0.1;

  private double shooterDegreesAllowedError = 1;
  private double shooterRPMAllowedError = 150;

  public SmartShoot(Shooter shooter, Feeder feeder, Peripherals peripherals, Lights lights, TOF tof, double shooterDegrees, double shooterRPM, double feederRPM) {
    this.shooter = shooter;
    this.feeder = feeder;
    this.peripherals = peripherals;
    this.lights = lights;
    this.tof = tof;
    this.shooterDegrees = shooterDegrees;
    this.shooterRPM = shooterRPM;
    this.feederRPM = feederRPM;
    addRequirements(this.shooter, this.feeder);
  }

  public SmartShoot(Shooter shooter, Feeder feeder, Peripherals peripherals, Lights lights, TOF tof, double shooterDegrees, double shooterRPM, double feederRPM, double timeout) {
    this.shooter = shooter;
    this.feeder = feeder;
    this.peripherals = peripherals;
    this.lights = lights;
    this.tof = tof;
    this.shooterDegrees = shooterDegrees;
    this.shooterRPM = shooterRPM;
    this.feederRPM = feederRPM;
    this.timeout = timeout;
    addRequirements(this.shooter, this.feeder);
  }

  @Override
  public void initialize() {
    this.startTime = Timer.getFPGATimestamp();
    this.hasShot = false;
  }

  @Override
  public void execute() {
    this.shooter.set(this.shooterDegrees, this.shooterRPM);
    if (Math.abs(this.shooter.getAngleDegrees() - this.shooterDegrees) <= this.shooterDegreesAllowedError && Math.abs(this.shooter.getFlywheelRPM() - this.shooterRPM) <= this.shooterRPMAllowedError){
      this.feeder.set(this.feederRPM);
    } else {
      this.feeder.set(0);
    }
    if (this.tof.getFeederDistMillimeters() >= Constants.SetPoints.FEEDER_TOF_THRESHOLD_MM){
      this.hasShot = true;
      this.shotTime = Timer.getFPGATimestamp();
    }
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    if (Timer.getFPGATimestamp() - this.startTime >= this.timeout){
      System.out.println("Shooter time out");
      return true;
    } else if (this.hasShot && Timer.getFPGATimestamp() - this.shotTime >= this.shotPauseTime){
      return true;
    } else {
      return false;
    }
  }
}
