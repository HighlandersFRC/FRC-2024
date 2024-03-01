package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
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

  private boolean haveShot;
  private boolean haveReachedSetpoint;

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
    this.haveShot = false;
    this.haveReachedSetpoint = false;
    this.startTime = Timer.getFPGATimestamp();
  }

  @Override
  public void execute() {

  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
