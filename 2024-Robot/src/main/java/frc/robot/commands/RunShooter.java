package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class RunShooter extends Command {
  Shooter shooter;
  double angle;
  double RPM;
  double startTime = Timer.getFPGATimestamp();

  public RunShooter(Shooter shooter, double angle, double RPM) {
    this.shooter = shooter;
    this.angle = angle;
    this.RPM = RPM;
    addRequirements(this.shooter);
  }

  @Override
  public void initialize() {
    startTime = Timer.getFPGATimestamp();
  }

  @Override
  public void execute() {
    this.shooter.setShooter(this.angle, this.RPM);
    System.out.println("SHOOTER");
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    if (Timer.getFPGATimestamp() - startTime > 4){
      return true;
    } else {
      return false;
    }
  }
}