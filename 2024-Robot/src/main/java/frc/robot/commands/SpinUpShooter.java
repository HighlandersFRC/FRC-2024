package frc.robot.commands;

import java.util.ArrayList;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Peripherals;
import frc.robot.subsystems.Shooter;

public class SpinUpShooter extends Command {
  private Shooter shooter;
  private Peripherals peripherals;

  private double speakerElevationDegrees;
  private double[] shooterSetpoints;
  private double shooterDegrees;
  private double shooterRPM;

  public SpinUpShooter(Shooter shooter, Peripherals peripherals) {
    this.shooter = shooter;
    this.peripherals = peripherals;
    addRequirements(this.shooter);
  }

  @Override
  public void initialize() {
    this.speakerElevationDegrees = this.peripherals.getFrontCamTargetTy();
    this.shooterDegrees = Constants.SetPoints.SHOOTER_DOWN_ANGLE_DEG + 4;
    this.shooterRPM = 3000;
  }

  @Override
  public void execute() {
    double id = this.peripherals.getFrontCamID();

    boolean canSeeTag = false;
    // for (double id : ids){
      if (id == 7 || id == 4){
        canSeeTag = true;
      }
    // }

    if (canSeeTag){
      this.speakerElevationDegrees = this.peripherals.getFrontCamTargetTy();
      this.shooterSetpoints = Constants.SetPoints.getShooterValuesFromAngle(speakerElevationDegrees);
      this.shooterDegrees = shooterSetpoints[0];
      this.shooterRPM = shooterSetpoints[1];
    }

    // this.shooter.set(this.shooterDegrees, this.shooterRPM);
    this.shooter.set(this.shooterDegrees, 0);
    // this.shooter.setFlywheelRPM(3000);
    // if (Math.abs(this.shooter.getAngleDegrees() - Constants.SetPoints.SHOOTER_DOWN_ANGLE_DEG) < 2){
    //   this.shooter.setAnglePercent(0);
    // } else {
    //   this.shooter.setAngleTorque(-5, 0.15);
    // }
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
