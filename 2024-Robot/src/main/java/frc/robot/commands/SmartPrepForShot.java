package frc.robot.commands;

import java.util.ArrayList;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Lights;
import frc.robot.subsystems.Peripherals;
import frc.robot.subsystems.Shooter;

public class SmartPrepForShot extends Command {
  private Shooter shooter;
  private Peripherals peripherals;
  private Lights lights;

  private double[] shooterValues;
  private double shooterDegrees;
  private double shooterRPM;

  private double speakerElevationDegrees;
  private double speakerAngleDegrees;

  public static boolean canSeeTag;

  public SmartPrepForShot(Shooter shooter, Peripherals peripherals, Lights lights) {
    this.shooter = shooter;
    this.peripherals = peripherals;
    this.lights = lights;
    addRequirements(this.shooter, this.lights);
  }

  @Override
  public void initialize() {
    this.speakerElevationDegrees = 0;
    this.speakerAngleDegrees = 0;
    this.shooterValues = Constants.SetPoints.getShooterValuesFromAngle(this.speakerElevationDegrees);
    this.shooterDegrees = this.shooterValues[0];
    this.shooterRPM = this.shooterValues[1];
    lights.setCommandRunning(true);
    lights.setStrobePurple();
  }

  @Override
  public void execute() {
    ArrayList<Integer> ids = this.peripherals.getFrontCamIDs();

    canSeeTag = false;
    for (int id : ids){
      if (id == 7 || id == 4){
        canSeeTag = true;
      }
    }

    if (canSeeTag){
      lights.setStrobeGreen();
      this.speakerElevationDegrees = this.peripherals.getFrontCamTargetTy();
      this.speakerAngleDegrees = this.peripherals.getFrontCamTargetTx();
      if (this.speakerElevationDegrees < 90 && this.speakerAngleDegrees < 90){
        this.shooterValues = Constants.SetPoints.getShooterValuesFromAngle(this.speakerElevationDegrees);
        this.shooterDegrees = this.shooterValues[0];
        this.shooterRPM = this.shooterValues[1];
      }
      this.shooter.set(this.shooterDegrees, this.shooterRPM);
    } else {
      this.shooter.set(30, 6000);
    }

  }

  @Override
  public void end(boolean interrupted) {
    this.shooter.setAngleTorque(-10, 0.3);
    lights.clearAnimations();
    lights.setCommandRunning(false);
  }

  @Override
  public boolean isFinished() {
    if (this.shooterDegrees > 90){
      return true;
    } else {
      return false;
    }
  }
}