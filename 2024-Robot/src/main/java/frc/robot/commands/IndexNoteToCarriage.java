package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.sensors.TOF;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class IndexNoteToCarriage extends Command {
  private Feeder feeder;
  private Climber climber;
  private Intake intake;
  private TOF tof;
  private Shooter shooter;
  private boolean haveNote = false;
  private double timeToCenterNote = 0.55;
  private double haveNoteTime;
  public IndexNoteToCarriage(Feeder feeder, Climber climber, Intake intake, TOF tof, Shooter shooter) {
    this.feeder = feeder;
    this.climber = climber;
    this.intake = intake;
    this.tof = tof;
    this.shooter = shooter;
    addRequirements(feeder, climber, intake, shooter);
  }

  @Override
  public void initialize() {
    this.haveNoteTime = 0.0;
    this.haveNote = false;
  }

  @Override
  public void execute() {
    if (this.tof.getCarriageDistMillimeters() <= Constants.SetPoints.CARRIAGE_TOF_THRESHOLD_MM){
      if (!this.haveNote){
        this.haveNoteTime = Timer.getFPGATimestamp();
      }
      this.haveNote = true;
    }

    if (Math.abs(this.climber.getCarriageRotationDegrees() - Constants.SetPoints.CarriageRotation.kDOWN.degrees) < 2){
      this.feeder.set(-150);
    } else {
      this.feeder.setTorque(5, 0.5);
    }
    this.climber.setTrapRollerTorque(-20, 0.5);
    this.climber.setCarriageRotation(Constants.SetPoints.CarriageRotation.kDOWN);
    this.intake.setRollers(-150);
    this.intake.setAngleTorqueCurrent(15, 0.6);
    if (this.shooter.getFlywheelRPM() > 100){
      this.shooter.setFlywheelPercent(-0.1);
    } else {
      this.shooter.setFlywheelPercent(0);
    }
  }

  @Override
  public void end(boolean interrupted) {
    this.feeder.set(0.0);
    this.climber.setTrapRollerTorque(0, 0);
    this.intake.setRollers(0.0);
  }

  @Override
  public boolean isFinished() {
    if (Timer.getFPGATimestamp() - haveNoteTime > timeToCenterNote && this.haveNote){
      return true;
    } else {
      return false;
    }
  }
}