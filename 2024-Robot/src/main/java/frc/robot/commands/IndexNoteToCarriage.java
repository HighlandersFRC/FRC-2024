package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.sensors.Proximity;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class IndexNoteToCarriage extends Command {
  private Feeder feeder;
  private Climber climber;
  private Intake intake;
  private Proximity proximity;
  private Shooter shooter;
  private boolean haveNote = false;
  private double timeToCenterNote = 0.7;
  private double haveNoteTime;
  public IndexNoteToCarriage(Feeder feeder, Climber climber, Intake intake, Proximity proximity, Shooter shooter) {
    this.feeder = feeder;
    this.climber = climber;
    this.intake = intake;
    this.proximity = proximity;
    this.shooter = shooter;
    this.timeToCenterNote = 0.7;
    addRequirements(feeder, climber, intake, shooter);
  }

  public IndexNoteToCarriage(Feeder feeder, Climber climber, Intake intake, Proximity proximity, Shooter shooter, double timeToCenter) {
    this.feeder = feeder;
    this.climber = climber;
    this.intake = intake;
    this.proximity= proximity;
    this.shooter = shooter;
    this.timeToCenterNote = timeToCenter;
    addRequirements(feeder, climber, intake, shooter);
  }

  @Override
  public void initialize() {
    this.haveNoteTime = 0.0;
    this.haveNote = false;
  }

  @Override
  public void execute() {
    if (this.proximity.getCarriageProximity()){
      if (!this.haveNote){
        this.haveNoteTime = Timer.getFPGATimestamp();
        // System.out.println("1");
      }
      this.haveNote = true;
    }

    this.feeder.set(-150);
    this.climber.setTrapRollerTorque(-20, 0.5);
    this.climber.setCarriageRotationDegrees(Constants.SetPoints.CarriageRotation.kDOWN.degrees - 5);
    this.intake.setRollers(-150);
    this.intake.setAngle(Constants.SetPoints.IntakePosition.kUP);
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