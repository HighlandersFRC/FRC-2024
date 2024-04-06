package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.OI;
import frc.robot.sensors.Proximity;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class TrapIndexNoteToCarriage extends Command {
  private Feeder feeder;
  private Climber climber;
  private Intake intake;
  private Proximity proximity;
  private Shooter shooter;
  private boolean haveNote = false;
  private boolean noteInPlace = false;
  private boolean noteInMiddle = false;
  private double timeToCenterNote = 0.7;
  private double haveNoteTime;

  public TrapIndexNoteToCarriage(Feeder feeder, Climber climber, Intake intake, Proximity proximity, Shooter shooter) {
    this.feeder = feeder;
    this.climber = climber;
    this.intake = intake;
    this.proximity = proximity;
    this.shooter = shooter;
    this.timeToCenterNote = 0.7;
    addRequirements(feeder, climber, intake, shooter);
  }

  @Override
  public void initialize() {
    this.haveNoteTime = 0.0;
    this.haveNote = false;
    this.noteInPlace = false;
    this.noteInMiddle = false;
  }

  @Override
  public void execute() {
    if (this.proximity.getCarriageProximity() && !this.haveNote){
      this.haveNoteTime = Timer.getFPGATimestamp();
      this.haveNote = true;
    //   System.out.println("1");
    } else if (!this.proximity.getCarriageProximity() && this.haveNote){
        this.noteInMiddle = true;
        // System.out.println("2");
    } else if (this.proximity.getCarriageProximity() && this.noteInMiddle){
        this.noteInPlace = true;
        // System.out.println("3");
    }
    // System.out.println("haveNote: " + this.haveNote);
    // System.out.println("noteInPlace: " + this.noteInPlace);

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
    if (OI.getOperatorLB()){
      return true; 
    } else if (noteInPlace){
      return true;
    } else {
      return false;
    }
  }
}