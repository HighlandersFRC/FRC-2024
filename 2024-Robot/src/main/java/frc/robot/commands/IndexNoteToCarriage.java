package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.sensors.TOF;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;

public class IndexNoteToCarriage extends Command {
  private Feeder feeder;
  private Climber climber;
  private Intake intake;
  private TOF tof;
  private boolean haveNote = false;
  private double timeToCenterNote = 0.55;
  private double haveNoteTime;
  public IndexNoteToCarriage(Feeder feeder, Climber climber, Intake intake, TOF tof) {
    this.feeder = feeder;
    this.climber = climber;
    this.intake = intake;
    this.tof = tof;
    addRequirements(feeder, climber, intake);
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

    this.feeder.set(-150);
    this.climber.setTrapRollerTorque(-20, 0.5);
    this.intake.setRollers(-150);
    this.intake.setAngleTorqueCurrent(15, 0.6);
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