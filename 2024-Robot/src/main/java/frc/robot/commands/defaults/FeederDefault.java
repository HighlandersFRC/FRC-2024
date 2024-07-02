package frc.robot.commands.defaults;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.OI;
import frc.robot.sensors.Proximity;
import frc.robot.subsystems.Feeder;

public class FeederDefault extends Command {
  private Feeder feeder;
  private Proximity proximity;

  private boolean haveNote;
  private boolean haveCarriageNote;
  private boolean haveFeederNote;

  public FeederDefault(Feeder feeder, Proximity proximity) {
    this.feeder = feeder;
    this.proximity = proximity;
    addRequirements(this.feeder);
  }

  @Override
  public void initialize() {
    this.feeder.setPercent(0);
    this.haveCarriageNote = false;
    this.haveNote = false;
    this.haveFeederNote = false;
    if (this.proximity.getShooterProximity()){
      this.haveNote = true;
    }
  }

  @Override
  public void execute() {
    if (this.proximity.getCarriageProximity()){
      this.haveCarriageNote = true;
    }

    if (this.proximity.getShooterProximity()){
      this.haveNote = true;
    }

    if (this.proximity.getFeederProximity()){
      this.haveFeederNote = true;
    }
    
     if (OI.getOperatorLB()){
      this.feeder.set(0);
    } else if (!this.proximity.getCarriageProximity() && !this.proximity.getShooterProximity() && !this.proximity.getFeederProximity()){
      // System.out.println("first");
      this.feeder.set(450);
    } else if (!this.proximity.getCarriageProximity() && !this.proximity.getShooterProximity() && this.proximity.getFeederProximity()){
      this.feeder.setPercent(0);
    } else if (this.haveNote && !this.proximity.getShooterProximity()){
      this.feeder.setPercent(0);
    } else if (this.haveCarriageNote && !this.haveNote && !this.haveFeederNote){
      // System.out.println("second");
      this.feeder.set(450);
    } else if (this.haveNote){
      // System.out.println("third");
      this.feeder.set(100);
    } else {
      this.feeder.setPercent(0.0);
    }
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
