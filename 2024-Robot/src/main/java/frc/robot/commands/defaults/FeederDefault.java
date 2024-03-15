package frc.robot.commands.defaults;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.sensors.Proximity;
import frc.robot.sensors.TOF;
import frc.robot.subsystems.Feeder;

public class FeederDefault extends Command {
  private Feeder feeder;
  private TOF tof;
  private Proximity proximity;

  private boolean haveNote;
  private boolean haveCarriageNote;

  public FeederDefault(Feeder feeder, TOF tof, Proximity proximity) {
    this.feeder = feeder;
    this.tof = tof;
    this.proximity = proximity;
    addRequirements(this.feeder);
  }

  @Override
  public void initialize() {
    System.out.println("Default starts");
    this.haveCarriageNote = false;
    this.haveNote = false;
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
    System.out.println("Have note: " + haveNote);
    // SmartDashboard.putBoolean("have note", haveNote);
    System.out.println("Have carriage note: " + haveCarriageNote);
    // SmartDashboard.putBoolean("have carriage note", haveCarriageNote);
    if (this.haveCarriageNote && !this.haveNote){
      this.feeder.set(120);
      System.out.println("1");
    } else if (this.haveNote && !this.proximity.getShooterProximity()){
      this.feeder.setPercent(0);
      System.out.println("2");
    } else if (this.haveNote){
      this.feeder.set(70);
      System.out.println("3");
    } else {
      this.feeder.setPercent(0.0);
      System.out.println("4");
    }
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
