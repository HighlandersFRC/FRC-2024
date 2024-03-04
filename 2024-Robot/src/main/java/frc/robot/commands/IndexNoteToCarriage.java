// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

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
  private double timeToCenterNote = 0.8;
  private double haveNoteTime;
  /** Creates a new IndexNoteToCarriage. */
  public IndexNoteToCarriage(Feeder feeder, Climber climber, Intake intake, TOF tof) {
    this.feeder = feeder;
    this.climber = climber;
    this.intake = intake;
    this.tof = tof;
    addRequirements(feeder, climber, intake);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.haveNoteTime = 0.0;
    this.haveNote = false;
    if (this.tof.getCarriageDistMillimeters() <= Constants.SetPoints.CARRIAGE_TOF_THRESHOLD_MM){
      this.haveNote = true;
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.println("tof: " + this.tof.getCarriageDistMillimeters());
    if (this.tof.getCarriageDistMillimeters() <= Constants.SetPoints.CARRIAGE_TOF_THRESHOLD_MM){
      if (!this.haveNote){
        this.haveNoteTime = Timer.getFPGATimestamp();
      }
      this.haveNote = true;
    }
    // if (haveNote){
    //   if (Timer.getFPGATimestamp() - haveNoteTime > timeToCenterNote){
    //     feeder.set(0.0);
    //   } else {
    //     feeder.set(-60);
    //   }
    // } else {
      feeder.set(-60);
      climber.setTrapRollerTorque(-15, 0.3);
      intake.setRollers(60);
    // }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    feeder.set(0.0);
    climber.setTrapRollerTorque(0, 0);
    intake.setRollers(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (Timer.getFPGATimestamp() - haveNoteTime > timeToCenterNote && this.haveNote){
      return true;
    } else {
      return false;
    }
  }
}
