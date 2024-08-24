// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class RunShooter extends Command {
  Shooter shooter;
  Feeder feeder;
  Intake intake;
  double left;
  double right;
  double startTime;
  /** Creates a new RunShooter. */
  public RunShooter(Shooter shooter, Feeder feeder, Intake intake, double left, double right) {
    this.shooter = shooter;
    this.feeder = feeder;
    this.intake = intake;
    this.left = left;
    this.right = right;
    addRequirements(shooter);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    startTime = Timer.getFPGATimestamp();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shooter.setShooterRPM(left, right);
    // if (Timer.getFPGATimestamp() - startTime > 2){
    //   feeder.setPercent(0.5);
    //   intake.setPercent(0.4);
    // }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.setShooterPercent(0.0, 0.0);
    feeder.setPercent(0.0);
    intake.setPercent(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
