// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class ReverseFeeder extends Command {
  /** Creates a new ReverseFeeder. */
  Intake intake;
  Feeder feeder;
  Shooter shooter;
  public ReverseFeeder(Intake intake, Feeder feeder, Shooter shooter) {
    this.intake = intake;
    this.feeder = feeder;
    this.shooter = shooter;
    addRequirements(this.intake, this.feeder, this.shooter);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    intake.setPercent(-0.5);
    feeder.setPercent(-0.5);
    shooter.setShooterPercent(-0.5, -0.5);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.setPercent(0.0);
    feeder.setPercent(0.0);
    shooter.setShooterPercent(0.0, 0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
