// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;

public class RunElevator extends Command {
  Elevator elevator;
  double leftPosition;
  double rightPosition;

  /** Creates a new RunClimber. */
  public RunElevator(Elevator elevator, double leftPosition, double rightPosition) {
    this.elevator = elevator;
    this.leftPosition = leftPosition;
    this.rightPosition = rightPosition;
    addRequirements(elevator);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    this.elevator.setPercents(leftPosition, rightPosition);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    this.elevator.setPercents(0.0, 0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
