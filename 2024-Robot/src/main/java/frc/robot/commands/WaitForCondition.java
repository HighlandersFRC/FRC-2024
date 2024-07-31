// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;

public class WaitForCondition extends Command {
  private final BooleanSupplier condition;
  public WaitForCondition(BooleanSupplier condition) {
    this.condition = condition;
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    Logger.recordOutput("started", condition.getAsBoolean());
    return condition.getAsBoolean();
  }
}
