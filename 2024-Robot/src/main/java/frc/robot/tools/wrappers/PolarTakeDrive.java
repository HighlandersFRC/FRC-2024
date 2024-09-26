// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.tools.wrappers;

import org.json.JSONObject;

import edu.wpi.first.wpilibj2.command.Command;

public abstract class PolarTakeDrive extends Command {
  /** Creates a new PolarTakeDrive. */
  public abstract int getPathPointIndex();

  public abstract void initialize();

  @Override
  public abstract void execute();

  @Override
  public abstract void end(boolean interrupted);

  public abstract void from(int pointIndex, JSONObject pathJSON, int toIndex);

  @Override
  abstract public boolean isFinished();
}
