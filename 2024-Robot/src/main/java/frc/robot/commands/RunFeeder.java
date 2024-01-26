// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Feeder;

public class RunFeeder extends Command {
  Feeder feeder;
  double RPM;

  public RunFeeder(Feeder feeder, double RPM) {
    this.feeder = feeder;
    this.RPM = RPM;
    addRequirements(feeder);
  }

  @Override
  public void initialize() {
    feeder.set(RPM);
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {
    feeder.setPercent(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
