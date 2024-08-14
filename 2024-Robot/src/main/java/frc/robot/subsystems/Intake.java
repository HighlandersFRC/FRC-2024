// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
  private final TalonFX centeringIntake = new TalonFX(10, "Canivore");
  private final TalonFX intake = new TalonFX(11, "Canivore");

  /** Creates a new Intake. */
  public Intake() {

  }

  public void setPercent(double percent){
    centeringIntake.set(percent);
    intake.set(-percent);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
