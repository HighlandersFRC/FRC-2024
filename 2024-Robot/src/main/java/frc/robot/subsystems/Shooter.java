// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
  private final TalonFX leftShooter = new TalonFX(13, "Canivore");
  private final TalonFX rightShooter = new TalonFX(14, "Canivore");

  /** Creates a new Shooter. */
  public Shooter() {}

  public void setShooterPercent(double left, double right){
    leftShooter.set(left);
    rightShooter.set(right);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
