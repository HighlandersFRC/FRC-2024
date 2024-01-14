// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
  /** Creates a new Shooter. */
  // TalonFX shooterLeft = new TalonFX(11, "Canivore");
  // TalonFX shooterRight = new TalonFX(12, "Canivore");
  TalonFX shooter2Prototype = new TalonFX(12, "Canivore");

  public Shooter() {}

  public void setPercent(double left){
    // shooterLeft.set(-left);
    // shooterRight.set(right);
    shooter2Prototype.set(left);
  }

  @Override
  public void periodic() {
    // SmartDashboard.putNumber("Shooter Speed", shooter2Prototype.getRotorVelocity().getValue());
    // This method will be called once per scheduler run
  }
}
