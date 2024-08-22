// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {
  private final TalonFX leftShooter = new TalonFX(Constants.CANInfo.SHOOTER_LEFT_MOTOR_ID, "Canivore");
  private final TalonFX rightShooter = new TalonFX(Constants.CANInfo.SHOOTER_RIGHT_MOTOR_ID, "Canivore");
  private final TalonFX shooterAngle = new TalonFX(Constants.CANInfo.SHOOTER_ANGLE_MOTOR_ID, "Canivore");

  /** Creates a new Shooter. */
  public Shooter() {}

  public void setShooterPercent(double left, double right){
    leftShooter.set(-left);
    rightShooter.set(-right);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
