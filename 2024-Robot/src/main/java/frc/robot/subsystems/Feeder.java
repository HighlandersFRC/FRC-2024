// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Feeder extends SubsystemBase {
    private final TalonFX feeder = new TalonFX(Constants.CANInfo.FEEDER_ROLLER_MOTOR_ID, "Canivore");
  /** Creates a new Feeder. */
  public Feeder() {}

  public void setPercent(double percent){
    feeder.set(-percent);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
