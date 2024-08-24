// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Feeder extends SubsystemBase {
  private final TalonFX feeder = new TalonFX(Constants.CANInfo.FEEDER_ROLLER_MOTOR_ID, "Canivore");

  private final TalonFXConfiguration feederConfiguration = new TalonFXConfiguration();

  private final VelocityTorqueCurrentFOC rollerFalconVelocityRequest = new VelocityTorqueCurrentFOC(0, 0, 0, 0, false, false, false);
  /** Creates a new Feeder. */
  public Feeder() {}

  public void init(){
    this.feederConfiguration.Slot0.kP = 12.5;
    this.feederConfiguration.Slot0.kI = 0.0;
    this.feederConfiguration.Slot0.kD = 0.0;
    this.feederConfiguration.Slot0.kS = 0.0;
    this.feederConfiguration.CurrentLimits.StatorCurrentLimit = 60;
    this.feederConfiguration.CurrentLimits.SupplyCurrentLimit = 60;
    this.feederConfiguration.CurrentLimits.StatorCurrentLimitEnable = true;
    this.feederConfiguration.CurrentLimits.SupplyCurrentLimitEnable = true;
    this.feeder.getConfigurator().apply(this.feederConfiguration);
    this.feeder.setNeutralMode(NeutralModeValue.Brake);

  }

  public void setPercent(double percent){
    feeder.set(-percent);
  }

  public void setRPM(double RPM){
    feeder.setControl(rollerFalconVelocityRequest.withVelocity(Constants.RPMToRPS(RPM) * Constants.Ratios.FEEDER_ROLLER_GEAR_RATIO));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
