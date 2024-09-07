// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
  private final TalonFX centeringIntake = new TalonFX(Constants.CANInfo.INTAKE_CENTER_MOTOR_ID, "Canivore");
  private final TalonFX intake = new TalonFX(Constants.CANInfo.INTAKE_ROLLER_MOTOR_ID, "Canivore");

  private final DigitalInput beamBreak = new DigitalInput(9);

  private final TalonFXConfiguration centeringConfiguration = new TalonFXConfiguration();
  private final TalonFXConfiguration rollerConfiguration = new TalonFXConfiguration();

public boolean getBeamBreak() {
  return beamBreak.get();
}

  /** Creates a new Intake. */
  public Intake() {}
  public void init() {
    this.centeringConfiguration.Slot0.kP = 10;    
    this.centeringConfiguration.Slot0.kI = 0;
    this.centeringConfiguration.Slot0.kD = 0.1;
    this.centeringConfiguration.Slot0.kS = 4;
    this.centeringConfiguration.CurrentLimits.StatorCurrentLimit = 60;
    this.centeringConfiguration.CurrentLimits.SupplyCurrentLimit = 60;
    this.centeringConfiguration.CurrentLimits.StatorCurrentLimitEnable = true;
    this.centeringConfiguration.CurrentLimits.SupplyCurrentLimitEnable = true;
    // this.centeringConfiguration.TorqueCurrent.PeakForwardTorqueCurrent = 50;
    // this.centeringConfiguration.TorqueCurrent.PeakReverseTorqueCurrent = -50;
    this.centeringIntake.getConfigurator().apply(this.centeringConfiguration);
    this.centeringIntake.setNeutralMode(NeutralModeValue.Coast);
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
