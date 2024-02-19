// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Elevator extends SubsystemBase {
  private final TalonFX leftElevatorKraken = new TalonFX(Constants.CANInfo.ELEVATOR_LEFT_MOTOR_ID, Constants.CANInfo.CANBUS_NAME);
  private final TalonFXConfiguration leftElevatorKrakenConfiguration = new TalonFXConfiguration();
  private final PositionTorqueCurrentFOC leftElevatorKrakenPositionRequest = new PositionTorqueCurrentFOC(0, 0, 0, 0, false, false, false);

  private final TalonFX rightElevatorKraken = new TalonFX(Constants.CANInfo.ELEVATOR_RIGHT_MOTOR_ID, Constants.CANInfo.CANBUS_NAME);
  private final TalonFXConfiguration rightElevatorKrakenConfiguration = new TalonFXConfiguration();
  private final PositionTorqueCurrentFOC rightElevatorKrakenPositionRequest = new PositionTorqueCurrentFOC(0, 0, 0, 0, false, false, false);
  /** Creates a new Elevator. */
  public Elevator() {}

  public void init(){
    // this.leftElevatorKrakenConfiguration.Slot0.kP = 1;
    // this.leftElevatorKrakenConfiguration.Slot0.kI = 0;
    // this.leftElevatorKrakenConfiguration.Slot0.kD = 0.1;
    // this.leftElevatorKrakenConfiguration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    // this.leftElevatorKraken.getConfigurator().apply(this.leftElevatorKrakenConfiguration);
    // this.leftElevatorKraken.setNeutralMode(NeutralModeValue.Brake);
    // this.leftElevatorKraken.setPosition(0);

    // this.rightElevatorKrakenConfiguration.Slot0.kP = 1;
    // this.rightElevatorKrakenConfiguration.Slot0.kI = 0;
    // this.rightElevatorKrakenConfiguration.Slot0.kD = 0.1;
    // this.rightElevatorKrakenConfiguration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    // this.rightElevatorKraken.getConfigurator().apply(this.rightElevatorKrakenConfiguration);
    // this.rightElevatorKraken.setNeutralMode(NeutralModeValue.Brake);
    // this.rightElevatorKraken.setPosition(0);
  }

  public void setPercents(double left, double right){
    this.leftElevatorKraken.set(left);
    this.rightElevatorKraken.set(-right);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
