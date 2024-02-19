// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Carriage extends SubsystemBase {
  private final TalonFX rollerKraken = new TalonFX(Constants.CANInfo.ELEVATOR_CARRIAGE_MOTOR_ID, Constants.CANInfo.CANBUS_NAME);
  private final TalonFXConfiguration rollerKrakenConfiguration = new TalonFXConfiguration();
  private final PositionTorqueCurrentFOC rollerKrakenPositionRequest = new PositionTorqueCurrentFOC(0, 0, 0, 0, false, false, false);

  private final Servo angleServo = new Servo(Constants.CANInfo.CARRIAGE_SERVO_CHANNEL);
  /** Creates a new Carriage. */
  public Carriage() {}

  public void init(){
    // this.rollerKrakenConfiguration.Slot0.kP = 1;
    // this.rollerKrakenConfiguration.Slot0.kI = 0;
    // this.rollerKrakenConfiguration.Slot0.kD = 0.1;
    // this.rollerKrakenConfiguration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    // this.rollerKraken.getConfigurator().apply(this.rollerKrakenConfiguration);
    // this.rollerKraken.setNeutralMode(NeutralModeValue.Brake);
    // this.rollerKraken.setPosition(0);
  }

  public void setPercent(double percent){
    this.rollerKraken.set(percent);
  }

  public void setServoAngle(double angle){
    angleServo.setAngle(angle);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
