// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.commands.defaults.ClimberDefault;
import frc.robot.commands.defaults.IntakeDefault;
import frc.robot.tools.EMBrake;

public class Climber extends SubsystemBase {
  private Lights lights;

  private final TalonFX leftClimberFalcon = new TalonFX(Constants.CANInfo.CLIMBER_LEFT_MOTOR_ID, Constants.CANInfo.CANBUS_NAME);
  private final TalonFXConfiguration leftClimberFalconConfiguration = new TalonFXConfiguration();
  private final PositionTorqueCurrentFOC leftClimberFalconPositionRequest = new PositionTorqueCurrentFOC(0, 0, 0, 0, false, false, false);

  private final TalonFX rightClimberFalcon = new TalonFX(Constants.CANInfo.CLIMBER_RIGHT_MOTOR_ID, Constants.CANInfo.CANBUS_NAME);
  private final TalonFXConfiguration rightClimberFalconConfiguration = new TalonFXConfiguration();
  private final PositionTorqueCurrentFOC rightClimberFalconPositionRequest = new PositionTorqueCurrentFOC(0, 0, 0, 0, false, false, false);

  private final EMBrake brake = new EMBrake(Constants.CANInfo.CLIMBER_BRAKE_PORT);

  /** Creates a new Climber. */
  public Climber(Lights lights) {
    this.lights = lights;
    // setDefaultCommand(new ClimberDefault(this));
  }

  public void init(){
    // this.leftClimberFalconConfiguration.Slot0.kP = 1;
    // this.leftClimberFalconConfiguration.Slot0.kI = 0;
    // this.leftClimberFalconConfiguration.Slot0.kD = 0.1;
    // this.leftClimberFalconConfiguration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    // this.leftClimberFalcon.getConfigurator().apply(this.leftClimberFalconConfiguration);
    // this.leftClimberFalcon.setNeutralMode(NeutralModeValue.Brake);
    // this.leftClimberFalcon.setPosition(0);

    // this.rightClimberFalconConfiguration.Slot0.kP = 1;
    // this.rightClimberFalconConfiguration.Slot0.kI = 0;
    // this.rightClimberFalconConfiguration.Slot0.kD = 0.1;
    // this.rightClimberFalconConfiguration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    // this.rightClimberFalcon.getConfigurator().apply(this.rightClimberFalconConfiguration);
    // this.rightClimberFalcon.setNeutralMode(NeutralModeValue.Brake);
    // this.rightClimberFalcon.setPosition(0);
  }

  public void setPercents(double left, double right){
    this.leftClimberFalcon.set(left);
    this.rightClimberFalcon.set(-right);
  }

  public void lock(){
    brake.lock();
  }
  public void unlock(){
    brake.unlock();
  }
  public void toggle(){
    brake.toggle();
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
