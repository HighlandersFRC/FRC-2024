// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
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

  private final TalonFX elevatorFalconFollower = new TalonFX(Constants.CANInfo.ELEVATOR_FOLLOWER_MOTOR_ID, Constants.CANInfo.CANBUS_NAME);
  private final TalonFXConfiguration elevatorFalconFollowerConfiguration = new TalonFXConfiguration();
  
  private final TalonFX elevatorFalconMaster = new TalonFX(Constants.CANInfo.ELEVATOR_MASTER_MOTOR_ID, Constants.CANInfo.CANBUS_NAME);
  private final TalonFXConfiguration elevatorFalconMasterConfiguration = new TalonFXConfiguration();
  private final PositionTorqueCurrentFOC elevatorFalconPositionRequest = new PositionTorqueCurrentFOC(0, 0, 0, 0, false, false, false);

  private final TalonFX trapRollerFalcon = new TalonFX(Constants.CANInfo.TRAP_ROLLER_MOTOR_ID, Constants.CANInfo.CANBUS_NAME);
  private final TalonFXConfiguration trapRollerFalconConfiguration = new TalonFXConfiguration();

  /** Creates a new Climber. */
  public Climber(Lights lights) {
    this.lights = lights;
    setDefaultCommand(new ClimberDefault(this));
  }

  public void init(){
    double elevatorFalconP = 0;
    double elevatorFalconI = 0;
    double elevatorFalconD = 0;

    this.elevatorFalconFollowerConfiguration.Slot0.kP = elevatorFalconP;
    this.elevatorFalconFollowerConfiguration.Slot0.kI = elevatorFalconI;
    this.elevatorFalconFollowerConfiguration.Slot0.kD = elevatorFalconD;
    this.elevatorFalconFollowerConfiguration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    this.elevatorFalconFollower.getConfigurator().apply(this.elevatorFalconFollowerConfiguration);
    this.elevatorFalconFollower.setNeutralMode(NeutralModeValue.Brake);
    this.elevatorFalconFollower.setPosition(0);

    this.elevatorFalconMasterConfiguration.Slot0.kP = elevatorFalconP;
    this.elevatorFalconMasterConfiguration.Slot0.kI = elevatorFalconI;
    this.elevatorFalconMasterConfiguration.Slot0.kD = elevatorFalconD;
    this.elevatorFalconMasterConfiguration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    this.elevatorFalconMaster.getConfigurator().apply(this.elevatorFalconMasterConfiguration);
    this.elevatorFalconMaster.setNeutralMode(NeutralModeValue.Brake);
    this.elevatorFalconMaster.setPosition(0);

    this.elevatorFalconFollower.setControl(new Follower(Constants.CANInfo.ELEVATOR_MASTER_MOTOR_ID, true));

    this.trapRollerFalconConfiguration.Slot0.kP = 0;

  }

  public void setElevatorPosition(double positionMeters){
    if (positionMeters > Constants.SetPoints.ELEVATOR_TOP_POSITION_M){
      this.elevatorFalconMaster.setControl(this.elevatorFalconPositionRequest.withPosition(Constants.SetPoints.elevatorMetersToRotations(Constants.SetPoints.ELEVATOR_TOP_POSITION_M)));
    } else if (positionMeters < Constants.SetPoints.ELEVATOR_BOTTOM_POSITION_M){
      this.elevatorFalconMaster.setControl(this.elevatorFalconPositionRequest.withPosition(Constants.SetPoints.elevatorMetersToRotations(Constants.SetPoints.ELEVATOR_BOTTOM_POSITION_M)));
    } else {
      this.elevatorFalconMaster.setControl(this.elevatorFalconPositionRequest.withPosition(Constants.SetPoints.elevatorMetersToRotations(positionMeters)));
    }
  }

  public void setElevatorPercent(double percent){
    this.elevatorFalconMaster.set(percent);
  }
  
  @Override
  public void periodic() {}
}
