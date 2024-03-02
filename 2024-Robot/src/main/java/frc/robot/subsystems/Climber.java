// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.commands.defaults.ClimberDefault;
import frc.robot.commands.defaults.IntakeDefault;
import frc.robot.sensors.TOF;
import frc.robot.tools.EMBrake;
import frc.robot.tools.controlloops.PID;

public class Climber extends SubsystemBase {
  private Lights lights;

  private final TalonFX elevatorFalconFollower = new TalonFX(Constants.CANInfo.ELEVATOR_FOLLOWER_MOTOR_ID, Constants.CANInfo.CANBUS_NAME);
  private final TalonFXConfiguration elevatorFalconFollowerConfiguration = new TalonFXConfiguration();
  
  private final TalonFX elevatorFalconMaster = new TalonFX(Constants.CANInfo.ELEVATOR_MASTER_MOTOR_ID, Constants.CANInfo.CANBUS_NAME);
  private final TalonFXConfiguration elevatorFalconMasterConfiguration = new TalonFXConfiguration();
  private final PositionTorqueCurrentFOC elevatorFalconPositionRequest = new PositionTorqueCurrentFOC(0, 0, 0, 0, false, false, false);
  private final TorqueCurrentFOC elevatorFalconTorqueRequest = new TorqueCurrentFOC(0, 0, 0, false, false, false);

  private final TalonFX trapRollerFalcon = new TalonFX(Constants.CANInfo.TRAP_ROLLER_MOTOR_ID);
  private final TalonFXConfiguration trapRollerFalconConfiguration = new TalonFXConfiguration();
  private final TorqueCurrentFOC trapRollerFalconTorqueRequest = new TorqueCurrentFOC(0, 0, 0, false, false, false);

  private final CANSparkMax carriageRotationNeo = new CANSparkMax(Constants.CANInfo.CARRIAGE_ROTATION_MOTOR_ID, MotorType.kBrushless);
  private final CANcoder rotationCanCoder = new CANcoder(Constants.CANInfo.CARRIAGE_ROTATION_CANCODER_ID, Constants.CANInfo.CANBUS_NAME);

  private double carriageRotationSetpoint = Constants.SetPoints.CARRIAGE_BOTTOM_ROTATION_DEG;
  private final PID rotationPID;
  private final double kP = 0.0;
  private final double kI = 0.0;
  private final double kD = 0.0;
  private final double kG = 0.0;

  /** Creates a new Climber. */
  public Climber(Lights lights) {
    this.lights = lights;
    setDefaultCommand(new ClimberDefault(this));

    this.rotationPID = new PID(this.kP, this.kI, this.kD);
    this.rotationPID.setMaxOutput(0.2);
    this.rotationPID.setMinOutput(-0.2);
    this.rotationPID.setSetPoint(Constants.SetPoints.CARRIAGE_BOTTOM_ROTATION_DEG);
    this.rotationPID.updatePID(getCarriageRotationDegrees());
  }

  public void init(){
    double elevatorFalconP = 0;
    double elevatorFalconI = 0;
    double elevatorFalconD = 0;

    this.elevatorFalconFollowerConfiguration.Slot0.kP = elevatorFalconP;
    this.elevatorFalconFollowerConfiguration.Slot0.kI = elevatorFalconI;
    this.elevatorFalconFollowerConfiguration.Slot0.kD = elevatorFalconD;
    this.elevatorFalconFollowerConfiguration.CurrentLimits.StatorCurrentLimitEnable = true;
    this.elevatorFalconFollowerConfiguration.CurrentLimits.SupplyCurrentLimitEnable = true;
    this.elevatorFalconFollowerConfiguration.CurrentLimits.StatorCurrentLimit = 60;
    this.elevatorFalconFollowerConfiguration.CurrentLimits.SupplyCurrentLimit = 60;
    this.elevatorFalconFollowerConfiguration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    this.elevatorFalconFollower.getConfigurator().apply(this.elevatorFalconFollowerConfiguration);
    this.elevatorFalconFollower.setNeutralMode(NeutralModeValue.Brake);
    this.elevatorFalconFollower.setPosition(0);

    this.elevatorFalconMasterConfiguration.Slot0.kP = elevatorFalconP;
    this.elevatorFalconMasterConfiguration.Slot0.kI = elevatorFalconI;
    this.elevatorFalconMasterConfiguration.Slot0.kD = elevatorFalconD;
    this.elevatorFalconMasterConfiguration.CurrentLimits.StatorCurrentLimitEnable = true;
    this.elevatorFalconMasterConfiguration.CurrentLimits.SupplyCurrentLimitEnable = true;
    this.elevatorFalconMasterConfiguration.CurrentLimits.StatorCurrentLimit = 60;
    this.elevatorFalconMasterConfiguration.CurrentLimits.SupplyCurrentLimit = 60;
    this.elevatorFalconMasterConfiguration.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    this.elevatorFalconMaster.getConfigurator().apply(this.elevatorFalconMasterConfiguration);
    this.elevatorFalconMaster.setNeutralMode(NeutralModeValue.Brake);
    this.elevatorFalconMaster.setPosition(0);

    // this.elevatorFalconFollower.setControl(new Follower(Constants.CANInfo.ELEVATOR_MASTER_MOTOR_ID, true));

    this.trapRollerFalconConfiguration.Slot0.kP = 0;
    this.trapRollerFalconConfiguration.Slot0.kI = 0;
    this.trapRollerFalconConfiguration.Slot0.kD = 0;
    this.trapRollerFalconConfiguration.CurrentLimits.StatorCurrentLimitEnable = true;
    this.trapRollerFalconConfiguration.CurrentLimits.SupplyCurrentLimitEnable = true;
    this.trapRollerFalconConfiguration.CurrentLimits.StatorCurrentLimit = 60;
    this.trapRollerFalconConfiguration.CurrentLimits.SupplyCurrentLimit = 60;
    this.trapRollerFalconConfiguration.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    this.trapRollerFalcon.getConfigurator().apply(this.trapRollerFalconConfiguration);
    this.trapRollerFalcon.setNeutralMode(NeutralModeValue.Brake);
  }

  public void setElevatorPositionMeters(double positionMeters){
    if (positionMeters > Constants.SetPoints.ELEVATOR_TOP_POSITION_M){
      this.elevatorFalconMaster.setControl(this.elevatorFalconPositionRequest.withPosition(Constants.Ratios.elevatorMetersToRotations(Constants.SetPoints.ELEVATOR_TOP_POSITION_M)));
    } else if (positionMeters < Constants.SetPoints.ELEVATOR_BOTTOM_POSITION_M){
      this.elevatorFalconMaster.setControl(this.elevatorFalconPositionRequest.withPosition(Constants.Ratios.elevatorMetersToRotations(Constants.SetPoints.ELEVATOR_BOTTOM_POSITION_M)));
    } else {
      this.elevatorFalconMaster.setControl(this.elevatorFalconPositionRequest.withPosition(Constants.Ratios.elevatorMetersToRotations(positionMeters)));
    }
  }

  public void setElevatorPositionRotations(double positionRotations){
    if (positionRotations > Constants.SetPoints.ELEVATOR_TOP_POSITION_M){
      this.elevatorFalconMaster.setControl(this.elevatorFalconPositionRequest.withPosition(Constants.SetPoints.ELEVATOR_TOP_POSITION_M));
    } else if (positionRotations < Constants.SetPoints.ELEVATOR_BOTTOM_POSITION_M){
      this.elevatorFalconMaster.setControl(this.elevatorFalconPositionRequest.withPosition(Constants.SetPoints.ELEVATOR_BOTTOM_POSITION_M));
    } else {
      this.elevatorFalconMaster.setControl(this.elevatorFalconPositionRequest.withPosition(positionRotations));
    }
  }

  public void setElevatorTorque(double current, double maxPercent){
    this.elevatorFalconMaster.setControl(this.elevatorFalconTorqueRequest.withOutput(current).withMaxAbsDutyCycle(maxPercent));
    this.elevatorFalconFollower.setControl(this.elevatorFalconTorqueRequest.withOutput(-current).withMaxAbsDutyCycle(maxPercent));
  }

  public void setElevatorPercent(double percent){
    this.elevatorFalconMaster.set(percent);
  }

  public void setTrapRollerPercent(double percent){
    this.trapRollerFalcon.set(percent);
  }

  public void setTrapRollerTorque(double current, double maxPercent){
    this.trapRollerFalcon.setControl(this.trapRollerFalconTorqueRequest.withOutput(current).withMaxAbsDutyCycle(maxPercent));
  }

  public void setCarriageRotationDegrees(double degrees){
    if (getElevatorPositionMeters() < Constants.SetPoints.CARRIAGE_CLEARANCE_ELEVATOR_HEIGH_M){
      this.carriageRotationSetpoint = Constants.SetPoints.CARRIAGE_BOTTOM_ROTATION_DEG;
      return;
    }
    if (degrees < Constants.SetPoints.CARRIAGE_BOTTOM_ROTATION_DEG){
      this.carriageRotationSetpoint = Constants.SetPoints.CARRIAGE_BOTTOM_ROTATION_DEG;
    } else if (degrees > Constants.SetPoints.CARRIAGE_TOP_ROTATION_DEG){
      this.carriageRotationSetpoint = Constants.SetPoints.CARRIAGE_TOP_ROTATION_DEG;
    } else {
      this.carriageRotationSetpoint = degrees;
    }
  }

  public void setCarriageRotationDegrees(Constants.SetPoints.CarriageRotation carriageRotation){
    double degrees = carriageRotation.degrees;
    if (getElevatorPositionMeters() < Constants.SetPoints.CARRIAGE_CLEARANCE_ELEVATOR_HEIGH_M){
      this.carriageRotationSetpoint = Constants.SetPoints.CARRIAGE_BOTTOM_ROTATION_DEG;
      return;
    }
    if (degrees < Constants.SetPoints.CARRIAGE_BOTTOM_ROTATION_DEG){
      this.carriageRotationSetpoint = Constants.SetPoints.CARRIAGE_BOTTOM_ROTATION_DEG;
    } else if (degrees > Constants.SetPoints.CARRIAGE_TOP_ROTATION_DEG){
      this.carriageRotationSetpoint = Constants.SetPoints.CARRIAGE_TOP_ROTATION_DEG;
    } else {
      this.carriageRotationSetpoint = degrees;
    }
  }

  public void setCarriageRotationPercent(double percent){
    this.carriageRotationNeo.set(percent);
  }

  public double getElevatorPositionMeters(){
    return Constants.Ratios.elevatorRotationsToMeters(this.elevatorFalconMaster.getPosition().getValueAsDouble());
  }

  public double getElevatorPositionRotations(){
    return this.elevatorFalconMaster.getPosition().getValueAsDouble();
  }

  public double getTrapRollerRPM(){
    return this.trapRollerFalcon.getVelocity().getValueAsDouble() / Constants.Ratios.TRAP_ROLLER_GEAR_RATIO;
  }
  
  public double getElevatorCurrent(){
    return this.elevatorFalconMaster.getStatorCurrent().getValueAsDouble();
  }

  public double getElevatorVelocityMPS(){
    return Constants.Ratios.elevatorRotationsToMeters(this.elevatorFalconMaster.getVelocity().getValueAsDouble());
  }

  public double getElevatorVelocityRPS(){
    return this.elevatorFalconMaster.getVelocity().getValueAsDouble();
  }

  public double getCarriageRotationDegrees(){
    return this.rotationCanCoder.getAbsolutePosition().getValueAsDouble();
  }

  @Override
  public void periodic() {
    boolean climbMaster = false;
    boolean climbFollower = false;
    SmartDashboard.putNumber("Elevator Meters", getElevatorPositionMeters());

    if(elevatorFalconMaster.getMotorVoltage().getValue() != 0){
      climbMaster = true;
    }
    if(elevatorFalconFollower.getMotorVoltage().getValue() != 0){
      climbFollower = true;
    }

    SmartDashboard.putBoolean(" Climber Master Motor", climbMaster);
    SmartDashboard.putBoolean(" Climber Follower Motor", climbFollower);
  }

  public void teleopPeriodic(){
    //DO NOT REMOVE FOR COMP
    this.rotationPID.setSetPoint(this.carriageRotationSetpoint);
    this.rotationPID.updatePID(getCarriageRotationDegrees());
    double result = this.rotationPID.getResult() + Math.cos(Math.toRadians(getCarriageRotationDegrees())) * this.kG;
    setCarriageRotationPercent(result);
    //DO NOT REMOVE FOR COMP
  }
}
