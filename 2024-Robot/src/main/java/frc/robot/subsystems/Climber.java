// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.lang.annotation.Documented;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkMax;
import com.revrobotics.REVLibError;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.commands.defaults.ClimberDefault;
import frc.robot.commands.defaults.IntakeDefault;
import frc.robot.sensors.Proximity;
import frc.robot.sensors.TOF;
import frc.robot.tools.EMBrake;
import frc.robot.tools.controlloops.PID;

public class Climber extends SubsystemBase {
  private Lights lights;

  private final TalonFX elevatorFalconFollower = new TalonFX(Constants.CANInfo.ELEVATOR_FOLLOWER_MOTOR_ID, Constants.CANInfo.CANBUS_NAME);
  private final TalonFXConfiguration elevatorFalconFollowerConfiguration = new TalonFXConfiguration();
  
  private final TalonFX elevatorFalconMaster = new TalonFX(Constants.CANInfo.ELEVATOR_MASTER_MOTOR_ID, Constants.CANInfo.CANBUS_NAME);
  private final TalonFXConfiguration elevatorFalconMasterConfiguration = new TalonFXConfiguration();
  private final MotionMagicTorqueCurrentFOC elevatorFalconPositionRequest = new MotionMagicTorqueCurrentFOC(0, 0, 0, false, false, false);
  private final TorqueCurrentFOC elevatorFalconTorqueRequest = new TorqueCurrentFOC(0, 0, 0, false, false, false);

  private final TalonFX trapRollerFalcon = new TalonFX(Constants.CANInfo.TRAP_ROLLER_MOTOR_ID);
  private final TalonFXConfiguration trapRollerFalconConfiguration = new TalonFXConfiguration();
  private final TorqueCurrentFOC trapRollerFalconTorqueRequest = new TorqueCurrentFOC(0, 0, 0, false, false, false);

  private final CANSparkMax carriageRotationNeo = new CANSparkMax(Constants.CANInfo.CARRIAGE_ROTATION_MOTOR_ID, MotorType.kBrushless);
  private final RelativeEncoder carriageEncoder;
  private final CANcoder rotationCanCoder = new CANcoder(Constants.CANInfo.CARRIAGE_ROTATION_CANCODER_ID);
  DigitalInput elevatorLimitSwitch = new DigitalInput(0);

  private double carriageRotationSetpoint = Constants.SetPoints.CARRIAGE_BOTTOM_ROTATION_DEG;
  private final PID rotationPID;
  private final double kP = 0.012;
  private final double kI = 0.0;
  private final double kD = 0.0;
  private final double kG = 0.015;

  /**
   * Checks the status of various components related to the climber mechanism.
   *
   * @return {@code true} if all components are functioning correctly; {@code false} otherwise.
  */
  public boolean getClimberCAN() {
    if(elevatorFalconFollower.clearStickyFault_BootDuringEnable() == StatusCode.OK && elevatorFalconMaster.clearStickyFault_BootDuringEnable() == StatusCode.OK && trapRollerFalcon.clearStickyFault_BootDuringEnable() == StatusCode.OK && rotationCanCoder.clearStickyFault_BadMagnet() == StatusCode.OK && carriageRotationNeo.setIdleMode(IdleMode.kBrake) == REVLibError.kOk) {
      return true;
    } else return false;
  }

  /**
   * Constructs a new instance of the Climber.
   * 
   * @param lights The lights subsystem.
   * @param tof The Time-of-Flight (TOF) sensor used by the Climber.
   */
  public Climber(Lights lights, TOF tof, Proximity proximity) {
    this.lights = lights;
    setDefaultCommand(new ClimberDefault(this, tof, proximity));

    this.rotationPID = new PID(this.kP, this.kI, this.kD);
    this.rotationPID.setMaxOutput(1);
    this.rotationPID.setMinOutput(-1);
    this.rotationPID.setSetPoint(Constants.SetPoints.CARRIAGE_BOTTOM_ROTATION_DEG);
    this.rotationPID.updatePID(getCarriageRotations());
    carriageEncoder = carriageRotationNeo.getEncoder();
  }

  public void init(){
    double elevatorFalconP = 0;
    double elevatorFalconI = 0;
    double elevatorFalconD = 0;

    double elevatorFalconAccel = 0.0;
    double elevatorFalconCruiseVel = 0.0;
    double elevatorFalconJerk = 0.0;

    this.elevatorFalconFollowerConfiguration.Slot0.kP = elevatorFalconP;
    this.elevatorFalconFollowerConfiguration.Slot0.kI = elevatorFalconI;
    this.elevatorFalconFollowerConfiguration.Slot0.kD = elevatorFalconD;
    this.elevatorFalconFollowerConfiguration.MotionMagic.MotionMagicAcceleration = elevatorFalconAccel;
    this.elevatorFalconFollowerConfiguration.MotionMagic.MotionMagicCruiseVelocity = elevatorFalconCruiseVel;
    this.elevatorFalconFollowerConfiguration.MotionMagic.MotionMagicJerk = elevatorFalconJerk;
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
    this.elevatorFalconMasterConfiguration.MotionMagic.MotionMagicAcceleration = elevatorFalconAccel;
    this.elevatorFalconMasterConfiguration.MotionMagic.MotionMagicCruiseVelocity = elevatorFalconCruiseVel;
    this.elevatorFalconMasterConfiguration.MotionMagic.MotionMagicJerk = elevatorFalconJerk;
    this.elevatorFalconMasterConfiguration.CurrentLimits.StatorCurrentLimitEnable = true;
    this.elevatorFalconMasterConfiguration.CurrentLimits.SupplyCurrentLimitEnable = true;
    this.elevatorFalconMasterConfiguration.CurrentLimits.StatorCurrentLimit = 60;
    this.elevatorFalconMasterConfiguration.CurrentLimits.SupplyCurrentLimit = 60;
    this.elevatorFalconMasterConfiguration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    this.elevatorFalconMaster.getConfigurator().apply(this.elevatorFalconMasterConfiguration);
    this.elevatorFalconMaster.setNeutralMode(NeutralModeValue.Brake);
    this.elevatorFalconMaster.setPosition(0);

    // this.elevatorFalconFollower.setControl(new Follower(Constants.CANInfo.ELEVATOR_MASTER_MOTOR_ID, false));

    this.trapRollerFalconConfiguration.Slot0.kP = 0;
    this.trapRollerFalconConfiguration.Slot0.kI = 0;
    this.trapRollerFalconConfiguration.Slot0.kD = 0;
    this.trapRollerFalconConfiguration.CurrentLimits.StatorCurrentLimitEnable = true;
    this.trapRollerFalconConfiguration.CurrentLimits.SupplyCurrentLimitEnable = true;
    this.trapRollerFalconConfiguration.CurrentLimits.StatorCurrentLimit = 60;
    this.trapRollerFalconConfiguration.CurrentLimits.SupplyCurrentLimit = 60;
    this.trapRollerFalconConfiguration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    this.trapRollerFalcon.getConfigurator().apply(this.trapRollerFalconConfiguration);
    this.trapRollerFalcon.setNeutralMode(NeutralModeValue.Brake);
  }

  /**
   * Sets the elevator position based on the specified position in meters.
   *
   * @param positionMeters The desired elevator position in meters.
  */
  public void setElevatorPositionMeters(double positionMeters){
    // if (positionMeters > Constants.SetPoints.ELEVATOR_TOP_POSITION_M){
    //   this.elevatorFalconMaster.setControl(this.elevatorFalconPositionRequest.withPosition(Constants.Ratios.elevatorMetersToRotations(Constants.SetPoints.ELEVATOR_TOP_POSITION_M)));
    // } else if (positionMeters < Constants.SetPoints.ELEVATOR_BOTTOM_POSITION_M){
    //   this.elevatorFalconMaster.setControl(this.elevatorFalconPositionRequest.withPosition(Constants.Ratios.elevatorMetersToRotations(Constants.SetPoints.ELEVATOR_BOTTOM_POSITION_M)));
    // } else {
    //   this.elevatorFalconMaster.setControl(this.elevatorFalconPositionRequest.withPosition(Constants.Ratios.elevatorMetersToRotations(positionMeters)));
    // }
    if (positionMeters < getElevatorPositionMeters() + 0.02 && positionMeters > getElevatorPositionMeters() - 0.02){
      setElevatorTorque(0.0, 0.0);
    } else {
      if (positionMeters > getElevatorPositionMeters()){
        setElevatorTorque(20, 0.5);
      } else {
        setElevatorTorque(-20, 0.5);
      }
    }
  }

  /**
   * Sets the elevator position based on the specified elevator position in meters.
   *
   * @param elevatorPosition The desired elevator position.
  */
  public void setElevatorPosition(Constants.SetPoints.ElevatorPosition elevatorPosition){
    double positionMeters = elevatorPosition.meters;
    if (positionMeters < getElevatorPositionMeters() + 0.02 && positionMeters > getElevatorPositionMeters() - 0.02){
      setElevatorTorque(0.0, 0.0);
    } else {
      if (positionMeters > getElevatorPositionMeters()){
        setElevatorTorque(20, 0.5);
      } else {
        setElevatorTorque(-20, 0.5);
      }
    }
  }

  /**
   * Sets the elevator position in rotations.
   *
   * @param positionRotations The desired position of the elevator in rotations.
  */
  public void setElevatorPositionRotations(double positionRotations){
    // if (positionRotations > Constants.SetPoints.ELEVATOR_TOP_POSITION_M){
    //   this.elevatorFalconMaster.setControl(this.elevatorFalconPositionRequest.withPosition(Constants.SetPoints.ELEVATOR_TOP_POSITION_M));
    // } else if (positionRotations < Constants.SetPoints.ELEVATOR_BOTTOM_POSITION_M){
    //   this.elevatorFalconMaster.setControl(this.elevatorFalconPositionRequest.withPosition(Constants.SetPoints.ELEVATOR_BOTTOM_POSITION_M));
    // } else {
    //   this.elevatorFalconMaster.setControl(this.elevatorFalconPositionRequest.withPosition(positionRotations));
    // }
    if (positionRotations == getElevatorPositionRotations()){
      setElevatorTorque(0.0, 0.0);
    } else {
      if (positionRotations > getElevatorPositionRotations()){
        setElevatorTorque(-50, 0.5);
      } else {
        setElevatorTorque(20, 0.5);
      }
    }
  }

  /**
   * Sets the torque control parameters for both the elevator Falcon Master and Follower.
   *
   * @param current The desired current output in amps.
   * @param maxPercent The maximum absolute duty cycle percentage.
  */
  public void setElevatorTorque(double current, double maxPercent){
    this.elevatorFalconMaster.setControl(this.elevatorFalconTorqueRequest.withOutput(current).withMaxAbsDutyCycle(maxPercent));
    this.elevatorFalconFollower.setControl(this.elevatorFalconTorqueRequest.withOutput(current).withMaxAbsDutyCycle(maxPercent));
  }

  /**
   * Sets the output percentage of the elevator Falcon Master.
   *
   * @param percent The desired output percentage of the elevator Falcon Master.
  */
  public void setElevatorPercent(double percent){
    this.elevatorFalconMaster.set(percent);
    this.elevatorFalconFollower.set(percent);
  }

  /**
   * Sets the encoder position for both the elevator Falcon Master and Follower.
   *
   * @param position The desired encoder position for the elevator.
  */
  public void setElevatorEncoderPosition(double position){
    this.elevatorFalconMaster.setPosition(position);
    this.elevatorFalconFollower.setPosition(position);
  }

  /**
   * Sets the output percentage of the trap roller Falcon.
   *
   * @param percent The desired output percentage of the trap roller Falcon.
  */
  public void setTrapRollerPercent(double percent){
    this.trapRollerFalcon.set(percent);
  }

  /**
   * Sets the torque control parameters for the trap roller Falcon.
   *
   * @param current The desired current output in amps.
   * @param maxPercent The maximum absolute duty cycle percentage.
  */
  public void setTrapRollerTorque(double current, double maxPercent){
    this.trapRollerFalcon.setControl(this.trapRollerFalconTorqueRequest.withOutput(current).withMaxAbsDutyCycle(maxPercent));
  }

  /**
   * Sets the setpoint for the carriage rotation in degrees.
   *
   * @param degrees The desired angle for the carriage rotation in degrees.
  */
  public void setCarriageRotationDegrees(double degrees){
    this.carriageRotationSetpoint = degrees;
  }

  /**
   * Sets the setpoint for the carriage rotation.
   *
   * @param carriageRotation The desired carriage rotation setpoint.
  */
  public void setCarriageRotation(Constants.SetPoints.CarriageRotation carriageRotation){
    double degrees = carriageRotation.degrees;
    this.carriageRotationSetpoint = degrees;
  }

  /**
   * Resets the position of the elevator to zero.
  */
  public void zeroElevator(){
    this.elevatorFalconMaster.setPosition(0.0);
    this.elevatorFalconFollower.setPosition(0.0);
  }

  /**
   * Sets the percentage of rotation for the carriage.
   *
   * @param percent The desired percentage of rotation for the carriage.
  */
  public void setCarriageRotationPercent(double percent){
    this.carriageRotationNeo.set(percent);
  }

  /**
   * Retrieves the position of the elevator in meters.
   *
   * @return The position of the elevator in meters.
  */
  public double getElevatorPositionMeters(){
    return Constants.Ratios.elevatorRotationsToMeters(this.elevatorFalconMaster.getPosition().getValueAsDouble());
  }

  /**
   * Retrieves the position of the elevator in rotations.
   *
   * @return The position of the elevator in rotations.
  */
  public double getElevatorPositionRotations(){
    return this.elevatorFalconMaster.getPosition().getValueAsDouble();
  }

  /**
   * Retrieves the velocity of the trap roller in Rotations Per Second (RPS).
   *
   * @return The velocity of the trap roller in RPS.
  */
  public double getTrapRollerRPS(){
    return this.trapRollerFalcon.getVelocity().getValueAsDouble() / Constants.Ratios.TRAP_ROLLER_GEAR_RATIO;
  }
  
  /**
   * Retrieves the stator current of the elevator.
   *
   * @return The stator current of the elevator.
  */
  public double getElevatorCurrent(){
    return this.elevatorFalconMaster.getStatorCurrent().getValueAsDouble();
  }

  /**
   * Retrieves the velocity of the elevator in Meters Per Second (MPS).
   *
   * @return The velocity of the elevator in MPS.
  */
  public double getElevatorVelocityMPS(){
    return Constants.Ratios.elevatorRotationsToMeters(this.elevatorFalconMaster.getVelocity().getValueAsDouble());
  }

  /**
   * Retrieves the velocity of the elevator in Rotations Per Second (RPS).
   *
   * @return The velocity of the elevator in RPS.
  */
  public double getElevatorVelocityRPS(){
    return this.elevatorFalconMaster.getVelocity().getValueAsDouble();
  }

  /**
   * Retrieves the rotation angle of the carriage in rotations.
   *
   * @return The rotation angle of the carriage in rotations.
  */
  public double getCarriageRotations(){
    return this.rotationCanCoder.getPosition().getValueAsDouble();
  }

  /**
   * Retrieves the rotation angle of the carriage in degrees.
   *
   * @return The rotation angle of the carriage in degrees.
  */
  public double getCarriageRotationDegrees(){
    return getCarriageRotations() * 360.0;
  }

  /**
   * Retrieves the state of the elevator limit switch.
   *
   * @return {@code true} if the elevator limit switch is not triggered, {@code false} otherwise.
  */
  public boolean getElevatorLimitSwitch(){
    //default returns false when triggered so this flips it
    return elevatorLimitSwitch.get();
    // if (elevatorLimitSwitch.get()){
    //   return false;
    // } else {
    //   return true;
    // }
  }

  @Override
  public void periodic() {
    // boolean climbMaster = false;
    // boolean climbFollower = false;
    // SmartDashboard.putNumber("Elevator Meters", getElevatorPositionMeters());
    // SmartDashboard.putNumber("Elevator Rotations", getElevatorPositionRotations());
    // Logger.recordOutput("Elevator Meters", getElevatorPositionMeters());
    // Logger.recordOutput("Elevator Rotations", getElevatorPositionRotations());
    // if(elevatorFalconMaster.getMotorVoltage().getValue() != 0){
    //   climbMaster = true;
    // }
    // if(elevatorFalconFollower.getMotorVoltage().getValue() != 0){
    //   climbFollower = true;
    // }

    SmartDashboard.putBoolean("Elevator Limit Switch", elevatorLimitSwitch.get());

    // if (getElevatorLimitSwitch()){
    //   setElevatorEncoderPosition(0.0);
    // }
    // System.out.println("Carriage RPM: " + carriageEncoder.getVelocity());
    // System.out.println("Carriage amps: " + carriageRotationNeo.getOutputCurrent());
    // SmartDashboard.putBoolean(" Climber Master Motor", climbMaster);
    // SmartDashboard.putBoolean(" Climber Follower Motor", climbFollower);

    //DO NOT REMOVE FOR COMP
    this.rotationPID.setSetPoint(this.carriageRotationSetpoint);
    this.rotationPID.updatePID(getCarriageRotationDegrees());
    double result = this.rotationPID.getResult() + Math.sin(Math.toRadians(getCarriageRotationDegrees())) * this.kG;
    setCarriageRotationPercent(result);
    SmartDashboard.putNumber("Carriage Rotation", getCarriageRotationDegrees());
    //DO NOT REMOVE FOR COMP
  }

  public void teleopPeriodic(){
    
  }
}
