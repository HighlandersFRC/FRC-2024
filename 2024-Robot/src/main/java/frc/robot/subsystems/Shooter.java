// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DynamicMotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.commands.AngleShooter;

public class Shooter extends SubsystemBase {
  private final TalonFX leftShooter = new TalonFX(Constants.CANInfo.SHOOTER_LEFT_MOTOR_ID, "Canivore");
  private final TalonFX rightShooter = new TalonFX(Constants.CANInfo.SHOOTER_RIGHT_MOTOR_ID, "Canivore");
  private final TalonFX shooterAngle = new TalonFX(Constants.CANInfo.SHOOTER_ANGLE_MOTOR_ID, "Canivore");

  private final TalonFXConfiguration flywheelConfiguration = new TalonFXConfiguration();
  private final TalonFXConfiguration angleConfiguration = new TalonFXConfiguration();
  
  private final PositionTorqueCurrentFOC anglePositionMotionProfileRequest = new PositionTorqueCurrentFOC(0.0);
  // private final DynamicMotionMagicTorqueCurrentFOC anglePositionMotionProfileRequest = new DynamicMotionMagicTorqueCurrentFOC(0, 0, 0, 0, 0, 0, false, false, false);
  private final VelocityTorqueCurrentFOC flywheelVelocityRequest = new VelocityTorqueCurrentFOC(0, 0, 0, 0, false, false, false);
  /** Creates a new Shooter. */
  public Shooter() {}

  public void init() {
    this.flywheelConfiguration.Slot0.kP = 7;
    this.flywheelConfiguration.Slot0.kI = 0;
    this.flywheelConfiguration.Slot0.kD = 0;
    this.flywheelConfiguration.Slot0.kS = 0;
    this.flywheelConfiguration.Slot0.kV = 0.5;
    
    // this.angleConfiguration.Feedback.SensorToMechanismRatio = Constants.Ratios.SHOOTER_ANGLE_GEAR_RATIO;
    this.angleConfiguration.Slot0.kP = 3;
    this.angleConfiguration.Slot0.kI = 0.0;
    this.angleConfiguration.Slot0.kD = 0.5;
    this.angleConfiguration.Slot0.kS = 5;
    this.angleConfiguration.Slot0.GravityType = GravityTypeValue.Elevator_Static;

    leftShooter.getConfigurator().apply(flywheelConfiguration);
    rightShooter.getConfigurator().apply(flywheelConfiguration);
    shooterAngle.getConfigurator().apply(angleConfiguration);
  }

  public void setShooterPercent(double left, double right){
    leftShooter.set(-left);
    rightShooter.set(-right);
  }

  public void setShooterAngle(double angle /* degrees */) {
    this.angleConfiguration.Slot0.kG = 15*Math.sin(getShooterAngle()*Math.PI/180);
    shooterAngle.getConfigurator().apply(angleConfiguration);
    this.shooterAngle.setControl(this.anglePositionMotionProfileRequest.withPosition(((angle-20)/360)*Constants.Ratios.SHOOTER_ANGLE_GEAR_RATIO));
    
  }

  public double getShooterAngle() {
    SmartDashboard.putString("Angle Request", this.shooterAngle.getAppliedControl().getControlInfo().toString());
    SmartDashboard.putNumber("otherValue ", shooterAngle.getRotorPosition().getValueAsDouble());
    return (((shooterAngle.getRotorPosition().getValueAsDouble())*360)/Constants.Ratios.SHOOTER_ANGLE_GEAR_RATIO+20);
  }

  public void setShooterRPM(double left, double right){
    this.leftShooter.setControl(this.flywheelVelocityRequest.withVelocity(Constants.RPMToRPS(left) * Constants.Ratios.SHOOTER_FLYWHEEL_GEAR_RATIO));
    this.rightShooter.setControl(this.flywheelVelocityRequest
        .withVelocity(Constants.RPMToRPS(-right) * Constants.Ratios.SHOOTER_FLYWHEEL_GEAR_RATIO));
  }

  public double getLeftShooterRPM() {
    return (Constants.RPSToRPM(leftShooter.getRotorVelocity().getValueAsDouble())) / Constants.Ratios.SHOOTER_FLYWHEEL_GEAR_RATIO;
  }

  public double getRightShooterRPM() {
    return (Constants.RPSToRPM(rightShooter.getRotorVelocity().getValueAsDouble())) / Constants.Ratios.SHOOTER_FLYWHEEL_GEAR_RATIO;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
