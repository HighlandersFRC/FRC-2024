// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.tools.math.Vector;

// **Zero Wheels with the bolt head showing on the left when the front side(battery) is facing down/away from you**

public class SwerveModule extends SubsystemBase {
  private final TalonFX angleMotor;
  private final TalonFX driveMotor;
  private final int moduleNumber;
  private final CANcoder canCoder;

  PositionTorqueCurrentFOC positionTorqueFOCRequest = new PositionTorqueCurrentFOC(0, 0, 0, 0, false, false, false);
  VelocityTorqueCurrentFOC velocityTorqueFOCRequest = new VelocityTorqueCurrentFOC(0, 0, 0, 0, false, false, false);
  VelocityTorqueCurrentFOC velocityTorqueFOCRequestAngleMotor = new VelocityTorqueCurrentFOC(0, 0, 0, 1, false, false, false);

  boolean swerveCan1;
  boolean swerveCan2;
  boolean swerveCan3;
  boolean swerveCan4;
  boolean frontRight;
  boolean frontLeft;
  boolean backLeft;
  boolean backRight;
  boolean can1;
  boolean can2;
  boolean can3;
  boolean can4;

  /** Creates a new SwerveModule. */
  public SwerveModule(int mModuleNum, TalonFX mAngleMotor, TalonFX mDriveMotor, CANcoder mCanCoder) {
    // creates values for a single module
    moduleNumber = mModuleNum;
    angleMotor = mAngleMotor;
    driveMotor = mDriveMotor;
    canCoder = mCanCoder;
  }

  public double torqueAngle(){
    // math to find turn angle
    double length = Constants.Physical.ROBOT_LENGTH/2, width = Constants.Physical.ROBOT_WIDTH/2, angle;
    length -= Constants.Physical.MODULE_OFFSET;
    width -= Constants.Physical.MODULE_OFFSET;

    switch(moduleNumber){
      case 1:
      angle = (Math.atan2(-width, length)) - Math.PI;
      break;
      case 2:
      angle = Math.atan2(width, length);
      break;
      case 3:
      angle =  Math.PI + Math.atan2(width, -length);
      break;
      case 4:
      angle =  (2 * Math.PI) + (Math.atan2(-width, -length));
      break;
      default: 
      angle = 1;
    }
    return angle;
  }

  public void init(){
    // sets all of the configurations for the motors
    TalonFXConfiguration angleMotorConfig = new TalonFXConfiguration();
    TalonFXConfiguration driveMotorConfig = new TalonFXConfiguration();

    // angleMotorConfig.Slot0.kP = 300.0;
    // angleMotorConfig.Slot0.kI = 0.0;
    // angleMotorConfig.Slot0.kD = 7.5;

    // angleMotorConfig.Slot1.kP = 3.0;
    // angleMotorConfig.Slot1.kI = 0.0;
    // angleMotorConfig.Slot1.kD = 0.0;

    angleMotorConfig.Slot0.kP = 350.0;
    angleMotorConfig.Slot0.kI = 0.0;
    angleMotorConfig.Slot0.kD = 15;

    angleMotorConfig.Slot1.kP = 3.0;
    angleMotorConfig.Slot1.kI = 0.0;
    angleMotorConfig.Slot1.kD = 0.0;

    angleMotorConfig.TorqueCurrent.PeakForwardTorqueCurrent = 60;
    angleMotorConfig.TorqueCurrent.PeakReverseTorqueCurrent = -60;

    angleMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    angleMotorConfig.ClosedLoopRamps.TorqueClosedLoopRampPeriod = 0.1;

    angleMotorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
    angleMotorConfig.Feedback.FeedbackRemoteSensorID = canCoder.getDeviceID();
    angleMotorConfig.Feedback.SensorToMechanismRatio = 1.0;
    angleMotorConfig.Feedback.RotorToSensorRatio = Constants.Ratios.STEER_GEAR_RATIO;
    
    driveMotorConfig.Slot0.kP = 6.5;
    driveMotorConfig.Slot0.kI = 0.0;
    driveMotorConfig.Slot0.kD = 0.0;
    driveMotorConfig.Slot0.kV = 0.0;

    driveMotorConfig.TorqueCurrent.PeakForwardTorqueCurrent = 60;
    driveMotorConfig.TorqueCurrent.PeakReverseTorqueCurrent = -60;

    driveMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    driveMotorConfig.ClosedLoopRamps.TorqueClosedLoopRampPeriod = 0.1;

    double absolutePosition = canCoder.getAbsolutePosition().getValue();
    angleMotor.setPosition(absolutePosition);
    driveMotor.setPosition(0.0);

    angleMotor.getConfigurator().apply(angleMotorConfig);
    driveMotor.getConfigurator().apply(driveMotorConfig);
  }

  public void setWheelPID(double angle, double velocity){
    // method used to move wheel
    angleMotor.setControl(positionTorqueFOCRequest.withPosition(degreesToRotations(Math.toDegrees(angle))));
    driveMotor.setControl(velocityTorqueFOCRequest.withVelocity(wheelToDriveMotorRotations(velocity)));
  }

  public void setDrivePID(double velocity){
    driveMotor.setControl(velocityTorqueFOCRequest.withVelocity(wheelToDriveMotorRotations(velocity)));
  }

  public double wheelToSteerMotorRotations(double rotations){
    return (rotations * Constants.Ratios.STEER_GEAR_RATIO);
  }

  public double steerMotorToWheelRotations(double rotations){
    return (rotations / Constants.Ratios.STEER_GEAR_RATIO);
  }

  public double wheelToDriveMotorRotations(double rotations){
    return (rotations * Constants.Ratios.DRIVE_GEAR_RATIO);
  }

  public double driveMotorToWheelRotations(double rotations){
    return (rotations / Constants.Ratios.DRIVE_GEAR_RATIO);
  }

  public double degreesToRotations(double degrees){
    return degrees / 360;
  }

  public double rotationsToDegrees(double rotations){
    return rotations * 360;
  }

  public double MPSToRPS(double mps){
    return (mps * Constants.Physical.WHEEL_ROTATION_PER_METER);
  }

  public double RPSToMPS(double rps){
    return (rps / Constants.Physical.WHEEL_ROTATION_PER_METER);
  }

  public void moveAngleMotor(double speed){
    angleMotor.set(0.2);
  }

  public void moveDriveMotor(double speed){
    driveMotor.set(-0.5);
  }

  public double getModuleDistance(){
    double position = driveMotor.getPosition().getValue();
    // double wheelRotations = (position * Constants.Wheel_Rotations_In_A_Meter) / Constants.GEAR_RATIO;
    double wheelRotations = driveMotorToWheelRotations(position);
    double distance = RPSToMPS(wheelRotations);
    return distance;
  }

  public double getWheelPosition(){
    double position = angleMotor.getPosition().getValue();
    return Constants.rotationsToRadians(position);
  }

  public double getWheelSpeed(){
    double speed = driveMotorToWheelRotations(driveMotor.getVelocity().getValue());
    return speed;
  }

  public double getAngleVelocity(){
    return angleMotor.getVelocity().getValue();
  }

  public double getWheelSpeedWithoutGearRatio(){
    return driveMotor.getVelocity().getValue();
  }

  public double getAngleMotorPosition(){
    double degrees = rotationsToDegrees(wheelToSteerMotorRotations(angleMotor.getPosition().getValue()));
    return (Math.toRadians(degrees));
  }

  public double getCanCoderPosition(){
    return canCoder.getAbsolutePosition().getValue();
  }

  public double getCanCoderPositionRadians(){
    return Constants.rotationsToRadians(canCoder.getAbsolutePosition().getValue());
  }

  public double getGroundSpeed(){
    return RPSToMPS(getWheelSpeed());
  }

  public double getAngleMotorSetpoint(){
    return angleMotor.getClosedLoopReference().getValue();
  }

  public double getDriveMotorSetpoint(){
    return RPSToMPS(driveMotorToWheelRotations(driveMotor.getClosedLoopReference().getValue()));
  }

  public double getJoystickAngle(double joystickY, double joystickX) {
    double joystickAngle = Math.atan2(-joystickX, -joystickY);
    return joystickAngle;
  }

  public double getJoystickPosition(double joystickY, double joystickX){
    double position = joystickY * joystickY + joystickX * joystickX;
    return position;
  }
  
  // main drive method
  public void drive(Vector vector, double turnValue, double navxAngle){
    if(Math.abs(vector.getI()) < 0.001 && Math.abs(vector.getJ()) < 0.001 && Math.abs(turnValue) < 0.01) {
      // stops motors when joysticks are at 0
      driveMotor.setControl(velocityTorqueFOCRequest.withVelocity(0.0));
      angleMotor.setControl(velocityTorqueFOCRequestAngleMotor.withVelocity(0.0));
    }
    else {
      double angleWanted = Math.atan2(vector.getJ(), vector.getI());
      double wheelPower = Math.sqrt(Math.pow(vector.getI(), 2) + Math.pow(vector.getJ(), 2));

      // field centric math
      double angleWithNavx = angleWanted + navxAngle;

      double xValueWithNavx = wheelPower * Math.cos(angleWithNavx);
      double yValueWithNavx = wheelPower * Math.sin(angleWithNavx);

      double turnX = turnValue * (Constants.angleToUnitVectorI(torqueAngle()));
      double turnY = turnValue * (Constants.angleToUnitVectorJ(torqueAngle()));

      // adds turn and strafe vectors
      Vector finalVector = new Vector();
      finalVector.setI(xValueWithNavx + turnX);
      finalVector.setJ(yValueWithNavx + turnY);

      double finalAngle = -Math.atan2(finalVector.getJ(), finalVector.getI());
      double finalVelocity = Math.sqrt(Math.pow(finalVector.getI(), 2) + Math.pow(finalVector.getJ(), 2));

      if (finalVelocity > Constants.Physical.TOP_SPEED){
        finalVelocity = Constants.Physical.TOP_SPEED;
      }

      double velocityRPS = (MPSToRPS(finalVelocity));

      double currentAngle = getWheelPosition();
      double currentAngleBelow360 = (getWheelPosition()) % (Math.toRadians(360));

      // runs angle through optimizer to optimize wheel motion
      double setpointAngle = findClosestAngle(currentAngleBelow360, finalAngle);
      double setpointAngleFlipped = findClosestAngle(currentAngleBelow360, finalAngle + Math.PI);

      // used to make drive motor move less the more the angle motor is from its setpoint
      double angleDifference = Math.abs(currentAngleBelow360 - finalAngle);
      double adjustedVelocity = ((Math.cos(angleDifference)) * velocityRPS);

      // moves wheel
      if (Math.abs(setpointAngle) <= Math.abs(setpointAngleFlipped)){
        setWheelPID(currentAngle + setpointAngle, adjustedVelocity);
      } else {
        setWheelPID(currentAngle + setpointAngleFlipped, adjustedVelocity);
      }
  }
}

  // optimizer
  public double findClosestAngle(double angleA, double angleB){
    double direction = angleB - angleA;

    if (Math.abs(direction) > Math.PI){
      direction = -(Math.signum(direction) * (2 * Math.PI)) + direction;
    }
    return direction;
  }

  @Override
  public void periodic() {
      if(driveMotor.getDeviceID() == 1 && driveMotor.getSupplyVoltage().getValue() != 0.0 && angleMotor.getDeviceID() == 2 && angleMotor.getSupplyVoltage().getValue() != 0.0){
        frontRight = true;
        // System.out.println("front right - " + driveMotor.getSupplyVoltage().getValue() + " + " + angleMotor.getSupplyVoltage().getValue());
      } 
      if(driveMotor.getDeviceID() == 3 && driveMotor.getSupplyVoltage().getValue() != 0.0 && angleMotor.getDeviceID() == 4 && angleMotor.getSupplyVoltage().getValue() != 0.0){
        frontLeft = true;
        // System.out.println("front left - " + driveMotor.getSupplyVoltage().getValue() + " + " + angleMotor.getSupplyVoltage().getValue());
      } 
      if(driveMotor.getDeviceID() == 5 && driveMotor.getSupplyVoltage().getValue() != 0.0 && angleMotor.getDeviceID() == 6 && angleMotor.getSupplyVoltage().getValue() != 0.0){
        backLeft = true;
        // System.out.println("back left - " + driveMotor.getSupplyVoltage().getValue() + " + " + angleMotor.getSupplyVoltage().getValue());
      }
      if(driveMotor.getDeviceID() == 7 && driveMotor.getSupplyVoltage().getValue() != 0.0 && angleMotor.getDeviceID() == 8 && angleMotor.getSupplyVoltage().getValue() != 0.0){
        backRight = true;        
        // System.out.println("back right - " + driveMotor.getSupplyVoltage().getValue() + " + " + angleMotor.getSupplyVoltage().getValue());
      }

    if(canCoder.getDeviceID() == 1 && canCoder.getSupplyVoltage().getValue() != 0 && frontRight == true){
      swerveCan1 = true;
      // System.out.println("can 1 - " + canCoder.getSupplyVoltage().getValue());
    } else if(canCoder.getDeviceID() == 2 && canCoder.getSupplyVoltage().getValue() != 0 && frontLeft == true){
      swerveCan2 = true;
      // System.out.println("can 2 - " + canCoder.getSupplyVoltage().getValue());
    } else if(canCoder.getDeviceID() == 3 && canCoder.getSupplyVoltage().getValue() != 0 && backLeft == true){
      swerveCan3 = true;
      // System.out.println("can 3 - " + canCoder.getSupplyVoltage().getValue());
    } else if(canCoder.getDeviceID() == 4 && canCoder.getSupplyVoltage().getValue() != 0 && backRight == true){
      swerveCan4 = true;
      // System.out.println("can 4 - " + canCoder.getSupplyVoltage().getValue());
    }

    SmartDashboard.putBoolean(" Swerve can(1)", swerveCan1);
    SmartDashboard.putBoolean(" Swerve can(2)", swerveCan2);
    SmartDashboard.putBoolean(" Swerve can(3)", swerveCan3);
    SmartDashboard.putBoolean(" Swerve can(4)", swerveCan4);    
  }
}