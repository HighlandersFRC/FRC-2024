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

  boolean isDriveFlipped;
  boolean isAngleFlipped;

  /**
   * Constructs a new SwerveModule instance.
   *
   * @param mModuleNum  The module number.
   * @param mAngleMotor The TalonFX motor used for controlling the angle of the module.
   * @param mDriveMotor The TalonFX motor used for driving the module.
   * @param mCanCoder   The CANCoder sensor used for feedback control.
   */
  public SwerveModule(int mModuleNum, TalonFX mAngleMotor, TalonFX mDriveMotor, CANcoder mCanCoder, boolean isDriveFlipped, boolean isAngleFlipped) {
    // creates values for a single module
    moduleNumber = mModuleNum;
    angleMotor = mAngleMotor;
    driveMotor = mDriveMotor;
    canCoder = mCanCoder;
    this.isDriveFlipped = isDriveFlipped;
    this.isAngleFlipped = isAngleFlipped;
  }
  /**
   * Calculates the torque angle for the swerve module.
   * 
   * @return The torque angle in radians.
   */
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

    angleMotorConfig.TorqueCurrent.PeakForwardTorqueCurrent = 70;
    angleMotorConfig.TorqueCurrent.PeakReverseTorqueCurrent = -70;

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

    driveMotorConfig.TorqueCurrent.PeakForwardTorqueCurrent = 160;
    driveMotorConfig.TorqueCurrent.PeakReverseTorqueCurrent = -160;

    driveMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    driveMotorConfig.ClosedLoopRamps.TorqueClosedLoopRampPeriod = 0.1;

    double absolutePosition = canCoder.getAbsolutePosition().getValue();
    angleMotor.setPosition(absolutePosition);
    driveMotor.setPosition(0.0);

    angleMotor.getConfigurator().apply(angleMotorConfig);
    driveMotor.getConfigurator().apply(driveMotorConfig);
  }
  
  /**
   * Sets the position and velocity PID for the swerve module's wheel.
   * 
   * @param angle    The desired angle for the wheel.
   * @param velocity The desired velocity for the wheel.
   */
  public void setWheelPID(double angle, double velocity){
    // method used to move wheel
    angleMotor.setControl(positionTorqueFOCRequest.withPosition(degreesToRotations(Math.toDegrees(angle))));
    driveMotor.setControl(velocityTorqueFOCRequest.withVelocity(wheelToDriveMotorRotations(velocity)));
  }
  
  /**
   * Sets the drive motor velocity
   * @param velocity - Wheel Velocity in RPS
   */
  public void setDrivePID(double velocity){
    driveMotor.setControl(velocityTorqueFOCRequest.withVelocity(wheelToDriveMotorRotations(velocity)));
  }

  /**
   * Converts wheel to motor rotations for the steer motor
   * @param rotations - Wheel angle in rotations
   * @return - Motor angle in rotations
   */
  public double wheelToSteerMotorRotations(double rotations){
    return (rotations * Constants.Ratios.STEER_GEAR_RATIO);
  }
  
  /**
   * Converts motor to wheel rotations for the steer motor
   * @param rotations - Motor angle in rotations
   * @return Wheel angle in rotations
   */
  public double steerMotorToWheelRotations(double rotations){
    return (rotations / Constants.Ratios.STEER_GEAR_RATIO);
  }

  /**
   * Converts wheel to motor rotations for the drive motor
   * @param rotations - Wheel rotations
   * @return Motor rotations
   */
  public double wheelToDriveMotorRotations(double rotations){
    return (rotations * Constants.Ratios.DRIVE_GEAR_RATIO);
  }

  /**
   * Converts wheel to motor rotations for the drive motor
   * @param rotations - Motor rotations
   * @return Wheel rotations
   */
  public double driveMotorToWheelRotations(double rotations){
    return (rotations / Constants.Ratios.DRIVE_GEAR_RATIO);
  }

  public double degreesToRotations(double degrees){
    return degrees / 360;
  }

  public double rotationsToDegrees(double rotations){
    return rotations * 360;
  }

  /**
   * Ground meters to wheel rotations
   * @param mps - Meters Per Second
   * @return - Wheel Rotations per Second
   */
  public double MPSToRPS(double mps){
    return (mps * Constants.Physical.WHEEL_ROTATION_PER_METER);
  }

  /**
   * Ground meters to wheel rotations
   * @param rps - Meters Per Second
   * @return - Wheel Rotations per Second
   */
  public double RPSToMPS(double rps){
    return (rps / Constants.Physical.WHEEL_ROTATION_PER_METER);
  }

  public void moveAngleMotor(double speed){
    angleMotor.set(0.2);
  }

  public void moveDriveMotor(double speed){
    driveMotor.set(-0.5);
  }

  /**
   * Gets the distance traveled by the swerve module's wheel.
   * 
   * @return The distance traveled by the wheel in meters.
   */
  public double getModuleDistance(){
    double position = driveMotor.getPosition().getValue();
    // double wheelRotations = (position * Constants.Wheel_Rotations_In_A_Meter) / Constants.GEAR_RATIO;
    double wheelRotations = driveMotorToWheelRotations(position);
    double distance = RPSToMPS(wheelRotations);
    return distance;
  }
  
  /**
   * Retrieves Wheel Angle
   * @return Wheel angle in radians
   */
  public double getWheelPosition(){
    double position = angleMotor.getPosition().getValue();
    return Constants.rotationsToRadians(position);
  }

  /**
   * Retrieves the current wheel velocity
   * @return Wheel velocity in RPS
   */
  public double getWheelSpeed(){
    double speed = driveMotorToWheelRotations(driveMotor.getVelocity().getValue());
    return speed;
  }

  /**
   * Retrieves the velocity of the angle of the wheel
   * @return Wheel Angle Velocity in RPS
   */
  public double getAngleVelocity(){
    return angleMotor.getVelocity().getValue();
  }

  /**
   * Retrieves the velocity of the drive motor rotor
   * @return Rotor velocity (RPS)
   */
  public double getWheelSpeedWithoutGearRatio(){
    return driveMotor.getVelocity().getValue();
  }

  /**
   * Retrieves the angle of the motor rotor
   * @return Angle Rotor Position (radians)
   */
  public double getAngleMotorPosition(){
    double degrees = rotationsToDegrees(wheelToSteerMotorRotations(angleMotor.getPosition().getValue()));
    return (Math.toRadians(degrees));
  }

  /**
   * Retrieves the absolute position of the CANCoder
   * @return CANCoder absolute position (rotations)
   */
  public double getCanCoderPosition(){
    return canCoder.getAbsolutePosition().getValue();
  }

  /**
   * Retrieves the absolute position of the CANCoder
   * @return CANCoder absolute position (radians)
   */
  public double getCanCoderPositionRadians(){
    return Constants.rotationsToRadians(canCoder.getAbsolutePosition().getValue());
  }

  /**
   * Retrieves the current ground speed of the swerve module
   * @return Speed (MPS)
   */
  public double getGroundSpeed(){
    return RPSToMPS(getWheelSpeed());
  }
  
  /**
   * Retrieves the setpoint of the angle motor
   * @return Angle motor setpoint (rotations)
   */
  public double getAngleMotorSetpoint(){
    return angleMotor.getClosedLoopReference().getValue();
  }

  /**
   * Retrieves the setpoint of the drive motor
   * @return Drive motor setpoint (MPS)
   */
  public double getDriveMotorSetpoint(){
    return RPSToMPS(driveMotorToWheelRotations(driveMotor.getClosedLoopReference().getValue()));
  }

  /**
   * Calculates the angle of the joystick input relative to the positive y-axis.
   * 
   * @param joystickY The y-component of the joystick input.
   * @param joystickX The x-component of the joystick input.
   * @return The angle of the joystick input in radians, relative to the positive y-axis.
  */
  public double getJoystickAngle(double joystickY, double joystickX) {
    double joystickAngle = Math.atan2(-joystickX, -joystickY);
    return joystickAngle;
  }

  /**
   * Calculates the position magnitude of the joystick input.
   * 
   * @param joystickY The y-component of the joystick input.
   * @param joystickX The x-component of the joystick input.
   * @return The magnitude of the joystick position.
   */
  public double getJoystickPosition(double joystickY, double joystickX){
    double position = joystickY * joystickY + joystickX * joystickX;
    return position;
  }
  
  /**
   * Drives the swerve module based on the given control vector, turn value, and current orientation.
   * 
   * @param vector The control vector representing the desired direction and magnitude of movement.
   * @param turnValue The turn value representing the rotation to be applied.
   * @param navxAngle The current orientation angle from the IMU sensor.
   */
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

  /**
   * Finds the closest angle between two given angles, considering wrap-around at 360 degrees.
   *
   * @param angleA The first angle.
   * @param angleB The second angle.
   * @return The closest angle from angle A to angle B, taking into account wrap-around.
   */
  public double findClosestAngle(double angleA, double angleB){
    double direction = angleB - angleA;

    if (Math.abs(direction) > Math.PI){
      direction = -(Math.signum(direction) * (2 * Math.PI)) + direction;
    }
    return direction;
  }

  @Override
  public void periodic() {
    //   if(driveMotor.getDeviceID() == 1 && driveMotor.getSupplyVoltage().getValue() != 0.0 && angleMotor.getDeviceID() == 2 && angleMotor.getSupplyVoltage().getValue() != 0.0){
    //     frontRight = true;
    //     // System.out.println("front right - " + driveMotor.getSupplyVoltage().getValue() + " + " + angleMotor.getSupplyVoltage().getValue());
    //   } 
    //   if(driveMotor.getDeviceID() == 3 && driveMotor.getSupplyVoltage().getValue() != 0.0 && angleMotor.getDeviceID() == 4 && angleMotor.getSupplyVoltage().getValue() != 0.0){
    //     frontLeft = true;
    //     // System.out.println("front left - " + driveMotor.getSupplyVoltage().getValue() + " + " + angleMotor.getSupplyVoltage().getValue());
    //   } 
    //   if(driveMotor.getDeviceID() == 5 && driveMotor.getSupplyVoltage().getValue() != 0.0 && angleMotor.getDeviceID() == 6 && angleMotor.getSupplyVoltage().getValue() != 0.0){
    //     backLeft = true;
    //     // System.out.println("back left - " + driveMotor.getSupplyVoltage().getValue() + " + " + angleMotor.getSupplyVoltage().getValue());
    //   }
    //   if(driveMotor.getDeviceID() == 7 && driveMotor.getSupplyVoltage().getValue() != 0.0 && angleMotor.getDeviceID() == 8 && angleMotor.getSupplyVoltage().getValue() != 0.0){
    //     backRight = true;        
    //     // System.out.println("back right - " + driveMotor.getSupplyVoltage().getValue() + " + " + angleMotor.getSupplyVoltage().getValue());
    //   }

    // if(canCoder.getDeviceID() == 1 && canCoder.getSupplyVoltage().getValue() != 0 && frontRight == true){
    //   swerveCan1 = true;
    //   // System.out.println("can 1 - " + canCoder.getSupplyVoltage().getValue());
    // } else if(canCoder.getDeviceID() == 2 && canCoder.getSupplyVoltage().getValue() != 0 && frontLeft == true){
    //   swerveCan2 = true;
    //   // System.out.println("can 2 - " + canCoder.getSupplyVoltage().getValue());
    // } else if(canCoder.getDeviceID() == 3 && canCoder.getSupplyVoltage().getValue() != 0 && backLeft == true){
    //   swerveCan3 = true;
    //   // System.out.println("can 3 - " + canCoder.getSupplyVoltage().getValue());
    // } else if(canCoder.getDeviceID() == 4 && canCoder.getSupplyVoltage().getValue() != 0 && backRight == true){
    //   swerveCan4 = true;
    //   // System.out.println("can 4 - " + canCoder.getSupplyVoltage().getValue());
    // }

    // SmartDashboard.putBoolean(" Swerve can(1)", swerveCan1);
    // SmartDashboard.putBoolean(" Swerve can(2)", swerveCan2);
    // SmartDashboard.putBoolean(" Swerve can(3)", swerveCan3);
    // SmartDashboard.putBoolean(" Swerve can(4)", swerveCan4);    
  }
}