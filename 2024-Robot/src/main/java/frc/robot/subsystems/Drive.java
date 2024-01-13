// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.json.JSONArray;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.OI;
import frc.robot.tools.controlloops.PID;
import frc.robot.tools.math.Vector;
import frc.robot.commands.defaults.DriveDefault;

// **Zero Wheels with the bolt head showing on the left when the front side(battery) is facing down/away from you**

public class Drive extends SubsystemBase {
  private final TalonFX frontRightDriveMotor = new TalonFX(1, "Canivore");
  private final TalonFX frontRightAngleMotor = new TalonFX(2, "Canivore");
  private final TalonFX frontLeftDriveMotor = new TalonFX(3, "Canivore");
  private final TalonFX frontLeftAngleMotor = new TalonFX(4, "Canivore");
  private final TalonFX backLeftDriveMotor = new TalonFX(5, "Canivore");
  private final TalonFX backLeftAngleMotor = new TalonFX(6, "Canivore");
  private final TalonFX backRightDriveMotor = new TalonFX(7, "Canivore");
  private final TalonFX backRightAngleMotor = new TalonFX(8, "Canivore");

  private final CANcoder frontRightCanCoder = new CANcoder(1, "Canivore");
  private final CANcoder frontLeftCanCoder = new CANcoder(2, "Canivore");
  private final CANcoder backLeftCanCoder = new CANcoder(3, "Canivore");
  private final CANcoder backRightCanCoder = new CANcoder(4, "Canivore");

  // creates all 4 modules
  private final SwerveModule frontRight = new SwerveModule(1, frontRightAngleMotor, frontRightDriveMotor, frontRightCanCoder);
  private final SwerveModule frontLeft = new SwerveModule(2, frontLeftAngleMotor, frontLeftDriveMotor, frontLeftCanCoder);
  private final SwerveModule backLeft = new SwerveModule(3, backLeftAngleMotor, backLeftDriveMotor, backLeftCanCoder);
  private final SwerveModule backRight = new SwerveModule(4, backRightAngleMotor, backRightDriveMotor, backRightCanCoder);

  Peripherals peripherals;

  // xy position of module based on robot width and distance from edge of robot
  private final double moduleX = ((Constants.Physical.ROBOT_LENGTH)/2) - Constants.Physical.MODULE_OFFSET;
  private final double moduleY = ((Constants.Physical.ROBOT_WIDTH)/2) - Constants.Physical.MODULE_OFFSET;

  // Locations for the swerve drive modules relative to the robot center.
  Translation2d m_frontLeftLocation = new Translation2d(moduleX, moduleY);
  Translation2d m_frontRightLocation = new Translation2d(moduleX, -moduleY);
  Translation2d m_backLeftLocation = new Translation2d(-moduleX, moduleY);
  Translation2d m_backRightLocation = new Translation2d(-moduleX, -moduleY);

  // odometry
  private double currentX = 0;
  private double currentY = 0;
  private double currentTheta = 0;

  private double estimatedX = 0.0;
  private double estimatedY = 0.0;
  private double estimatedTheta = 0.0;

  private double previousEstimateX = 0.0;
  private double previousEstimateY = 0.0;
  private double previousEstimateTheta = 0.0;

  private double averagedX = 0.0;
  private double averagedY = 0.0;
  private double averagedTheta = 0.0;

  private double initTime;
  private double currentTime;
  private double previousTime;
  private double timeDiff;

  private double previousX = 0;
  private double previousY = 0;
  private double previousTheta = 0;

  // array for fused odometry
  private double[] currentFusedOdometry = new double[3];

  // Creating my kinematics object using the module locations
  SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
  m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation
  );

  SwerveDrivePoseEstimator m_odometry; 
  Pose2d m_pose;

  double initAngle;
  double setAngle;
  double diffAngle;

  // path following PID values
  private double xP = 4.0; 
  private double xI = 0.0;
  private double xD = 1.2;

  private double yP = 4.0;
  private double yI = 0.0;
  private double yD = 1.2;

  private double thetaP = 3.1;
  private double thetaI = 0.0;
  private double thetaD = 0.8;

  private PID xPID = new PID(xP, xI, xD);
  private PID yPID = new PID(yP, yI, yD);
  private PID thetaPID = new PID(thetaP, thetaI, thetaD);

  private String fieldSide = "red";

  private int lookAheadDistance = 5;
  
  /** Creates a new SwerveDriveSubsystem. */
  public Drive(Peripherals peripherals) {
    this.peripherals = peripherals;

    SwerveModulePosition[] swerveModulePositions = new SwerveModulePosition[4];
    swerveModulePositions[0] = new SwerveModulePosition(0, new Rotation2d(frontLeft.getCanCoderPositionRadians()));
    swerveModulePositions[1] = new SwerveModulePosition(0, new Rotation2d(frontRight.getCanCoderPositionRadians()));
    swerveModulePositions[2] = new SwerveModulePosition(0, new Rotation2d(backLeft.getCanCoderPositionRadians()));
    swerveModulePositions[3] = new SwerveModulePosition(0, new Rotation2d(backRight.getCanCoderPositionRadians()));

    Pose2d m_pose = new Pose2d();

    m_odometry = new SwerveDrivePoseEstimator(m_kinematics, new Rotation2d((Math.toRadians(peripherals.getPigeonAngle()))), swerveModulePositions, m_pose);
  }

  public void init(String fieldSide){
    // sets configurations when run on robot initalization
    this.fieldSide = fieldSide;

    frontRight.init();
    frontLeft.init();
    backRight.init();
    backLeft.init();

    frontRightAngleMotor.setInverted(true);
    frontLeftAngleMotor.setInverted(true);
    backRightAngleMotor.setInverted(true);
    backLeftAngleMotor.setInverted(true);

    frontRightDriveMotor.setInverted(false);
    frontLeftDriveMotor.setInverted(false);
    backRightDriveMotor.setInverted(false);
    backLeftDriveMotor.setInverted(false);

    xPID.setMinOutput(-4.9);
    xPID.setMaxOutput(4.9);

    yPID.setMinOutput(-4.9);
    yPID.setMaxOutput(4.9);

    thetaPID.setMinOutput(-(Constants.Physical.TOP_SPEED)/(Constants.Physical.ROBOT_RADIUS));
    thetaPID.setMaxOutput((Constants.Physical.TOP_SPEED)/(Constants.Physical.ROBOT_RADIUS));

    setDefaultCommand(new DriveDefault(this));
  }

  // method to zeroIMU mid match and reset odometry with zeroed angle
  public void zeroIMU(){
    peripherals.zeroPigeon();
    SwerveModulePosition[] swerveModulePositions = new SwerveModulePosition[4];
    swerveModulePositions[0] = new SwerveModulePosition(frontLeft.getModuleDistance(), new Rotation2d(frontLeft.getCanCoderPositionRadians()));
    swerveModulePositions[1] = new SwerveModulePosition(frontRight.getModuleDistance(), new Rotation2d(frontRight.getCanCoderPositionRadians()));
    swerveModulePositions[2] = new SwerveModulePosition(backLeft.getModuleDistance(), new Rotation2d(backLeft.getCanCoderPositionRadians()));
    swerveModulePositions[3] = new SwerveModulePosition(backRight.getModuleDistance(), new Rotation2d(backRight.getCanCoderPositionRadians()));
    m_odometry.resetPosition(new Rotation2d((Math.toRadians(peripherals.getPigeonAngle()))), swerveModulePositions, new Pose2d(new Translation2d(getFusedOdometryX(), getFusedOdometryY()), new Rotation2d((Math.toRadians(peripherals.getPigeonAngle())))));
  }

  public void setPigeonAfterAuto() {
    peripherals.setPigeonAngle((peripherals.getPigeonAngle() + 180)%360);
  }

  public void setPigeonAngle(double angle){
    peripherals.setPigeonAngle(angle);
  }

  public double getPigeonAngle(){
    return peripherals.getPigeonAngle();
  }

  public void setWheelsStraight(){
    frontRight.setWheelPID(0.0, 0.0);
    frontLeft.setWheelPID(0.0, 0.0);
    backLeft.setWheelPID(0.0, 0.0);
    backRight.setWheelPID(0.0, 0.0);
  }

  public void autoInit(JSONArray pathPoints){
    // runs at start of autonomous
    // System.out.println("Auto init");
    JSONArray firstPoint = pathPoints.getJSONArray(0);
    double firstPointX = firstPoint.getDouble(1);
    double firstPointY = firstPoint.getDouble(2);
    double firstPointAngle = firstPoint.getDouble(3);

    // changing odometry if on blue side, don't need to change y because it will be the same for autos on either side
    if(getFieldSide() == "blue") {
      firstPointX = Constants.Physical.FIELD_LENGTH - firstPointX;
      firstPointAngle = Math.PI - firstPointAngle;
    }
        
    peripherals.setPigeonAngle(Math.toDegrees(firstPointAngle));
    SwerveModulePosition[] swerveModulePositions = new SwerveModulePosition[4];
    swerveModulePositions[0] = new SwerveModulePosition(frontLeft.getModuleDistance(), new Rotation2d(frontLeft.getCanCoderPositionRadians()));
    swerveModulePositions[1] = new SwerveModulePosition(frontRight.getModuleDistance(), new Rotation2d(frontRight.getCanCoderPositionRadians()));
    swerveModulePositions[2] = new SwerveModulePosition(backLeft.getModuleDistance(), new Rotation2d(backLeft.getCanCoderPositionRadians()));
    swerveModulePositions[3] = new SwerveModulePosition(backRight.getModuleDistance(), new Rotation2d(backRight.getCanCoderPositionRadians()));
    m_odometry.resetPosition(new Rotation2d(firstPointAngle), swerveModulePositions, new Pose2d(new Translation2d(firstPointX, firstPointY), new Rotation2d(firstPointAngle)));
        
    estimatedX = getOdometryX();
    estimatedY = getOdometryY();
    estimatedTheta = getOdometryAngle();

    previousEstimateX = estimatedX;
    previousEstimateY = estimatedY;
    previousEstimateTheta = estimatedTheta;

    currentX = getOdometryX();
    currentY = getOdometryY();
    currentTheta = getOdometryAngle();

    previousX = currentX;
    previousY = currentY;
    previousTheta = currentTheta;

    averagedX = (estimatedX + currentX)/2;
    averagedY = (estimatedY + currentY)/2;   
    averagedTheta = (estimatedTheta + currentTheta)/2;

    initTime = Timer.getFPGATimestamp();

    updateOdometryFusedArray();
  }

  public void setFieldSide(String side){
    fieldSide = side;
  }

  public String getFieldSide(){
    return fieldSide;
  }

  public double getCurrentTime(){
    return currentTime;
  }

  // method to update odometry by fusing prediction, encoder rotations, and camera values
  public void updateOdometryFusedArray(){
    
  }

  public double[] getModuleStates(){
    double[] states = {
      frontRight.getCanCoderPosition() * 360.0, frontRight.getGroundSpeed(),
      frontLeft.getCanCoderPosition() * 360.0, frontLeft.getGroundSpeed(),
      backLeft.getCanCoderPosition() * 360.0, frontRight.getGroundSpeed(),
      backRight.getCanCoderPosition() * 360.0, frontRight.getGroundSpeed(),
    };
    return states;
  }

  public double[] getModuleSetpoints(){
    double[] setpoints = {
      frontRight.getAngleMotorSetpoint(), frontRight.getDriveMotorSetpoint(),
      frontLeft.getAngleMotorSetpoint(), frontLeft.getDriveMotorSetpoint(),
      backLeft.getAngleMotorSetpoint(), backLeft.getDriveMotorSetpoint(),
      backRight.getAngleMotorSetpoint(), backRight.getDriveMotorSetpoint(),
    };
    return setpoints;
  }

  public double[] getAngleMotorVelocity(){
    double[] velocity = {
      frontRight.getAngleVelocity(), frontLeft.getAngleVelocity(), backLeft.getAngleVelocity(), backRight.getAngleVelocity()
    };
    return velocity;
  }

  public double[] getOdometry(){
    double[] odometry = {
      getOdometryX(), getOdometryY(), getOdometryAngle()
    };
    return odometry;
  }

  public double getFusedOdometryX() {
    return currentFusedOdometry[0];
  }

  public double getFusedOdometryY() {
    return currentFusedOdometry[1];
  }

  public double getFusedOdometryTheta(){
    return currentFusedOdometry[2];
  }

  public double getOdometryX() {
    return m_odometry.getEstimatedPosition().getX();
  }

  public double getOdometryY() {
    return m_odometry.getEstimatedPosition().getY();
  }

  public double getOdometryAngle() {
    return m_odometry.getEstimatedPosition().getRotation().getRadians();
  }

  public void driveAutoAligned(double degreesFromPlacement) {
    updateOdometryFusedArray();

    double turn = degreesFromPlacement;

    double originalX = -(Math.copySign(OI.getDriverLeftY() * OI.getDriverLeftY(), OI.getDriverLeftY()));
    double originalY = -(Math.copySign(OI.getDriverLeftX() * OI.getDriverLeftX(), OI.getDriverLeftX()));

    if(Math.abs(originalX) < 0.05) {
        originalX = 0;
    }
    if(Math.abs(originalY) < 0.05) {
        originalY = 0;
    }

    double pigeonAngle = Math.toRadians(peripherals.getPigeonAngle());
    double xPower = getAdjustedX(originalX, originalY);
    double yPower = getAdjustedY(originalX, originalY);

    double xSpeed = xPower * Constants.Physical.TOP_SPEED;
    double ySpeed = yPower * Constants.Physical.TOP_SPEED;

    Vector controllerVector = new Vector(xSpeed, ySpeed);

    frontLeft.drive(controllerVector, turn, pigeonAngle);
    frontRight.drive(controllerVector, turn, pigeonAngle);
    backLeft.drive(controllerVector, turn, pigeonAngle);
    backRight.drive(controllerVector, turn, pigeonAngle);
}
  
  public void teleopDrive() {
    updateOdometryFusedArray();
    double turnLimit = 0.5;

    if(OI.driverController.getLeftBumper()) {
        // activate speedy spin
        turnLimit = 1;
    }

    // this is correct, X is forward in field, so originalX should be the y on the joystick
    double originalX = -(Math.copySign(OI.getDriverLeftY() * OI.getDriverLeftY(), OI.getDriverLeftY()));
    double originalY = -(Math.copySign(OI.getDriverLeftX() * OI.getDriverLeftX(), OI.getDriverLeftX()));

    if(Math.abs(originalX) < 0.075) {
        originalX = 0;
    }
    if(Math.abs(originalY) < 0.075) {
        originalY = 0;
    }

    double turn = turnLimit * ((Math.copySign(OI.getDriverRightX() * OI.getDriverRightX() * OI.getDriverRightX(), OI.getDriverRightX())) * (Constants.Physical.TOP_SPEED)/(Constants.Physical.ROBOT_RADIUS));
    double pigeonAngle = Math.toRadians(peripherals.getPigeonAngle());
    double xPower = getAdjustedX(originalX, originalY);
    double yPower = getAdjustedY(originalX, originalY);

    double xSpeed = xPower * Constants.Physical.TOP_SPEED;
    double ySpeed = yPower * Constants.Physical.TOP_SPEED;

    Vector controllerVector = new Vector(xSpeed, ySpeed);

    frontLeft.drive(controllerVector, turn, pigeonAngle);
    frontRight.drive(controllerVector, turn, pigeonAngle);
    backLeft.drive(controllerVector, turn, pigeonAngle);
    backRight.drive(controllerVector, turn, pigeonAngle);
  }

  // method run in autonomous that accepts a velocity vector of xy velocities, as well as how much to spin per second
  public void autoDrive(Vector vector, double turnRadiansPerSec){
    updateOdometryFusedArray();

    double pigeonAngle = Math.toRadians(peripherals.getPigeonAngle());

    double[] odometryList = new double[3];

    odometryList[0] = getFusedOdometryX();
    odometryList[1] = getFusedOdometryY();
    odometryList[2] = getFusedOdometryTheta();

    frontLeft.drive(vector, turnRadiansPerSec, pigeonAngle);
    frontRight.drive(vector, turnRadiansPerSec, pigeonAngle);
    backLeft.drive(vector, turnRadiansPerSec, pigeonAngle);
    backRight.drive(vector, turnRadiansPerSec, pigeonAngle);
  }

  // Autonomous algorithm
  public double[] pidController(double currentX, double currentY, double currentTheta, double time, JSONArray pathPoints) {
    if(time < pathPoints.getJSONArray(pathPoints.length() - 1).getDouble(0)) {
        JSONArray currentPoint = pathPoints.getJSONArray(0);
        JSONArray targetPoint = pathPoints.getJSONArray(0);
        for(int i = 0; i < pathPoints.length(); i ++) {
            if(i == pathPoints.length() - lookAheadDistance) {
                currentPoint = pathPoints.getJSONArray(i + 1);
                targetPoint = pathPoints.getJSONArray((i + (lookAheadDistance - 1)));
                break;
            }

            currentPoint = pathPoints.getJSONArray(i + 1);
            JSONArray previousPoint = pathPoints.getJSONArray(i);
            
            double currentPointTime = currentPoint.getDouble(0);
            double previousPointTime = previousPoint.getDouble(0);

            if(time >= previousPointTime && time < currentPointTime){
                targetPoint = pathPoints.getJSONArray(i + (lookAheadDistance - 1));
                break;
            }
        }
        
        double targetTime = targetPoint.getDouble(0);
        double targetX = targetPoint.getDouble(1);
        double targetY = targetPoint.getDouble(2);
        double targetTheta = targetPoint.getDouble(3);

        // System.out.println("X: " + targetX + " Y: " + targetY + " Theta: " + targetTheta);

        if(getFieldSide() == "blue") {
            targetX = Constants.Physical.FIELD_LENGTH - targetX;
            targetTheta = Math.PI - targetTheta;
        }

        if (targetTheta - currentTheta > Math.PI){
            targetTheta -= 2 * Math.PI;
        } else if (targetTheta - currentTheta < -Math.PI){
            targetTheta += 2 * Math.PI;
        }

        double currentPointTime = currentPoint.getDouble(0);
        double currentPointX = currentPoint.getDouble(1);
        double currentPointY = currentPoint.getDouble(2);
        double currentPointTheta = currentPoint.getDouble(3);

        if(getFieldSide() == "blue") {
            currentPointX = Constants.Physical.FIELD_LENGTH - currentPointX;
            currentPointTheta = Math.PI - currentPointTheta;
        }

        double feedForwardX = (targetX - currentPointX)/(targetTime - currentPointTime);
        double feedForwardY = (targetY - currentPointY)/(targetTime - currentPointTime);
        double feedForwardTheta = -(targetTheta - currentPointTheta)/(targetTime - currentPointTime);

        xPID.setSetPoint(targetX);
        yPID.setSetPoint(targetY);
        thetaPID.setSetPoint(targetTheta);
        
        xPID.updatePID(currentX);
        yPID.updatePID(currentY);
        thetaPID.updatePID(currentTheta);

        double xVelNoFF = xPID.getResult();
        double yVelNoFF = yPID.getResult();
        double thetaVelNoFF = -thetaPID.getResult();

        double xVel = feedForwardX + xVelNoFF;
        double yVel = feedForwardY + yVelNoFF;
        double thetaVel = feedForwardTheta + thetaVelNoFF;

        double[] velocityArray = new double[3];

        velocityArray[0] = xVel;
        velocityArray[1] = -yVel;
        velocityArray[2] = thetaVel;

        // System.out.println("Target Point: " + targetPoint);

        return velocityArray;
    }
    else {
        double[] velocityArray = new double[3];

        velocityArray[0] = 0;
        velocityArray[1] = 0;
        velocityArray[2] = 0;

        return velocityArray;
    }
  }
  
  @Override
  public void periodic() {

  }

  // get Joystick adjusted y-value
  public double getAdjustedY(double originalX, double originalY){
    double adjustedY = originalY * Math.sqrt((1-(Math.pow(originalX, 2))/2));
    return adjustedY;
  }

  // get Joystick adjusted x-value
  public double getAdjustedX(double originalX, double originalY){
    double adjustedX = originalX * Math.sqrt((1-(Math.pow(originalY, 2))/2));
    return adjustedX;
  }
}