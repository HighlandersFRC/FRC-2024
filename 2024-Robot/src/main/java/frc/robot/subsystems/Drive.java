// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.Arrays;

import org.apache.commons.math3.linear.MatrixUtils;
import org.apache.commons.math3.linear.RealMatrix;
import org.json.JSONArray;
import org.json.JSONObject;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
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
  private final TalonFX frontRightDriveMotor = new TalonFX(Constants.CANInfo.FRONT_RIGHT_DRIVE_MOTOR_ID, Constants.CANInfo.CANBUS_NAME);
  private final TalonFX frontRightAngleMotor = new TalonFX(Constants.CANInfo.FRONT_RIGHT_ANGLE_MOTOR_ID, Constants.CANInfo.CANBUS_NAME);
  private final TalonFX frontLeftDriveMotor = new TalonFX(Constants.CANInfo.FRONT_LEFT_DRIVE_MOTOR_ID, Constants.CANInfo.CANBUS_NAME);
  private final TalonFX frontLeftAngleMotor = new TalonFX(Constants.CANInfo.FRONT_LEFT_ANGLE_MOTOR_ID, Constants.CANInfo.CANBUS_NAME);
  private final TalonFX backLeftDriveMotor = new TalonFX(Constants.CANInfo.BACK_LEFT_DRIVE_MOTOR_ID, Constants.CANInfo.CANBUS_NAME);
  private final TalonFX backLeftAngleMotor = new TalonFX(Constants.CANInfo.BACK_LEFT_ANGLE_MOTOR_ID, Constants.CANInfo.CANBUS_NAME);
  private final TalonFX backRightDriveMotor = new TalonFX(Constants.CANInfo.BACK_RIGHT_DRIVE_MOTOR_ID, Constants.CANInfo.CANBUS_NAME);
  private final TalonFX backRightAngleMotor = new TalonFX(Constants.CANInfo.BACK_RIGHT_ANGLE_MOTOR_ID, Constants.CANInfo.CANBUS_NAME);

  private final CANcoder frontRightCanCoder = new CANcoder(Constants.CANInfo.FRONT_RIGHT_MODULE_CANCODER_ID, Constants.CANInfo.CANBUS_NAME);
  private final CANcoder frontLeftCanCoder = new CANcoder(Constants.CANInfo.FRONT_LEFT_MODULE_CANCODER_ID, Constants.CANInfo.CANBUS_NAME);
  private final CANcoder backLeftCanCoder = new CANcoder(Constants.CANInfo.BACK_LEFT_MODULE_CANCODER_ID, Constants.CANInfo.CANBUS_NAME);
  private final CANcoder backRightCanCoder = new CANcoder(Constants.CANInfo.BACK_RIGHT_MODULE_CANCODER_ID, Constants.CANInfo.CANBUS_NAME);

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

  //Odometry tracker
  private NetworkTable odometryTrackerTable = NetworkTableInstance.getDefault().getTable("odometry_tracking");
  private NetworkTableEntry odometryTrackerData = odometryTrackerTable.getEntry("odometry_data");

  private NetworkTable measurementTable = NetworkTableInstance.getDefault().getTable("measurements");
  private NetworkTableEntry tagPoseMeasurements = measurementTable.getEntry("tag");
  private NetworkTableEntry triangulationPoseMeasurements = measurementTable.getEntry("triangulation");

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

  private double lastLoopTime = Timer.getFPGATimestamp();  

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

  private double thetaP = 2.7;
  private double thetaI = 0.0;
  private double thetaD = 2.0;

  private PID xPID = new PID(xP, xI, xD);
  private PID yPID = new PID(yP, yI, yD);
  private PID thetaPID = new PID(thetaP, thetaI, thetaD);

  private String fieldSide = "blue";

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

    thetaPID.setMinOutput(-3);
    thetaPID.setMaxOutput(3);

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

    // changing odometry if on red side, don't need to change y because it will be the same for autos on either side
    if(this.fieldSide == "blue") {
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
        
    currentFusedOdometry[0] = firstPointX;
    currentFusedOdometry[1] = firstPointY;
    currentFusedOdometry[2] = firstPointAngle;

    currentX = currentFusedOdometry[0];
    currentY = currentFusedOdometry[1];
    currentTheta = currentFusedOdometry[2];

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

  public void updateOdometryFusedArray(){
    double navxOffset = Math.toRadians(peripherals.getPigeonAngle());

    SwerveModulePosition[] swerveModulePositions = new SwerveModulePosition[4];
    swerveModulePositions[0] = new SwerveModulePosition(frontLeft.getModuleDistance(), new Rotation2d(frontLeft.getCanCoderPositionRadians()));
    swerveModulePositions[1] = new SwerveModulePosition(frontRight.getModuleDistance(), new Rotation2d(frontRight.getCanCoderPositionRadians()));
    swerveModulePositions[2] = new SwerveModulePosition(backLeft.getModuleDistance(), new Rotation2d(backLeft.getCanCoderPositionRadians()));
    swerveModulePositions[3] = new SwerveModulePosition(backRight.getModuleDistance(), new Rotation2d(backRight.getCanCoderPositionRadians()));
        
    m_pose = m_odometry.update(new Rotation2d((navxOffset)), swerveModulePositions);

    currentX = getOdometryX();
    currentY = getOdometryY();
    currentTheta = navxOffset;

    // JSONArray frontCamCoordinates = this.peripherals.getFrontCamBasedPosition();
    // JSONObject frontCamLatencies = this.peripherals.getFrontCamLatencies();

    // double cameraBasedX = frontCamCoordinates.getDouble(0);
    // double cameraBasedY = frontCamCoordinates.getDouble(1);
    // Pose2d cameraBasedPosition = new Pose2d(new Translation2d(cameraBasedX, cameraBasedY), new Rotation2d(navxOffset));
    // m_odometry.addVisionMeasurement(cameraBasedPosition, Timer.getFPGATimestamp() - frontCamLatencies.getDouble("tl") - frontCamLatencies.getDouble("cl"));

    currentTime = Timer.getFPGATimestamp() - initTime;
    timeDiff = currentTime - previousTime;

    averagedX = (currentX + averagedX)/2;
    averagedY = (currentY + averagedY)/2;
    averagedTheta = (currentTheta + averagedTheta)/2;

    previousX = averagedX;
    previousY = averagedY;
    previousTheta = averagedTheta;
    previousTime = currentTime;
    previousEstimateX = estimatedX;
    previousEstimateY = estimatedY;
    previousEstimateTheta = estimatedTheta;

    currentFusedOdometry[0] = averagedX;
    currentFusedOdometry[1] = averagedY;
    currentFusedOdometry[2] = currentTheta;
  }

  // // method to update odometry by fusing prediction, encoder rotations, and camera values
  // public void updateOdometryFusedArray(){
  //   double pigeonAngle = Math.toRadians(peripherals.getPigeonAngle());

  //   //angle in field coordinate system, 0 = +x axis
  //   double fieldPigeonAngle = pigeonAngle;
  //   if (this.fieldSide == "red"){
  //     fieldPigeonAngle += Math.PI;
  //   }

  //   //maximum ammount the position of the robot could change by in one loop through
  //   double dt = Timer.getFPGATimestamp() - this.lastLoopTime;
  //   this.lastLoopTime = Timer.getFPGATimestamp();
  //   double maxChange = dt * Constants.Physical.TOP_SPEED;

  //   SwerveModulePosition[] swerveModulePositions = new SwerveModulePosition[4];
  //   swerveModulePositions[0] = new SwerveModulePosition(frontRight.getModuleDistance(), new Rotation2d(frontRight.getCanCoderPositionRadians()));
  //   swerveModulePositions[1] = new SwerveModulePosition(frontLeft.getModuleDistance(), new Rotation2d(frontLeft.getCanCoderPositionRadians()));
  //   swerveModulePositions[2] = new SwerveModulePosition(backLeft.getModuleDistance(), new Rotation2d(backLeft.getCanCoderPositionRadians()));
  //   swerveModulePositions[3] = new SwerveModulePosition(backRight.getModuleDistance(), new Rotation2d(backRight.getCanCoderPositionRadians()));

  //   //json data from all cameras
  //   JSONObject allCamResults = peripherals.getCameraMeasurements();

  //   boolean haveBackCam = !allCamResults.isNull("BackCam");
  //   JSONObject backCamResults = new JSONObject();
  //   if (haveBackCam){
  //     backCamResults = allCamResults.getJSONObject("BackCam");
  //   }

  //   boolean haveFrontCam = !allCamResults.isNull("FrontCam");
  //   JSONObject frontCamResults = new JSONObject();
  //   if (haveFrontCam){
  //     frontCamResults = allCamResults.getJSONObject("FrontCam");
  //   }

  //   boolean haveLeftCam = !allCamResults.isNull("LeftCam");
  //   JSONObject leftCamResults = new JSONObject();
  //   if (haveLeftCam){
  //     leftCamResults = allCamResults.getJSONObject("LeftCam");
  //   }

  //   boolean haveRightCam = !allCamResults.isNull("RightCam");
  //   JSONObject rightCamResults = new JSONObject();
  //   if (haveRightCam){
  //     rightCamResults = allCamResults.getJSONObject("RightCam");
  //   }

  //   double backCamTL = 9999;
  //   double backCamCL = 9999;
  //   JSONArray backCamBotPose = new JSONArray();
  //   JSONArray backCamFiducialResults = new JSONArray();
  //   if (haveBackCam){
  //     backCamTL = backCamResults.getDouble("tl") / 1000;
  //     backCamCL = backCamResults.getDouble("cl") / 1000;
  //     backCamBotPose = backCamResults.getJSONArray("botpose_wpiblue");
  //     backCamFiducialResults = backCamResults.getJSONArray("Fiducial");
  //     for (int i = 0; i < backCamFiducialResults.length(); i ++){
  //       JSONObject fiducial = (JSONObject) backCamFiducialResults.get(i);
  //       int id = fiducial.getInt("fID");
  //       if (id < 1 || id > 16){
  //         backCamFiducialResults.remove(i);
  //         i --;
  //       }
  //     }
  //   }
  //   double frontCamTL = 9999;
  //   double frontCamCL = 9999;
  //   JSONArray frontCamBotPose = new JSONArray();
  //   JSONArray frontCamFiducialResults = new JSONArray();
  //   if (haveFrontCam){
  //     frontCamTL = frontCamResults.getDouble("tl") / 1000;
  //     frontCamCL = frontCamResults.getDouble("cl") / 1000;
  //     frontCamBotPose = frontCamResults.getJSONArray("botpose_wpiblue");
  //     frontCamFiducialResults = frontCamResults.getJSONArray("Fiducial");
  //     for (int i = 0; i < frontCamFiducialResults.length(); i ++){
  //       JSONObject fiducial = (JSONObject) frontCamFiducialResults.get(i);
  //       int id = fiducial.getInt("fID");
  //       if (id < 1 || id > 16){
  //         frontCamFiducialResults.remove(i);
  //         i --;
  //       }
  //     }
  //   }
  //   double leftCamTL = 9999;
  //   double leftCamCL = 9999;
  //   JSONArray leftCamBotPose = new JSONArray();
  //   JSONArray leftCamFiducialResults = new JSONArray();
  //   if (haveLeftCam){
  //     leftCamTL = leftCamResults.getDouble("tl") / 1000;
  //     leftCamCL = leftCamResults.getDouble("cl") / 1000;
  //     leftCamBotPose = leftCamResults.getJSONArray("botpose_wpiblue");
  //     leftCamFiducialResults = leftCamResults.getJSONArray("Fiducial");
  //     for (int i = 0; i < leftCamFiducialResults.length(); i ++){
  //       JSONObject fiducial = (JSONObject) leftCamFiducialResults.get(i);
  //       int id = fiducial.getInt("fID");
  //       if (id < 1 || id > 16){
  //         leftCamFiducialResults.remove(i);
  //         i --;
  //       }
  //     }
  //   }
  //   double rightCamTL = 9999;
  //   double rightCamCL = 9999;
  //   JSONArray rightCamBotPose = new JSONArray();
  //   JSONArray rightCamFiducialResults = new JSONArray();
  //   if (haveRightCam){
  //     rightCamTL = rightCamResults.getDouble("tl") / 1000;
  //     rightCamCL = rightCamResults.getDouble("cl") / 1000;
  //     rightCamBotPose = rightCamResults.getJSONArray("botpose_wpiblue");
  //     rightCamFiducialResults = rightCamResults.getJSONArray("Fiducial");
  //     for (int i = 0; i < rightCamFiducialResults.length(); i ++){
  //       JSONObject fiducial = (JSONObject) rightCamFiducialResults.get(i);
  //       int id = fiducial.getInt("fID");
  //       if (id < 1 || id > 16){
  //         rightCamFiducialResults.remove(i);
  //         i --;
  //       }
  //     }
  //   }

  //   //combine fiducial data from all cameras, marked with which camera it came from
  //   JSONArray fiducialResults = new JSONArray();
  //   for (int i = 0; i < backCamFiducialResults.length(); i ++){
  //     JSONObject fiducial = (JSONObject) backCamFiducialResults.get(i);
  //     int id = fiducial.getInt("fID");
  //     if (id >= 1 && id <= 16){
  //       fiducial.put("camera", "back_cam");
  //       fiducialResults.put(fiducial);
  //     }
  //   }
  //   for (int i = 0; i < frontCamFiducialResults.length(); i ++){
  //     JSONObject fiducial = (JSONObject) frontCamFiducialResults.get(i);
  //     int id = fiducial.getInt("fID");
  //     if (id >= 1 && id <= 16){
  //       fiducial.put("camera", "front_cam");
  //       fiducialResults.put(fiducial);
  //     }
  //   }
  //   for (int i = 0; i < leftCamFiducialResults.length(); i ++){
  //     JSONObject fiducial = (JSONObject) leftCamFiducialResults.get(i);
  //     int id = fiducial.getInt("fID");
  //     if (id >= 1 && id <= 16){
  //       fiducial.put("camera", "left_cam");
  //       fiducialResults.put(fiducial);
  //     }
  //   }
  //   for (int i = 0; i < rightCamFiducialResults.length(); i ++){
  //     JSONObject fiducial = (JSONObject) rightCamFiducialResults.get(i);
  //     int id = fiducial.getInt("fID");
  //     if (id >= 1 && id <= 16){
  //       fiducial.put("camera", "right_cam");
  //       fiducialResults.put(fiducial);
  //     }
  //   }

  //   int numTracks = fiducialResults.length();

  //   //2d poses defining lines passing through offset tag positions (offset by camera offset from robot center) and the robot center
  //   //each JSONObject is of the sform:
  //   //{
  //   //  "x": float (x in field coordinates, meters),
  //   //  "y": float (y in field coordinates, meters),
  //   //  "theta": float (angle in field coordinates, radians),
  //   //  "camera": String (camera name, e.g. "back_cam"),
  //   //  "id": int (id number of AprilTag used for track) 
  //   //}
  //   ArrayList<JSONObject> horizontalTagPoses = new ArrayList<JSONObject>();

  //   //distances from offset tag positions (offset by camera offset from robot center) to the robot center
  //   //each JSONObject is of the form:
  //   //{
  //   //  "x": float (x in field coordinates, meters),
  //   //  "y": float (y in field coordinates, meters),
  //   //  "dist": float (distance from robot center to offset target, meters),
  //   //  "camera": String (camera name, e.g. "back_cam"),
  //   //  "id": int (id number of AprilTag used for track)
  //   //}
  //   ArrayList<JSONObject> verticalTagDistances = new ArrayList<JSONObject>();

  //   //calculate distances and field centric angles to tags from robot center
  //   for (int i = 0; i < fiducialResults.length(); i ++){
  //     JSONObject fiducial = (JSONObject) fiducialResults.get(i);
  //     int id = fiducial.getInt("fID");
  //     double cameraOffsetX = 0;
  //     double cameraOffsetY = 0;
  //     double cameraOffsetZ = 0;
  //     double cameraOffsetPitch = 0;
  //     double cameraOffsetTheta = 0;
  //     String camera = fiducial.getString("camera");
  //     //3d camera offset in field coordinates (meters and radians)
  //     if (camera == "back_cam"){
  //         cameraOffsetX = Constants.Vision.BACK_CAMERA_POSITION_POLAR[0] * Math.cos(Constants.Vision.BACK_CAMERA_POSITION_POLAR[1] + fieldPigeonAngle);
  //         cameraOffsetY = Constants.Vision.BACK_CAMERA_POSITION_POLAR[0] * Math.sin(Constants.Vision.BACK_CAMERA_POSITION_POLAR[1] + fieldPigeonAngle);
  //         cameraOffsetZ = Constants.Vision.BACK_CAMERA_POSE[2];
  //         cameraOffsetTheta = Constants.Vision.BACK_CAMERA_POSE[5];
  //         cameraOffsetPitch = Constants.Vision.BACK_CAMERA_POSE[4];
  //     } else if (camera == "front_cam"){
  //         cameraOffsetX = Constants.Vision.FRONT_CAMERA_POSITION_POLAR[0] * Math.cos(Constants.Vision.FRONT_CAMERA_POSITION_POLAR[1] + fieldPigeonAngle);
  //         cameraOffsetY = Constants.Vision.FRONT_CAMERA_POSITION_POLAR[0] * Math.sin(Constants.Vision.FRONT_CAMERA_POSITION_POLAR[1] + fieldPigeonAngle);
  //         cameraOffsetZ = Constants.Vision.FRONT_CAMERA_POSE[2];
  //         cameraOffsetTheta = Constants.Vision.FRONT_CAMERA_POSE[5];
  //         cameraOffsetPitch = Constants.Vision.FRONT_CAMERA_POSE[4];
  //     } else if (camera == "left_cam"){
  //         cameraOffsetX = Constants.Vision.LEFT_CAMERA_POSITION_POLAR[0] * Math.cos(Constants.Vision.LEFT_CAMERA_POSITION_POLAR[1] + fieldPigeonAngle);
  //         cameraOffsetY = Constants.Vision.LEFT_CAMERA_POSITION_POLAR[0] * Math.sin(Constants.Vision.LEFT_CAMERA_POSITION_POLAR[1] + fieldPigeonAngle);
  //         cameraOffsetZ = Constants.Vision.LEFT_CAMERA_POSE[2];
  //         cameraOffsetTheta = Constants.Vision.LEFT_CAMERA_POSE[5];
  //         cameraOffsetPitch = Constants.Vision.LEFT_CAMERA_POSE[4];
  //     } else if (camera == "right_cam"){
  //         cameraOffsetX = Constants.Vision.RIGHT_CAMERA_POSITION_POLAR[0] * Math.cos(Constants.Vision.RIGHT_CAMERA_POSITION_POLAR[1] + fieldPigeonAngle);
  //         cameraOffsetY = Constants.Vision.RIGHT_CAMERA_POSITION_POLAR[0] * Math.sin(Constants.Vision.RIGHT_CAMERA_POSITION_POLAR[1] + fieldPigeonAngle);
  //         cameraOffsetZ = Constants.Vision.RIGHT_CAMERA_POSE[2];
  //         cameraOffsetTheta = Constants.Vision.RIGHT_CAMERA_POSE[5];
  //         cameraOffsetPitch = Constants.Vision.RIGHT_CAMERA_POSE[4];
  //     }
  //     //pose to add to horizontalTagPoses
  //     JSONObject pose = new JSONObject();
  //     pose.put("x", Constants.Vision.TAG_POSES[id - 1][0] - cameraOffsetX);
  //     pose.put("y", Constants.Vision.TAG_POSES[id - 1][1] - cameraOffsetY);
  //     pose.put("theta", -Constants.degreesToRadians(fiducial.getDouble("tx")) + cameraOffsetTheta + fieldPigeonAngle);
  //     pose.put("camera", camera);
  //     pose.put("id", id);
  //     horizontalTagPoses.add(pose);

  //     //distance info to add to verticalTagDistances
  //     double verticalAngle = Constants.degreesToRadians(fiducial.getDouble("ty")) + cameraOffsetPitch;
  //     JSONObject dist = new JSONObject();
  //     dist.put("x", Constants.Vision.TAG_POSES[id - 1][0] - cameraOffsetX);
  //     dist.put("y", Constants.Vision.TAG_POSES[id - 1][1] - cameraOffsetY);
  //     dist.put("dist", -(Constants.Vision.TAG_POSES[id - 1][2] - cameraOffsetZ) / Math.tan(verticalAngle));
  //     dist.put("camera", camera);
  //     dist.put("id", id);
  //     verticalTagDistances.add(dist);
  //   }   

  //   //angle of elevation distance and tag angle approach
  //   for (int i = 0; i < numTracks; i ++){
  //       JSONObject horizontalTagPose = horizontalTagPoses.get(i);
  //       double dist = verticalTagDistances.get(i).getDouble("dist");
  //       double x = horizontalTagPose.getDouble("x") + dist * Math.cos(horizontalTagPose.getDouble("theta"));
  //       double y = horizontalTagPose.getDouble("y") + dist * Math.sin(horizontalTagPose.getDouble("theta"));
  //       int id = horizontalTagPose.getInt("id");
  //       double xOffset = x - Constants.Vision.TAG_POSES[id - 1][0];
  //       double yOffset = y - Constants.Vision.TAG_POSES[id - 1][1];
  //       Matrix<N3, N1> standardDeviation = new Matrix<>(Nat.N3(), Nat.N1());
  //       if (Constants.getDistance(currentX, currentY, x, y) > maxChange){
  //         double dif = Constants.getDistance(currentX, currentY, x, y) - maxChange;
  //         standardDeviation.set(0, 0, Constants.Vision.getTriStdDevX(xOffset, yOffset) * Constants.Vision.getTagDistStdDevScalar(dist) + Math.pow(dif, Constants.Vision.ODOMETRY_JUMP_STANDARD_DEVIATION_DEGREE) * Constants.Vision.ODOMETRY_JUMP_STANDARD_DEVIATION_SCALAR);
  //         standardDeviation.set(1, 0, Constants.Vision.getTriStdDevY(xOffset, yOffset) * Constants.Vision.getTagDistStdDevScalar(dist) + Math.pow(dif, Constants.Vision.ODOMETRY_JUMP_STANDARD_DEVIATION_DEGREE) * Constants.Vision.ODOMETRY_JUMP_STANDARD_DEVIATION_SCALAR);
  //       } else {
  //         standardDeviation.set(0, 0, Constants.Vision.getTriStdDevX(xOffset, yOffset));
  //         standardDeviation.set(1, 0, Constants.Vision.getTriStdDevY(xOffset, yOffset));
  //       }
  //       standardDeviation.set(2, 0, 0);
  //       if (horizontalTagPose.getString("camera") == "back_cam"){
  //         // m_odometry.addVisionMeasurement(new Pose2d(new Translation2d(x, y), new Rotation2d(pigeonAngle)), Timer.getFPGATimestamp() - (backCamTL + backCamCL), standardDeviation);
  //       } else if (horizontalTagPose.getString("camera") == "front_cam"){
  //         // m_odometry.addVisionMeasurement(new Pose2d(new Translation2d(x, y), new Rotation2d(pigeonAngle)), Timer.getFPGATimestamp() - (frontCamTL + frontCamCL), standardDeviation);
  //       } else if (horizontalTagPose.getString("camera") == "left_cam"){
  //         // m_odometry.addVisionMeasurement(new Pose2d(new Translation2d(x, y), new Rotation2d(pigeonAngle)), Timer.getFPGATimestamp() - (leftCamTL + leftCamCL), standardDeviation);
  //       } else if (horizontalTagPose.getString("camera") == "right_cam"){
  //         // m_odometry.addVisionMeasurement(new Pose2d(new Translation2d(x, y), new Rotation2d(pigeonAngle)), Timer.getFPGATimestamp() - (rightCamTL + rightCamCL), standardDeviation);
  //       }
  //   }

  //   //AprilTag pose estimation approach
  //   if (backCamBotPose.length() == 6){
  //     double x = (double) backCamBotPose.get(0);
  //     double y = (double) backCamBotPose.get(1);
  //     if (x != 0 && y != 0 && backCamFiducialResults.length() != 0){
  //       int id = ((JSONObject) backCamFiducialResults.get(0)).getInt("fID");
  //       int numBackTracks = backCamFiducialResults.length();
  //       double xOffset = x - Constants.Vision.TAG_POSES[id - 1][0];
  //       double yOffset = y - Constants.Vision.TAG_POSES[id - 1][1];
  //       double distToTag = Constants.getDistance(xOffset, yOffset, 0, 0);
  //       Matrix<N3, N1> standardDeviation = new Matrix<>(Nat.N3(), Nat.N1());
  //       if (Constants.getDistance(currentX, currentY, x, y) > maxChange){
  //         double dif = Constants.getDistance(currentX, currentY, x, y) - maxChange;
  //         standardDeviation.set(0, 0, Constants.Vision.getNumTagStdDevScalar(numBackTracks) * (Constants.Vision.getTagDistStdDevScalar(distToTag) + Math.pow(dif, Constants.Vision.ODOMETRY_JUMP_STANDARD_DEVIATION_DEGREE) * Constants.Vision.ODOMETRY_JUMP_STANDARD_DEVIATION_SCALAR));
  //         standardDeviation.set(1, 0, Constants.Vision.getNumTagStdDevScalar(numBackTracks) * (Constants.Vision.getTagDistStdDevScalar(distToTag) + Math.pow(dif, Constants.Vision.ODOMETRY_JUMP_STANDARD_DEVIATION_DEGREE) * Constants.Vision.ODOMETRY_JUMP_STANDARD_DEVIATION_SCALAR));
  //       } else {
  //         standardDeviation.set(0, 0, Constants.Vision.getNumTagStdDevScalar(numBackTracks) * (Constants.Vision.getTagDistStdDevScalar(distToTag)));
  //         standardDeviation.set(1, 0, Constants.Vision.getNumTagStdDevScalar(numBackTracks) * (Constants.Vision.getTagDistStdDevScalar(distToTag)));
  //       }
  //       standardDeviation.set(2, 0, 0);
  //       m_odometry.addVisionMeasurement(new Pose2d(new Translation2d(x, y), new Rotation2d(pigeonAngle)), Timer.getFPGATimestamp() - (backCamTL + backCamCL), standardDeviation);
  //     }
  //   }
  //   if (frontCamBotPose.length() == 6){
  //     double x = (double) frontCamBotPose.get(0);
  //     double y = (double) frontCamBotPose.get(1);
  //     if (x != 0 && y != 0 && frontCamFiducialResults.length() != 0){
  //       int id = ((JSONObject) frontCamFiducialResults.get(0)).getInt("fID");
  //       int numFrontTracks = frontCamFiducialResults.length();
  //       double xOffset = x - Constants.Vision.TAG_POSES[id - 1][0];
  //       double yOffset = y - Constants.Vision.TAG_POSES[id - 1][1];
  //       double distToTag = Constants.getDistance(xOffset, yOffset, 0, 0);
  //       Matrix<N3, N1> standardDeviation = new Matrix<>(Nat.N3(), Nat.N1());
  //       if (Constants.getDistance(currentX, currentY, x, y) > maxChange){
  //         double dif = Constants.getDistance(currentX, currentY, x, y) - maxChange;
  //         standardDeviation.set(0, 0, Constants.Vision.getNumTagStdDevScalar(numFrontTracks) * Constants.Vision.getTagDistStdDevScalar(distToTag) + Math.pow(dif, Constants.Vision.ODOMETRY_JUMP_STANDARD_DEVIATION_DEGREE) * Constants.Vision.ODOMETRY_JUMP_STANDARD_DEVIATION_SCALAR);
  //         standardDeviation.set(1, 0, Constants.Vision.getNumTagStdDevScalar(numFrontTracks) * Constants.Vision.getTagDistStdDevScalar(distToTag) + Math.pow(dif, Constants.Vision.ODOMETRY_JUMP_STANDARD_DEVIATION_DEGREE) * Constants.Vision.ODOMETRY_JUMP_STANDARD_DEVIATION_SCALAR);
  //       } else {
  //         standardDeviation.set(0, 0, Constants.Vision.getNumTagStdDevScalar(numFrontTracks) * Constants.Vision.getTagDistStdDevScalar(distToTag));
  //         standardDeviation.set(1, 0, Constants.Vision.getNumTagStdDevScalar(numFrontTracks) * Constants.Vision.getTagDistStdDevScalar(distToTag));
  //       }
  //       standardDeviation.set(2, 0, 0);
  //       m_odometry.addVisionMeasurement(new Pose2d(new Translation2d(x, y), new Rotation2d(pigeonAngle)), Timer.getFPGATimestamp() - (frontCamTL + frontCamCL), standardDeviation);
  //     }
  //   }
  //   if (leftCamBotPose.length() == 6){
  //     double x = (double) leftCamBotPose.get(0);
  //     double y = (double) leftCamBotPose.get(1);
  //     if (x != 0 && y != 0 && leftCamFiducialResults.length() != 0){
  //       int id = ((JSONObject) leftCamFiducialResults.get(0)).getInt("fID");
  //       int numLeftTracks = leftCamFiducialResults.length();
  //       double xOffset = x - Constants.Vision.TAG_POSES[id - 1][0];
  //       double yOffset = y - Constants.Vision.TAG_POSES[id - 1][1];
  //       double distToTag = Constants.getDistance(xOffset, yOffset, 0, 0);
  //       Matrix<N3, N1> standardDeviation = new Matrix<>(Nat.N3(), Nat.N1());
  //       if (Constants.getDistance(currentX, currentY, x, y) > maxChange){
  //         double dif = Constants.getDistance(currentX, currentY, x, y) - maxChange;
  //         standardDeviation.set(0, 0, Constants.Vision.getNumTagStdDevScalar(numLeftTracks) * (Constants.Vision.getTagDistStdDevScalar(distToTag) + Math.pow(dif, Constants.Vision.ODOMETRY_JUMP_STANDARD_DEVIATION_DEGREE) * Constants.Vision.ODOMETRY_JUMP_STANDARD_DEVIATION_SCALAR));
  //         standardDeviation.set(1, 0, Constants.Vision.getNumTagStdDevScalar(numLeftTracks) * (Constants.Vision.getTagDistStdDevScalar(distToTag) + Math.pow(dif, Constants.Vision.ODOMETRY_JUMP_STANDARD_DEVIATION_DEGREE) * Constants.Vision.ODOMETRY_JUMP_STANDARD_DEVIATION_SCALAR));
  //       } else {
  //         standardDeviation.set(0, 0, Constants.Vision.getNumTagStdDevScalar(numLeftTracks) * (Constants.Vision.getTagDistStdDevScalar(distToTag)));
  //         standardDeviation.set(1, 0, Constants.Vision.getNumTagStdDevScalar(numLeftTracks) * (Constants.Vision.getTagDistStdDevScalar(distToTag)));
  //       }
  //       standardDeviation.set(2, 0, 0);
  //       m_odometry.addVisionMeasurement(new Pose2d(new Translation2d(x, y), new Rotation2d(pigeonAngle)), Timer.getFPGATimestamp() - (leftCamTL + leftCamCL), standardDeviation);
  //     }
  //   }
  //   if (rightCamBotPose.length() == 6){
  //     double x = (double) rightCamBotPose.get(0);
  //     double y = (double) rightCamBotPose.get(1);
  //     if (x != 0 && y != 0 && rightCamFiducialResults.length() != 0){
  //       int id = ((JSONObject) rightCamFiducialResults.get(0)).getInt("fID");
  //       int numRightTracks = rightCamFiducialResults.length();
  //       double xOffset = x - Constants.Vision.TAG_POSES[id - 1][0];
  //       double yOffset = y - Constants.Vision.TAG_POSES[id - 1][1];
  //       double distToTag = Constants.getDistance(xOffset, yOffset, 0, 0);
  //       Matrix<N3, N1> standardDeviation = new Matrix<>(Nat.N3(), Nat.N1());
  //       if (Constants.getDistance(currentX, currentY, x, y) > maxChange){
  //         double dif = Constants.getDistance(currentX, currentY, x, y) - maxChange;
  //         standardDeviation.set(0, 0, Constants.Vision.getNumTagStdDevScalar(numRightTracks) * (Constants.Vision.getTagDistStdDevScalar(distToTag) + Math.pow(dif, Constants.Vision.ODOMETRY_JUMP_STANDARD_DEVIATION_DEGREE) * Constants.Vision.ODOMETRY_JUMP_STANDARD_DEVIATION_SCALAR));
  //         standardDeviation.set(1, 0, Constants.Vision.getNumTagStdDevScalar(numRightTracks) * (Constants.Vision.getTagDistStdDevScalar(distToTag) + Math.pow(dif, Constants.Vision.ODOMETRY_JUMP_STANDARD_DEVIATION_DEGREE) * Constants.Vision.ODOMETRY_JUMP_STANDARD_DEVIATION_SCALAR));
  //       } else {
  //         standardDeviation.set(0, 0, Constants.Vision.getNumTagStdDevScalar(numRightTracks) * (Constants.Vision.getTagDistStdDevScalar(distToTag)));
  //         standardDeviation.set(1, 0, Constants.Vision.getNumTagStdDevScalar(numRightTracks) * (Constants.Vision.getTagDistStdDevScalar(distToTag)));
  //       }
  //       standardDeviation.set(2, 0, 0);
  //       m_odometry.addVisionMeasurement(new Pose2d(new Translation2d(x, y), new Rotation2d(pigeonAngle)), Timer.getFPGATimestamp() - (rightCamTL + rightCamCL), standardDeviation);
  //     }
  //   }

  //   //feed in encoders and pigeon and get pose estimation
  //   m_pose = m_odometry.update(new Rotation2d(pigeonAngle), swerveModulePositions);

  //   double finalX = m_pose.getX();
  //   double finalY = m_pose.getY();
    
  //   // System.out.println("Update X:" + finalX + " Y: " + " Theta: " + pigeonAngle);

  //   currentFusedOdometry[0] = finalX;
  //   currentFusedOdometry[1] = finalY;
  //   currentFusedOdometry[2] = pigeonAngle;

  //   currentX = currentFusedOdometry[0];
  //   currentY = currentFusedOdometry[1];
  //   currentTheta = currentFusedOdometry[2];

  //   //odometry data to send to odometry tracking tool
  //   JSONObject trackerData = new JSONObject();
  //   //final odometry pose
  //   JSONObject pose = new JSONObject();
  //   pose.put("x", finalX);
  //   pose.put("y", finalY);
  //   pose.put("theta", fieldPigeonAngle);
  //   //list of tags tracked by which cameras
  //   JSONArray tracks = new JSONArray();
  //   for (int i = 0; i < numTracks; i ++){
  //     JSONObject track = new JSONObject();
  //     JSONObject fiducial = (JSONObject) fiducialResults.get(i);
  //     track.put("camera", fiducial.getString("camera"));
  //     track.put("fID", fiducial.getInt("fID"));
  //     tracks.put(track);
  //   }
  //   trackerData.put("time", Timer.getFPGATimestamp());
  //   trackerData.put("pose", pose);
  //   trackerData.put("tracks", tracks);
  //   odometryTrackerData.setString(trackerData.toString());
  // }

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

  public void autoRobotCentricTurn(double turn){
    frontLeft.drive(new Vector(0, 0), turn, 0.0);
    frontRight.drive(new Vector(0, 0), turn, 0.0);
    backLeft.drive(new Vector(0, 0), turn, 0.0);
    backRight.drive(new Vector(0, 0), turn, 0.0);
    updateOdometryFusedArray();
  }

  public void autoRobotCentricDrive(Vector velocityVector, double turnRadiansPerSec){
    updateOdometryFusedArray();
    frontLeft.drive(velocityVector, turnRadiansPerSec, 0);
    frontRight.drive(velocityVector, turnRadiansPerSec, 0);
    backLeft.drive(velocityVector, turnRadiansPerSec, 0);
    backRight.drive(velocityVector, turnRadiansPerSec, 0);
  }
  
  public void teleopDrive() {
    updateOdometryFusedArray();
    double turnLimit = 0.35;

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

  //get current velocity vector of the robot in field coordinats, m/s
  public Vector getRobotVelocityVector(){
    Vector velocityVector = new Vector(0, 0);
    double pigeonAngleRadians = Math.toRadians(this.peripherals.getPigeonAngle());

    double frV = this.frontRight.getGroundSpeed();
    double frTheta = this.frontRight.getWheelPosition() + pigeonAngleRadians;
    double frVX = frV * Math.cos(frTheta);
    double frVY = frV * Math.sin(frTheta);
    double flV = this.frontLeft.getGroundSpeed();
    double flTheta = this.frontLeft.getWheelPosition() + pigeonAngleRadians;
    double flVX = flV * Math.cos(flTheta);
    double flVY = flV * Math.sin(flTheta);
    double blV = this.backLeft.getGroundSpeed();
    double blTheta = this.backLeft.getWheelPosition() + pigeonAngleRadians;
    double blVX = blV * Math.cos(blTheta);
    double blVY = blV * Math.sin(blTheta);
    double brV = this.backRight.getGroundSpeed();
    double brTheta = this.backRight.getWheelPosition() + pigeonAngleRadians;
    double brVX = brV * Math.cos(brTheta);
    double brVY = brV * Math.sin(brTheta);

    velocityVector.setI(frVX + flVX + blVX + brVX);
    velocityVector.setJ(frVY + flVY + blVY + brVY);
    
    // System.out.println("VVector: <" + velocityVector.getI() + ", " + velocityVector.getJ() + ">");

    return velocityVector;
  }

  public Vector getRobotAccelerationVector(){
    Vector accelerationVector = new Vector();

    Vector robotCentricAccelerationVector = this.peripherals.getPigeonLinAccel();
    double accelerationMagnitude = Constants.getDistance(robotCentricAccelerationVector.getI(), robotCentricAccelerationVector.getJ(), 0, 0);
    accelerationVector.setI(accelerationMagnitude * Math.cos(Math.toRadians(this.peripherals.getPigeonAngle())));
    accelerationVector.setJ(accelerationMagnitude * Math.sin(Math.toRadians(this.peripherals.getPigeonAngle())));

    return accelerationVector;
  }

  public JSONArray getPathPoint(JSONArray path, double time){
    for (int i = 0; i < path.length() - 1; i ++){
      JSONArray currentPoint = path.getJSONArray(i + 1);
      JSONArray previousPoint = path.getJSONArray(i);
      double currentPointTime = currentPoint.getDouble(0);
      double previousPointTime = previousPoint.getDouble(0);
      if (time >= previousPointTime && time < currentPointTime){
        return currentPoint;
      }
    }
    if (time < path.getJSONArray(0).getDouble(0)){
      return path.getJSONArray(0);
    } else {
      return path.getJSONArray(path.length() - 1);
    }
  }

  // Autonomous algorithm
  public double[] pidController(double currentX, double currentY, double currentTheta, double time, JSONArray pathPoints, boolean pickupNote) {
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

        if(this.fieldSide == "blue") {
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

        if(this.fieldSide == "blue") {
          currentPointX = Constants.Physical.FIELD_LENGTH - currentPointX;
          currentPointTheta = Math.PI - currentPointTheta;
        }

        if (pickupNote){
          double angleToNote =  peripherals.getBackCamTargetTx();
          double differenceX = Math.abs(targetX - currentPointX);
          double differenceY = Math.abs(targetY - currentPointY);
          double r = (Math.sqrt((differenceX * differenceX) + (differenceY * differenceY)));
          double adjustedX = r * (Math.cos((targetTheta + Math.PI) - angleToNote));
          double adjustedY = r * (Math.sin((targetTheta + Math.PI) - angleToNote));
          targetX = targetX + adjustedX;
          targetY = targetY + (2.6 * adjustedY);
          targetTheta = targetTheta - angleToNote;
        }

        double feedForwardX = (targetX - currentPointX)/(targetTime - currentPointTime);
        double feedForwardY = (targetY - currentPointY)/(targetTime - currentPointTime);
        // double feedForwardTheta = -(targetTheta - currentPointTheta)/(targetTime - currentPointTime);
        double feedForwardTheta = 0;

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

        // System.out.println("Targ - X: " + targetX + " Y: " + targetY + " Theta: " + targetTheta);
        // System.out.println("PID side: " + this.fieldSide);

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
    updateOdometryFusedArray();
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