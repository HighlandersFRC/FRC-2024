package frc.robot.subsystems;

import java.net.InetAddress;
import java.util.ArrayList;

import org.json.JSONArray;
import org.json.JSONObject;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.commands.defaults.PeripheralsDefault;
import frc.robot.tools.math.Vector;

public class Peripherals extends SubsystemBase {
  private NetworkTable backCam = NetworkTableInstance.getDefault().getTable("limelight-back");
  private NetworkTableEntry backCamJSON = backCam.getEntry("json");
  private NetworkTableEntry backCamTrack = backCam.getEntry("tv");
  private NetworkTable frontCam = NetworkTableInstance.getDefault().getTable("limelight-front");
  private NetworkTableEntry frontCamJSON = frontCam.getEntry("json");
  private NetworkTableEntry frontCamTy = frontCam.getEntry("ty");
  private NetworkTableEntry frontCamTx = frontCam.getEntry("tx");
  private NetworkTableEntry frontCamTl = frontCam.getEntry("tl");
  private NetworkTableEntry frontCamCl = frontCam.getEntry("cl");
  private NetworkTableEntry frontCamHB = frontCam.getEntry("hb");
  private NetworkTableEntry frontCamIDs = frontCam.getEntry("tid");
  private NetworkTableEntry frontCamIDSet = frontCam.getEntry("priorityid");
  private NetworkTableEntry frontCamRobotTagPose = frontCam.getEntry("botpose_targetspace");
  private NetworkTableEntry frontCamRobotFieldPose = frontCam.getEntry("botpose_wpiblue");
  private NetworkTableEntry backCamTx = backCam.getEntry("tx");  
  private NetworkTableEntry backCamTy = backCam.getEntry("ty");
  private NetworkTableEntry backCamConfidence = backCam.getEntry("conf");
  private NetworkTable leftCam = NetworkTableInstance.getDefault().getTable("limelight-left");
  private NetworkTableEntry leftCamTl = leftCam.getEntry("tl");
  private NetworkTableEntry leftCamCl = leftCam.getEntry("cl");
  private NetworkTableEntry leftCamJSON = leftCam.getEntry("json");
  private NetworkTableEntry leftCamRobotFieldPose = leftCam.getEntry("botpose_wpiblue");
  private NetworkTableEntry leftCamRobotTagPose = leftCam.getEntry("botpose_targetspace");
  private NetworkTable rightCam = NetworkTableInstance.getDefault().getTable("limelight-right");
  private NetworkTableEntry rightCamTl = rightCam.getEntry("tl");
  private NetworkTableEntry rightCamCl = rightCam.getEntry("cl");
  private NetworkTableEntry rightCamJSON = rightCam.getEntry("json");
  private NetworkTableEntry rightCamRobotFieldPose = rightCam.getEntry("botpose_wpiblue");
  private NetworkTableEntry rightCamRobotTagPose = rightCam.getEntry("botpose_targetspace");
  private String fieldSide = "none";

  private double[] noTrackLimelightArray = new double[6];

  private Pigeon2 pigeon = new Pigeon2(0, "Canivore");
  private Pigeon2Configuration pigeonConfig = new Pigeon2Configuration();

  public Peripherals() {
  }

  // public void setLimelightLights(String limelight, int ledMode) {
  //   if (limelight == "left") {
  //     leftCam.getEntry("ledMode").setNumber(ledMode);
  //   } else if (limelight == "right") {
  //     rightCam.getEntry("ledMode").setNumber(ledMode);
  //   } else if (limelight == "back") {
  //     backCam.getEntry("ledMode").setNumber(ledMode);
  //   } else if (limelight == "front") {
  //     frontCam.getEntry("ledMode").setNumber(ledMode);
  //   }
  // }

  /**
   * Checks the connectivity of Limelight devices.
   * 
   * @return true if both Limelights are reachable, false otherwise.
   */
  public boolean limelightsConnected() {
    boolean reachable = true;
    // try {
    //   InetAddress address3 = InetAddress.getByName("10.44.99.43"); // back limelight ip address
    //   if(!address3.isReachable(100)) {
    //     reachable = false;
    //   }
    // } catch (Exception e) {
    //   System.out.println("What an absolute L piece of code: " + e);
    // }
    try {
      InetAddress address4 = InetAddress.getByName("10.44.99.44"); // front limelight ip address
      if(!address4.isReachable(100)) {
        reachable = false;
      }
    } catch (Exception e) {
      System.out.println("What an absolute W piece of code: " + e);
    }
    // System.out.println("Back Cam: " + );
    // System.out.println("Front Cam: " + );
    return reachable;
  }

  /**
   * Initializes the peripherals.
   */
  public void setFieldSide(String fieldSide){
    this.fieldSide = fieldSide;
    // if (fieldSide == "blue"){
    //   // System.out.println("blue side");
    //   frontCamIDSet.setInteger(7);
    // } else {
    //   // System.out.println("red side");
    //   frontCamIDSet.setInteger(4);
    // }
  }
  public void init() {
    pigeonConfig.MountPose.MountPosePitch = -85.28816223144531;
    pigeonConfig.MountPose.MountPoseRoll = 32.49882125854492;
    pigeonConfig.MountPose.MountPoseYaw = 0.19013315439224243;
    pigeon.getConfigurator().apply(pigeonConfig);
    zeroPigeon();
    noTrackLimelightArray[0] = 0;
    noTrackLimelightArray[1] = 0;
    noTrackLimelightArray[2] = 0;
    noTrackLimelightArray[3] = 0;
    noTrackLimelightArray[4] = 0;
    noTrackLimelightArray[5] = 0;
    setDefaultCommand(new PeripheralsDefault(this));
  }

  /**
   * Checks if the back camera is tracking a target.
   *
   * @return {@code true} if the back camera is tracking a target, {@code false} otherwise.
   */
  public boolean getBackCamTrack() {
    if (backCam.getEntry("tv").getInteger(0) == 1) {
      return true;
    } else
      return false;
  }

  /**
   * Retrieves the Y-Axis Rotation of the robot based on the front camera
   * @return Y-Axis rotation in radians
   */
  public double getFrontCamTargetTy() {
    // JSONObject results = new JSONObject(this.frontCamJSON.getString("{Results: {}}")).getJSONObject("Results");
    // if (results.isNull("Fiducial")) {
    //   return 100;
    // }
    // JSONArray fiducials = results.getJSONArray("Fiducial");
    // for (int i = 0; i < fiducials.length(); i++) {
    //   int id = ((JSONObject) fiducials.get(i)).getInt("fID");
    //   if (id == 7 || id == 4) {
    //     return ((JSONObject) fiducials.get(i)).getDouble("ty");
    //   }
    // }
    // return 100;
    return frontCamTy.getDouble(100);
  }

  /**
   * Retrieves the X-Axis Rotation of the robot based on the front camera
   * @return X-Axis rotation in radians
   */
  public double getFrontCamTargetTx() {
    // JSONObject results = new JSONObject(this.frontCamJSON.getString("{Results: {}}")).getJSONObject("Results");
    // if (results.isNull("Fiducial")) {
    //   return 100;
    // }
    // JSONArray fiducials = results.getJSONArray("Fiducial");
    // for (int i = 0; i < fiducials.length(); i++) {
    //   int id = ((JSONObject) fiducials.get(i)).getInt("fID");
    //   if (id == 7 || id == 4) {
    //     return ((JSONObject) fiducials.get(i)).getDouble("tx");
    //   }
    // }
    // return 100;
    return frontCamTx.getDouble(100);
  }

  public double getFrontCamHB(){
    return frontCamHB.getDouble(-1);
  }

  /**
   * Retrieves all fiducial IDs of the front camera
   * @return ArrayList of fiducial IDs
   */
  public ArrayList<Integer> getFrontCamIDs() {
    JSONObject results = new JSONObject(this.frontCamJSON.getString("{Results: {}}")).getJSONObject("Results");
    if (results.isNull("Fiducial")) {
      return new ArrayList<Integer>();
    }
    ArrayList<Integer> ids = new ArrayList<Integer>();
    JSONArray fiducials = results.getJSONArray("Fiducial");
    for (int i = 0; i < fiducials.length(); i++) {
      ids.add(((JSONObject) fiducials.get(i)).getInt("fID"));
    }
    return ids;
  }

  public double getFrontCamID() {
    return NetworkTableInstance.getDefault().getTable("limelight-front").getEntry("tid").getDouble(0);
  }

  /**
   * Retrieves the latencies of the front camera
   * @return JSONObject {"tl": target latency, "cl": camera latency}
   */
  public JSONObject getFrontCamLatencies() {
    JSONObject latencies = new JSONObject();
    latencies.put("tl", this.frontCamTl.getDouble(0) / 1000);
    latencies.put("cl", this.frontCamCl.getDouble(0) / 1000);
    return latencies;
  }

  public double getFrontCameraLatency() {
    double latency = frontCamTl.getDouble(-1) + frontCamCl.getDouble(-1);
    return latency;
  }

  public double getLeftCameraLatency() {
    double latency = leftCamTl.getDouble(-1) + leftCamCl.getDouble(-1);
    return latency;
  }

  public double getRightCameraLatency() {
    double latency = rightCamTl.getDouble(-1) + rightCamCl.getDouble(-1);
    return latency;
  }

  /**
   * Retrieves the X-Axis Rotation of the robot based on the back camera
   * @return X-Axis rotation in radians
   */
  public double getBackCamTargetTx() {
    return backCamTx.getDouble(0.0);
  }

  public double getBackCamTargetTy(){
    return backCamTy.getDouble(100);
  }

  public double getBackCamTargetConfidence(){
    return backCamConfidence.getDouble(0.0);
  }

  /**
   * Sets the pipeline of the front camera
   * @param pipeline - index to set the pipeline to 
   */
  public void setFrontCamPipeline(int pipeline) {
    frontCam.getEntry("pipeline").setNumber(pipeline);
  }

  /**
   * Sets the pipeline of the back camera
   * @param pipeline - index to set the pipeline to 
   */
  public void setBackCamPipeline(int pipeline) {
    backCam.getEntry("pipeline").setNumber(pipeline);
  }

  /**
   * Sets the pipeline of the left camera
   * @param pipeline - index to set the pipeline to 
   */
  public void setLeftCamPipeline(int pipeline) {
    leftCam.getEntry("pipeline").setNumber(pipeline);
  }

  /**
   * Sets the pipeline of the right camera
   * @param pipeline - index to set the pipeline to 
   */
  public void setRightCamPipeline(int pipeline) {
    rightCam.getEntry("pipeline").setNumber(pipeline);
  }

  /**
   * Gets pipeline index from the front camera
   * @return index of current pipeline
   */
  public int getFrontCamPipeline() {
    return (int) frontCam.getEntry("pipeline").getInteger(5);
  }

  /**
   * Gets pipeline index from the back camera
   * @return index of current pipeline
   */
  public int getBackCamPipeline() {
    return (int) backCam.getEntry("pipeline").getInteger(5);
  }

  /**
   * Gets pipeline index from the left camera
   * @return index of current pipeline
   */
  public int getLeftCamPipeline() {
    return (int) leftCam.getEntry("pipeline").getInteger(5);
  }

  /**
   * Gets pipeline index from the right camera
   * @return index of current pipeline
   */
  public int getRightCamPipeline() {
    return (int) rightCam.getEntry("pipeline").getInteger(5);
  }

  /**
   * Sets the IMU angle to 0
   */
  public void zeroPigeon() {
    setPigeonAngle(0.0);
  }

  /**
   * Sets the angle of the IMU
   * @param degrees - Angle to be set to the IMU
   */
  public void setPigeonAngle(double degrees) {
    pigeon.setYaw(degrees);
  }

  /**
   * Retrieves the yaw of the robot
   * @return Yaw in degrees
   */
  public double getPigeonAngle() {
    return pigeon.getYaw().getValueAsDouble();
  }

  public double getPigeonAngularVelocity() {
    return Math.abs(pigeon.getAngularVelocityZDevice().getValueAsDouble());
  }

  public double getPigeonAngularVelocityW() {
    return pigeon.getAngularVelocityZWorld().getValueAsDouble();
  }

  /**
   * Retrieves the acceleration vector of the robot
   * @return Current acceleration vector of the robot
   */
  public Vector getPigeonLinAccel() {
    Vector accelVector = new Vector();
    accelVector.setI(pigeon.getAccelerationX().getValueAsDouble() / Constants.Physical.GRAVITY_ACCEL_MS2);
    accelVector.setJ(pigeon.getAccelerationY().getValueAsDouble() / Constants.Physical.GRAVITY_ACCEL_MS2);
    return accelVector;
  }

  /**
   * Retrieves the horizontal distance from an april tag based on the front camera
   * @return Horizontal distance to april tag
   */
  public double getFrontHorizontalDistToTag() {
    double[] pose = this.frontCamRobotTagPose.getDoubleArray(new double[] { 0, 0, 1000, 0, 0, 0 });
    return -pose[2];
  }

  /**
   * Retrieves the horizontal distance from an april tag based on the left camera
   * @return Horizontal distance to april tag
   */
  public double getLeftHorizontalDistToTag() {
    double[] pose = this.leftCamRobotTagPose.getDoubleArray(new double[] { 0, 0, 1000, 0, 0, 0 });
    return -pose[2];
  }

  /**
   * Retrieves the horizontal distance from an april tag based on the right camera
   * @return Horizontal distance to april tag
   */
  public double getRightHorizontalDistToTag() {
    double[] pose = this.rightCamRobotTagPose.getDoubleArray(new double[] { 0, 0, 1000, 0, 0, 0 });
    return -pose[2];
  }

  /**
   * Retrieves the robot's position based on the front camera image.
   *
   * @return A JSONArray containing the robot's position data (X and Y coordinates), 
   *         or null if the data is unavailable.
   */
  public JSONArray getFrontCamBasedPosition() {
    JSONArray fieldPosArray = new JSONArray();
    double[] result = new double[7];
    double tagDist = 99999;
    JSONArray noTrack = new JSONArray();
    noTrack.put(0);
    noTrack.put(0);
    try {
      result = this.frontCamRobotFieldPose.getDoubleArray(noTrackLimelightArray);
      tagDist = getFrontHorizontalDistToTag();
    } catch (Exception e) {
      return noTrack;
    }
    if (tagDist > 2.25 || tagDist == 0) {
      return noTrack;
    }
    if (result[0] == 0 || result[1] == 0) {
      return noTrack;
    }
    double fieldX = result[0];
    double fieldY = result[1];
    fieldPosArray.put(0, fieldX);
    fieldPosArray.put(1, fieldY);
    // System.out.println("Back X: " + fieldX + " Y: " + fieldY + " Dist: " +
    // tagDist);
    return fieldPosArray;
  }

  /**
   * Retrieves the robot's position based on the raw front camera image.
   *
   * @return A JSONArray containing the robot's position data (X and Y coordinates), 
   *         or null if the data is unavailable.
   */
  public JSONArray getRawFrontCamBasedPosition() {
    JSONArray fieldPosArray = new JSONArray();
    double[] result = new double[7];
    JSONArray noTrack = new JSONArray();
    noTrack.put(0);
    noTrack.put(0);
    try {
      result = this.frontCamRobotFieldPose.getDoubleArray(noTrackLimelightArray);
    } catch (Exception e) {
      return noTrack;
    }
    if (result[0] == 0 || result[1] == 0) {
      return noTrack;
    }
    double fieldX = result[0];
    double fieldY = result[1];
    fieldPosArray.put(0, fieldX);
    fieldPosArray.put(1, fieldY);
    // System.out.println("Front X: " + fieldX + " Y: " + fieldY + " Dist: " +
    // tagDist);
    return fieldPosArray;
  }

  public JSONArray getLeftCamBasedPosition(){
    JSONArray fieldPosArray = new JSONArray();
    double[] result = new double[7];
    double tagDist = 99999;
    JSONArray noTrack = new JSONArray();
    noTrack.put(0);
    noTrack.put(0);
    try {
      result = this.leftCamRobotFieldPose.getDoubleArray(noTrackLimelightArray);
      tagDist = getFrontHorizontalDistToTag();
    } catch (Exception e) {
      return noTrack;
    }
    if (tagDist > 2.25 || tagDist == 0){
      return noTrack;
    }
    if (result[0] == 0 || result[1] == 0){
      return noTrack;
    }
    double fieldX = result[0];
    double fieldY = result[1];
    fieldPosArray.put(0, fieldX);
    fieldPosArray.put(1, fieldY);
    // System.out.println("Back X: " + fieldX + " Y: " + fieldY + " Dist: " + tagDist);
    return fieldPosArray;
  }

  /**
   * Retrieves the robot's position based on the raw left camera image.
   *
   * @return A JSONArray containing the robot's position data (X and Y coordinates), 
   *         or null if the data is unavailable.
   */
  public JSONArray getRawLeftCamBasedPosition() {
    JSONArray fieldPosArray = new JSONArray();
    double[] result = new double[7];
    JSONArray noTrack = new JSONArray();
    noTrack.put(0);
    noTrack.put(0);
    try {
      result = this.leftCamRobotFieldPose.getDoubleArray(noTrackLimelightArray);
    } catch (Exception e) {
      return noTrack;
    }
    if (result[0] == 0 || result[1] == 0) {
      return noTrack;
    }
    double fieldX = result[0];
    double fieldY = result[1];
    fieldPosArray.put(0, fieldX);
    fieldPosArray.put(1, fieldY);
    // System.out.println("Left X: " + fieldX + " Y: " + fieldY + " Dist: " +
    // tagDist);
    return fieldPosArray;
  }

  public JSONArray getRightCamBasedPosition(){
    JSONArray fieldPosArray = new JSONArray();
    double[] result = new double[7];
    double tagDist = 99999;
    JSONArray noTrack = new JSONArray();
    noTrack.put(0);
    noTrack.put(0);
    try {
      result = this.rightCamRobotFieldPose.getDoubleArray(noTrackLimelightArray);
      tagDist = getFrontHorizontalDistToTag();
    } catch (Exception e) {
      return noTrack;
    }
    if (tagDist > 2.25 || tagDist == 0){
      return noTrack;
    }
    if (result[0] == 0 || result[1] == 0){
      return noTrack;
    }
    double fieldX = result[0];
    double fieldY = result[1];
    fieldPosArray.put(0, fieldX);
    fieldPosArray.put(1, fieldY);
    // System.out.println("Back X: " + fieldX + " Y: " + fieldY + " Dist: " + tagDist);
    return fieldPosArray;
  }


  /**
   * Retrieves the robot's position based on the raw right camera image.
   *
   * @return A JSONArray containing the robot's position data (X and Y coordinates), 
   *         or null if the data is unavailable.
   */
  public JSONArray getRawRightCamBasedPosition() {
    JSONArray fieldPosArray = new JSONArray();
    double[] result = new double[7];
    JSONArray noTrack = new JSONArray();
    noTrack.put(0);
    noTrack.put(0);
    try {
      result = this.rightCamRobotFieldPose.getDoubleArray(noTrackLimelightArray);
    } catch (Exception e) {
      return noTrack;
    }
    if (result[0] == 0 || result[1] == 0) {
      return noTrack;
    }
    double fieldX = result[0];
    double fieldY = result[1];
    fieldPosArray.put(0, fieldX);
    fieldPosArray.put(1, fieldY);
    // System.out.println("Right X: " + fieldX + " Y: " + fieldY + " Dist: " +
    // tagDist);
    return fieldPosArray;
  }

  /**
   * Gathers camera measurements and combines them into a single JSONObject.
   * 
   * @return A JSONObject containing the combined camera measurements.
   */
  public JSONObject getCameraMeasurements() {
    JSONObject allCamResults = new JSONObject();
    JSONObject backCamResults = new JSONObject(this.backCamJSON.getString("{'Results': {}}")).getJSONObject("Results");
    JSONObject frontCamResults = new JSONObject(this.frontCamJSON.getString("{'Results': {}}"))
        .getJSONObject("Results");
    JSONObject leftCamResults = new JSONObject(this.leftCamJSON.getString("{'Results': {}}")).getJSONObject("Results");
    JSONObject rightCamResults = new JSONObject(this.rightCamJSON.getString("{'Results': {}}"))
        .getJSONObject("Results");
    if (!backCamResults.isNull("Fiducial")) {
      allCamResults.put("BackCam", backCamResults);
    }
    if (!frontCamResults.isNull("Fiducial")) {
      allCamResults.put("FrontCam", frontCamResults);
    }
    if (!leftCamResults.isNull("Fiducial")) {
      allCamResults.put("LeftCam", leftCamResults);
    }
    if (!rightCamResults.isNull("Fiducial")) {
      allCamResults.put("RightCam", rightCamResults);
    }
    return allCamResults;
  }

  @Override
  public void periodic() {
    // if (OI.isBlueSide()) {
    //   setFieldSide("blue");
    // } else {
    //   setFieldSide("red");
    // }
    // Logger.recordOutput("Camera Measurements", getCameraMeasurements().toString());
    // Logger.recordOutput("Pigeon Angle", getPigeonAngle());
    // Vector accelVector = getPigeonLinAccel();
    // Logger.recordOutput("Pigeon Acceleration Vector Magnitude", accelVector.magnitude());
    // Logger.recordOutput("Pigeon Acceleration Vector Magnitude", Math.atan2(accelVector.getI(), accelVector.getJ()));

    // ConnectionInfo[] info = NetworkTableInstance.getDefault().getConnections();
    // for (ConnectionInfo i : info){
    // System.out.println(i.remote_ip);
    // System.out.println(i.remote_id);
    // }
    SmartDashboard.putNumber("ty", getFrontCamTargetTy());
    SmartDashboard.putNumber("ty direct", frontCamTy.getDouble(0));
    SmartDashboard.putNumber("device", getPigeonAngularVelocity());
    SmartDashboard.putNumber("world", getPigeonAngularVelocityW());
    SmartDashboard.putNumber("yaw", getPigeonAngle());
  }
}
