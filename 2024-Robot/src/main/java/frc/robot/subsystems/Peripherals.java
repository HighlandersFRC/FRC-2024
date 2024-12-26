package frc.robot.subsystems;

import java.net.InetAddress;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.json.JSONArray;
import org.json.JSONObject;
import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
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
  // private NetworkTableEntry backCamTrack = backCam.getEntry("tv");
  private NetworkTable frontCam = NetworkTableInstance.getDefault().getTable("limelight-front");
  private NetworkTableEntry frontCamJSON = frontCam.getEntry("json");
  private NetworkTableEntry frontCamTy = frontCam.getEntry("ty");
  private NetworkTableEntry frontCamTx = frontCam.getEntry("tx");
  private NetworkTableEntry frontCamTl = frontCam.getEntry("tl");
  private NetworkTableEntry frontCamCl = frontCam.getEntry("cl");
  private NetworkTableEntry frontCamHB = frontCam.getEntry("hb");
  // private NetworkTableEntry frontCamIDSet = frontCam.getEntry("priorityid");
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
  private PhotonCamera photonCamera = new PhotonCamera("9281_Front");

  private double[] noTrackLimelightArray = new double[6];

  AprilTagFieldLayout aprilTagFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();

  private Pigeon2 pigeon = new Pigeon2(0, "Canivore");
  private Pigeon2Configuration pigeonConfig = new Pigeon2Configuration();
  Transform3d robotToCam = new Transform3d(new Translation3d(0.0, 0.0, 0.5), new Rotation3d(0, 0, 0));

  PhotonPoseEstimator photonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout,
      PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, photonCamera, robotToCam);

  public Peripherals() {
  }

  public double[] getEstimatedGlobalPose() {
    Transform3d ret = new Transform3d(0, 0, 0, new Rotation3d(0, 0, 0));
    var result = photonCamera.getLatestResult();
    if (result.getMultiTagResult().estimatedPose.isPresent) {
      ret = result.getMultiTagResult().estimatedPose.best;
    }
    double[] pose = { ret.getX(), ret.getY(), getPigeonAngle() };
    return pose;
  }

  /**
   * Checks the connectivity of Limelight devices.
   * 
   * @return true if both Limelights are reachable, false otherwise.
   */
  public boolean limelightsConnected() {
    boolean reachable = true;
    try {
      InetAddress address4 = InetAddress.getByName("10.44.99.44"); // front limelight ip address
      if (!address4.isReachable(100)) {
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
   * Initializes the Peripherals subsystem.
   * 
   * This method sets up the IMU configuration, mount pose, and zeroes the IMU.
   * It also applies the default command to the Peripherals subsystem.
   */
  public void init() {
    // Set the mount pose configuration for the IMU
    pigeonConfig.MountPose.MountPosePitch = -85.28813934326172;
    pigeonConfig.MountPose.MountPoseRoll = 32.49883270263672;
    pigeonConfig.MountPose.MountPoseYaw = 0.1901332437992096;

    // Apply the IMU configuration
    pigeon.getConfigurator().apply(pigeonConfig);

    // Zero the IMU angle
    zeroPigeon();

    // Initialize the array for no track data
    noTrackLimelightArray[0] = 0;
    noTrackLimelightArray[1] = 0;
    noTrackLimelightArray[2] = 0;
    noTrackLimelightArray[3] = 0;
    noTrackLimelightArray[4] = 0;
    noTrackLimelightArray[5] = 0;

    // Set the default command for the Peripherals subsystem
    setDefaultCommand(new PeripheralsDefault(this));
  }

  public double getPhotonYaw() {
    double yaw = 0.0;
    var result = photonCamera.getLatestResult();
    Logger.recordOutput("has target", result.hasTargets());
    if (result.hasTargets()) {
      List<PhotonTrackedTarget> targets = result.getTargets();
      PhotonTrackedTarget target = result.getBestTarget();
      yaw = target.getYaw();
    }
    return yaw;
  }

  public double getPhotonPitch() {
    double pitch = 0.0;
    var result = photonCamera.getLatestResult();
    if (result.hasTargets()) {

      List<PhotonTrackedTarget> targets = result.getTargets();
      PhotonTrackedTarget target = result.getBestTarget();
      pitch = target.getPitch();
    }

    return pitch;
  }

  /**
   * Retrieves the latency of the front camera.
   * 
   * @return The latency of the front camera in seconds.
   */
  public double getPhotonFrontLatency() {
    return photonCamera.getLatestResult().getLatencyMillis() / 1000.0;
  }

  public Pose2d getRobotPoseViaTrig(PhotonTrackedTarget trackedTarget) {
    double pitch = trackedTarget.getPitch();
    double yaw = trackedTarget.getYaw();
    int id = trackedTarget.getFiducialId();
    double cameraPitch = 0;
    double cameraHeight = Constants.inchesToMeters(30.68);
    double[] tagPose = Constants.Vision.TAG_POSES[id - 1];
    double tagHeight = tagPose[2];
    double tagX = tagPose[0];
    double tagY = tagPose[1];

    double distToTag = (tagHeight - cameraHeight) / Math.tan(Math.toRadians(pitch + cameraPitch));
    Logger.recordOutput("Distance to Tag", distToTag);
    Logger.recordOutput("yaw to Tag", yaw);
    // double txProjOntoGroundPlane = Math.atan((Math.tan(yaw)) / Math.cos(pitch));
    double xFromTag = distToTag * Math.cos(Math.toRadians(yaw));
    double yFromTag = distToTag * Math.sin(Math.toRadians(yaw));
    Logger.recordOutput("x to Tag", xFromTag);
    Logger.recordOutput("y to Tag", yFromTag);

    double fieldPoseX = -xFromTag + tagX;
    double fieldPoseY = yFromTag + tagY;
    Pose2d pose = new Pose2d(fieldPoseX, fieldPoseY, new Rotation2d(getPigeonAngle()));
    return pose;
  }

  /**
   * Checks if the back camera is tracking a target.
   *
   * @return {@code true} if the back camera is tracking a target, {@code false}
   *         otherwise.
   */
  public boolean getBackCamTrack() {
    if (backCam.getEntry("tv").getInteger(0) == 1) {
      return true;
    } else
      return false;
  }

  /**
   * Retrieves the Y-Axis Rotation of the robot based on the front camera
   * 
   * @return Y-Axis rotation in radians
   */
  public double getFrontCamTargetTy() {
    return frontCamTy.getDouble(100);
  }

  /**
   * Retrieves the X-Axis Rotation of the robot based on the front camera
   * 
   * @return X-Axis rotation in radians
   */
  public double getFrontCamTargetTx() {
    return frontCamTx.getDouble(100);
  }

  /**
   * Retrieves the horizontal distance from an april tag based on the front
   * camera.
   *
   * @return The horizontal distance to the april tag in meters.
   *         If no target is detected or the distance is not available, the
   *         function returns -1.
   */
  public double getFrontCamHB() {
    return frontCamHB.getDouble(-1);
  }

  /**
   * Retrieves all fiducial IDs of the front camera
   * 
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

  /**
   * Retrieves the ID of the target detected by the front camera.
   * 
   * @return The ID of the target detected by the front camera.
   *         The ID is a double value. If no target is detected, the function
   *         returns 0.
   */
  public double getFrontCamID() {
    return NetworkTableInstance.getDefault().getTable("limelight-front").getEntry("tid").getDouble(0);
  }

  /**
   * Retrieves the latencies of the front camera
   * 
   * @return JSONObject {"tl": target latency, "cl": camera latency}
   */
  public JSONObject getFrontCamLatencies() {
    JSONObject latencies = new JSONObject();
    latencies.put("tl", this.frontCamTl.getDouble(0) / 1000);
    latencies.put("cl", this.frontCamCl.getDouble(0) / 1000);
    return latencies;
  }

  /**
   * Retrieves the latency of the front camera.
   * 
   * @return The latency of the front camera in milliseconds.
   *         The latency is calculated as the sum of the target latency (tl) and
   *         the camera latency (cl).
   *         If either of these values is not available, the function returns -1.
   */
  public double getFrontCameraLatency() {
    double latency = frontCamTl.getDouble(-1) + frontCamCl.getDouble(-1);
    return latency;
  }

  /**
   * Retrieves the latency of the left camera.
   * 
   * @return The latency of the left camera in milliseconds.
   *         The latency is calculated as the sum of the target latency (tl) and
   *         the camera latency (cl).
   *         If either of these values is not available, the function returns -1.
   */
  public double getLeftCameraLatency() {
    double latency = leftCamTl.getDouble(-1) + leftCamCl.getDouble(-1);
    return latency;
  }

  /**
   * Retrieves the latency of the right camera.
   * 
   * @return The latency of the right camera in milliseconds.
   *         The latency is calculated as the sum of the target latency (tl) and
   *         the camera latency (cl).
   *         If either of these values is not available, the function returns -1.
   */
  public double getRightCameraLatency() {
    double latency = rightCamTl.getDouble(-1) + rightCamCl.getDouble(-1);
    return latency;
  }

  /**
   * Retrieves the X-Axis Rotation of the robot based on the back camera
   * 
   * @return X-Axis rotation in radians
   */
  public double getBackCamTargetTx() {
    return backCamTx.getDouble(0.0);
  }

  /**
   * Retrieves the vertical distance from an april tag based on the back camera.
   * 
   * @return The vertical distance to the april tag in meters.
   *         If no target is detected, the function returns 100 meters.
   */
  public double getBackCamTargetTy() {
    return backCamTy.getDouble(100);
  }

  /**
   * Retrieves the confidence of the target detected by the back camera.
   * 
   * @return The confidence of the target detected by the back camera.
   *         The confidence value is a double between 0.0 and 1.0, inclusive.
   *         A value of 1.0 indicates a high confidence in the detection, while a
   *         value of 0.0 indicates a low confidence.
   *         If no target is detected, the function returns 0.0.
   */
  public double getBackCamTargetConfidence() {
    return backCamConfidence.getDouble(0.0);
  }

  /**
   * Sets the pipeline of the front camera
   * 
   * @param pipeline - index to set the pipeline to
   */
  public void setFrontCamPipeline(int pipeline) {
    frontCam.getEntry("pipeline").setNumber(pipeline);
  }

  /**
   * Sets the pipeline of the back camera
   * 
   * @param pipeline - index to set the pipeline to
   */
  public void setBackCamPipeline(int pipeline) {
    backCam.getEntry("pipeline").setNumber(pipeline);
  }

  /**
   * Sets the pipeline of the left camera
   * 
   * @param pipeline - index to set the pipeline to
   */
  public void setLeftCamPipeline(int pipeline) {
    leftCam.getEntry("pipeline").setNumber(pipeline);
  }

  /**
   * Sets the pipeline of the right camera
   * 
   * @param pipeline - index to set the pipeline to
   */
  public void setRightCamPipeline(int pipeline) {
    rightCam.getEntry("pipeline").setNumber(pipeline);
  }

  /**
   * Gets pipeline index from the front camera
   * 
   * @return index of current pipeline
   */
  public int getFrontCamPipeline() {
    return (int) frontCam.getEntry("pipeline").getInteger(5);
  }

  /**
   * Gets pipeline index from the back camera
   * 
   * @return index of current pipeline
   */
  public int getBackCamPipeline() {
    return (int) backCam.getEntry("pipeline").getInteger(5);
  }

  /**
   * Gets pipeline index from the left camera
   * 
   * @return index of current pipeline
   */
  public int getLeftCamPipeline() {
    return (int) leftCam.getEntry("pipeline").getInteger(5);
  }

  /**
   * Gets pipeline index from the right camera
   * 
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
   * 
   * @param degrees - Angle to be set to the IMU
   */
  public void setPigeonAngle(double degrees) {
    pigeon.setYaw(degrees);
  }

  /**
   * Retrieves the yaw of the robot
   * 
   * @return Yaw in degrees
   */
  public double getPigeonAngle() {
    return pigeon.getYaw().getValueAsDouble();
  }

  /**
   * Retrieves the absolute angular velocity of the IMU's Z-axis in device
   * coordinates.
   *
   * @return The absolute angular velocity of the IMU's Z-axis in device
   *         coordinates.
   *         The value is in radians per second.
   */
  public double getPigeonAngularVelocity() {
    return Math.abs(pigeon.getAngularVelocityZDevice().getValueAsDouble());
  }

  /**
   * Retrieves the absolute angular velocity of the IMU's Z-axis in world
   * coordinates.
   *
   * @return The absolute angular velocity of the IMU's Z-axis in world
   *         coordinates.
   *         The value is in radians per second.
   */
  public double getPigeonAngularVelocityW() {
    return pigeon.getAngularVelocityZWorld().getValueAsDouble();
  }

  /**
   * Retrieves the acceleration vector of the robot
   * 
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
   * 
   * @return Horizontal distance to april tag
   */
  public double getFrontHorizontalDistToTag() {
    double[] pose = this.frontCamRobotTagPose.getDoubleArray(new double[] { 0, 0, 1000, 0, 0, 0 });
    return -pose[2];
  }

  /**
   * Retrieves the horizontal distance from an april tag based on the left camera
   * 
   * @return Horizontal distance to april tag
   */
  public double getLeftHorizontalDistToTag() {
    double[] pose = this.leftCamRobotTagPose.getDoubleArray(new double[] { 0, 0, 1000, 0, 0, 0 });
    return -pose[2];
  }

  /**
   * Retrieves the horizontal distance from an april tag based on the right camera
   * 
   * @return Horizontal distance to april tag
   */
  public double getRightHorizontalDistToTag() {
    double[] pose = this.rightCamRobotTagPose.getDoubleArray(new double[] { 0, 0, 1000, 0, 0, 0 });
    return -pose[2];
  }

  /**
   * Retrieves the robot's position based on the front camera image.
   *
   * @return A JSONArray containing the robot's position data (X and Y
   *         coordinates),
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
   * @return A JSONArray containing the robot's position data (X and Y
   *         coordinates),
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

  public JSONArray getLeftCamBasedPosition() {
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
   * Retrieves the robot's position based on the raw left camera image.
   *
   * @return A JSONArray containing the robot's position data (X and Y
   *         coordinates),
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

  /**
   * Retrieves the robot's position based on the raw right camera image.
   *
   * @return A JSONArray containing the robot's position data (X and Y
   *         coordinates),
   *         or null if the data is unavailable.
   *         The X and Y coordinates represent the position of the robot in the
   *         field.
   */
  public JSONArray getRightCamBasedPosition() {
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
    // System.out.println("Right X: " + fieldX + " Y: " + fieldY + " Dist: " +
    // tagDist);
    return fieldPosArray;
  }

  /**
   * Retrieves the robot's position based on the raw right camera image.
   *
   * @return A JSONArray containing the robot's position data (X and Y
   *         coordinates),
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
    Logger.recordOutput("MultiTag", getEstimatedGlobalPose());
    var result = photonCamera.getLatestResult();
    if (result.hasTargets()) {
      PhotonTrackedTarget target = result.getBestTarget();
      Pose2d robotPose = getRobotPoseViaTrig(target);
      Logger.recordOutput("Trig Localiazation", robotPose);
    }
  }
}
