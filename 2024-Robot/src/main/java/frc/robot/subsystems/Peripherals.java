package frc.robot.subsystems;

import java.util.ArrayList;

import org.json.JSONArray;
import org.json.JSONObject;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.commands.defaults.PeripheralsDefault;
import frc.robot.tools.math.Vector;

public class Peripherals extends SubsystemBase {
  private NetworkTable backCam = NetworkTableInstance.getDefault().getTable("limelight-back");
  private NetworkTableEntry backCamJSON = backCam.getEntry("json");
  private NetworkTableEntry backCamTx = backCam.getEntry("tx");
  private NetworkTable frontCam = NetworkTableInstance.getDefault().getTable("limelight-front");
  private NetworkTableEntry frontCamJSON = frontCam.getEntry("json");
  private NetworkTableEntry frontCamTy = frontCam.getEntry("ty");
  private NetworkTableEntry frontCamTx = frontCam.getEntry("tx");
  private NetworkTableEntry frontCamTl = frontCam.getEntry("tl");
  private NetworkTableEntry frontCamCl = frontCam.getEntry("cl");
  private NetworkTableEntry frontCamRobotTagPose = frontCam.getEntry("botpose_targetspace");
  private NetworkTableEntry frontCamRobotFieldPose = frontCam.getEntry("botpose_wpiblue");
  private NetworkTable leftCam = NetworkTableInstance.getDefault().getTable("limelight-left");
  private NetworkTableEntry leftCamJSON = leftCam.getEntry("json");
  private NetworkTableEntry leftCamRobotFieldPose = leftCam.getEntry("botpose_wpiblue");
  private NetworkTableEntry leftCamRobotTagPose = leftCam.getEntry("botpose_targetspace");
  private NetworkTable rightCam = NetworkTableInstance.getDefault().getTable("limelight-right");
  private NetworkTableEntry rightCamJSON = rightCam.getEntry("json");
  private NetworkTableEntry rightCamRobotFieldPose = rightCam.getEntry("botpose_wpiblue");
  private NetworkTableEntry rightCamRobotTagPose = rightCam.getEntry("botpose_targetspace");

  private double[] noTrackLimelightArray = new double[6];

  private Pigeon2 pigeon = new Pigeon2(0, "Canivore");
  private Pigeon2Configuration pigeonConfig = new Pigeon2Configuration();
  
  public Peripherals() {}

  public void init() {
    pigeonConfig.MountPose.MountPosePitch = -85.28813934326172;
    pigeonConfig.MountPose.MountPoseRoll = 32.49883270263672;
    pigeonConfig.MountPose.MountPoseYaw = 0.1901332437992096;
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

  public double getFrontCamTargetTy(){
    JSONObject results = new JSONObject(this.frontCamJSON.getString("{Results: {}}")).getJSONObject("Results");
    if (results.isNull("Fiducial")){
      return 100;
    }
    JSONArray fiducials = results.getJSONArray("Fiducial");
    for (int i = 0; i < fiducials.length(); i ++){
      int id = ((JSONObject) fiducials.get(i)).getInt("fID");
      if (id == 7 || id == 4){
        return ((JSONObject) fiducials.get(i)).getDouble("ty");
      }
    }
    return 100;
  }

  public double getFrontCamTargetTx(){
    JSONObject results = new JSONObject(this.frontCamJSON.getString("{Results: {}}")).getJSONObject("Results");
    if (results.isNull("Fiducial")){
      return 100;
    }
    JSONArray fiducials = results.getJSONArray("Fiducial");
    for (int i = 0; i < fiducials.length(); i ++){
      int id = ((JSONObject) fiducials.get(i)).getInt("fID");
      if (id == 7 || id == 4){
        return ((JSONObject) fiducials.get(i)).getDouble("tx");
      }
    }
    return 100;
  }

  public ArrayList<Integer> getFrontCamIDs(){
    JSONObject results = new JSONObject(this.frontCamJSON.getString("{Results: {}}")).getJSONObject("Results");
    if (results.isNull("Fiducial")){
      return new ArrayList<Integer>();
    }
    ArrayList<Integer> ids = new ArrayList<Integer>();
    JSONArray fiducials = results.getJSONArray("Fiducial");
    for (int i = 0; i < fiducials.length(); i ++){
      ids.add(((JSONObject) fiducials.get(i)).getInt("fID"));
    }
    return ids;
  }

  public JSONObject getFrontCamLatencies(){
    JSONObject latencies = new JSONObject();
    latencies.put("tl", this.frontCamTl.getDouble(0) / 1000);
    latencies.put("cl", this.frontCamCl.getDouble(0) / 1000);
    return latencies;
  }

  public double getBackCamTargetTx(){
    return backCamTx.getDouble(0.0) * Math.PI / 180;
  }

  public void setFrontCamPipeline(int pipeline){
    frontCam.getEntry("pipeline").setNumber(pipeline);
  }

  public void setBackCamPipeline(int pipeline){
    backCam.getEntry("pipeline").setNumber(pipeline);
  }

  public void setLeftCamPipeline(int pipeline){
    leftCam.getEntry("pipeline").setNumber(pipeline);
  }

  public void setRightCamPipeline(int pipeline){
    rightCam.getEntry("pipeline").setNumber(pipeline);
  }

  public int getFrontCamPipeline(){
    return (int) frontCam.getEntry("pipeline").getInteger(5);
  }

  public int getBackCamPipeline(){
    return (int) backCam.getEntry("pipeline").getInteger(5);
  }

  public int getLeftCamPipeline(){
    return (int) leftCam.getEntry("pipeline").getInteger(5);
  }

  public int getRightCamPipeline(){
    return (int) rightCam.getEntry("pipeline").getInteger(5);
  }

  public void zeroPigeon(){
    setPigeonAngle(0.0);
  }

  public void setPigeonAngle(double degrees){
    pigeon.setYaw(degrees);
  }

  public double getPigeonAngle(){
    return pigeon.getYaw().getValueAsDouble();
  }

  public Vector getPigeonLinAccel(){
    Vector accelVector = new Vector();
    accelVector.setI(pigeon.getAccelerationX().getValueAsDouble() / Constants.Physical.GRAVITY_ACCEL_MS2);
    accelVector.setJ(pigeon.getAccelerationY().getValueAsDouble() / Constants.Physical.GRAVITY_ACCEL_MS2);
    return accelVector;
  }

  public double getFrontHorizontalDistToTag(){
    double[] pose = this.frontCamRobotTagPose.getDoubleArray(new double[] {0, 0, 1000, 0, 0, 0});
    return -pose[2];
  }

  public double getLeftHorizontalDistToTag(){
    double[] pose = this.leftCamRobotTagPose.getDoubleArray(new double[] {0, 0, 1000, 0, 0, 0});
    return -pose[2];
  }

  public double getRightHorizontalDistToTag(){
    double[] pose = this.rightCamRobotTagPose.getDoubleArray(new double[] {0, 0, 1000, 0, 0, 0});
    return -pose[2];
  }

  public JSONArray getFrontCamBasedPosition(){
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

  public JSONArray getRawFrontCamBasedPosition(){
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
    if (result[0] == 0 || result[1] == 0){
      return noTrack;
    }
    double fieldX = result[0];
    double fieldY = result[1];
    fieldPosArray.put(0, fieldX);
    fieldPosArray.put(1, fieldY);
    // System.out.println("Front X: " + fieldX + " Y: " + fieldY + " Dist: " + tagDist);
    return fieldPosArray;
  }

  public JSONArray getRawLeftCamBasedPosition(){
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
    if (result[0] == 0 || result[1] == 0){
      return noTrack;
    }
    double fieldX = result[0];
    double fieldY = result[1];
    fieldPosArray.put(0, fieldX);
    fieldPosArray.put(1, fieldY);
    // System.out.println("Left X: " + fieldX + " Y: " + fieldY + " Dist: " + tagDist);
    return fieldPosArray;
  }

  public JSONArray getRawRightCamBasedPosition(){
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
    if (result[0] == 0 || result[1] == 0){
      return noTrack;
    }
    double fieldX = result[0];
    double fieldY = result[1];
    fieldPosArray.put(0, fieldX);
    fieldPosArray.put(1, fieldY);
    // System.out.println("Right X: " + fieldX + " Y: " + fieldY + " Dist: " + tagDist);
    return fieldPosArray;
  }

  public JSONObject getCameraMeasurements(){
    JSONObject allCamResults = new JSONObject();
    JSONObject backCamResults = new JSONObject(this.backCamJSON.getString("{'Results': {}}")).getJSONObject("Results");
    JSONObject frontCamResults = new JSONObject(this.frontCamJSON.getString("{'Results': {}}")).getJSONObject("Results");
    JSONObject leftCamResults = new JSONObject(this.leftCamJSON.getString("{'Results': {}}")).getJSONObject("Results");
    JSONObject rightCamResults = new JSONObject(this.rightCamJSON.getString("{'Results': {}}")).getJSONObject("Results");
    if (!backCamResults.isNull("Fiducial")){
      allCamResults.put("BackCam", backCamResults);
    }
    if (!frontCamResults.isNull("Fiducial")){
      allCamResults.put("FrontCam", frontCamResults);
    }
    if (!leftCamResults.isNull("Fiducial")){
      allCamResults.put("LeftCam", leftCamResults);
    }
    if (!rightCamResults.isNull("Fiducial")){
      allCamResults.put("RightCam", rightCamResults);
    }
    return allCamResults;
  }

  @Override
  public void periodic() {
    // ConnectionInfo[] info = NetworkTableInstance.getDefault().getConnections();
    // for (ConnectionInfo i : info){
    //   System.out.println(i.remote_ip);
    //   System.out.println(i.remote_id);
    // }
  }
}
