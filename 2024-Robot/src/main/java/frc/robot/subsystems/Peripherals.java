package frc.robot.subsystems;

import org.json.JSONArray;
import org.json.JSONObject;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.networktables.ConnectionInfo;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.commands.defaults.PeripheralsDefault;
import frc.robot.sensors.Navx;
import frc.robot.sensors.TOF;

public class Peripherals extends SubsystemBase {
  private NetworkTable backCam = NetworkTableInstance.getDefault().getTable("limelight-back");
  private NetworkTableEntry backCamJSON = backCam.getEntry("json");
  private NetworkTable frontCam = NetworkTableInstance.getDefault().getTable("limelight-front");
  private NetworkTableEntry frontCamJSON = frontCam.getEntry("json");
  private NetworkTable leftCam = NetworkTableInstance.getDefault().getTable("limelight-left");
  private NetworkTableEntry leftCamJSON = leftCam.getEntry("json");
  private NetworkTable rightCam = NetworkTableInstance.getDefault().getTable("limelight-right");
  private NetworkTableEntry rightCamJSON = rightCam.getEntry("json");

  private double[] noTrackLimelightArray = new double[6];

  private Pigeon2 pigeon = new Pigeon2(0, "Canivore");
  private Pigeon2Configuration pigeonConfig = new Pigeon2Configuration();
  
  public Peripherals() {}

  public void init() {
    pigeonConfig.MountPose.MountPosePitch = 85.25395965576172;
    pigeonConfig.MountPose.MountPoseRoll = 126.1924057006836;
    pigeonConfig.MountPose.MountPoseYaw = -0.5872021913528442;
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

  public JSONObject getCameraMeasurements(){
    JSONObject allCamResults = new JSONObject();
    JSONObject backCamResults = new JSONObject(backCamJSON.getString("{'Results': {}}")).getJSONObject("Results");
    JSONObject frontCamResults = new JSONObject(frontCamJSON.getString("{'Results': {}}")).getJSONObject("Results");
    JSONObject leftCamResults = new JSONObject(leftCamJSON.getString("{'Results': {}}")).getJSONObject("Results");
    JSONObject rightCamResults = new JSONObject(rightCamJSON.getString("{'Results': {}}")).getJSONObject("Results");
    allCamResults.put("BackCam", backCamResults);
    allCamResults.put("FrontCam", frontCamResults);
    allCamResults.put("LeftCam", leftCamResults);
    allCamResults.put("RightCam", rightCamResults);
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
