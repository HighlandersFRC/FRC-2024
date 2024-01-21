package frc.robot.subsystems;

import org.json.JSONArray;
import org.json.JSONObject;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.kauailabs.navx.frc.AHRS;

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

public class Peripherals extends SubsystemBase {
  private final static AHRS ahrs = new AHRS(Port.kMXP);

  private final static Navx navx = new Navx(ahrs);

  private Lights lights;

  private NetworkTable backCam = NetworkTableInstance.getDefault().getTable("limelight-back");
  private NetworkTableEntry backCamAngleHorizontal = backCam.getEntry("tx");
  private NetworkTableEntry backCamAngleVertical = backCam.getEntry("ty");
  private NetworkTableEntry backCamTableLatency = backCam.getEntry("tl");
  private NetworkTableEntry backCamCameraLatency = backCam.getEntry("cl");
  private NetworkTableEntry backCamArea = backCam.getEntry("ta");
  private NetworkTableEntry backCamRobotPose = backCam.getEntry("botpose");
  private NetworkTableEntry backCamAprilTagPose = backCam.getEntry("targetpose_cameraspace");
  private NetworkTableEntry backCamJSON = backCam.getEntry("json");
  private NetworkTableEntry backCamTagToRobotPose = backCam.getEntry("botpose_targetspace");

  private NetworkTable frontCam = NetworkTableInstance.getDefault().getTable("limelight-front");
  private NetworkTableEntry frontCamAngleHorizontal = frontCam.getEntry("tx");
  private NetworkTableEntry frontCamAngleVertical = frontCam.getEntry("ty");
  private NetworkTableEntry fromCamTableLatency = frontCam.getEntry("tl");
  private NetworkTableEntry frontCamCameraLatency = frontCam.getEntry("cl");
  private NetworkTableEntry frontCamArea = frontCam.getEntry("ta");
  private NetworkTableEntry frontCamRobotPose = frontCam.getEntry("botpose");
  private NetworkTableEntry frontCamTagPose = frontCam.getEntry("targetpose_cameraspace");
  private NetworkTableEntry frontCamJSON = frontCam.getEntry("json");
  private NetworkTableEntry frontCamTagToRobotPose = frontCam.getEntry("botpose_targetspace");

  private double[] noTrackLimelightArray = new double[6];

  private Pigeon2 pigeon = new Pigeon2(0, "Canivore");
  private Pigeon2Configuration pigeonConfig = new Pigeon2Configuration();
  
  public Peripherals(Lights lights) {
   this.lights = lights;
  }

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

  public void setFrontPipeline(int pipeline){
    frontCam.getEntry("pipeline").setNumber(pipeline);
  }

  public void setBackPipeline(int pipeline){
    backCam.getEntry("pipeline").setNumber(pipeline);
  }

  public int getFrontLimelightPipeline(){
    return (int) frontCam.getEntry("pipeline").getInteger(5);
  }

  public int getBackLimelightPipeline(){
    return (int) backCam.getEntry("pipeline").getInteger(5);
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
    JSONObject backCamResults = new JSONObject(backCamJSON.getString("{}")).getJSONObject("Results");
    JSONObject frontCamResults = new JSONObject(frontCamJSON.getString("{}")).getJSONObject("Results");
    allCamResults.put("BackCam", backCamResults);
    allCamResults.put("FrontCam", frontCamResults);
    return allCamResults;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
