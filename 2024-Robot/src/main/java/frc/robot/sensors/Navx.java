package frc.robot.sensors;

import com.kauailabs.navx.frc.AHRS;


public class Navx {
  private double m_originalAngle;
  private double m_originalYaw;
  private AHRS imu;
  /** Creates a new Navx. */
 
    public Navx(AHRS navx) {
      imu = navx;
      m_originalAngle = imu.getAngle();
      m_originalYaw = imu.getYaw();
  }

  public Navx(AHRS navx, Double startAngle) {
      imu = navx;
      m_originalAngle = startAngle;
      m_originalYaw = imu.getYaw();
  }

  public double currentAngle() {
        return -(imu.getAngle() - m_originalAngle);
  }

  public double getRawAngle() {
      return -(imu.getAngle());
  }

  public double getRawPitch() {
      return imu.getPitch();
  }

  public double getRawYaw() {
      return imu.getYaw();
  }

  public double getRawRoll() {
      return imu.getRoll();
  }

  public double currentPitch() {
      return imu.getPitch();
  }

  public double currentRoll() {
      return imu.getRoll();
  }

  public double currentYaw() {
      return -((imu.getYaw()) - m_originalYaw);
  }

  public boolean isMoving() {
      return imu.isMoving();
  }
    
  public double currentAccelerometerX() {
      return imu.getWorldLinearAccelX();
  }

  public double currentAccelerometerY() {
      return imu.getWorldLinearAccelY();
  }

  public double currentAccelerometerZ() {
      return imu.getWorldLinearAccelZ();
  }

  public boolean isOn() {
      return imu.isConnected();
  }

  public boolean isMagCalibrated() {
      return imu.isMagnetometerCalibrated();
  }

  public boolean isAutoCalibrating() {
      return imu.isCalibrating();
  }

  public boolean isMagInerference() {
      return imu.isMagneticDisturbance();
  }

  public void softResetAngle() {
      m_originalAngle = imu.getAngle();
  }

  public void setNavxAngle(double angle) {
      m_originalAngle = m_originalAngle + angle;
  }

  public void softResetYaw() {
      m_originalYaw = imu.getYaw();
  }

  public double getAngleRate() {
      return (imu.getRate());
  }
  
}