package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.commands.defaults.IntakeDefault;

public class Intake extends SubsystemBase {
  private Lights lights;

  private final TalonFX angleFalcon = new TalonFX(Constants.CANInfo.INTAKE_ANGLE_MOTOR_ID, Constants.CANInfo.CANBUS_NAME);
  private final TalonFXConfiguration angleFalconConfiguration = new TalonFXConfiguration();
  private final PositionTorqueCurrentFOC anglefalconPositionRequest = new PositionTorqueCurrentFOC(0, 0, 0, 0, false, false, false);

  private final TalonFX rollerFalcon = new TalonFX(Constants.CANInfo.INTAKE_ROLLER_MOTOR_ID, Constants.CANInfo.CANBUS_NAME);
  private final TalonFXConfiguration rollerFalconConfiguration = new TalonFXConfiguration();
  private final VelocityTorqueCurrentFOC rollerFalconVelocityRequest = new VelocityTorqueCurrentFOC(0, 0, 0, 0, false, false, false);

  public Intake(Lights lights) {
    this.lights = lights;
    setDefaultCommand(new IntakeDefault(this));
  }

  public void init(){
    this.angleFalconConfiguration.Slot0.kP = 0.1;
    this.angleFalconConfiguration.Slot0.kI = 0;
    this.angleFalconConfiguration.Slot0.kD = 0;
    this.angleFalcon.getConfigurator().apply(this.angleFalconConfiguration);
    this.angleFalcon.setNeutralMode(NeutralModeValue.Brake);
    this.angleFalcon.setPosition(0);

    this.rollerFalconConfiguration.Slot0.kP = 0.1;    
    this.rollerFalconConfiguration.Slot0.kI = 0;
    this.rollerFalconConfiguration.Slot0.kD = 0;
    this.rollerFalcon.getConfigurator().apply(this.rollerFalconConfiguration);
    this.rollerFalcon.setNeutralMode(NeutralModeValue.Coast);
  }

  //Set intake state, rotationAngle in degrees, rollerVelocity in RPM
  public void setIntake(double rotationAngle, double rollerVelocity){
    if (rotationAngle > Constants.SetPoints.INTAKE_DOWN_ANGLE_DEG){
      // System.out.println("2 " + rotationAngle);
      this.angleFalcon.setControl(this.anglefalconPositionRequest.withPosition(Constants.SetPoints.INTAKE_DOWN_ANGLE_ROT * Constants.Ratios.INTAKE_ANGLE_GEAR_RATIO));
    } else if (rotationAngle < Constants.SetPoints.INTAKE_UP_ANGLE_DEG){
      // System.out.println("3 " + rotationAngle);
      this.angleFalcon.setControl(this.anglefalconPositionRequest.withPosition(Constants.SetPoints.INTAKE_UP_ANGLE_ROT * Constants.Ratios.INTAKE_ANGLE_GEAR_RATIO));
    } else {
      // System.out.println("4 " + rotationAngle);
      this.angleFalcon.setControl(this.anglefalconPositionRequest.withPosition(Constants.degreesToRotations(rotationAngle) * Constants.Ratios.INTAKE_ANGLE_GEAR_RATIO));
    }
    this.rollerFalcon.setControl(this.rollerFalconVelocityRequest.withVelocity(Constants.RPMToRPS(rollerVelocity) * Constants.Ratios.INTAKE_ROLLER_GEAR_RATIO));
  }

  //Set intake state, position either kUP or kDOWN, rollerVelocity in RPM
  public void setIntake(Constants.SetPoints.IntakePosition position, double rollerVelocity){
    if (position.angle < Constants.SetPoints.INTAKE_DOWN_ANGLE_DEG){
      // System.out.println("2 " + position.angle);
      this.angleFalcon.setControl(this.anglefalconPositionRequest.withPosition(Constants.SetPoints.INTAKE_DOWN_ANGLE_ROT * Constants.Ratios.INTAKE_ANGLE_GEAR_RATIO));
    } else if (position.angle > Constants.SetPoints.INTAKE_UP_ANGLE_DEG){
      // System.out.println("3 " + position.angle);
      this.angleFalcon.setControl(this.anglefalconPositionRequest.withPosition(Constants.SetPoints.INTAKE_UP_ANGLE_ROT * Constants.Ratios.INTAKE_ANGLE_GEAR_RATIO));
    } else {
      // System.out.println("4 " + position.angle);
      this.angleFalcon.setControl(this.anglefalconPositionRequest.withPosition(Constants.degreesToRotations(position.angle) * Constants.Ratios.INTAKE_ANGLE_GEAR_RATIO));
    }
  }

  //Set intake angle in rotations
  public void setIntakeAngle(double rotationAngle){
    if (rotationAngle < Constants.SetPoints.INTAKE_DOWN_ANGLE_DEG){
      this.angleFalcon.setControl(this.anglefalconPositionRequest.withPosition(Constants.SetPoints.INTAKE_DOWN_ANGLE_ROT * Constants.Ratios.INTAKE_ANGLE_GEAR_RATIO));
    } else if (rotationAngle > Constants.SetPoints.INTAKE_UP_ANGLE_DEG){
      this.angleFalcon.setControl(this.anglefalconPositionRequest.withPosition(Constants.SetPoints.INTAKE_UP_ANGLE_ROT * Constants.Ratios.INTAKE_ANGLE_GEAR_RATIO));
    } else {
      this.angleFalcon.setControl(this.anglefalconPositionRequest.withPosition(Constants.degreesToRotations(rotationAngle) * Constants.Ratios.INTAKE_ANGLE_GEAR_RATIO));
    }
  }

  //Set intake angle with setpoint
  public void setIntakeAngle(Constants.SetPoints.IntakePosition position){
    if (position.angle < Constants.SetPoints.INTAKE_DOWN_ANGLE_DEG){
      this.angleFalcon.setControl(this.anglefalconPositionRequest.withPosition(Constants.SetPoints.INTAKE_DOWN_ANGLE_ROT * Constants.Ratios.INTAKE_ANGLE_GEAR_RATIO));
    } else if (position.angle > Constants.SetPoints.INTAKE_UP_ANGLE_DEG){
      this.angleFalcon.setControl(this.anglefalconPositionRequest.withPosition(Constants.SetPoints.INTAKE_UP_ANGLE_ROT * Constants.Ratios.INTAKE_ANGLE_GEAR_RATIO));
    } else {
      this.angleFalcon.setControl(this.anglefalconPositionRequest.withPosition(Constants.degreesToRotations(position.angle) * Constants.Ratios.INTAKE_ANGLE_GEAR_RATIO));
    }
  }

  //Set intake roller velocity in RPM
  public void setIntakeRollers(double rollerVelocity){
    this.rollerFalcon.setControl(this.rollerFalconVelocityRequest.withVelocity(Constants.RPMToRPS(rollerVelocity) * Constants.Ratios.INTAKE_ROLLER_GEAR_RATIO));
  }

  //Set intake roller percent
  public void setIntakePercent(double percent){
    this.rollerFalcon.set(percent);
  }

  public double getRollerVelocity(){
    return this.rollerFalcon.getVelocity().getValue() / Constants.Ratios.INTAKE_ROLLER_GEAR_RATIO;
  }

  public double getIntakeAngleRotations(){
    return this.angleFalcon.getPosition().getValue() / Constants.Ratios.INTAKE_ANGLE_GEAR_RATIO;
  }

  public double getIntakeAngleDegrees(){
    return Constants.rotationsToDegrees(this.angleFalcon.getPosition().getValue() / Constants.Ratios.INTAKE_ANGLE_GEAR_RATIO);
  }

  //Get value of intake rotation limit switch
  public boolean getAngleLimitSwitch(){
    if (angleFalcon.getReverseLimit().getValue().value == 1){
      return true;
    } else {
      return false;
    }
  }

  //Set encoder position of the intake angle in rotations
  public void setAngleEncoderPosition(double position){
    angleFalcon.setPosition(position);
  }

  //Constantly set roller velocity PID
  public void teleopPeriodic(){
    SmartDashboard.putNumber("Intake Angle", getIntakeAngleDegrees());
  }

  @Override
  public void periodic() {}
}
