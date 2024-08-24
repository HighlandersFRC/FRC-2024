package frc.robot;

import org.littletonrobotics.junction.LoggedRobot;

import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.DriveAutoAligned;
import frc.robot.commands.RunIntake;
import frc.robot.commands.RunShooter;
import frc.robot.commands.ZeroAngleMidMatch;
import frc.robot.sensors.Proximity;
import frc.robot.sensors.TOF;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Lights;
import frc.robot.subsystems.Peripherals;
import frc.robot.subsystems.Shooter;

public class Robot extends LoggedRobot {
  private Command m_autonomousCommand;

  //Sensors
  private TOF tof = new TOF();
  private Proximity proximity = new Proximity();

  //Subsystems
  private Lights lights = new Lights(tof);
  private Peripherals peripherals = new Peripherals();
  private Drive drive = new Drive(peripherals);
  private Intake intake = new Intake();
  private Feeder feeder = new Feeder();
  private Shooter shooter = new Shooter();
  // private Logger logger = Logger.getInstance();

  private double shooterAngleDegreesTuning = 0;
  private double shooterRPMTuning = 0;
  private double startTime = Timer.getFPGATimestamp();
  private boolean checkedCAN = false;

  String fieldSide = "blue";

  @Override
  public void robotInit() {
    // System.out.println("Starting");
    // Logger.addDataReceiver(new WPILOGWriter()); // Log to a USB stick ("/U/logs")
    // Logger.addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables
    // new PowerDistribution(1, ModuleType.kRev); // Enables power distribution logging
    // Logger.recordMetadata("Code", "Running");
    // Logger.disableDeterministicTimestamps(); // See "Deterministic Timestamps" in the "Understanding Data Flow" page
    // Logger.start(); // Start logging! No more data receivers, replay sources, or metadata values may be added.
    // System.out.println("Started Logger");
    this.fieldSide = "blue";
    SmartDashboard.putNumber("Shooter Angle Degrees (tuning)", 0);
    SmartDashboard.putNumber("Shooter RPM (input)", 0);

    lights.init(fieldSide);
    peripherals.init();
    drive.init(fieldSide);
    feeder.init();
    shooter.init();

    PortForwarder.add(5800, "limelight.local", 5800);
    PortForwarder.add(5801, "limelight.local", 5801);
    
    PortForwarder.add(5800, "limelight-front.local", 5800);
    PortForwarder.add(5801, "limelight-front.local", 5801);
    
    PortForwarder.add(5800, "limelight-back.local", 5800);
    PortForwarder.add(5801, "limelight-back.local", 5801);

    PortForwarder.add(5800, "limelight-left.local", 5800);
    PortForwarder.add(5801, "limelight-left.local", 5801);

    PortForwarder.add(5800, "limelight-right.local", 5800);
    PortForwarder.add(5801, "limelight-right.local", 5801);

    PortForwarder.add(5800, "10.44.99.41", 5800);
    PortForwarder.add(5801, "10.44.99.41", 5801);

    PortForwarder.add(5800, "10.44.99.42", 5800);
    PortForwarder.add(5801, "10.44.99.42", 5801);

    PortForwarder.add(5800, "10.44.99.43", 5800);
    PortForwarder.add(5801, "10.44.99.43", 5801);

    PortForwarder.add(5800, "10.44.99.44", 5800);
    PortForwarder.add(5801, "10.44.99.44", 5801);
    lights.clearAnimations();
    lights.setCommandRunning(true);
    lights.setRGBFade();

  }
 
  @Override
  public void robotPeriodic() {
    shooterAngleDegreesTuning = SmartDashboard.getNumber("Shooter Angle Degrees (tuning)", 0);
    shooterRPMTuning = SmartDashboard.getNumber("Shooter RPM (input)", 0);
    CommandScheduler.getInstance().run();

    // try{
    //   Logger.recordOutput("Localization Odometry", drive.getLocalizationOdometry());
    // } catch(Exception e) {
    //   System.out.println("Problem with logging");
    // }

    // try{
    //   Logger.recordOutput("Wheel Odometry", drive.getOdometry());
    // } catch(Exception e) {
    //   System.out.println("Problem with logging");
    // }

    // try{
    //   Logger.recordOutput("MT2 Odometry", drive.getMT2Odometry());
    // } catch(Exception e) {
    //   System.out.println("Problem with logging");
    // }
    // Logger.recordOutput("Swerve Module States", drive.getModuleStates());
    // Logger.recordOutput("Swerve Module Setpoints", drive.getModuleSetpoints());
    // Logger.recordOutput("IMU", peripherals.getPigeonAngle());

    lights.periodic();
    tof.periodic();
    proximity.periodic();

    // drive.periodic(); // remove for competition
    peripherals.periodic();

    // System.out.println("0-1: " + (t1 - t0));

    // SmartDashboard.putNumber("Carriage Rotation", climber.getCarriageRotationDegrees());
  }

  @Override
  public void disabledInit() {
    OI.driverController.setRumble(RumbleType.kBothRumble, 0);
    OI.operatorController.setRumble(RumbleType.kBothRumble, 0);
    lights.clearAnimations();
    lights.setCommandRunning(true);
    lights.setRainbow();
  }

  @Override
  public void disabledPeriodic() {}

  @Override
  public void autonomousInit() {
    if (OI.isBlueSide()) {
      System.out.println("ON BLUE SIDE");
      fieldSide = "blue";
    } else {
      System.out.println("ON RED SIDE");
      fieldSide = "red";
    }
    this.drive.setFieldSide(fieldSide);
    this.peripherals.setFieldSide(fieldSide);
  }

  @Override
  public void autonomousPeriodic() {}

  @Override 
  public void teleopInit() {
    lights.setCommandRunning(false);
    lights.clearAnimations();
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

    if (OI.isBlueSide()) {
      fieldSide = "blue";
    } else {
      fieldSide = "red";
    }

    if (this.fieldSide == "red"){
      this.drive.setPigeonAfterAuto();
    }
    System.out.println("field side" + fieldSide);

    this.peripherals.setFieldSide(fieldSide);
    this.drive.setFieldSide(fieldSide);

    //CONTROLS

    //Driver
    OI.driverViewButton.whileTrue(new ZeroAngleMidMatch(drive));
    OI.driverX.whileTrue(new DriveAutoAligned(drive, peripherals));
    OI.driverRT.whileTrue(new RunIntake(intake, feeder, 0.8));
    OI.driverA.whileTrue(new RunShooter(shooter, feeder, intake, 2000, 4000));
  }

  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {}
}
