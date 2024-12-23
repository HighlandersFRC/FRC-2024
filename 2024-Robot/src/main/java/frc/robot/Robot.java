package frc.robot;

import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends LoggedRobot {
  private RobotContainer m_robotContainer = new RobotContainer();
  private Command m_autonomousCommand;

  // private Logger logger = Logger.getInstance();

  // private double shooterAngleDegreesTuning = 0;
  // private double shooterRPMTuning = 0;
  private double m_startTime = Timer.getFPGATimestamp();
  private boolean m_checkedCAN = false;

  String m_fieldSide = "blue";

  @Override
  public void robotInit() {
    System.out.println("Robot Init");
    Logger.addDataReceiver(new WPILOGWriter()); // Log to a USB stick ("/U/logs")
    Logger.addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables
    Logger.start(); // Start logging! No more data receivers, replay sources, or metadata values may
                    // be added.
    this.m_fieldSide = "blue";
    SmartDashboard.putNumber("Shooter Angle Degrees (tuning)", 0);
    SmartDashboard.putNumber("Shooter RPM (input)", 0);

    m_robotContainer.lights.init(m_fieldSide);
    m_robotContainer.peripherals.init();
    m_robotContainer.drive.init(m_fieldSide);
    m_robotContainer.intake.init();
    m_robotContainer.shooter.init();
    m_robotContainer.feeder.init();
    m_robotContainer.climber.init();

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

    PortForwarder.add(5800, "orangepi1.local", 5800);
    PortForwarder.add(5801, "orangepi1.local", 5801);

    PortForwarder.add(5800, "10.44.99.41", 5800);
    PortForwarder.add(5801, "10.44.99.41", 5801);

    PortForwarder.add(5800, "10.44.99.42", 5800);
    PortForwarder.add(5801, "10.44.99.42", 5801);

    PortForwarder.add(5800, "10.44.99.43", 5800);
    PortForwarder.add(5801, "10.44.99.43", 5801);

    PortForwarder.add(5800, "10.44.99.44", 5800);
    PortForwarder.add(5801, "10.44.99.44", 5801);

    PortForwarder.add(5800, "10.44.99.34", 5800);
    PortForwarder.add(5801, "10.44.99.34", 5801);

    m_robotContainer.lights.clearAnimations();
    m_robotContainer.lights.setCommandRunning(true);
    m_robotContainer.lights.setRGBFade();
    System.out.println("end robot init");
  }

  @Override
  public void robotPeriodic() {
    m_robotContainer.updateNoteInIntake();
    // checks CAN and limelights, blinks green if good and blinks yellow if bad
    if (!m_checkedCAN) {
      if (Timer.getFPGATimestamp() - m_startTime > 30 || m_robotContainer.peripherals.limelightsConnected()) {
        m_checkedCAN = true;
        m_robotContainer.lights.clearAnimations();
        m_robotContainer.lights.setCommandRunning(false);
        if (m_robotContainer.drive.getSwerveCAN() && m_robotContainer.shooter.getShooterCAN()
            && m_robotContainer.intake.getIntakeCAN() && m_robotContainer.feeder.getFeederCAN()
            && m_robotContainer.climber.getClimberCAN() && m_robotContainer.peripherals.limelightsConnected()) {
          m_robotContainer.lights.blinkGreen(3);
        } else {
          m_robotContainer.lights.clearAnimations();
          m_robotContainer.lights.setCommandRunning(true);
          m_robotContainer.lights.setStrobeYellow();
        }
      }
    }

    // shooterAngleDegreesTuning = SmartDashboard.getNumber("Shooter Angle Degrees
    // (tuning)", 0);
    // shooterRPMTuning = SmartDashboard.getNumber("Shooter RPM (input)", 0);
    CommandScheduler.getInstance().run();

    try {
      Logger.recordOutput("Localization Odometry", m_robotContainer.drive.getLocalizationOdometry());
    } catch (Exception e) {
      System.out.println("Problem with logging");
    }

    try {
      Logger.recordOutput("Wheel Odometry", m_robotContainer.drive.getOdometry());
    } catch (Exception e) {
      System.out.println("Problem with logging");
    }

    try {
      Logger.recordOutput("MT2 Odometry", m_robotContainer.drive.getMT2Odometry());
    } catch (Exception e) {
      System.out.println("Problem with logging");
    }
    Logger.recordOutput("Swerve Module States", m_robotContainer.drive.getModuleStates());
    Logger.recordOutput("Swerve Module Setpoints", m_robotContainer.drive.getModuleSetpoints());
    Logger.recordOutput("IMU", m_robotContainer.peripherals.getPigeonAngle());
    Logger.recordOutput("Note in Robot", m_robotContainer.getNoteInRobot());
    Constants.periodic();
    m_robotContainer.lights.periodic();
    m_robotContainer.intake.periodic();
    m_robotContainer.shooter.periodic();
    m_robotContainer.feeder.periodic();
    m_robotContainer.tof.periodic();
    m_robotContainer.proximity.periodic();

    m_robotContainer.peripherals.periodic();
    m_robotContainer.climber.periodic();
  }

  @Override
  public void disabledInit() {
    OI.driverController.setRumble(RumbleType.kBothRumble, 0);
    OI.operatorController.setRumble(RumbleType.kBothRumble, 0);
    m_robotContainer.lights.clearAnimations();
    m_robotContainer.lights.setCommandRunning(true);
    m_robotContainer.lights.setRainbow();
  }

  @Override
  public void disabledPeriodic() {
  }

  @Override
  public void autonomousInit() {
    if (OI.isBlueSide()) {
      System.out.println("ON BLUE SIDE");
      m_fieldSide = "blue";
    } else {
      System.out.println("ON RED SIDE");
      m_fieldSide = "red";
    }
    this.m_robotContainer.drive.setFieldSide(m_fieldSide);

    System.out.print("Selected Auto: ");

    this.m_robotContainer.intake.autoInit();
  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
    m_robotContainer.shooter.teleopInit();
    m_robotContainer.lights.setCommandRunning(false);
    m_robotContainer.lights.clearAnimations();
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    if (OI.isBlueSide()) {
      m_fieldSide = "blue";
    } else {
      m_fieldSide = "red";
    }

    if (this.m_fieldSide == "red") {
      this.m_robotContainer.drive.setPigeonAfterAuto();
    }
    System.out.println("field side" + m_fieldSide);

    this.m_robotContainer.drive.setFieldSide(m_fieldSide);
  }

  @Override
  public void teleopPeriodic() {
    // SmartDashboard.putBoolean("Feeder TOF", this.tof.getFeederDistMillimeters() <
    // Constants.SetPoints.FEEDER_TOF_THRESHOLD_MM);
    // SmartDashboard.putNumber("Feeder TOF Dist",
    // this.tof.getFeederDistMillimeters());
    m_robotContainer.intake.teleopPeriodic();
    m_robotContainer.shooter.teleopPeriodic();
    m_robotContainer.feeder.teleopPeriodic();
    m_robotContainer.climber.teleopPeriodic();
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {
  }

  @Override
  public void simulationInit() {
  }

  @Override
  public void simulationPeriodic() {
  }
}
