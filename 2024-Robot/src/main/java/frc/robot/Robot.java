// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;
import java.io.FileReader;

import org.json.JSONArray;
import org.json.JSONObject;
import org.json.JSONTokener;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.AutoParser;
import frc.robot.commands.AutoShoot;
import frc.robot.commands.DriveAutoAligned;
import frc.robot.commands.PresetAutoShoot;
import frc.robot.commands.RunClimber;
import frc.robot.commands.RunFeeder;
import frc.robot.commands.RunIntake;
import frc.robot.commands.RunIntakeAndFeeder;
import frc.robot.commands.RunShooter;
import frc.robot.commands.RunTrap;
import frc.robot.commands.SetClimber;
import frc.robot.commands.SmartIntake;
import frc.robot.commands.SmartShoot;
import frc.robot.commands.TurnToTarget;
import frc.robot.commands.ZeroAngleMidMatch;
import frc.robot.commands.autos.FourPieceCloseAuto;
import frc.robot.commands.autos.ThreePieceFarBottomAuto;
import frc.robot.sensors.TOF;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Lights;
import frc.robot.subsystems.Peripherals;
import frc.robot.subsystems.Shooter;

public class Robot extends LoggedRobot {
  private Command m_autonomousCommand;

  //Subsystems
  private Lights lights = new Lights();
  private Peripherals peripherals = new Peripherals();
  private Drive drive = new Drive(peripherals);
  private Intake intake = new Intake();
  private Shooter shooter = new Shooter();
  private Feeder feeder = new Feeder();
  private Climber climber = new Climber(lights);

  //Sensors
  private TOF tof = new TOF();

  // private Logger logger = Logger.getInstance();
  
  File fourPieceCloseFile;
  JSONArray fourPieceCloseJSON;
  File threePieceFarBottomFile;
  JSONArray threePieceFarBottomJSON;

  String fieldSide = "blue";

  Command auto;

  @Override
  public void robotInit() {
    // Logger.recordMetadata("ProjectName", "MyProject"); // Set a metadata value

    // if (isReal()) {
    //   Logger.addDataReceiver(new WPILOGWriter()); // Log to a USB stick ("/U/logs")
    //   Logger.addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables
    //   new PowerDistribution(1, ModuleType.kRev); // Enables power distribution logging
    // } else {
    //   setUseTiming(false); // Run as fast as possible
    //   String logPath = LogFileUtil.findReplayLog(); // Pull the replay log from AdvantageScope (or prompt the user)
    //   Logger.setReplaySource(new WPILOGReader(logPath)); // Read replay log
    //   Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim"))); // Save outputs to a new log
    // }

    // // Logger.disableDeterministicTimestamps() // See "Deterministic Timestamps" in the "Understanding Data Flow" page
    // Logger.start(); // Start logging! No more data receivers, replay sources, or metadata values may be added.

    this.fieldSide = "blue";

    lights.init(fieldSide);
    peripherals.init();
    drive.init(fieldSide);
    intake.init();
    shooter.init();
    feeder.init();
    climber.init();

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

    try {
      this.fourPieceCloseFile = new File("/home/lvuser/deploy/2PieceCenterPart1.json");
      FileReader scanner = new FileReader(this.fourPieceCloseFile);
      JSONObject pathRead = new JSONObject(new JSONTokener(scanner));
      this.fourPieceCloseJSON = (JSONArray) pathRead.get("sampled_points");
    } catch(Exception e) {
      System.out.println("ERROR WITH PATH FILE " + e);
    }
    try {
      this.threePieceFarBottomFile = new File("/home/lvuser/deploy/2PieceCenterPart1.json");
      FileReader scanner = new FileReader(this.threePieceFarBottomFile);
      JSONObject pathRead = new JSONObject(new JSONTokener(scanner));
      this.threePieceFarBottomJSON = (JSONArray) pathRead.get("sampled_points");
    } catch(Exception e) {
      System.out.println("ERROR WITH PATH FILE " + e);
    }

    // this.auto = new AutoParser(drive, intake, feeder, shooter, peripherals, "CommandTest");
    // this.auto.schedule();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();

    // Logger.recordOutput("Odometry", drive.getOdometry());

    lights.periodic();
    intake.periodic();
    shooter.periodic();
    feeder.periodic();

    // drive.periodic(); // remove for competition
    peripherals.periodic();
    climber.periodic();
  }

  @Override
  public void disabledInit() {
    OI.driverController.setRumble(RumbleType.kBothRumble, 0);
    OI.operatorController.setRumble(RumbleType.kBothRumble, 0);
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

    // System.out.println("Selected Auto: ");
    // if (OI.is4PieceCloseAuto()) {
    //   System.out.println("Four Piece Close");
    //   this.auto = new FourPieceCloseAuto(drive, peripherals, intake, feeder, shooter, climber, lights, tof);
    //   this.drive.autoInit(this.fourPieceCloseJSON);
    // } else if (OI.is3PieceFarBottomAuto()){
      // System.out.println("Three Piece Far Bottom");
      // this.auto = new ThreePieceFarBottomAuto(drive, peripherals, intake, feeder, shooter, climber, lights, tof);
      // this.drive.autoInit(this.threePieceFarBottomJSON);
    // } else {
    //   System.out.println("NO AUTO SELECTED");
    // }

    
    this.intake.autoInit();

    //THIS MUST BE LAST!!!
    try {
      // this.auto.schedule();
    } catch (Exception e){
      System.out.println("No auto is selected");
    }
    //THIS MUST BE LAST!!!
  }

  @Override
  public void autonomousPeriodic() {}

  @Override 
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

    //CONTROLS

    //Driver
    OI.driverX.whileTrue(new DriveAutoAligned(drive, peripherals));
    OI.driverViewButton.whileTrue(new ZeroAngleMidMatch(drive));
    OI.driverRT.whileTrue(new SmartIntake(intake, feeder, climber, lights, tof, Constants.SetPoints.IntakePosition.kDOWN, 1200,  600));
    OI.driverLT.whileTrue(new RunIntakeAndFeeder(intake, feeder, climber, Constants.SetPoints.IntakePosition.kUP, -800, -800, -0.4));
    OI.driverY.whileTrue(new AutoShoot(drive, shooter, feeder, peripherals, lights, tof, 1200));
    OI.driverA.whileTrue(new TurnToTarget(drive, peripherals));
    OI.driverB.whileTrue(new PresetAutoShoot(drive, shooter, feeder, peripherals, lights, tof, 23.5, 7500, 1200, 0));

    //Operator
  }

  @Override
  public void teleopPeriodic() {
    // SmartDashboard.putBoolean("Feeder TOF", this.tof.getFeederDistMillimeters() < Constants.SetPoints.FEEDER_TOF_THRESHOLD_MM);
    // SmartDashboard.putNumber("Feeder TOF Dist", this.tof.getFeederDistMillimeters());
    intake.teleopPeriodic();
    shooter.teleopPeriodic();
    feeder.teleopPeriodic();
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
