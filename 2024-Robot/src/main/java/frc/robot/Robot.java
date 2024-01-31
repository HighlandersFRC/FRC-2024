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
import frc.robot.commands.RunClimber;
import frc.robot.commands.RunFeeder;
import frc.robot.commands.RunIntake;
import frc.robot.commands.RunIntakeAndFeeder;
import frc.robot.commands.RunShooter;
import frc.robot.commands.SmartIntake;
import frc.robot.commands.SmartShoot;
import frc.robot.commands.ZeroAngleMidMatch;
import frc.robot.commands.autos.FivePieceAuto;
import frc.robot.commands.autos.FourPieceCloseAuto;
import frc.robot.commands.autos.FourPieceOneFarAuto;
import frc.robot.commands.autos.FourPieceTwoFarAuto;
import frc.robot.commands.autos.ThreePieceBottomAuto;
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
  
  File pathingFile;
  String pathString;

  JSONObject pathRead;
  JSONArray pathJSON;

  String fieldSide = "red";

  Command auto;

  @Override
  public void robotInit() {
    Logger.recordMetadata("ProjectName", "MyProject"); // Set a metadata value

    if (isReal()) {
      Logger.addDataReceiver(new WPILOGWriter()); // Log to a USB stick ("/U/logs")
      Logger.addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables
      new PowerDistribution(1, ModuleType.kRev); // Enables power distribution logging
    } else {
      setUseTiming(false); // Run as fast as possible
      String logPath = LogFileUtil.findReplayLog(); // Pull the replay log from AdvantageScope (or prompt the user)
      Logger.setReplaySource(new WPILOGReader(logPath)); // Read replay log
      Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim"))); // Save outputs to a new log
    }

    // Logger.disableDeterministicTimestamps() // See "Deterministic Timestamps" in the "Understanding Data Flow" page
    Logger.start(); // Start logging! No more data receivers, replay sources, or metadata values may be added.

    // if (OI.isBlueSide()) {
    //   System.out.println("ON BLUE SIDE");
    //   fieldSide = "blue";
    // } else {
    //   System.out.println("ON RED SIDE");
    //   fieldSide = "red";
    // }

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

    //Auto selection here...
    // if (OI.is4PieceCloseAuto()) {
    //   System.out.println("4 piece");
    //   try {
    //     pathingFile = new File("/home/lvuser/deploy/2PieceCenterPart1.json");
    //     FileReader scanner = new FileReader(pathingFile);
    //     pathRead = new JSONObject(new JSONTokener(scanner));
    //     pathJSON = (JSONArray) pathRead.get("sampled_points");
    //   } catch(Exception e) {
    //     System.out.println("ERROR WITH PATH FILE " + e);
    //   }
    // } else if (OI.is5PieceAuto()) {
      try {
        pathingFile = new File("/home/lvuser/deploy/2PieceCenterPart1.json");
        FileReader scanner = new FileReader(pathingFile);
        pathRead = new JSONObject(new JSONTokener(scanner));
        pathJSON = (JSONArray) pathRead.get("sampled_points");
      } catch (Exception e) {
        System.out.println("ERROR WITH PATH FILE " + e);
      }
    // } else if (OI.is4Piece1FarAuto()) {
    //   try {
    //     pathingFile = new File("/home/lvuser/deploy/4Piece1FarPart1.json");
    //     FileReader scanner = new FileReader(pathingFile);
    //     pathRead = new JSONObject(new JSONTokener(scanner));
    //     pathJSON = (JSONArray) pathRead.get("sampled_points");
    //   } catch (Exception e) {
    //     System.out.println("ERROR WITH PATH FILE " + e);
    //   }
    // } else if (OI.is3PieceBottomAuto()) {
    //   try {
    //     pathingFile = new File("/home/lvuser/deploy/3PieceBottomPart1.json");
    //     FileReader scanner = new FileReader(pathingFile);
    //     pathRead = new JSONObject(new JSONTokener(scanner));
    //     pathJSON = (JSONArray) pathRead.get("sampled_points");
    //   } catch (Exception e) {
    //     System.out.println("ERROR WITH PATH FILE " + e);
    //   }
    // } else if (OI.is4Piece2FarAuto()){
    //   try {
    //     pathingFile = new File("/home/lvuser/deploy/4Piece2FarPart1.json");
    //     FileReader scanner = new FileReader(pathingFile);
    //     pathRead = new JSONObject(new JSONTokener(scanner));
    //     pathJSON = (JSONArray) pathRead.get("sampled_points");
    //   } catch (Exception e) {
    //     System.out.println("ERROR WITH PATH FILE " + e);
    //   }
    // }

    // if (OI.is4PieceCloseAuto()) {
    //   // drive.useCameraInOdometry();
    //   this.auto = new FourPieceCloseAuto(drive, peripherals);
    //   auto.schedule();
    // } else if (OI.is5PieceAuto()) {
      this.auto = new FivePieceAuto(drive, peripherals, intake, feeder, shooter, lights, tof);
      auto.schedule();
    // } else if (OI.is4Piece1FarAuto()){
    //   this.auto = new FourPieceOneFarAuto(drive, peripherals);
    //   auto.schedule();
    // } else if (OI.is3PieceBottomAuto()) {
    //   this.auto = new ThreePieceBottomAuto();
    //   auto.schedule();
    // } else if (OI.is4Piece2FarAuto()){
    //   this.auto = new FourPieceTwoFarAuto();
    //   auto.schedule();
    // } else {
    //   System.out.println("NO AUTO SELECTED");
    // }

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

    drive.periodic(); // remove for competition
    peripherals.periodic();
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
    try {
      this.auto.schedule();
    } catch (Exception e){
      System.out.println("No auto is selected");
    }
    this.intake.autoInit();
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
    OI.driverRT.whileTrue(new SmartIntake(intake, feeder, lights, tof, Constants.SetPoints.IntakePosition.kDOWN, 1200,  600));
    OI.driverLT.whileTrue(new RunIntakeAndFeeder(intake, feeder, Constants.SetPoints.IntakePosition.kUP, -800, -800));
    // OI.driverA.whileTrue(new SmartShoot(shooter, feeder, peripherals, lights, tof, 40, 6500, 2000));
    // OI.driverB.whileTrue(new SmartShoot(shooter, feeder, peripherals, lights, tof, 30, 4000, 2000));
    // OI.driverX.whileTrue(new RunClimber(climber, 0.6, 0.6));
    // OI.driverY.whileTrue(new RunClimber(climber, 0.0, 0.6));
    // OI.driverRB.whileTrue(new RunClimber(climber, 0.6, 0.0));
    OI.driverY.whileTrue(new AutoShoot(shooter, feeder, peripherals, lights, tof, 600));

    //Operator
  }

  @Override
  public void teleopPeriodic() {
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
