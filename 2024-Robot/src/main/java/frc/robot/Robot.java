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
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.DriveAutoAligned;
import frc.robot.commands.RunIntake;
import frc.robot.commands.Shoot;
import frc.robot.commands.ZeroAngleMidMatch;
import frc.robot.commands.autos.FivePieceAuto;
import frc.robot.commands.autos.FourPieceCloseAuto;
import frc.robot.commands.autos.FourPieceOneFarAuto;
import frc.robot.commands.autos.FourPieceTwoFarAuto;
import frc.robot.commands.autos.ThreePieceBottomAuto;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Lights;
import frc.robot.subsystems.Peripherals;
import frc.robot.subsystems.Shooter;

public class Robot extends LoggedRobot {
  private Command m_autonomousCommand;

  private Lights lights = new Lights();
  private Peripherals peripherals = new Peripherals(lights);
  private Drive drive = new Drive(peripherals);
  private Intake intake = new Intake(lights);
  private Shooter shooter = new Shooter(lights);
  private Feeder feeder = new Feeder(lights);

  // private Logger logger = Logger.getInstance();
  
  File pathingFile;
  String pathString;

  JSONObject pathRead;
  JSONArray pathJSON;

  String fieldSide = "red";

  SequentialCommandGroup auto;

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

    if (OI.isBlueSide()) {
      System.out.println("ON BLUE SIDE");
      fieldSide = "blue";
    } else {
      System.out.println("ON RED SIDE");
      fieldSide = "red";
    }

    lights.init(fieldSide);
    peripherals.init();
    drive.init(fieldSide);
    intake.init();
    shooter.init();
    feeder.init();

    PortForwarder.add(5800, "limelight.local", 5800);
    PortForwarder.add(5801, "limelight.local", 5801);

    PortForwarder.add(5800, "10.44.99.13", 5800);
    PortForwarder.add(5801, "10.44.99.13", 5801);

    PortForwarder.add(5800, "10.44.99.57", 5800);
    PortForwarder.add(5801, "10.44.99.57", 5801);

    PortForwarder.add(5800, "10.44.99.58", 5800);
    PortForwarder.add(5801, "10.44.99.58", 5801);

    PortForwarder.add(5800, "10.44.99.65", 5800);
    PortForwarder.add(5801, "10.44.99.65", 5801);

    PortForwarder.add(5800, "10.44.99.41", 5800);
    PortForwarder.add(5801, "10.44.99.41", 5801);

    PortForwarder.add(5800, "10.44.99.42", 5800);
    PortForwarder.add(5801, "10.44.99.42", 5801);

    //Auto selection here...
    if (OI.is4PieceCloseAuto()) {
      System.out.println("4 piece");
      try {
        pathingFile = new File("/home/lvuser/deploy/2PieceCenterPart1.json");
        FileReader scanner = new FileReader(pathingFile);
        pathRead = new JSONObject(new JSONTokener(scanner));
        pathJSON = (JSONArray) pathRead.get("sampled_points");
      } catch(Exception e) {
        System.out.println("ERROR WITH PATH FILE " + e);
      }
    } else if (OI.is5PieceAuto()) {
      try {
        pathingFile = new File("/home/lvuser/deploy/2PieceCenterPart1.json");
        FileReader scanner = new FileReader(pathingFile);
        pathRead = new JSONObject(new JSONTokener(scanner));
        pathJSON = (JSONArray) pathRead.get("sampled_points");
      } catch (Exception e) {
        System.out.println("ERROR WITH PATH FILE " + e);
      }
    } else if (OI.is4Piece1FarAuto()) {
      try {
        pathingFile = new File("/home/lvuser/deploy/4Piece1FarPart1.json");
        FileReader scanner = new FileReader(pathingFile);
        pathRead = new JSONObject(new JSONTokener(scanner));
        pathJSON = (JSONArray) pathRead.get("sampled_points");
      } catch (Exception e) {
        System.out.println("ERROR WITH PATH FILE " + e);
      }
    } else if (OI.is3PieceBottomAuto()) {
      try {
        pathingFile = new File("/home/lvuser/deploy/3PieceBottomPart1.json");
        FileReader scanner = new FileReader(pathingFile);
        pathRead = new JSONObject(new JSONTokener(scanner));
        pathJSON = (JSONArray) pathRead.get("sampled_points");
      } catch (Exception e) {
        System.out.println("ERROR WITH PATH FILE " + e);
      }
    } else if (OI.is4Piece2FarAuto()){
      try {
        pathingFile = new File("/home/lvuser/deploy/4Piece2FarPart1.json");
        FileReader scanner = new FileReader(pathingFile);
        pathRead = new JSONObject(new JSONTokener(scanner));
        pathJSON = (JSONArray) pathRead.get("sampled_points");
      } catch (Exception e) {
        System.out.println("ERROR WITH PATH FILE " + e);
      }
    }

    if (OI.is4PieceCloseAuto()) {
      // drive.useCameraInOdometry();
      this.auto = new FourPieceCloseAuto(drive, peripherals);
      auto.schedule();
    } else if (OI.is5PieceAuto()) {
      this.auto = new FivePieceAuto(drive, peripherals);
      auto.schedule();
    } else if (OI.is4Piece1FarAuto()){
      this.auto = new FourPieceOneFarAuto(drive, peripherals);
      auto.schedule();
    } else if (OI.is3PieceBottomAuto()) {
      this.auto = new ThreePieceBottomAuto();
      auto.schedule();
    } else if (OI.is4Piece2FarAuto()){
      this.auto = new FourPieceTwoFarAuto();
      auto.schedule();
    } else {
      System.out.println("NO AUTO SELECTED");
    }
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
    drive.autoInit(this.pathJSON);
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
    OI.driverRT.whileTrue(new RunIntake(intake, Constants.SetPoints.IntakePosition.kDOWN, 1000));
    OI.driverLT.whileTrue(new RunIntake(intake, Constants.SetPoints.IntakePosition.kDOWN, -1000));
    OI.driverY.whileTrue(new Shoot(shooter, feeder, 1000));

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
