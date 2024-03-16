// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos;

import java.io.File;
import java.io.FileReader;

import org.json.JSONArray;
import org.json.JSONObject;
import org.json.JSONTokener;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.commands.AutoIntake;
import frc.robot.commands.AutoPrepForShot;
import frc.robot.commands.AutoShoot;
import frc.robot.commands.AutonomousFollower;
import frc.robot.commands.IdleShooter;
import frc.robot.commands.PresetAutoShoot;
import frc.robot.commands.RunFeeder;
import frc.robot.commands.RunIntake;
import frc.robot.commands.RunShooter;
import frc.robot.commands.SetCarriage;
import frc.robot.commands.ShootWhilePathingAndIntaking;
import frc.robot.commands.SmartIntake;
import frc.robot.commands.SmartShoot;
import frc.robot.commands.SpinUpShooter;
import frc.robot.commands.StopDriving;
import frc.robot.commands.TurnToTarget;
import frc.robot.sensors.Proximity;
import frc.robot.sensors.TOF;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Lights;
import frc.robot.subsystems.Peripherals;
import frc.robot.subsystems.Shooter;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class FivePieceAuto extends SequentialCommandGroup {
  private File pathingFile;
  private JSONArray pathJSON;
  private JSONObject pathRead;

  private File pathingFile2;
  private JSONArray pathJSON2;
  private JSONObject pathRead2;

  private File pathingFile3;
  private JSONArray pathJSON3;
  private JSONObject pathRead3;

  private File pathingFile4;
  private JSONArray pathJSON4;
  private JSONObject pathRead4;

  /** Creates a new FivePieceAuto. */
  public FivePieceAuto(Drive drive, Peripherals peripherals, Intake intake, Feeder feeder, Shooter shooter, Climber climber, Lights lights, TOF tof, Proximity proximity) {
    try {
      pathingFile = new File("/home/lvuser/deploy/4PieceClosePart1.json");
      FileReader scanner = new FileReader(pathingFile);
      pathRead = new JSONObject(new JSONTokener(scanner));
      pathJSON = (JSONArray) pathRead.get("sampled_points");
    }
    catch(Exception e) {
      System.out.println("ERROR WITH PATH FILE " + e);
    }

    try {
      pathingFile2 = new File("/home/lvuser/deploy/4PieceClosePart2.json");
      FileReader scanner2 = new FileReader(pathingFile2);
      pathRead2 = new JSONObject(new JSONTokener(scanner2));
      pathJSON2 = (JSONArray) pathRead2.get("sampled_points");
    }
    catch(Exception e) {
      System.out.println("ERROR WITH PATH FILE " + e);
    }

    try {
      pathingFile3 = new File("/home/lvuser/deploy/4PieceClosePart3.json");
      FileReader scanner3 = new FileReader(pathingFile3);
      pathRead3 = new JSONObject(new JSONTokener(scanner3));
      pathJSON3 = (JSONArray) pathRead3.get("sampled_points");
    }
    catch(Exception e) {
      System.out.println("ERROR WITH PATH FILE " + e);
    }

    try {
      pathingFile4 = new File("/home/lvuser/deploy/5PiecePart4.json");
      FileReader scanner4 = new FileReader(pathingFile4);
      pathRead4 = new JSONObject(new JSONTokener(scanner4));
      pathJSON4 = (JSONArray) pathRead4.get("sampled_points");
    }
    catch(Exception e) {
      System.out.println("ERROR WITH PATH FILE " + e);
    }


    addRequirements(drive, intake, feeder, shooter, lights);

    addCommands(
      new ParallelDeadlineGroup(
        new PresetAutoShoot(drive, shooter, feeder, peripherals, lights, tof, 58, 3000, 1200, 10),
        new RunIntake(intake, Constants.SetPoints.IntakePosition.kDOWN, 1200),
        new SetCarriage(climber, Constants.SetPoints.CarriageRotation.kDOWN, 10, 0.1, false)
      ),
      new ParallelDeadlineGroup(
        new AutoIntake(intake, feeder, climber, lights, tof, proximity, Constants.SetPoints.IntakePosition.kDOWN, 1200, 500, 3),
        new ParallelCommandGroup(
          new SequentialCommandGroup(
            new AutonomousFollower(drive, lights, peripherals, pathJSON, 0, false, false),
            new TurnToTarget(drive, peripherals)
          ),
          new AutoPrepForShot(shooter, tof, 0, 5000)
        )
      ),
      new ParallelDeadlineGroup(
        new AutoShoot(drive, shooter, feeder, peripherals, lights, tof, 1200, 1),
        new SetCarriage(climber, Constants.SetPoints.CarriageRotation.kDOWN, 10, 0.1, false)
      ),
      new ParallelDeadlineGroup(
        new AutoIntake(intake, feeder, climber, lights, tof, proximity, Constants.SetPoints.IntakePosition.kDOWN, 1200, 500, 3),
        new ParallelCommandGroup(
          new SequentialCommandGroup(
            new AutonomousFollower(drive, lights, peripherals, pathJSON2, 0, false, false),
            new TurnToTarget(drive, peripherals)
          ),
          new AutoPrepForShot(shooter, tof, 0, 5000)
        )
      ),
      new ParallelDeadlineGroup(
        new AutoShoot(drive, shooter, feeder, peripherals, lights, tof, 1200, 1),
        new SetCarriage(climber, Constants.SetPoints.CarriageRotation.kDOWN, 10, 0.1, false)
      ),
      new ParallelDeadlineGroup(
        new AutoIntake(intake, feeder, climber, lights, tof, proximity, Constants.SetPoints.IntakePosition.kDOWN, 1200, 500, 3),
        new ParallelCommandGroup(
          new SequentialCommandGroup(
            new AutonomousFollower(drive, lights, peripherals, pathJSON3, 0, false, false),
            new TurnToTarget(drive, peripherals)
          ),
          new AutoPrepForShot(shooter, tof, 0, 5000)
        )
      ),
      new ParallelDeadlineGroup(
        new AutoShoot(drive, shooter, feeder, peripherals, lights, tof, 1200, 1),
        new SetCarriage(climber, Constants.SetPoints.CarriageRotation.kDOWN, 10, 0.1, false)
      ),
      new ParallelDeadlineGroup(
        new AutonomousFollower(drive, lights, peripherals, pathJSON4, 0, false, false),
        new AutoIntake(intake, feeder, climber, lights, tof, proximity, Constants.SetPoints.IntakePosition.kDOWN, 1200, 500),
        new AutoPrepForShot(shooter, tof, 0, 7000)
      ),
      new ParallelDeadlineGroup(
        new AutoShoot(drive, shooter, feeder, peripherals, lights, tof, 1200, 0.75),
        new SetCarriage(climber, Constants.SetPoints.CarriageRotation.kDOWN, 10, 0.1, false)
      ),

      //End
      new ParallelCommandGroup(
        new RunFeeder(feeder, 0),
        new IdleShooter(shooter, 0),
        new RunIntake(intake, 0, 0),
        new StopDriving(drive)
      ),
      new WaitCommand(3)
    );
  }
}
