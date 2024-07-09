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
import frc.robot.commands.AddVisionMeasurement;
import frc.robot.commands.AutoIntake;
import frc.robot.commands.AutoPrepForShot;
import frc.robot.commands.AutoPositionalShoot;
import frc.robot.commands.AutonomousFollower;
import frc.robot.commands.IdleShooter;
import frc.robot.commands.RunFeeder;
import frc.robot.commands.RunIntake;
import frc.robot.commands.SetCarriage;
import frc.robot.commands.StopDriving;
import frc.robot.sensors.Proximity;
import frc.robot.sensors.TOF;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Lights;
import frc.robot.subsystems.Peripherals;
import frc.robot.subsystems.Shooter;

public class FourPieceAmpSideAuto extends SequentialCommandGroup {
  private File pathingFile0;
  private JSONArray pathJSON0;
  private JSONObject pathRead0;

  private File pathingFile;
  private JSONArray pathJSON;
  private JSONObject pathRead;

  private File pathingFile2;
  private JSONArray pathJSON2;
  private JSONObject pathRead2;

  private File pathingFile3;
  private JSONArray pathJSON3;
  private JSONObject pathRead3;

  public FourPieceAmpSideAuto(Drive drive, Peripherals peripherals, Intake intake, Feeder feeder, Shooter shooter, Climber climber, Lights lights, TOF tof, Proximity proximity) {
    try {
      pathingFile0 = new File("/home/lvuser/deploy/3PieceAmpSidePart0.json");
      FileReader scanner0 = new FileReader(pathingFile0);
      pathRead0 = new JSONObject(new JSONTokener(scanner0));
      pathJSON0 = (JSONArray) pathRead0.get("sampled_points");
    }
    catch(Exception e) {
      System.out.println("ERROR WITH PATH FILE " + e);
    }
    
    try {
      pathingFile = new File("/home/lvuser/deploy/3PieceAmpSidePart1.json");
      FileReader scanner = new FileReader(pathingFile);
      pathRead = new JSONObject(new JSONTokener(scanner));
      pathJSON = (JSONArray) pathRead.get("sampled_points");
    }
    catch(Exception e) {
      System.out.println("ERROR WITH PATH FILE " + e);
    }

    try {
      pathingFile2 = new File("/home/lvuser/deploy/3PieceAmpSidePart2.json");
      FileReader scanner2 = new FileReader(pathingFile2);
      pathRead2 = new JSONObject(new JSONTokener(scanner2));
      pathJSON2 = (JSONArray) pathRead2.get("sampled_points");
    }
    catch(Exception e) {
      System.out.println("ERROR WITH PATH FILE " + e);
    }

    try {
      pathingFile3 = new File("/home/lvuser/deploy/4PieceAmpSidePart3.json");
      FileReader scanner3 = new FileReader(pathingFile3);
      pathRead3 = new JSONObject(new JSONTokener(scanner3));
      pathJSON3 = (JSONArray) pathRead3.get("sampled_points");
    }
    catch(Exception e) {
      System.out.println("ERROR WITH PATH FILE " + e);
    }
    addCommands(
      new ParallelDeadlineGroup(
        new AutonomousFollower(drive, lights, peripherals, pathJSON0, 0, false, false, 0, proximity),
        // new LineUpWhilePathing(drive, lights, peripherals, pathJSON, 0, false, false, 0, proximity),
        new AutoPrepForShot(shooter, proximity, 30, 6000)
      ),
      new AddVisionMeasurement(drive, peripherals),
      new ParallelDeadlineGroup(
        new AutoPositionalShoot(drive, shooter, feeder, peripherals, lights, proximity, 1200, 30, 6000, 2, true),
        new SetCarriage(climber, Constants.SetPoints.CarriageRotation.kDOWN, 30, 0.1, false)
      ),
      new ParallelDeadlineGroup(
        new AutonomousFollower(drive, lights, peripherals, pathJSON, 0, false, false, 0, proximity),
        // new LineUpWhilePathing(drive, lights, peripherals, pathJSON, 0, false, false, 0, proximity),
        new SequentialCommandGroup(
          new AutoIntake(intake, feeder, climber, lights, tof, proximity, Constants.SetPoints.IntakePosition.kDOWN, 1200, 600, false, false),
          new SetCarriage(climber, Constants.SetPoints.CarriageRotation.kDOWN, 10, 0.1, false)
        ),
        new AutoPrepForShot(shooter, proximity, 30, 6000)
      ),
      new AddVisionMeasurement(drive, peripherals),
      new ParallelDeadlineGroup(
        new AutoPositionalShoot(drive, shooter, feeder, peripherals, lights, proximity, 1200, 30, 6000, 2, true),
        new SetCarriage(climber, Constants.SetPoints.CarriageRotation.kDOWN, 30, 0.1, false)
      ),
      new ParallelDeadlineGroup(
        new AutonomousFollower(drive, lights, peripherals, pathJSON3, 0, false, false, 0, proximity),
        // new LineUpWhilePathing(drive, lights, peripherals, pathJSON2, 0, false, false, 0, proximity),
        new SequentialCommandGroup(
          new AutoIntake(intake, feeder, climber, lights, tof, proximity, Constants.SetPoints.IntakePosition.kDOWN, 1200, 600, false, false),
          new SetCarriage(climber, Constants.SetPoints.CarriageRotation.kDOWN, 10, 0.1, false)
        ),
        new AutoPrepForShot(shooter, proximity, 30, 6000)
      ),
      new AddVisionMeasurement(drive, peripherals),
      new ParallelDeadlineGroup(
        new AutoPositionalShoot(drive, shooter, feeder, peripherals, lights, proximity, 1200, 30, 6000, 2, true),
        new SetCarriage(climber, Constants.SetPoints.CarriageRotation.kDOWN, 30, 0.1, false)
      ),
      new ParallelDeadlineGroup(
        new AutonomousFollower(drive, lights, peripherals, pathJSON2, 0, false, false, 0, proximity),
        // new LineUpWhilePathing(drive, lights, peripherals, pathJSON3, 0, false, false, 0, proximity),
        new SequentialCommandGroup(
          new AutoIntake(intake, feeder, climber, lights, tof, proximity, Constants.SetPoints.IntakePosition.kDOWN, 1200, 600, false, false),
          new SetCarriage(climber, Constants.SetPoints.CarriageRotation.kDOWN, 30, 0.1, false)
        ),
        new AutoPrepForShot(shooter, proximity, 30, 6000)
      ),
      new AddVisionMeasurement(drive, peripherals),
      new ParallelDeadlineGroup(
        new AutoPositionalShoot(drive, shooter, feeder, peripherals, lights, proximity, 1200, 30, 6000, 2, true),
        new SetCarriage(climber, Constants.SetPoints.CarriageRotation.kDOWN, 30, 0.1, false)
      ),
      // new ParallelDeadlineGroup(
      //   // new AutonomousFollower(drive, lights, peripherals, pathJSON3, 0, false, false, 3.25, proximity),
      //   new LineUpWhilePathing(drive, lights, peripherals, pathJSON4, 0, false, false, 0, proximity),
      //   new SequentialCommandGroup(
      //     new AutoIntake(intake, feeder, climber, lights, tof, proximity, Constants.SetPoints.IntakePosition.kDOWN, 1200, 600, false, false),
      //     new SetCarriage(climber, Constants.SetPoints.CarriageRotation.kDOWN, 30, 0.1, false)
      //   ),
      //   new AutoPrepForShot(shooter, proximity, 28, 6000)
      // ),
      // new ParallelDeadlineGroup(
      //   new AutoPositionalShoot(drive, shooter, feeder, peripherals, lights, proximity, 1200, 28, 6000, 2),
      //   new SetCarriage(climber, Constants.SetPoints.CarriageRotation.kDOWN, 30, 0.1, false)
      // ),

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
