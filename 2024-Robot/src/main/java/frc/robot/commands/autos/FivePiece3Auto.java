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
import frc.robot.commands.PresetAutoShoot;
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

public class FivePiece3Auto extends SequentialCommandGroup {
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

  private File pathingFile5;
  private JSONArray pathJSON5;
  private JSONObject pathRead5;

  /** Creates a new FivePieceAuto. */
  public FivePiece3Auto(Drive drive, Peripherals peripherals, Intake intake, Feeder feeder, Shooter shooter, Climber climber, Lights lights, TOF tof, Proximity proximity) {
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
      pathingFile4 = new File("/home/lvuser/deploy/5Piece3Note.json");
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
        new PresetAutoShoot(drive, shooter, feeder, peripherals, lights, proximity, 58, 4500, 1200, 0),
        new RunIntake(intake, Constants.SetPoints.IntakePosition.kDOWN, 1200),
        new SetCarriage(climber, Constants.SetPoints.CarriageRotation.kDOWN, 30, 0.1, false)
      ),
      new ParallelDeadlineGroup(
        new AutoIntake(intake, feeder, climber, lights, tof, proximity, Constants.SetPoints.IntakePosition.kDOWN, 1200, 500, 3, false, false),
        new ParallelCommandGroup(
          new SequentialCommandGroup(
            new AutonomousFollower(drive, lights, peripherals, pathJSON, 0, false, false, 0, proximity)
          ),
          new AutoPrepForShot(shooter, proximity, 38, 5500)
        )
      ),
      new ParallelDeadlineGroup(
        new AutoPositionalShoot(drive, shooter, feeder, peripherals, lights, proximity, 1200, 38, 5500, 2, true),
        new AddVisionMeasurement(drive, peripherals),
        new SetCarriage(climber, Constants.SetPoints.CarriageRotation.kDOWN, 30, 0.1, false)
      ),
      new ParallelDeadlineGroup(
        new AutoIntake(intake, feeder, climber, lights, tof, proximity, Constants.SetPoints.IntakePosition.kDOWN, 1200, 500, 3, false, false),
        new ParallelCommandGroup(
          new SequentialCommandGroup(
            new AutonomousFollower(drive, lights, peripherals, pathJSON2, 0, false, false, 0, proximity)
          ),
          new AutoPrepForShot(shooter, proximity, 38, 5500)
        )
      ),
      new ParallelDeadlineGroup(
        new AutoPositionalShoot(drive, shooter, feeder, peripherals, lights, proximity, 1200, 38, 5500, 2, true),
        new AddVisionMeasurement(drive, peripherals),
        new SetCarriage(climber, Constants.SetPoints.CarriageRotation.kDOWN, 30, 0.1, false)
      ),
      new ParallelDeadlineGroup(
        new AutoIntake(intake, feeder, climber, lights, tof, proximity, Constants.SetPoints.IntakePosition.kDOWN, 1200, 500, 3, false, false),
        new ParallelCommandGroup(
          new SequentialCommandGroup(
            new AutonomousFollower(drive, lights, peripherals, pathJSON3, 0, false, false, 0, proximity)
          ),
          new AutoPrepForShot(shooter, proximity, 38, 5500)
        )
      ),
      new ParallelDeadlineGroup(
        new AutoPositionalShoot(drive, shooter, feeder, peripherals, lights, proximity, 1200, 38, 5500, 2, true),
        new AddVisionMeasurement(drive, peripherals),
        new SetCarriage(climber, Constants.SetPoints.CarriageRotation.kDOWN, 30, 0.1, false)
      ),
      new ParallelDeadlineGroup(
        new AutonomousFollower(drive, lights, peripherals, pathJSON4, 0, false, false, 0, proximity),
        new AutoIntake(intake, feeder, climber, lights, tof, proximity, Constants.SetPoints.IntakePosition.kDOWN, 1200, 500, false, false),
        new AutoPrepForShot(shooter, proximity, 25, 7000)
      ),
      new ParallelDeadlineGroup(
        new AutoPositionalShoot(drive, shooter, feeder, peripherals, lights, proximity, 1200, 25, 6500, 0.75, true),
        new AddVisionMeasurement(drive, peripherals),
        new RunIntake(intake, 0, 0),
        new SetCarriage(climber, Constants.SetPoints.CarriageRotation.kDOWN, 30, 0.1, false)
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
