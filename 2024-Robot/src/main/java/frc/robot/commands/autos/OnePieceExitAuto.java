package frc.robot.commands.autos;

import java.io.File;
import java.io.FileReader;

import org.json.JSONArray;
import org.json.JSONObject;
import org.json.JSONTokener;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.AutoShoot;
import frc.robot.commands.AutonomousFollower;
import frc.robot.sensors.Proximity;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Lights;
import frc.robot.subsystems.Peripherals;
import frc.robot.subsystems.Shooter;

public class OnePieceExitAuto extends SequentialCommandGroup {
  private File pathingFile;
  private JSONArray pathJSON;
  private JSONObject pathRead;

  public OnePieceExitAuto(Drive drive, Peripherals peripherals, Feeder feeder, Shooter shooter, Lights lights, Proximity proximity) {
    try {
      pathingFile = new File("/home/lvuser/deploy/1PieceExit.json");
      FileReader scanner = new FileReader(pathingFile);
      pathRead = new JSONObject(new JSONTokener(scanner));
      pathJSON = (JSONArray) pathRead.get("sampled_points");
    }
    catch(Exception e) {
      System.out.println("ERROR WITH PATH FILE " + e);
    }

    addCommands(
      new WaitCommand(8),
      new AutonomousFollower(drive, lights, peripherals, pathJSON, 0, false, false, 0, proximity),
      new AutoShoot(drive, shooter, feeder, peripherals, lights, proximity, 1200, 40, 5500, 3, true)
    );
  }
}
