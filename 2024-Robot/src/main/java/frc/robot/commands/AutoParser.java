package frc.robot.commands;

import java.io.File;
import java.io.FileReader;
import java.util.ArrayList;

import org.json.JSONArray;
import org.json.JSONObject;
import org.json.JSONTokener;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.templates.ArrayListSequentialCommandGroup;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Peripherals;
import frc.robot.subsystems.Shooter;
public class AutoParser extends ArrayListSequentialCommandGroup {
  private Drive drive;
  private Intake intake;
  private Feeder feeder;
  private Shooter shooter;
  private Peripherals peripherals;

  private double autoInitTime = Timer.getFPGATimestamp();
  private String pathName;

  private File pathingFile;
  private JSONObject pathJSON;
  private JSONArray sampledPoints;
  private JSONArray pathCommands;

  private ArrayList<Command> commands = new ArrayList<Command>();

  public AutoParser(Drive drive, Intake intake, Feeder feeder, Shooter shooter, Peripherals peripherals, String pathName) {
    
    this.drive = drive;
    this.intake = intake;
    this.feeder = feeder;
    this.shooter = shooter;
    this.peripherals = peripherals;

    this.pathName = pathName;

    try {
      this.pathingFile = new File("/home/lvuser/deploy/" + this.pathName + ".json");
      FileReader scanner = new FileReader(this.pathingFile);
      this.pathJSON = new JSONObject(new JSONTokener(scanner));
      this.sampledPoints = (JSONArray) this.pathJSON.get("sampled_points");
      this.pathCommands = (JSONArray) this.pathJSON.get("commands");
    } catch(Exception e) {
      System.out.println("whoopsies... " + e);
    }
    
    addRequirements(drive, intake, feeder, shooter, peripherals);

    
  }
}
