package frc.robot.commands;

import java.io.File;
import java.io.FileReader;
import java.util.ArrayList;

import org.json.JSONArray;
import org.json.JSONObject;
import org.json.JSONTokener;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.commands.templates.ArrayListParallelDeadlineGroup;
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

  private ArrayList<Command> autoCommands = new ArrayList<Command>();

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

    this.drive.autoInit(this.sampledPoints);

    //Path parsing and command construction
    JSONArray sortedHaltingPathCommands = new JSONArray();
    JSONArray sortedNonHaltingPathCommands = new JSONArray();
    for (int i  = 0; i < this.pathCommands.length(); i ++){
      JSONObject pathCommand = (JSONObject) this.pathCommands.get(i);
      double time = pathCommand.getDouble("trigger");
      boolean halting = pathCommand.getBoolean("halting");
      if (halting){
        for (int j = 0; j < sortedHaltingPathCommands.length(); j ++){
          if (time < ((JSONObject) sortedHaltingPathCommands.get(j)).getDouble("trigger") || j == sortedHaltingPathCommands.length() - 1){
            sortedHaltingPathCommands.put(j, pathCommand);
            break;
          }
        }
      } else {
        for (int j = 0; j < sortedNonHaltingPathCommands.length(); j ++){
          if (time < ((JSONObject) sortedNonHaltingPathCommands.get(j)).getDouble("trigger") || j == sortedNonHaltingPathCommands.length() - 1){
            sortedNonHaltingPathCommands.put(j, pathCommand);
            break;
          }
        }
      }
    }
    double previousHaltingTime = 0;
    for (int i = 0; i < sortedHaltingPathCommands.length(); i ++){
      JSONObject haltingPathCommand = (JSONObject) sortedHaltingPathCommands.get(i);
      double haltingTime = haltingPathCommand.getDouble("trigger");
      String haltingType = haltingPathCommand.getString("type");

      ArrayList<Command> parallelCommands = new ArrayList<Command>();

      for (int j = 0; j < sortedNonHaltingPathCommands.length(); j ++){
        JSONObject nonHaltingCommand = (JSONObject) sortedNonHaltingPathCommands.get(j);
        double nonHaltingTime = nonHaltingCommand.getDouble("trigger");
        String nonHaltingType = nonHaltingCommand.getString("type");
        if (nonHaltingTime < haltingTime){
          switch (nonHaltingType){
            case "shoot":
              parallelCommands.add(
                new SequentialCommandGroup(
                  new WaitCommand(nonHaltingTime - previousHaltingTime),
                  new RunShooter(shooter, 0, 4000)
                )
              );
              break;
            case "intake_down":
              parallelCommands.add(
                new SequentialCommandGroup(
                  new WaitCommand(nonHaltingTime - previousHaltingTime),
                  new RunIntake(intake, Constants.SetPoints.IntakePosition.kDOWN, 1000)
                )
              );
              break;
            case "intake_up":
              parallelCommands.add(
                new SequentialCommandGroup(
                  new WaitCommand(nonHaltingTime - previousHaltingTime),
                  new RunIntake(intake, Constants.SetPoints.IntakePosition.kUP, 0)
                )
              );
              break;
            default:
          }
        }
      }
      ArrayListParallelDeadlineGroup parallelDeadlineGroup = new ArrayListParallelDeadlineGroup(
        new AutonomousFollower(drive, this.sampledPoints, previousHaltingTime, haltingTime, false),
        parallelCommands
      );
      this.autoCommands.add(parallelDeadlineGroup);
      switch (haltingType){
            case "shoot":
              parallelCommands.add(
                new RunShooter(shooter, 0, 4000)
              );
              break;
            case "intake_down":
              parallelCommands.add(
                new RunIntake(intake, Constants.SetPoints.IntakePosition.kDOWN, 1000)
              );
              break;
            case "intake_up":
              parallelCommands.add(
                new RunIntake(intake, Constants.SetPoints.IntakePosition.kUP, 0)
              );
              break;
            default:
          }

      previousHaltingTime = haltingTime;
    }
    this.autoCommands.add(
      new AutonomousFollower(drive, this.sampledPoints, previousHaltingTime, false)
    );

    addCommands(this.autoCommands);
  }
}
