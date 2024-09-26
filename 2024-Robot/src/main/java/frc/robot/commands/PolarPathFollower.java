// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import org.json.JSONArray;
import org.json.JSONObject;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Lights;
import frc.robot.subsystems.Peripherals;
import frc.robot.tools.wrappers.PolarTakeDrive;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class PolarPathFollower extends ParallelCommandGroup {
  /** Creates a new PolarPathFollower. */
  PolarTakeDrive follower;
  PurePursuitFollower defaultFollower;
  double endTime = 0, startTime = 0;
  boolean timerStarted = false;
  JSONObject pathJSON;
  TriggerCommand followerCommand;

  public PolarPathFollower(Drive drive, Lights lights, Peripherals peripherals, JSONObject pathJSON,
      HashMap<String, Supplier<Command>> commandMap, HashMap<String, BooleanSupplier> conditionMap) {
    defaultFollower = new PurePursuitFollower(drive, lights, peripherals, pathJSON.getJSONArray("sampled_points"),
        false);
    startTime = pathJSON.getJSONArray("sampled_points").getJSONObject(0).getDouble("time");
    follower = defaultFollower;
    followerCommand = new TriggerCommand(
        () -> startTime <= getPathTime(),
        follower,
        () -> false);
    this.pathJSON = pathJSON;
    ArrayList<Command> commands = new ArrayList<>();
    commands.add(followerCommand);
    for (int i = 0; i < pathJSON.getJSONArray("commands").length(); i++) {
      JSONObject command = pathJSON.getJSONArray("commands").getJSONObject(i);
      commands.add(addCommandsFromJSON(command, commandMap, conditionMap));
    }
    for (Command command : commands) {
      addCommands(command);
    }
  }

  private Command addCommandsFromJSON(JSONObject command, HashMap<String, Supplier<Command>> commandMap,
      HashMap<String, BooleanSupplier> conditionMap) {
    if (command.has("command")) {
      return singleCommandFromJSON(command, commandMap);
    } else if (command.has("branched_command")) {
      BooleanSupplier startSupplier = () -> command.getDouble("start") < getPathTime();
      BooleanSupplier endSupplier = () -> command.getDouble("end") <= getPathTime();
      JSONObject onTrue = command.getJSONObject("branched_command").getJSONObject("on_true");
      JSONObject onFalse = command.getJSONObject("branched_command").getJSONObject("on_true");
      BooleanSupplier condition = conditionMap.get(command.getJSONObject("branched_command").getString("condition"));
      return new TriggerCommand(
          startSupplier,
          new ConditionalCommand(
              addCommandsFromJSON(onTrue, commandMap, conditionMap),
              addCommandsFromJSON(onFalse, commandMap, conditionMap),
              condition),
          endSupplier);
    } else if (command.has("parallel_command_group")) {
      ArrayList<Command> commands = new ArrayList<Command>();
      for (int i = 0; i < command.getJSONObject("parallel_command_group").getJSONArray("commands").length(); i++) {
        commands.add(addCommandsFromJSON(
            command.getJSONObject("parallel_command_group").getJSONArray("commands").getJSONObject(i), commandMap,
            conditionMap));
      }
      return new TriggerCommand(() -> command.getDouble("start") < getPathTime(),
          new ParallelCommandGroup(
              commands.toArray(new Command[0])),
          () -> command.getDouble("end") <= getPathTime());
    } else if (command.has("parallel_deadline_group")) {
      Command deadlineCommand = addCommandsFromJSON(
          command.getJSONObject("parallel_deadline_group").getJSONArray("commands").getJSONObject(0), commandMap,
          conditionMap);
      ArrayList<Command> commands = new ArrayList<Command>();
      for (int i = 1; i < command.getJSONObject("parallel_deadline_group").getJSONArray("commands").length(); i++) {
        commands.add(addCommandsFromJSON(
            command.getJSONObject("parallel_deadline_group").getJSONArray("commands").getJSONObject(i), commandMap,
            conditionMap));
      }
      return new TriggerCommand(() -> command.getDouble("start") < getPathTime(),
          new ParallelDeadlineGroup(
              deadlineCommand,
              commands.toArray(new Command[0])),
          () -> command.getDouble("end") <= getPathTime());
    } else if (command.has("parallel_command_group")) {
      ArrayList<Command> commands = new ArrayList<Command>();
      for (int i = 0; i < command.getJSONObject("parallel_race_group").getJSONArray("commands").length(); i++) {
        commands.add(addCommandsFromJSON(
            command.getJSONObject("parallel_race_group").getJSONArray("commands").getJSONObject(i), commandMap,
            conditionMap));
      }
      return new TriggerCommand(() -> command.getDouble("start") < getPathTime(),
          new ParallelRaceGroup(
              commands.toArray(new Command[0])),
          () -> command.getDouble("end") <= getPathTime());
    } else if (command.has("sequential_command_group")) {
      ArrayList<Command> commands = new ArrayList<Command>();
      for (int i = 0; i < command.getJSONObject("sequential_command_group").getJSONArray("commands").length(); i++) {
        commands.add(addCommandsFromJSON(
            command.getJSONObject("sequential_command_group").getJSONArray("commands").getJSONObject(i), commandMap,
            conditionMap));
      }
      return new TriggerCommand(() -> command.getDouble("start") < getPathTime(),
          new SequentialCommandGroup(
              commands.toArray(new Command[0])),
          () -> command.getDouble("end") <= getPathTime());
    } else {
      throw new IllegalArgumentException("Invalid command JSON: " + command.toString());
    }
  }

  private Command singleCommandFromJSON(JSONObject command, HashMap<String, Supplier<Command>> commandMap) {
    Command runner = commandMap.get(command.getJSONObject("command").getString("name")).get();
    BooleanSupplier startSupplier = () -> command.getDouble("start") < getPathTime();
    BooleanSupplier endSupplier = () -> command.getDouble("end") <= getPathTime();
    if (runner instanceof PolarTakeDrive) {
      Runnable cancelPathFollower = new Runnable() {
        public void run(){
          int runFrom = getPointIndexFromTime(command.getDouble("start"));
          int runTo = getPointIndexFromTime(command.getDouble("end"));
          follower.cancel();
          follower = (PolarTakeDrive) runner;
          follower.from(runFrom, pathJSON, runTo);
        }
      };
      runner.beforeStarting(cancelPathFollower);
      Runnable cancelRunner = new Runnable(){
        public void run(){
          int runFrom = getPointIndexFromTime(command.getDouble("end"));
          int runTo = pathJSON.getJSONArray("sampled_points").length()-1;
          follower.cancel();
          follower = defaultFollower;
          follower.from(runFrom, pathJSON, runTo);
        }
      };
      runner.andThen(cancelRunner);
    }
    return new TriggerCommand(startSupplier, runner,
        endSupplier);
  }

  double getPathTime() {
    double retval;
    if (follower.isFinished() || !follower.isScheduled()) {
      if (!timerStarted) {
        endTime = Timer.getFPGATimestamp();
        timerStarted = true;
      }
      retval = Timer.getFPGATimestamp() - endTime;
      if (follower.isFinished()) {
        retval += pathJSON.getJSONArray("sampled_points")
            .getJSONObject(pathJSON.getJSONArray("sampled_points").length() - 1).getDouble("time");
      }
    } else {
      timerStarted = false;
      retval = pathJSON.getJSONArray("sampled_points")
          .getJSONObject(follower.getPathPointIndex()).getDouble("time");
    }
    Logger.recordOutput("Path Time", retval);
    Logger.recordOutput("Path Start Time", defaultFollower.pathStartTime);
    return retval;
  }

  private int getPointIndexFromTime(double time) {
    JSONArray points = pathJSON.getJSONArray("sampled_points");
    for (int i = 0; i < points.length(); i++) {
      JSONObject point = points.getJSONObject(i);
      if (time >= point.getDouble("time")) {
        return i;
      }
    }
    return points.length() - 1;
  }
}
