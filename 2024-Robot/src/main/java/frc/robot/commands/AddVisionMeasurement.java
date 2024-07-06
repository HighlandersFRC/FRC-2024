package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Peripherals;

public class AddVisionMeasurement extends Command {
  Drive drive;
  Peripherals peripherals;
  double robotAngle;
  boolean localized;
  public AddVisionMeasurement(Drive drive, Peripherals peripherals) {
    this.drive = drive;
    this.peripherals = peripherals;
  }

  @Override
  public void initialize() {
    localized = false;
    robotAngle = 0;
  }

  @Override
  public void execute() {
    System.out.println("localized: " + localized);
    robotAngle = peripherals.getPigeonAngle();
    LimelightHelpers.SetRobotOrientation("limelight-front", robotAngle, 0, 0, 0, 0, 0);
    LimelightHelpers.PoseEstimate mt2Front = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-front");
    if(Math.abs(peripherals.getPigeonAngularVelocity()) < 15) {
      if (mt2Front.tagCount != 0){
        System.out.println("adding");
        drive.addVisionMeasurementToOdometry(mt2Front.pose, mt2Front.timestampSeconds);
        localized = true;
      }
    }
  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    if (localized){
      return true;
    } else {
      return false;
    }
  }
}
