// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

public final class Constants {

  //Physical constants (e.g. field and robot dimensions)
  public static final class Physical {
    public static final double FIELD_WIDTH = 8.2;
    public static final double FIELD_LENGTH = 16.63;
    public static final double ROBOT_RADIUS = inchesToMeters(15.375);
    public static final double WHEEL_DIAMETER = inchesToMeters(4);
    public static final double WHEEL_CIRCUMFERENCE = Math.PI * WHEEL_DIAMETER;
    public static final double WHEEL_ROTATION_PER_METER = 1 / WHEEL_CIRCUMFERENCE;

    public static final double TOP_SPEED = feetToMeters(20);

    public static final double ROBOT_WIDTH = inchesToMeters(25);
    public static final double ROBOT_LENGTH = inchesToMeters(29);
    public static final double MODULE_OFFSET = inchesToMeters(2.5);

    public static final double SHOOTER_RESTING_ANGLE_DEG = 8;
  }

  //Subsystem setpoint constants
  public static final class SetPoints {
    //drive

    //intake
    public static final double INTAKE_DOWN_ANGLE_ROT = -0.375;
    public static final double INTAKE_UP_ANGLE_ROT = 0;
    public static final double INTAKE_DOWN_ANGLE_DEG = rotationsToDegrees(INTAKE_DOWN_ANGLE_ROT);
    public static final double INTAKE_UP_ANGLE_DEG = rotationsToDegrees(INTAKE_UP_ANGLE_ROT);

    //Intake up and down positions
    public enum IntakePosition {
      kDOWN(INTAKE_DOWN_ANGLE_DEG, INTAKE_DOWN_ANGLE_ROT), kUP(INTAKE_UP_ANGLE_DEG, INTAKE_UP_ANGLE_ROT);

      public final double degrees;
      public final double rotations;

      private IntakePosition(double degrees, double rotations){
        this.degrees = degrees;
        this.rotations = rotations;
      }
    }

    //shooter
    public static final double SHOOTER_DOWN_ANGLE_ROT = 0.02222;
    public static final double SHOOTER_MAX_ANGLE_ROT = 0.18;
    public static final double SHOOTER_DOWN_ANGLE_DEG = rotationsToDegrees(SHOOTER_DOWN_ANGLE_ROT);
    public static final double SHOOTER_MAX_ANGLE_DEG = rotationsToDegrees(SHOOTER_MAX_ANGLE_ROT);

    // {distance(inches), target angle(deg), hood angle(deg), RPM}
    public static final double [][] SHOOTING_LOOKUP_TABLE = {
      {1.4352, 13.80, 52, 4500},
      {2.2987, -3.64, 42, 5000},
      {3.3655, -12.68, 31, 5500},
      {3.6195, -19.93, 26, 6000},
      {5.9817, -23.34, 22, 6500}
    };

    public static double[] getShooterValues(double angle) {
        int lastIndex = SHOOTING_LOOKUP_TABLE.length - 1;
        if (angle > SHOOTING_LOOKUP_TABLE[0][1]) {
            //If the angle is closer than the first setpoint
            double[] returnArr = {SHOOTING_LOOKUP_TABLE[0][2], SHOOTING_LOOKUP_TABLE[0][3]};
            return returnArr;
        } else if (angle < SHOOTING_LOOKUP_TABLE[lastIndex][1]) {
            //If the angle is farther than the last setpoint
            double[] returnArr = {SHOOTING_LOOKUP_TABLE[lastIndex][2], SHOOTING_LOOKUP_TABLE[lastIndex][3]};
            return returnArr;
        } else {
            for (int i = 0; i < SHOOTING_LOOKUP_TABLE.length; i ++) {
                if (angle < SHOOTING_LOOKUP_TABLE[i][1] && angle > SHOOTING_LOOKUP_TABLE[i + 1][1]) {
                    //If the angle is in the table of setpoints
                    //Calculate where angle is between setpoints
                    double leftDif = angle - SHOOTING_LOOKUP_TABLE[i][1];
                    double percent = leftDif / (SHOOTING_LOOKUP_TABLE[i + 1][1] - SHOOTING_LOOKUP_TABLE[i][1]);

                    double hood1 = SHOOTING_LOOKUP_TABLE[i][2];
                    double rpm1 = SHOOTING_LOOKUP_TABLE[i][3];
                    double hood2 = SHOOTING_LOOKUP_TABLE[i + 1][2];
                    double rpm2 = SHOOTING_LOOKUP_TABLE[i + 1][3];

                    //Interpolate in-between values for hood angle and shooter rpm
                    double newHood = hood1 + (percent * (hood2 - hood1));
                    double newRPM = rpm1 + (percent * (rpm2 - rpm1));
                    
                    double[] returnArr = {newHood, newRPM};
                    return returnArr;
                }
            }
            //Should never run
            double[] returnArr = {0, 0};
            return returnArr;
        }  
    }

    //feeder

    //TOF
    public static final double FEEDER_TOF_THRESHOLD_MM = 150;
  }

  //Vision constants (e.g. camera offsets)
  public static final class Vision {
    //Poses of all 16 AprilTags, {x, y, z, theta}, in meters and radians
    public static final double[][] TAG_POSES = {
      {15.079502159004317, 0.2458724917449835, 1.3558547117094235, 2.0943951023931953},
      {16.18516637033274, 0.8836677673355348, 1.3558547117094235, 2.0943951023931953},  
      {16.57937515875032, 4.982727965455931, 1.4511049022098046, 3.141592653589793},    
      {16.57937515875032, 5.547879095758192, 1.4511049022098046, 3.141592653589793},    
      {14.700787401574804, 8.204216408432817, 1.3558547117094235, 4.71238898038469},    
      {1.841503683007366, 8.204216408432817, 1.3558547117094235, 4.71238898038469},     
      {-0.038100076200152405, 5.547879095758192, 1.4511049022098046, 0.0},
      {-0.038100076200152405, 4.982727965455931, 1.4511049022098046, 0.0},
      {0.35610871221742446, 0.8836677673355348, 1.3558547117094235, 1.0471975511965976},
      {1.4615189230378463, 0.2458724917449835, 1.3558547117094235, 1.0471975511965976}, 
      {11.90474980949962, 3.713233426466853, 1.3208026416052834, 5.235987755982989},    
      {11.90474980949962, 4.4983489966979935, 1.3208026416052834, 1.0471975511965976},  
      {11.220218440436883, 4.105156210312421, 1.3208026416052834, 3.141592653589793},   
      {5.320802641605283, 4.105156210312421, 1.3208026416052834, 0.0},
      {4.641351282702566, 4.4983489966979935, 1.3208026416052834, 2.0943951023931953},  
      {4.641351282702566, 3.713233426466853, 1.3208026416052834, 4.1887902047863905}
    };

    //Field of view angles
    public static final double LIMELIGHT_HFOV_DEG = 63.3;
    public static final double LIMELIGHT_VFOV_DEG = 49.7;
    public static final double LIMELIGHT_HFOV_RAD = LIMELIGHT_HFOV_DEG * Math.PI / 180;
    public static final double LIMELIGHT_VFOV_RAD = LIMELIGHT_VFOV_DEG * Math.PI / 180;

    //Camera IDs
    public static final int FRONT_CAMERA_ID = 4;
    public static final int BACK_CAMERA_ID = 3;
    public static final int LEFT_CAMERA_ID = 1;
    public static final int RIGHT_CAMERA_ID = 2;

    //Poses of cameras relative to robot, {x, y, z, rx, ry, rz}, in meters and radians
    public static final double[] FRONT_CAMERA_POSE = {0.3683, -0.01905, 0.23495, 0, degreesToRadians(33), 0};
    public static final double[] LEFT_CAMERA_POSE = {0.0172, 0.3429, 0.23495, 0, degreesToRadians(33), degreesToRadians(90)};
    public static final double[] RIGHT_CAMERA_POSE = {0.073025, -0.3429, 0.23495, 0, degreesToRadians(33), degreesToRadians(270)};
    public static final double[] BACK_CAMERA_POSE = {0, 0, 0, 0, 0, degreesToRadians(180)};
    public static final double[] FRONT_CAMERA_POSITION_POLAR = {getDistance(0, 0, FRONT_CAMERA_POSE[0], FRONT_CAMERA_POSE[1]), Math.atan2(FRONT_CAMERA_POSE[1], FRONT_CAMERA_POSE[0])};
    public static final double[] LEFT_CAMERA_POSITION_POLAR = {getDistance(0, 0, LEFT_CAMERA_POSE[0], LEFT_CAMERA_POSE[1]), Math.atan2(LEFT_CAMERA_POSE[1], LEFT_CAMERA_POSE[0])};
    public static final double[] RIGHT_CAMERA_POSITION_POLAR = {getDistance(0, 0, RIGHT_CAMERA_POSE[0], RIGHT_CAMERA_POSE[1]), Math.atan2(RIGHT_CAMERA_POSE[1], RIGHT_CAMERA_POSE[0])};
    public static final double[] BACK_CAMERA_POSITION_POLAR = {getDistance(0, 0, BACK_CAMERA_POSE[0], BACK_CAMERA_POSE[1]), Math.atan2(BACK_CAMERA_POSE[1], BACK_CAMERA_POSE[0])};

    //Standard deviation adjustments
    public static final double STANDARD_DEVIATION_SCALAR = 1;
    public static final double ODOMETRY_JUMP_STANDARD_DEVIATION_SCALAR = 1;
    public static final double ODOMETRY_JUMP_STANDARD_DEVIATION_DEGREE = 3;
    public static final double TAG_STANDARD_DEVIATION_DISTANCE = 2; //meters
    public static final double TAG_STANDARD_DEVIATION_FLATNESS = 5;

    //Limelight settings
    /*
    
    Resolution: 960x720 40fps
    Exposure: 3000
    Black Level Offset: 8
    Sensor Gain: 10.5
    Red Balance: 1380
    Blue Balance: 1410
    AprilTag Family: 16h11
    AprilTag Size: 165
    Detector Downscale: 1.5
    Quality Threshold: 2
    */

    //Standard deviation regressions
    //Increases standard deviation with distance from tag
    public static double getTagDistStdDevScalar(double dist){
      double a = TAG_STANDARD_DEVIATION_FLATNESS;
      double b = 1 - a * Math.pow(TAG_STANDARD_DEVIATION_DISTANCE, 2);
      return Math.max(1, a * Math.pow(dist, 2) + b);
    }

    // Decreases standard deviation with more tag present in one MegaTag botpose
    public static double getNumTagStdDevScalar(int numTags){
      if (numTags == 0){
        return 99999;
      } else if (numTags == 1){
        return 2;
      } else if (numTags == 2){
        return 1;
      } else {
        return 0.75;
      }
    }

    //Regression for standard deviation in x for fiducial measurements
    public static double getTagStdDevX(double xOffset, double yOffset){
      return Math.max(0, 0.005533021491867763 * (xOffset * xOffset + yOffset * yOffset) - 0.010807566510145635) * STANDARD_DEVIATION_SCALAR;
    }

    //Regression for standard deviation in y for fiducial measurements
    public static double getTagStdDevY(double xOffset, double yOffset){
      return Math.max(0, 0.0055 * (xOffset * xOffset + yOffset * yOffset) - 0.01941597810542626) * STANDARD_DEVIATION_SCALAR;
    }

    //Regression for standard deviation in x for triangulation measurements
    public static double getTriStdDevX(double xOffset, double yOffset){
      return Math.max(0, 0.004544133588821881 * (xOffset * xOffset + yOffset * yOffset) - 0.01955724864971872) * STANDARD_DEVIATION_SCALAR;
    }

    //Regression for standard deviation in y for triangulation measurements
    public static double getTriStdDevY(double xOffset, double yOffset){
      return Math.max(0, 0.002615358015002413 * (xOffset * xOffset + yOffset * yOffset) - 0.008955462032388808) * STANDARD_DEVIATION_SCALAR;
    }
  }

  //Gear ratios and conversions
  public static final class Ratios {
    //drive
    public static final double DRIVE_GEAR_RATIO = 6.75;
    public static final double STEER_GEAR_RATIO = 21.43;

    //intake
    public static final double INTAKE_ANGLE_GEAR_RATIO = 25.0;
    public static final double INTAKE_ROLLER_GEAR_RATIO = 2.0 + 1.0 / 3.0;

    //shooter
    public static final double SHOOTER_ANGLE_GEAR_RATIO = 96.0;
    public static final double SHOOTER_FLYWHEEL_GEAR_RATIO = 1.0 / 2.5;

    //feeder
    public static final double FEEDER_ROLLER_GEAR_RATIO = 4.0 / 3.0;
  }

  //Can info such as IDs
  public static final class CANInfo {
    public static final String CANBUS_NAME = "Canivore";

    //drive
    public static final int FRONT_RIGHT_DRIVE_MOTOR_ID = 1;
    public static final int FRONT_RIGHT_ANGLE_MOTOR_ID = 2;
    public static final int FRONT_LEFT_DRIVE_MOTOR_ID = 3;
    public static final int FRONT_LEFT_ANGLE_MOTOR_ID = 4;
    public static final int BACK_LEFT_DRIVE_MOTOR_ID = 5;
    public static final int BACK_LEFT_ANGLE_MOTOR_ID = 6;
    public static final int BACK_RIGHT_DRIVE_MOTOR_ID = 7;
    public static final int BACK_RIGHT_ANGLE_MOTOR_ID = 8;
    public static final int FRONT_RIGHT_MODULE_CANCODER_ID = 1;
    public static final int FRONT_LEFT_MODULE_CANCODER_ID = 2;
    public static final int BACK_LEFT_MODULE_CANCODER_ID = 3;
    public static final int BACK_RIGHT_MODULE_CANCODER_ID = 4;

    //intake
    public static final int INTAKE_ANGLE_MOTOR_ID = 10;
    public static final int INTAKE_ROLLER_MOTOR_ID = 11;

    //shooter
    public static final int SHOOTER_ANGLE_MOTOR_ID = 12;
    public static final int SHOOTER_FLYWHEEL_MASTER_MOTOR_ID = 13;
    public static final int SHOOTER_FLYWHEEL_FOLLOWER_MOTOR_ID = 14;
    public static final int SHOOTER_ANGLE_CANCODER_ID = 5;

    //feeder
    public static final int FEEDER_ROLLER_MOTOR_ID = 15;

    //TOF
    public static final int FEEDER_TOF_ID = 0;

    //Climber
    public static final int CLIMBER_LEFT_MOTOR_ID = 16;
    public static final int CLIMBER_RIGHT_MOTOR_ID = 17;
    public static final int CLIMBER_BRAKE_PORT = 0;
  }

  //Misc. controller values
  public static final class OperatorConstants {
    public static final double RIGHT_TRIGGER_DEADZONE = 0.1;
    public static final double LEFT_TRIGGER_DEADZONE = 0.1;
  }

  public static double inchesToMeters(double inches){
    return inches / 39.37;
  }

  public static double feetToMeters(double feet){
    return feet / 3.281;
  }

  public static double getDistance(double x1, double y1, double x2, double y2){
    return Math.sqrt(Math.pow(x2 - x1, 2) + Math.pow(y2 - y1, 2));
  }

  public static double rotationsToRadians(double rotations){
    return rotations * 2 * Math.PI;
  }

  public static double degreesToRotations(double degrees){
    return degrees / 360;
  }

  public static double rotationsToDegrees(double rotations){
    return rotations * 360;
  }

  public static double degreesToRadians(double degrees){
    return degrees * Math.PI / 180;
  }

  public static double angleToUnitVectorI(double angle){
    return (Math.cos(angle));
  }

  public static double angleToUnitVectorJ(double angle){
    return (Math.sin(angle));
  }

  public static double RPMToRPS(double RPM){
    return RPM / 60;
  }

  public static double RPSToRPM(double RPS){
    return RPS * 60;
  }
}
