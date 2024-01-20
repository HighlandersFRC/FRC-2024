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
    public static final double GEAR_RATIO = 6.75;
    public static final double STEER_GEAR_RATIO = 21.43;

    public static final double FALCON_TICS_PER_ROTATION = 2048;
    public static final double CANCODER_TICS_PER_ROTATION = 4096;
    public static final double NEO_TICS_PER_ROTATION = 42;

    public static final double ROBOT_WIDTH = inchesToMeters(25);
    public static final double ROBOT_LENGTH = inchesToMeters(29);
    public static final double MODULE_OFFSET = inchesToMeters(2.5);
  }

  //Subsystem setpoint constants
  public static final class Setpoints {
    public static final double INTAKE_DOWN_ANGLE = 0;
    public static final double INTAKE_UP_ANGLE = 0;

    //Intake up and down positions
    public enum IntakePosition {
      kDOWN(INTAKE_DOWN_ANGLE), kUP(INTAKE_UP_ANGLE);

      public final double angle;

      private IntakePosition(double angle){
        this.angle = angle;
      }
    }
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

    //Poses of cameras relative to robot, {x, y, z, rx, ry, rz}, in meters and radians
    public static final double[] BACK_CAMERA_POSE = {-0.263252, -0.1778, 0.2413, 0, 0, degreesToRadians(180)};
    public static final double[] FRONT_CAMERA_POSE = {0.263525, 0.1778, 0.2667, 0, degreesToRadians(33), 0};
    public static final double[] BACK_CAMERA_POSITION_POLAR = {getDistance(0, 0, BACK_CAMERA_POSE[0], BACK_CAMERA_POSE[1]), Math.atan2(BACK_CAMERA_POSE[1], BACK_CAMERA_POSE[0])};
    public static final double[] FRONT_CAMERA_POSITION_POLAR = {getDistance(0, 0, FRONT_CAMERA_POSE[0], FRONT_CAMERA_POSE[1]), Math.atan2(FRONT_CAMERA_POSE[1], FRONT_CAMERA_POSE[0])};

    //Standard deviation adjustments
    public static final double STANDARD_DEVIATION_SCALAR = 10;
    public static final double ODOMETRY_JUMP_STANDARD_DEVIATION_SCALAR = 0.25;
    public static final double ODOMETRY_JUMP_STANDARD_DEVIATION_DEGREE = 2;
    public static final double TAG_STANDARD_DEVIATION_DISTANCE = 3; //meters
    public static final double TAG_STANDARD_DEVIATION_FLATNESS = 5;

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

  public static final class OperatorConstants {
    int DRIVER_CONTROLLER_PORT = 0;
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

  public static double degreesToRadians(double degrees){
    return degrees * Math.PI / 180;
  }

  public static double angleToUnitVectorI(double angle){
    return (Math.cos(angle));
  }

  public static double angleToUnitVectorJ(double angle){
    return (Math.sin(angle));
  }
}
