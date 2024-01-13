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
    public static final double[] BACK_CAMERA_POSE = {-0.219075, -0.1524, 0.1524, 0, 0, Math.PI};
    public static final double[] FRONT_CAMERA_POSE = {0.219075, 0.1524, 0.288925, 0, 33 * Math.PI / 180, 0};
    public static final double[] BACK_CAMERA_POSITION_POLAR = {getDistance(0, 0, BACK_CAMERA_POSE[0], BACK_CAMERA_POSE[1]), Math.atan2(BACK_CAMERA_POSE[1], BACK_CAMERA_POSE[0])};
    public static final double[] FRONT_CAMERA_POSITION_POLAR = {getDistance(0, 0, FRONT_CAMERA_POSE[0], FRONT_CAMERA_POSE[1]), Math.atan2(FRONT_CAMERA_POSE[1], FRONT_CAMERA_POSE[0])};

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
}
