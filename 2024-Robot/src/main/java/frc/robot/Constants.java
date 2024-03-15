// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.tools.math.Vector;

public final class Constants {

  //Physical constants (e.g. field and robot dimensions)
  public static final class Physical {
    public static final double FIELD_WIDTH = 8.2;
    public static final double FIELD_LENGTH = 16.63;
    public static final double ROBOT_RADIUS = inchesToMeters(15.429);
    public static final double WHEEL_DIAMETER = inchesToMeters(4);
    public static final double WHEEL_CIRCUMFERENCE = Math.PI * WHEEL_DIAMETER;
    public static final double WHEEL_ROTATION_PER_METER = 1 / WHEEL_CIRCUMFERENCE;

    public static final double SPEAKER_DEPTH = inchesToMeters(18.11);

    public static final double TOP_SPEED = feetToMeters(20);

    public static final double ROBOT_LENGTH = inchesToMeters(25);
    public static final double ROBOT_WIDTH = inchesToMeters(28.5);
    public static final double MODULE_OFFSET = inchesToMeters(2.5);

    public static final double SHOOTER_RESTING_ANGLE_DEG = 8.0;
    public static final double FLYWHEEL_RADIUS_METERS = inchesToMeters(2.0);
    public static final double FLYWHEEL_CIRCUMFERENCE_METERS = 2.0 * Math.PI * FLYWHEEL_RADIUS_METERS;

    public static final double GRAVITY_ACCEL_MS2 = 9.806;

    public static double flywheelRPMToNoteMPS(double rpm){
      return rpm * (1.0 / 60.0) * FLYWHEEL_CIRCUMFERENCE_METERS;
    }

    public static double noteMPSToFlywheelRPM(double mps){
      return (mps / FLYWHEEL_CIRCUMFERENCE_METERS) * 60.0;
    }
  }

  //Subsystem setpoint constants
  public static final class SetPoints {
    //drive

    //intake
    public static final double INTAKE_DOWN_ANGLE_ROT = -0.365;
    public static final double INTAKE_UP_ANGLE_ROT = 0;
    public static final double INTAKE_DOWN_ANGLE_DEG = rotationsToDegrees(INTAKE_DOWN_ANGLE_ROT);
    public static final double INTAKE_UP_ANGLE_DEG = rotationsToDegrees(INTAKE_UP_ANGLE_ROT);

    //Intake up and down positions
    public enum IntakePosition {
      kDOWN(INTAKE_DOWN_ANGLE_DEG, INTAKE_DOWN_ANGLE_ROT),
      kUP(INTAKE_UP_ANGLE_DEG, INTAKE_UP_ANGLE_ROT);

      public final double degrees;
      public final double rotations;

      private IntakePosition(double degrees, double rotations){
        this.degrees = degrees;
        this.rotations = rotations;
      }
    }

    //shooter
    public static final double SHOOTER_CENTER_OFFSET_DEG = 22.0;
    public static final double SHOOTER_CENTER_OFFSET_ROT = degreesToRotations(SHOOTER_CENTER_OFFSET_DEG);
    public static final double SHOOTER_DOWN_ANGLE_ROT = 0.0;
    public static final double SHOOTER_MAX_ANGLE_ROT = 0.18;
    // public static final double SHOOTER_MAX_ANGLE_ROT = 0.07;
    public static final double SHOOTER_DOWN_ANGLE_DEG = rotationsToDegrees(SHOOTER_DOWN_ANGLE_ROT);
    public static final double SHOOTER_MAX_ANGLE_DEG = rotationsToDegrees(SHOOTER_MAX_ANGLE_ROT);

    // {distance(inches), target angle(deg), hood angle(deg), RPM}
    public static final double [][] SHOOTING_LOOKUP_TABLE = {
      {1.302, 11.87, 58, 4000, 1},
      {1.442, 10.20, 56, 4000, 1},
      {1.732, 6.70, 52, 4250, 1},
      {2.072, 3.57, 48, 4500, 1},
      {2.442, 0.92, 42, 4750, 1},
      {2.842, -1.09, 38, 5000, 1},
      {3.062, -1.95, 36.5, 5125, 1},
      {3.302, -2.55, 35, 5250, 1},
      {3.412, -3.13, 33, 5250, 1},
      {3.722, -4.01, 31.25, 5500, 1},
      {4.072, -5.03, 28.5, 5750, 1},
      {4.372, -5.69, 27.5, 6000, 1},
      {4.592, -5.91, 26, 6250, 1},
      {4.812, -6.27, 26, 6375, 1},
      {5.152, -6.59, 25.25, 6500, 0.75},
      {5.442, -7.16, 24, 6625, 0.75},
      {5.712, -8.26, 22.25, 6750, 0.75},
      {6.117, -8.51, 21.25, 7000, 0.75},
      {6.382, -8.60, 21, 7250, 0.75},
      {6.682, -8.82, 20.5, 7500, 0.75},
      {6.912, -9.20, 19.5, 7750, 0.75},
      {7.252, -9.44, 19.25, 8000, 0.75}
    };

    public static double getInterpolatedValue(int xIndex, int yIndex, double xValue){
        int lastIndex = SHOOTING_LOOKUP_TABLE.length - 1;
        if (xValue > SHOOTING_LOOKUP_TABLE[0][xIndex]) {
            //If the xValue is closer than the first setpoint
            double returnValue = SHOOTING_LOOKUP_TABLE[0][yIndex];
            return returnValue;
        } else if (xValue < SHOOTING_LOOKUP_TABLE[lastIndex][xIndex]) {
            //If the xValue is farther than the last setpoint
            double returnValue = SHOOTING_LOOKUP_TABLE[lastIndex][0];
            return returnValue;
        } else {
            for (int i = 0; i < SHOOTING_LOOKUP_TABLE.length; i ++) {
                if (xValue < SHOOTING_LOOKUP_TABLE[i][xIndex] && xValue > SHOOTING_LOOKUP_TABLE[i + 1][xIndex]) {
                    //If the xValue is in the table of setpoints
                    //Calculate where xValue is between setpoints
                    double leftDif = xValue - SHOOTING_LOOKUP_TABLE[i][xIndex];
                    double percent = leftDif / (SHOOTING_LOOKUP_TABLE[i + 1][xIndex] - SHOOTING_LOOKUP_TABLE[i][xIndex]);

                    double value1 = SHOOTING_LOOKUP_TABLE[i][yIndex];
                    double value2 = SHOOTING_LOOKUP_TABLE[i + 1][yIndex];

                    //Interpolate in-between values for value xValue and shooter rpm
                    double newValue = value1 + (percent * (value2 - value1));
                    
                    double returnValue = newValue;
                    return returnValue;
                }
            }
            //Should never run
            double returnValue = SHOOTING_LOOKUP_TABLE[0][yIndex];
            return returnValue;
        }
    }

    public static double[] getShooterValuesFromAngle(double angle) {
      return new double[] {getInterpolatedValue(1, 2, angle), getInterpolatedValue(1, 3, angle)};
      // return new double[] {25, getInterpolatedValue(1, 3, angle)}; 
    }

    public static double[] getShooterValuesFromDistance(double dist) {
      return new double[] {getInterpolatedValue(0, 2, dist), getInterpolatedValue(0, 3, dist)};
      // return new double[] {25, getInterpolatedValue(0, 3, dist)};
    }

    public static double getAllowedAngleErrFromAngle(double angle){
      return getInterpolatedValue(1, 4, angle);
    }

    public static double getDistFromAngle(double ty) {
      return getInterpolatedValue(1, 0, ty);
    }

    public static double[] getVelocityAdjustedSetpoint(double pigeonAngleDegrees, double speakerAngleDegrees, double shooterAngleDegrees, double shooterRPM, Vector robotVelocityMPS){
      double thetaI = Math.toRadians(pigeonAngleDegrees - speakerAngleDegrees);
      double phiI = Math.toRadians(shooterAngleDegrees);
      double rhoI = Constants.Physical.flywheelRPMToNoteMPS(shooterRPM);
      Vector robotVelocityVector = robotVelocityMPS;
      double vx = robotVelocityVector.getI();
      double vy = robotVelocityVector.getJ();

      double xI = rhoI * Math.cos(phiI) * Math.cos(thetaI);
      double yI = rhoI * Math.cos(phiI) * Math.sin(thetaI);
      double zI = rhoI * Math.sin(phiI);

      double xF = xI - vx;
      double yF = yI - vy;
      double zF = zI;

      double thetaF = Math.atan2(yF, xF);
      if (thetaF < 0){
        thetaF += 2 * Math.PI;
      }
      double phiF = Math.atan2(zF, Math.sqrt(Math.pow(xF, 2) + Math.pow(yF, 2)));
      double rhoF = Math.sqrt(Math.pow(xF, 2) + Math.pow(yF, 2) + Math.pow(zF, 2));

      double targetShooterDegrees = Math.toDegrees(phiF);
      double targetShooterRPM = Constants.Physical.noteMPSToFlywheelRPM(rhoF);

      double extraPigeonRotations = Math.floor((pigeonAngleDegrees / 360.0));
      double targetPigeonAngleDegrees = extraPigeonRotations * 360.0 + Math.toDegrees(thetaF);

      return new double[] {targetPigeonAngleDegrees, targetShooterDegrees, targetShooterRPM};
    }

    public static double getAddedTheta(double targetAngleDegrees, double distanceMeters, double depthAddedMeters) {
      targetAngleDegrees = standardizeAngleDegrees(targetAngleDegrees);
      double targetAngleRadians = Math.toRadians(targetAngleDegrees);
      targetAngleRadians -= (Math.PI)/2;
      double addedAngleRadians;
      if(targetAngleRadians >= Math.PI/2) {
        targetAngleRadians -= Math.PI;
        addedAngleRadians = (targetAngleRadians - Math.atan(Math.tan(targetAngleRadians) - depthAddedMeters/(distanceMeters*Math.cos(targetAngleRadians))));
      } else {
        addedAngleRadians = -(targetAngleRadians - Math.atan(Math.tan(targetAngleRadians) - depthAddedMeters/(distanceMeters*Math.cos(targetAngleRadians))));
      }
      return Math.toDegrees(addedAngleRadians);
    }
  
    public static double standardizeAngleDegrees(double angleDegrees) {
      if(angleDegrees >= 0 && angleDegrees < 360) {
        return angleDegrees;
      } else if(angleDegrees < 0) {
        while(angleDegrees < 0) {
          angleDegrees += 360;
        }
        return angleDegrees;
      } else if(angleDegrees >= 360) {
        while(angleDegrees >= 360) {
          angleDegrees -= 360;
        }
        return angleDegrees;
      } else {
        System.out.println("Weird ErroR");
        return angleDegrees;
      }
    }

    public static double getAdjustedPigeonAngle(double targetPigeonAngleDegrees, double distToSpeakerMeters){
      double adjustment = getAddedTheta(targetPigeonAngleDegrees, distToSpeakerMeters, (Constants.Physical.SPEAKER_DEPTH / 2));
      // System.out.println("Angle Adjustment: " + adjustment);
      return targetPigeonAngleDegrees + adjustment;
    }

    //feeder

    //TOF
    public static final double FEEDER_TOF_THRESHOLD_MM = 75;    
    public static final double CARRIAGE_TOF_THRESHOLD_MM = 95;
    public static final double INTAKE_TOF_THRESHOLD_MM = 200;

    //climber
    public static final double ELEVATOR_BOTTOM_POSITION_M = 0.0;
    public static final double ELEVATOR_TOP_POSITION_M = 0.43;
    public static final double ELEVATOR_AMP_POSITION_M = 0.22;
    public static final double CARRIAGE_CLEARANCE_ELEVATOR_HEIGH_M = 0.5;
    public static final double CARRIAGE_BOTTOM_ROTATION_DEG = -16.0;
    public static final double CARRIAGE_AMP_ROTATION_DEG = 210.0;
    public static final double CARRIAGE_TRAP_ROTATION_DEG = 215;

    public enum ElevatorPosition {
      kDOWN(ELEVATOR_BOTTOM_POSITION_M, Constants.Ratios.elevatorMetersToRotations(ELEVATOR_BOTTOM_POSITION_M)),
      kFIRST_EXTEND(0.5, Constants.Ratios.elevatorMetersToRotations(0.5)),
      kAMP(ELEVATOR_AMP_POSITION_M, Constants.Ratios.elevatorMetersToRotations(ELEVATOR_AMP_POSITION_M)),
      kUP(ELEVATOR_TOP_POSITION_M, Constants.Ratios.elevatorMetersToRotations(ELEVATOR_TOP_POSITION_M));

      public final double meters;
      public final double rotations;

      private ElevatorPosition(double meters, double rotations){
        this.meters = meters;
        this.rotations = rotations;
      }
    }

    public enum CarriageRotation {
      kDOWN(CARRIAGE_BOTTOM_ROTATION_DEG),
      kFEED(1),
      kAMP(CARRIAGE_AMP_ROTATION_DEG),
      kTRAP(CARRIAGE_TRAP_ROTATION_DEG);

      
      public final double degrees;

      private CarriageRotation(double degrees){
        this.degrees = degrees;
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
    Crosshair X: 0.23
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
    public static final double DRIVE_GEAR_RATIO = 5.9;
    public static final double STEER_GEAR_RATIO = 21.43;

    //intake
    public static final double INTAKE_ANGLE_GEAR_RATIO = 30.0;
    public static final double INTAKE_ROLLER_GEAR_RATIO = 24.0 / 11.0;

    //shooter
    public static final double SHOOTER_ANGLE_GEAR_RATIO = 225.0;
    public static final double SHOOTER_FLYWHEEL_GEAR_RATIO = 30.0 / 56.0;

    //feeder
    public static final double FEEDER_ROLLER_GEAR_RATIO = 3.0;

    //climber
    public static final double ELEVATOR_GEAR_RATIO = 23.52;
    public static final double TRAP_ROLLER_GEAR_RATIO = 3.0;
    public static final double CARRIAGE_ROTATION_GEAR_RATIO = 100.0;
    public static final double ELEVATOR_MOTOR_ROTATIONS_PER_METER = 219.254;

    public static double elevatorRotationsToMeters(double rotations){
      return rotations / ELEVATOR_MOTOR_ROTATIONS_PER_METER;
    }

    public static double elevatorMetersToRotations(double meters){
      return meters * ELEVATOR_MOTOR_ROTATIONS_PER_METER;
    }
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
    public static final int CARRIAGE_TOF_ID = 1;
    public static final int INTAKE_TOF_ID = 2;

    //proximity
    public static final int SHOOTER_PROXIMITY_PORT = 1;
    public static final int CARRIAGE_PROXIMITY_PORT = 2;
    public static final int FEEDER_PROXIMITY_PORT = 3;

    //Climber
    public static final int ELEVATOR_FOLLOWER_MOTOR_ID = 16;
    public static final int ELEVATOR_MASTER_MOTOR_ID = 17;
    public static final int TRAP_ROLLER_MOTOR_ID = 18;
    public static final int CARRIAGE_ROTATION_MOTOR_ID = 19;
    public static final int CARRIAGE_ROTATION_CANCODER_ID = 6;

    //Lights
    public static final int CANDLE_ID = 0;
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
