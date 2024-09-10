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
    public static final double SPEAKER_X = 0;
    public static final double SPEAKER_Y = inchesToMeters(220.347);

    public static final double TOP_SPEED = feetToMeters(25);

    public static final double ROBOT_LENGTH = inchesToMeters(28.5);
    public static final double ROBOT_WIDTH = inchesToMeters(28.5);
    public static final double MODULE_OFFSET = inchesToMeters(2.5);

    public static final double SHOOTER_RESTING_ANGLE_DEG = 8.0;
    public static final double FLYWHEEL_RADIUS_METERS = inchesToMeters(2.0);
    public static final double FLYWHEEL_CIRCUMFERENCE_METERS = 2.0 * Math.PI * FLYWHEEL_RADIUS_METERS;

    public static final double GRAVITY_ACCEL_MS2 = 9.806;

    public static final boolean isFrontRightDriveFlipped = false;
    public static final boolean isFrontRightAngleFlipped = true;
    public static final boolean isFrontLeftDriveFlipped = false;
    public static final boolean isFrontLeftAngleFlipped = true;
    public static final boolean isBackLeftDriveFlipped = false;
    public static final boolean isBackLeftAngleFlipped = false;
    public static final boolean isBackRightDriveFlipped = true;
    public static final boolean isBackRightAngleFlipped = true;


    /**
     * Converts flywheel RPM (Revolutions Per Minute) to linear speed in meters per second (m/s).
     *
     * @param rpm The flywheel speed in RPM.
     * @return The equivalent linear speed in meters per second (m/s).
     */
    public static double flywheelRPMToNoteMPS(double rpm){
      return rpm * (1.0 / 60.0) * FLYWHEEL_CIRCUMFERENCE_METERS;
    }
    /**
     * Converts linear speed in meters per second (m/s) to flywheel RPM (Revolutions Per Minute).
     *
     * @param mps The linear speed in meters per second (m/s).
     * @return The equivalent flywheel speed in RPM.
     */
    public static double noteMPSToFlywheelRPM(double mps){
      return (mps / FLYWHEEL_CIRCUMFERENCE_METERS) * 60.0;
    }
  }

  //Subsystem setpoint constants
  public static final class SetPoints {
    //drive

    //intake
    // public static final double INTAKE_DOWN_ANGLE_ROT = -0.32;
    public static final double INTAKE_DOWN_ANGLE_ROT = -0.32;
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
    public static final double SHOOTER_MAX_DEG = 270.0;
    public static final double SHOOTER_MIN_DEG = 20.0;
    public static final double SHOOTER_AMP_ANGLE_PRESET_DEG = 145.0;
    public static final double SHOOTER_CENTER_OFFSET_DEG = 0.0;
    public static final double SHOOTER_CENTER_OFFSET_ROT = degreesToRotations(SHOOTER_CENTER_OFFSET_DEG);
    public static final double SHOOTER_DOWN_ANGLE_ROT = 0.0;
    public static final double SHOOTER_MAX_ANGLE_ROT = 0.18 * Constants.Ratios.SHOOTER_ANGLE_GEAR_RATIO;
    // public static final double SHOOTER_MAX_ANGLE_ROT = 0.07;
    public static final double SHOOTER_DOWN_ANGLE_DEG = rotationsToDegrees(SHOOTER_DOWN_ANGLE_ROT);
    public static final double SHOOTER_MAX_ANGLE_DEG = rotationsToDegrees(SHOOTER_MAX_ANGLE_ROT);

    public static final double[] SHOOTER_SUBWOOFER_PRESET = {1750, 3500, 65};
    public static final double[] SHOOTER_PODIUM_PRESET = {2500, 5000, 40, 150};

    //bumper ty on comp field: 8.29 blue, 8.04 red

    //ty at wired red practice field: 8.13
    //ty: -12.55, current offset: 2.5
    //flip getInterpolated value edge setpoints
    // {distance(meters), target angle(deg), hood angle(deg), RPM, allowed hood angle error (deg), allowed robot angle error(deg)}
    public static final double LIMELIGHT_ANGLE_OFFSET = 2.5;
    public static final double DISTANCE_OFFSET = 0.1;
    public static final double [][] SHOOTING_LOOKUP_TABLE = {
      {1.295 + DISTANCE_OFFSET, 7.83 + LIMELIGHT_ANGLE_OFFSET, 60, 4500, 1, 2},
      {1.486 + DISTANCE_OFFSET, 4.78 + LIMELIGHT_ANGLE_OFFSET, 56, 4500, 1, 2},
      {1.762 + DISTANCE_OFFSET, 1.98 + LIMELIGHT_ANGLE_OFFSET, 53, 4750, 1, 2},
      {2.172 + DISTANCE_OFFSET, -2.05 + LIMELIGHT_ANGLE_OFFSET, 47, 5000, 0.75, 2},
      {2.575 + DISTANCE_OFFSET, -4.39 + LIMELIGHT_ANGLE_OFFSET, 43, 5125, 0.75, 2},
      {2.927 + DISTANCE_OFFSET, -6.57 + LIMELIGHT_ANGLE_OFFSET, 37.5, 5250, 0.75, 2},
      {3.479 + DISTANCE_OFFSET, -8.40 + LIMELIGHT_ANGLE_OFFSET, 34.5, 5500, 0.5, 1.5},
      {3.846 + DISTANCE_OFFSET, -9.49 + LIMELIGHT_ANGLE_OFFSET, 31.9, 5750, 0.5, 1.5},
      {4.247 + DISTANCE_OFFSET, -10.34 + LIMELIGHT_ANGLE_OFFSET, 28.7, 6000, 0.5, 1.5},
      {4.743 + DISTANCE_OFFSET, -11.08 + LIMELIGHT_ANGLE_OFFSET, 27.3, 6250, 0.5, 1.5},
      {5.293 + DISTANCE_OFFSET, -11.75 + LIMELIGHT_ANGLE_OFFSET, 26.2, 6500, 0.5, 1.5},
      {5.420 + DISTANCE_OFFSET, -12.22 + LIMELIGHT_ANGLE_OFFSET, 25.5, 6750, 0.35, 1.5},
      {5.596 + DISTANCE_OFFSET, -12.70 + LIMELIGHT_ANGLE_OFFSET, 24.5, 6825, 0.5, 1.5},
      {5.755 + DISTANCE_OFFSET, -13.48 + LIMELIGHT_ANGLE_OFFSET, 23.5, 7000, 0.5, 1.2},
      {6.06 + DISTANCE_OFFSET, -14.02 + LIMELIGHT_ANGLE_OFFSET, 22, 7500, 0.5, 1.2},
      {6.40 + DISTANCE_OFFSET, -14.29 + LIMELIGHT_ANGLE_OFFSET, 22, 7500, 0.5, 1.2},
      {6.9 + DISTANCE_OFFSET, -14.47 + LIMELIGHT_ANGLE_OFFSET, 22, 7500, 0.5, 1.2},
      {7.2 + DISTANCE_OFFSET, -14.56 + LIMELIGHT_ANGLE_OFFSET, 21, 7500, 0.5, 1.2},
      {7.4 + DISTANCE_OFFSET, -14.68 + LIMELIGHT_ANGLE_OFFSET, 21.25, 7500, 0.5, 1.2}
    };

    public static double[] getMovingAverageWeights(int numMeasurements){
      double n = numMeasurements;
      double divisor = (n * (n + 1.0)) / 2.0;
      double[] weights = new double[numMeasurements];
      for (int i = 0; i < weights.length; i ++){
        weights[i] = ((double) i + 1) / divisor;
      }
      return weights;
    }

    /**
     * Interpolates a value from a lookup table based on the given xValue.
     * 
     * @param xIndex  The index of the x-values in the lookup table.
     * @param yIndex  The index of the y-values in the lookup table.
     * @param xValue  The x-value for which to interpolate a y-value.
     * @return        The interpolated y-value corresponding to the given x-value.
     */
    public static double getInterpolatedValue(int xIndex, int yIndex, double xValue){
        int lastIndex = SHOOTING_LOOKUP_TABLE.length - 1;
        if (xValue < SHOOTING_LOOKUP_TABLE[0][xIndex]) {
            //If the xValue is closer than the first setpoint
            double returnValue = SHOOTING_LOOKUP_TABLE[0][yIndex];
            return returnValue;
        } else if (xValue > SHOOTING_LOOKUP_TABLE[lastIndex][xIndex]) {
            //If the xValue is farther than the last setpoint
            double returnValue = SHOOTING_LOOKUP_TABLE[lastIndex][yIndex];
            return returnValue;
        } else {
            for (int i = 0; i < SHOOTING_LOOKUP_TABLE.length; i ++) {
                if (xValue > SHOOTING_LOOKUP_TABLE[i][xIndex] && xValue < SHOOTING_LOOKUP_TABLE[i + 1][xIndex]) {
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

    /**
     * Calculates shooter values (flywheel velocity and note velocity) based on the given angle.
     *
     * @param angle The angle of the shooter.
     * @return An array containing the calculated flywheel velocity and note velocity.
     */
    public static double[] getShooterValuesFromAngle(double angle) {
      return new double[] {getInterpolatedValue(1, 2, angle), getInterpolatedValue(1, 3, angle)};
      // return new double[] {25, getInterpolatedValue(1, 3, angle)}; 
    }

    /**
     * Calculates shooter values (flywheel velocity and note velocity) based on the given distance.
     *
     * @param dist The distance from the shooter to the target.
     * @return An array containing the calculated flywheel velocity and note velocity.
     */
    public static double[] getShooterValuesFromDistance(double dist) {
      return new double[] {getInterpolatedValue(0, 2, dist), getInterpolatedValue(0, 3, dist)};
      // return new double[] {25, getInterpolatedValue(0, 3, dist)};
    }

    /**
     * Calculates the allowed angle error based on the given angle.
     *
     * @param angle The angle of the shooter.
     * @return The allowed angle error.
     */
    public static double getAllowedAngleErrFromAngle(double angle){
      return getInterpolatedValue(1, 4, angle);
    }

    public static double getAllowedDriveAngleErrFromAngle(double angle){
      return getInterpolatedValue(1, 5, angle);
    }

    /**
     * Calculates the distance from the target based on the given vertical angle.
     *
     * @param ty The vertical angle of the target.
     * @return The distance from the target.
     */
    public static double getDistFromAngle(double ty) {
      return getInterpolatedValue(1, 0, ty);
    }

    /**
     * Calculates the velocity-adjusted setpoint for the shooter and pigeon angles based on various parameters.
     *
     * @param pigeonAngleDegrees  The current angle of the pigeon in degrees.
     * @param speakerAngleDegrees The angle of the speaker in degrees.
     * @param shooterAngleDegrees The current angle of the shooter in degrees.
     * @param shooterRPM          The current RPM of the shooter.
     * @param robotVelocityMPS    The velocity vector of the robot in meters per second.
     * @return An array containing the adjusted setpoints for the pigeon angle, shooter angle, and shooter RPM.
     */
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

    /**
     * Calculates the additional angle (addedTheta) required to adjust for depth added to the target angle.
     *
     * @param targetAngleDegrees The target angle in degrees.
     * @param distanceMeters     The distance to the target in meters.
     * @param depthAddedMeters   The additional depth added to the target in meters.
     * @return The additional angle (addedTheta) in degrees.
     */
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

    /**
     * Standardizes an angle to be within the range [0, 360) degrees.
     *
     * @param angleDegrees The input angle in degrees.
     * @return The standardized angle within the range [0, 360) degrees.
     */
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
        // System.out.println("Weird ErroR");
        return angleDegrees;
      }
    }

    /**
     * Calculates the adjusted pigeon angle based on the target angle and distance to the speaker.
     *
     * @param targetPigeonAngleDegrees The target angle for the pigeon in degrees.
     * @param distToSpeakerMeters      The distance to the speaker in meters.
     * @return The adjusted pigeon angle in degrees.
     */
    public static double getAdjustedPigeonAngle(double targetPigeonAngleDegrees, double distToSpeakerMeters){
      double adjustment = getAddedTheta(targetPigeonAngleDegrees, distToSpeakerMeters, (Constants.Physical.SPEAKER_DEPTH / 2));
      // System.out.println("Angle Adjustment: " + adjustment);
      return targetPigeonAngleDegrees + adjustment;
    }

    //feeder

    //TOF
    public static final double FEEDER_TOF_THRESHOLD_MM = 75;    
    public static final double CARRIAGE_TOF_THRESHOLD_MM = 95;
    public static final double INTAKE_TOF_THRESHOLD_MM = 420;

    //climber
    public static final double ELEVATOR_BOTTOM_POSITION_M = 0.0;
    public static final double ELEVATOR_TOP_POSITION_M = 0.43;
    public static final double ELEVATOR_AMP_POSITION_M = 0.22;
    public static final double CARRIAGE_CLEARANCE_ELEVATOR_HEIGH_M = 0.5;
    public static final double CARRIAGE_BOTTOM_ROTATION_DEG = -16.0;
    //public static final double CARRIAGE_AMP_ROTATION_DEG = 210.0;
    public static final double CARRIAGE_AMP_ROTATION_DEG = 199.0;
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
    /**
     * Calculates the standard deviation scalar based on the distance from the tag.
     *
     * @param dist The distance from the tag.
     * @return The standard deviation scalar.
     */
    public static double getTagDistStdDevScalar(double dist){
      double a = TAG_STANDARD_DEVIATION_FLATNESS;
      double b = 1 - a * Math.pow(TAG_STANDARD_DEVIATION_DISTANCE, 2);
      return Math.max(1, a * Math.pow(dist, 2) + b);
    }

    /**
     * Calculates the standard deviation scalar based on the number of detected tags.
     *
     * @param numTags The number of detected tags.
     * @return The standard deviation scalar.
     */
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

    /**
     * Calculates the standard deviation of the x-coordinate based on the given offsets.
     *
     * @param xOffset The x-coordinate offset.
     * @param yOffset The y-coordinate offset.
     * @return The standard deviation of the x-coordinate.
     */
    public static double getTagStdDevX(double xOffset, double yOffset){
      return Math.max(0, 0.005533021491867763 * (xOffset * xOffset + yOffset * yOffset) - 0.010807566510145635) * STANDARD_DEVIATION_SCALAR;
    }

    /**
     * Calculates the standard deviation of the y-coordinate based on the given offsets.
     *
     * @param xOffset The x-coordinate offset.
     * @param yOffset The y-coordinate offset.
     * @return The standard deviation of the y-coordinate.
     */
    public static double getTagStdDevY(double xOffset, double yOffset){
      return Math.max(0, 0.0055 * (xOffset * xOffset + yOffset * yOffset) - 0.01941597810542626) * STANDARD_DEVIATION_SCALAR;
    }

    /**
     * Calculates the standard deviation in the x-coordinate for triangulation measurements.
     *
     * @param xOffset The x-coordinate offset.
     * @param yOffset The y-coordinate offset.
     * @return The standard deviation in the x-coordinate.
     */
    public static double getTriStdDevX(double xOffset, double yOffset){
      return Math.max(0, 0.004544133588821881 * (xOffset * xOffset + yOffset * yOffset) - 0.01955724864971872) * STANDARD_DEVIATION_SCALAR;
    }

    /**
     * Calculates the standard deviation in the y-coordinate for triangulation measurements.
     *
     * @param xOffset The x-coordinate offset.
     * @param yOffset The y-coordinate offset.
     * @return The standard deviation in the y-coordinate.
     */
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
    public static final double INTAKE_ROLLER_GEAR_RATIO = 2.0;
    public static final double INTAKE_CENTERING_ROLLERS_GEAR_RATIO = 30.0 / 12.0;

    //shooter
    public static final double SHOOTER_ANGLE_GEAR_RATIO = 107.0;
    public static final double SHOOTER_FLYWHEEL_GEAR_RATIO = 16.0 / 27.0;

    //feeder
    public static final double FEEDER_ROLLER_GEAR_RATIO = 33.0 / 16.0;

    //climber
    public static final double ELEVATOR_GEAR_RATIO = 23.52;
    public static final double TRAP_ROLLER_GEAR_RATIO = 3.0;
    public static final double CARRIAGE_ROTATION_GEAR_RATIO = 100.0;
    public static final double ELEVATOR_MOTOR_ROTATIONS_PER_METER = 219.254;

    /**
     * Converts elevator motor rotations to meters.
     *
     * @param rotations The number of rotations of the elevator motor.
     * @return The equivalent distance in meters.
     */
    public static double elevatorRotationsToMeters(double rotations){
      return rotations / ELEVATOR_MOTOR_ROTATIONS_PER_METER;
    }

    /**
     * Converts meters to elevator motor rotations.
     *
     * @param meters The distance in meters to be converted.
     * @return The equivalent number of rotations of the elevator motor.
     */
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
    public static final int INTAKE_CENTER_MOTOR_ID = 9;
    public static final int INTAKE_ROLLER_MOTOR_ID = 10;

    //shooter
    public static final int SHOOTER_ANGLE_MOTOR_ID = 12;
    public static final int SHOOTER_LEFT_MOTOR_ID = 14;
    public static final int SHOOTER_RIGHT_MOTOR_ID = 13;

    //feeder
    public static final int FEEDER_ROLLER_MOTOR_ID = 11;

    //TOF
    public static final int FEEDER_TOF_ID = 0;
    public static final int CARRIAGE_TOF_ID = 1;
    public static final int INTAKE_TOF_ID = 2;

    //proximity
    public static final int SHOOTER_PROXIMITY_PORT = 3;
    public static final int CARRIAGE_PROXIMITY_PORT = 2;
    public static final int FEEDER_PROXIMITY_PORT = 1;

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

  /**
   * Converts inches to meters.
   *
   * @param inches The length in inches to be converted.
   * @return The equivalent length in meters.
   */
  public static double inchesToMeters(double inches){
    return inches / 39.37;
  }

  /**
   * Converts feet to meters.
   *
   * @param inches The length in feet to be converted.
   * @return The equivalent length in meters.
   */
  public static double feetToMeters(double feet){
    return feet / 3.281;
  }

  /**
   * Calculates the Euclidean distance between two points in a 2D plane.
   *
   * @param x1 The x-coordinate of the first point.
   * @param y1 The y-coordinate of the first point.
   * @param x2 The x-coordinate of the second point.
   * @param y2 The y-coordinate of the second point.
   * @return The Euclidean distance between the two points.
   */
  public static double getDistance(double x1, double y1, double x2, double y2){
    return Math.sqrt(Math.pow(x2 - x1, 2) + Math.pow(y2 - y1, 2));
  }

  public static double getAngleToPoint(double x1, double y1, double x2, double y2){
    double deltaX = x2 - x1;
    double deltaY = y2 - y1;

    double angleInRadians = Math.atan2(deltaY, deltaX);

    double angleInDegrees = Math.toDegrees(angleInRadians);

    double standardizeAngleDegrees = SetPoints.standardizeAngleDegrees(angleInDegrees);

    // if (y1 > y2) {
      // System.out.println("running");
      return 180 + standardizeAngleDegrees;
    // }
    //   System.out.println("2");
    //   double temp = 180 - standardizeAngleDegrees;
    //   double j = 180 - temp;
    //   return 180 + j;
    // }
  }

  /**
   * Converts a quantity in rotations to radians.
   *
   * @param rotations The quantity in rotations to be converted.
   * @return The equivalent quantity in radians.
   */
  public static double rotationsToRadians(double rotations){
    return rotations * 2 * Math.PI;
  }

  /**
   * Converts a quantity in degrees to rotations.
   *
   * @param rotations The quantity in degrees to be converted.
   * @return The equivalent quantity in rotations.
   */
  public static double degreesToRotations(double degrees){
    return degrees / 360;
  }

  /**
   * Converts a quantity in rotations to degrees.
   *
   * @param rotations The quantity in rotations to be converted.
   * @return The equivalent quantity in degrees.
   */
  public static double rotationsToDegrees(double rotations){
    return rotations * 360;
  }

  /**
   * Converts a quantity in degrees to radians.
   *
   * @param rotations The quantity in degrees to be converted.
   * @return The equivalent quantity in radians.
   */
  public static double degreesToRadians(double degrees){
    return degrees * Math.PI / 180;
  }

  /**
   * Calculates the x-component of a unit vector given an angle in radians.
   *
   * @param angle The angle in radians.
   * @return The x-component of the unit vector.
   */
  public static double angleToUnitVectorI(double angle){
    return (Math.cos(angle));
  }

  /**
   * Calculates the y-component of a unit vector given an angle in radians.
   *
   * @param angle The angle in radians.
   * @return The y-component of the unit vector.
   */
  public static double angleToUnitVectorJ(double angle){
    return (Math.sin(angle));
  }

  /**
   * Converts revolutions per minute (RPM) to revolutions per second (RPS).
   *
   * @param RPM The value in revolutions per minute (RPM) to be converted.
   * @return The equivalent value in revolutions per second (RPS).
   */
  public static double RPMToRPS(double RPM){
    return RPM / 60;
  }

  /**
   * Converts revolutions per second (RPS) to revolutions per minute (RPM).
   *
   * @param RPM The value in revolutions per second (RPS) to be converted.
   * @return The equivalent value in revolutions per minute (RPM).
   */
  public static double RPSToRPM(double RPS){
    return RPS * 60;
  }
}
