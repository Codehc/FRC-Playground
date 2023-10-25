package frc.robot;

public class SwerveConstants {
  public static double DRIVE_KFF;
  public static double DRIVE_KS;
  public static double DRIVE_KV;
  public static double DRIVE_KA;

  public static double DRIVE_KP;
  public static double DRIVE_KI;
  public static double DRIVE_KD;

  public static double AZIMUTH_KP;
  public static double AZIMUTH_KI;
  public static double AZIMUTH_KD;

  public static double FL_OFFSET;
  public static double FR_OFFSET;
  public static double BL_OFFSET;
  public static double BR_OFFSET;

  // Auto constants
  public static double TRANSLATION_KP;
  public static double TRANSLATION_KI;
  public static double TRANSLATION_KD;

  public static double ROTATION_KP;
  public static double ROTATION_KI;
  public static double ROTATION_KD;

  // Snap constants
  public static double SNAP_TRANSLATION_KP;
  public static double SNAP_TRANSLATION_KI;
  public static double SNAP_TRANSLATION_KD;

  public static double SNAP_ROTATION_KP;
  public static double SNAP_ROTATION_KI;
  public static double SNAP_ROTATION_KD;

  public static double AUTO_MAX_VELOCITY;
  public static double AUTO_MAX_ACCELERATION;
  public static double AUTO_MAX_ROTATIONS_PER_SECOND;

  public static double SNAP_MAX_VELOCITY;
  public static double SNAP_MAX_ACCELERATION;
  public static double SNAP_MAX_ROTATIONS_PER_SECOND;

  public static double SNAP_TRANSLATE_TOLERANCE;
  public static double SNAP_ROTATE_TOLERANCE;

  public static double SNAP_TRANSLATION_RATE;
  public static double SNAP_ROTATION_RATE;

  public static int ENCODER_MEASUREMENT_PERIOD = 16;
  public static int ENCODER_MEASUREMENT_DEPTH = 4;

  // Command constants
  public static double CHARGE_KP;
  public static double CHARGE_ANGLE_TOLERANCE;

  // Drive constants
  public static double MAX_VELOCITY;
  public static double MAX_X_ACCELERATION;
  public static double MAX_X_DECELERATION;
  public static double MAX_Y_ACCELERATION;
  public static double MAX_Y_DECELERATION;
  public static double MAX_ROTATE_ACCELERATION;
  public static double MAX_ROTATE_DECELERATION;
  public static double MAX_ROTATIONS_PER_SECOND;
  public static double SPEED_MODIFIER;

  // Odometry constants
  public static double VISION_MEASUREMENT_SPEED_THRESHOLD = 0.5;

  public static void populate() {
    DRIVE_KFF = 0.25;
      /*DRIVE_KFF = 0;
      DRIVE_KS = 0.29;
      DRIVE_KV = 0.125;
      DRIVE_KA = 0;*/

      DRIVE_KP = 0.4;
      DRIVE_KI = 0;
      DRIVE_KD = 0;

      AZIMUTH_KP = 0.02; // 0.1 normally
      AZIMUTH_KI = 0;
      AZIMUTH_KD = 0;

      FL_OFFSET = 109.6875;
      FR_OFFSET = 88.418;
      BL_OFFSET = 85.342;
      BR_OFFSET = 130.3417;

      // Auto constants
      TRANSLATION_KP = 1.1;//9;//9;
      TRANSLATION_KI = 0;
      TRANSLATION_KD = 0;

      ROTATION_KP = 2;//6;//3;
      ROTATION_KI = 0;
      ROTATION_KD = 0;

      SNAP_TRANSLATION_KP = 2;
      SNAP_TRANSLATION_KI = 0;
      SNAP_TRANSLATION_KD = 0;

      SNAP_ROTATION_KP = 9;
      SNAP_ROTATION_KI = 0;
      SNAP_ROTATION_KD = 0;

      SNAP_TRANSLATE_TOLERANCE = 0.01;
      // Degrees
      SNAP_ROTATE_TOLERANCE = 0.1;

      SNAP_TRANSLATION_RATE = 0;
      SNAP_ROTATION_RATE = 0;

      AUTO_MAX_VELOCITY = 2.5;
      AUTO_MAX_ACCELERATION = 2.5;
      AUTO_MAX_ROTATIONS_PER_SECOND = 0.5;

      SNAP_MAX_VELOCITY = 1;
      SNAP_MAX_ACCELERATION = 0.5;
      SNAP_MAX_ROTATIONS_PER_SECOND = 0.25;

      // Command constants
      CHARGE_KP = 0.7; // TODO: Get value
      CHARGE_ANGLE_TOLERANCE = 1.5;

      // Drive constants
      MAX_VELOCITY = 4;
      MAX_X_ACCELERATION = 8;
      MAX_X_DECELERATION = 8;
      MAX_Y_ACCELERATION = 8;
      MAX_Y_DECELERATION = 8;
      MAX_ROTATE_ACCELERATION = 6;
      MAX_ROTATE_DECELERATION = 6;
      MAX_ROTATIONS_PER_SECOND = 0.75;
      SPEED_MODIFIER = 0.25;
  }
}