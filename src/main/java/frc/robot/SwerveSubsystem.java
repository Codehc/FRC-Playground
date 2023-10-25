// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Tracer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class SwerveSubsystem implements Subsystem {

  public SwerveSubsystem() {

    SwerveConstants.populate();

    frontLeftConstants = new SwerveModuleConstants(
            CANID.FRONT_LEFT_DRIVE, CANID.FRONT_LEFT_AZIMUTH, CANID.FRONT_LEFT_AZIMUTH_ENCODER,
            false, true,
            SwerveConstants.ENCODER_MEASUREMENT_PERIOD, SwerveConstants.ENCODER_MEASUREMENT_DEPTH,
            SwerveConstants.DRIVE_KFF, 0, 0, 0,
            SwerveConstants.DRIVE_KP, SwerveConstants.DRIVE_KI, SwerveConstants.DRIVE_KD,
            SwerveConstants.AZIMUTH_KP, SwerveConstants.AZIMUTH_KI, SwerveConstants.AZIMUTH_KD
    );

    // frontRight
    frontRightConstants = new SwerveModuleConstants(
            CANID.FRONT_RIGHT_DRIVE, CANID.FRONT_RIGHT_AZIMUTH, CANID.FRONT_RIGHT_AZIMUTH_ENCODER,
            true, true,
            SwerveConstants.ENCODER_MEASUREMENT_PERIOD, SwerveConstants.ENCODER_MEASUREMENT_DEPTH,
            SwerveConstants.DRIVE_KFF, 0, 0, 0,
            SwerveConstants.DRIVE_KP, SwerveConstants.DRIVE_KI, SwerveConstants.DRIVE_KD,
            SwerveConstants.AZIMUTH_KP, SwerveConstants.AZIMUTH_KI, SwerveConstants.AZIMUTH_KD
    );

    // backLeft
    backLeftConstants = new SwerveModuleConstants(
            CANID.BACK_LEFT_DRIVE, CANID.BACK_LEFT_AZIMUTH, CANID.BACK_LEFT_AZIMUTH_ENCODER,
            false, true,
            SwerveConstants.ENCODER_MEASUREMENT_PERIOD, SwerveConstants.ENCODER_MEASUREMENT_DEPTH,
            SwerveConstants.DRIVE_KFF, 0, 0, 0,
            SwerveConstants.DRIVE_KP, SwerveConstants.DRIVE_KI, SwerveConstants.DRIVE_KD,
            SwerveConstants.AZIMUTH_KP, SwerveConstants.AZIMUTH_KI, SwerveConstants.AZIMUTH_KD
    );

    // backRight
    backRightConstants = new SwerveModuleConstants(
            CANID.BACK_RIGHT_DRIVE, CANID.BACK_RIGHT_AZIMUTH, CANID.BACK_RIGHT_AZIMUTH_ENCODER,
            true, true,
            SwerveConstants.ENCODER_MEASUREMENT_PERIOD, SwerveConstants.ENCODER_MEASUREMENT_DEPTH,
            SwerveConstants.DRIVE_KFF, 0, 0, 0,
            SwerveConstants.DRIVE_KP, SwerveConstants.DRIVE_KI, SwerveConstants.DRIVE_KD,
            SwerveConstants.AZIMUTH_KP, SwerveConstants.AZIMUTH_KI, SwerveConstants.AZIMUTH_KD
    );

    frontLeftModule = new SwerveModule(frontLeftConstants, "Front Left");
    frontRightModule = new SwerveModule(frontRightConstants, "Front Right");
    backLeftModule = new SwerveModule(backLeftConstants, "Back Left");
    backRightModule = new SwerveModule(backRightConstants, "Back Right");

    swerveModules = new SwerveModule[] {
            frontLeftModule,
            frontRightModule,
            backLeftModule,
            backRightModule
    };

    SmartDashboard.putData("Pose", field);

    if (Robot.isSimulation()) {
      simAngle = Rotation2d.fromDegrees(0);
    }

    swerveDrivePoseEstimator = new SwerveDrivePoseEstimator(swerveDriveKinematics, getGyroAngle(), getModulePositions(), new Pose2d(), odoStdDevs, visionStdDevs);

    register();
  }

  private Rotation2d simAngle;
  private ChassisSpeeds desiredChassisSpeeds = new ChassisSpeeds();

  // frontLeft
  private final SwerveModuleConstants frontLeftConstants;
  private final SwerveModule frontLeftModule;

  // frontRight
  private final SwerveModuleConstants frontRightConstants;
  private final SwerveModule frontRightModule;

  // backLeft
  private final SwerveModuleConstants backLeftConstants;
  private final SwerveModule backLeftModule;

  // backRight
  private final SwerveModuleConstants backRightConstants;
  private final SwerveModule backRightModule;

  // frontLeft, frontRight, backLeft, backRight
  private final SwerveModule[] swerveModules;

  // Wheel base length in meters (from center of module to center of other module from BACK TO FRONT)
  private static final double wheelBaseLength = Units.inchesToMeters(32-2.625*2);
  // Wheel base width in meters (from center of module to center of other module from LEFT TO RIGHT)
  private static final double wheelBaseWidth = Units.inchesToMeters(26-2.625*2);

  private static final Translation2d frontLeftLocation = new Translation2d(wheelBaseLength / 2, wheelBaseWidth / 2);
  private static final Translation2d frontRightLocation = new Translation2d(wheelBaseLength / 2, -wheelBaseWidth / 2);
  private static final Translation2d backLeftLocation = new Translation2d(-wheelBaseLength / 2, wheelBaseWidth / 2);
  private static final Translation2d backRightLocation = new Translation2d(-wheelBaseLength / 2, -wheelBaseWidth / 2);

  private final Vector<N3> odoStdDevs = VecBuilder.fill(0.1, 0.1, 0.1);
  private final Vector<N3> visionStdDevs = VecBuilder.fill(0.5, 0.5, 0.5);
  //private final Vector<N3> visionStdDevs = VecBuilder.fill(2, 2, 2.3);
  // 2, 2, 2.3 vision
  //private final Vector<N3> visionStdDevs = VecBuilder.fill(0.45, 0.45, Math.toRadians(15));

  private final SwerveDriveKinematics swerveDriveKinematics = new SwerveDriveKinematics(frontLeftLocation, frontRightLocation, backLeftLocation, backRightLocation);
  private final SwerveDrivePoseEstimator swerveDrivePoseEstimator;

  private final Field2d field = new Field2d();

  private SwerveModuleState[] targetStates = new SwerveModuleState[] {new SwerveModuleState(), new SwerveModuleState(), new SwerveModuleState(), new SwerveModuleState()};

  private Tracer tracer = new Tracer();

  @Override
  public void periodic() {
    simAngle = simAngle.plus(Rotation2d.fromRadians(desiredChassisSpeeds.omegaRadiansPerSecond * 0.02));
    SmartDashboard.putNumber("Sim angle", simAngle.getDegrees());

    field.setRobotPose(getPose());

    ChassisSpeeds robotCompSpeeds = swerveDriveKinematics.toChassisSpeeds(targetStates[0], targetStates[1], targetStates[2], targetStates[3]);
    double speed = Math.sqrt(Math.pow(robotCompSpeeds.vxMetersPerSecond, 2) + Math.pow(robotCompSpeeds.vyMetersPerSecond, 2));

    SmartDashboard.putNumber("Swerve/Target Velos/X", robotCompSpeeds.vxMetersPerSecond);
    SmartDashboard.putNumber("Swerve/Target Velos/Y", robotCompSpeeds.vyMetersPerSecond);
    SmartDashboard.putNumber("Swerve/Target Velos/Theta", robotCompSpeeds.omegaRadiansPerSecond);
    SmartDashboard.putNumber("Swerve/Target Velos/Velocity", speed);

    SmartDashboard.putNumberArray("Swerve/Target Swerve Module States", new double[] {
            targetStates[0].angle.getDegrees(), targetStates[0].speedMetersPerSecond,
            targetStates[1].angle.getDegrees(), targetStates[1].speedMetersPerSecond,
            targetStates[2].angle.getDegrees(), targetStates[2].speedMetersPerSecond,
            targetStates[3].angle.getDegrees(), targetStates[3].speedMetersPerSecond,
    });

    ChassisSpeeds robotRelativeSpeed = getRobotRelativeSpeed();
    SmartDashboard.putNumber("Swerve/Actual Velos/X", robotRelativeSpeed.vxMetersPerSecond);
    SmartDashboard.putNumber("Swerve/Actual Velos/Y", robotRelativeSpeed.vyMetersPerSecond);
    SmartDashboard.putNumber("Swerve/Actual Velos/Theta", robotRelativeSpeed.omegaRadiansPerSecond);
    SmartDashboard.putNumber("Swerve/Actual Velos/Velocity", getOverallSpeed());

    double flSpeed = frontLeftModule.getSpeed();
    double frSpeed = frontRightModule.getSpeed();
    double blSpeed = backLeftModule.getSpeed();
    double brSpeed = backRightModule.getSpeed();

    double flAngle = frontLeftModule.getAzimuth();
    double frAngle = frontRightModule.getAzimuth();
    double blAngle = backLeftModule.getAzimuth();
    double brAngle = backRightModule.getAzimuth();

    SmartDashboard.putNumberArray("Swerve/Actual Swerve Module States", new double[] {
            flAngle, flSpeed,
            frAngle, frSpeed,
            blAngle, blSpeed,
            brAngle, brSpeed
    });

    tracer.addEpoch("Logged");

    for (int i = 0; i < swerveModules.length; i++) {
      swerveModules[i].setState(targetStates[i]);
    }

    tracer.addEpoch("Set States");

    tracer.printEpochs();

    updateOdometry();
  }

  public Rotation2d getGyroAngle() {
    return simAngle;
  }

  public void updateOdometry() {
    // Update odometry
    swerveDrivePoseEstimator.update(getGyroAngle(), getModulePositions());
  }
  
  /**
   * Drives the robot (field oriented control)
   * @param translateX Field relative X speed (m/s)
   * @param translateY Field relative Y speed (m/s)
   * @param rotate Rotation speed (rad/s)
   */
  public void driveFieldRelative(double translateX, double translateY, double rotate) {
    // Get the desired robot relative speeds from the desired field relative speeds
    ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(translateX, translateY, rotate, getGyroAngle());

    drive(chassisSpeeds);
  }

  /**
   * Drives the robot (robot oriented control)
   * @param translateX Robot relative X speed (m/s)
   * @param translateY Robot relative Y speed (m/s)
   * @param rotate Rotation speed (rad/s)
   */
  public void driveRobotRelative(double translateX, double translateY, double rotate) {
    // Create a chassisSpeeds object with the desired robot relative speeds
    ChassisSpeeds chassisSpeeds = new ChassisSpeeds(translateX, translateY, rotate);

    drive(chassisSpeeds);
  }

  /**
   * Drives the robot according to specified chassisSpeeds
   * @param chassisSpeeds The desired chassisSpeeds to drive the robot at
   */
  public void drive(ChassisSpeeds chassisSpeeds) {
    // Account for swerve drive drift while rotating
    chassisSpeeds = discretize(chassisSpeeds);

    desiredChassisSpeeds = chassisSpeeds;

    // Convert the robot relative speeds to swerve module states
    SwerveModuleState[] moduleStates = getSwerveDriveKinematics().toSwerveModuleStates(chassisSpeeds);

    // Normalize the speeds so that no module is going faster than the max speed
    SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, SwerveConstants.MAX_VELOCITY);

    // Set every swerve modules state to the desired state
    setModuleStates(moduleStates);
  }

  /**
   * Sets the state of every swerve module to the specified states
   * @param moduleStates The desired states to set the module states to
   */
  public void setModuleStates(SwerveModuleState[] moduleStates) {
    targetStates = moduleStates;
  }

  /**
   * Resets the robot's estimated pose
   * @param pose The pose to reset to
   */
  public void resetPoseEstimation(Pose2d pose) {
    swerveDrivePoseEstimator.resetPosition(getGyroAngle(), getModulePositions(), pose);
  }

  /**
   * Locks the swerve modules into the locked position
   */
  public void lockModules() {
    // Set the modules into the locked position
    // This will create a "X" pattern with the modules which will make the robot very difficult to rotate or move
    setModuleStates(new SwerveModuleState[]{
            new SwerveModuleState(0, Rotation2d.fromDegrees(45)),
            new SwerveModuleState(0, Rotation2d.fromDegrees(-45)),
            new SwerveModuleState(0, Rotation2d.fromDegrees(-45)),
            new SwerveModuleState(0, Rotation2d.fromDegrees(45))
    });
  }

  /**
   * Stops driving the modules but doesn't rotate them
   */
  public void stopDriving() {
    // Keep the modules' azimuth but stop driving
    setModuleStates(new SwerveModuleState[]{
            new SwerveModuleState(0, Rotation2d.fromDegrees(frontLeftModule.getAzimuth())),
            new SwerveModuleState(0, Rotation2d.fromDegrees(frontRightModule.getAzimuth())),
            new SwerveModuleState(0, Rotation2d.fromDegrees(backLeftModule.getAzimuth())),
            new SwerveModuleState(0, Rotation2d.fromDegrees(backRightModule.getAzimuth()))
    });
  }

  // TODO: Tune this to check to see if 4 is a good value. Source for 4 is 4738. <3 Alexander Hamilton
  public ChassisSpeeds discretize(ChassisSpeeds speeds) {
    double dt = 0.02;
    var desiredDeltaPose = new Pose2d(
            speeds.vxMetersPerSecond * dt,
            speeds.vyMetersPerSecond * dt,
            new Rotation2d(speeds.omegaRadiansPerSecond * dt * 4)
    );
    var twist = new Pose2d().log(desiredDeltaPose);

    return new ChassisSpeeds((twist.dx / dt), (twist.dy / dt), (speeds.omegaRadiansPerSecond));
  }

  public Rotation2d getAngle() {
    return swerveDrivePoseEstimator.getEstimatedPosition().getRotation();
  }

  /**
   * Get the robot's robot relative speed
   * @return The robot's robot relative speed
   */
  public ChassisSpeeds getRobotRelativeSpeed() {
    // Converts the module states to robot relative speeds
    return swerveDriveKinematics.toChassisSpeeds(frontLeftModule.getState(), frontRightModule.getState(), backLeftModule.getState(), backRightModule.getState());
  }

  /**
   * Get the robot's overall speed
   * @return The robot's speed in whatever direction it's going
   */
  public double getOverallSpeed() {
    // Get the robot's overall speed in whatever direction it's going (not velocity)

    // Converts the module states to robot relative speeds
    ChassisSpeeds robotCompSpeeds = getRobotRelativeSpeed();

    // Use pythagorean theorem to convert robot relative speeds in the X and Y directions to a single speed
    return Math.sqrt(Math.pow(robotCompSpeeds.vxMetersPerSecond, 2) + Math.pow(robotCompSpeeds.vyMetersPerSecond, 2));
  }

  /**
   * Gets the robot's field relative position
   * @return The robot's field relative position
   */
  public Pose2d getPose() {
    return swerveDrivePoseEstimator.getEstimatedPosition();
  }

  /**
   * Gets the array of swerve modules
   * @return The array of swerve module objects
   */
  public SwerveModule[] getSwerveModules() {
    return swerveModules;
  }

  /**
   * Gets the swerve drive kinematics object
   * @return The swerve drive kinematics object for this robot
   */
  public SwerveDriveKinematics getSwerveDriveKinematics() {
    return swerveDriveKinematics;
  }

  public SwerveModulePosition[] getModulePositions() {
    return new SwerveModulePosition[] {
            frontLeftModule.getPosition(),
            frontRightModule.getPosition(),
            backLeftModule.getPosition(),
            backRightModule.getPosition()
    };
  }

  public Field2d getField() {
    return field;
  }

  public Command getLockCommand() {
    return Commands.runOnce(this::lockModules);
  }

  public Command unlimpModules() {
    return Commands.runOnce(() -> {
      for (SwerveModule module : swerveModules) {
        module.unlimpModule();
      }
    }).ignoringDisable(true);
  }

  public Command limpModules() {
    return Commands.runOnce(() -> {
      for (SwerveModule module : swerveModules) {
        module.limpModule();
      }
    }).ignoringDisable(true);
  }
}