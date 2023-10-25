// Developed by Reza from Team Spyder 1622

package frc.robot;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderStatusFrame;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.NEO.StatusFrame;

public class SwerveModule {
    private static final double WHEEL_DIAMETER = Units.inchesToMeters(4);

    private static final double DRIVE_GEAR_RATIO = 6.75; // L2 modules
    private static final double AZIMUTH_GEAR_RATIO = 150.0/7.0;

    private final SwerveModuleConstants swerveModuleConstants;
    private final String name;

    private static final int DRIVE_FREE_CURRENT_LIMIT = 60;
    private static final int DRIVE_STALL_CURRENT_LIMIT = 40;
    private static final int AZIMUTH_CURRENT_LIMIT = 35;

    private final NEO azimuthMotor;
    private final NEO driveMotor;

    private CANCoder azimuthEncoder;

    private boolean initialized = false;

    /**
     * Instantiates a SwerveModule
     * @param swerveModuleConstants The constants to configure module with
     */
    public SwerveModule(SwerveModuleConstants swerveModuleConstants, String name) {
        this.name = name;
        this.azimuthMotor = new NEO(swerveModuleConstants.getAzimuthID());
        this.driveMotor = new NEO(swerveModuleConstants.getDriveID(), swerveModuleConstants.getDriveInverted());
        this.swerveModuleConstants = swerveModuleConstants;
        
        driveMotor.encoder.setPositionConversionFactor((WHEEL_DIAMETER * Math.PI) / (DRIVE_GEAR_RATIO));
        driveMotor.encoder.setVelocityConversionFactor((WHEEL_DIAMETER * Math.PI) / (60 * DRIVE_GEAR_RATIO));
        
        driveMotor.encoder.setMeasurementPeriod(swerveModuleConstants.getVeloMeasurementPeriod());
        driveMotor.encoder.setAverageDepth(swerveModuleConstants.getVeloMeasurementDepth());
        
        driveMotor.setSmartCurrentLimit(DRIVE_STALL_CURRENT_LIMIT, DRIVE_FREE_CURRENT_LIMIT);
        
        driveMotor.changeStatusFrame(StatusFrame.APPLIED_FAULTS_FOLLOWER, 65535);
        driveMotor.resetStatusFrame(StatusFrame.VELO_TEMP_VOLTAGE_CURRENT);
        driveMotor.resetStatusFrame(StatusFrame.POSITION);
        driveMotor.changeStatusFrame(StatusFrame.ANALOG_VOLTAGE_VELO_POS, 65535);
        driveMotor.changeStatusFrame(StatusFrame.ALTERNATE_VELO_POS, 65535);
        driveMotor.changeStatusFrame(StatusFrame.ABSOLUTE_ENCODER_POS, 65535);
        driveMotor.changeStatusFrame(StatusFrame.ABSOLUTE_ENCODER_VELO, 65535);
        
        driveMotor.pidController.setP(swerveModuleConstants.getDriveKP());
        driveMotor.pidController.setI(swerveModuleConstants.getDriveKI());
        driveMotor.pidController.setD(swerveModuleConstants.getDriveKD());
        driveMotor.pidController.setFF(swerveModuleConstants.getDriveKFF());
        
        azimuthMotor.encoder.setPositionConversionFactor(360.0 / AZIMUTH_GEAR_RATIO);
        azimuthMotor.encoder.setVelocityConversionFactor(360.0 / (AZIMUTH_GEAR_RATIO * 60));
        
        azimuthMotor.encoder.setMeasurementPeriod(swerveModuleConstants.getVeloMeasurementPeriod());
        azimuthMotor.encoder.setAverageDepth(swerveModuleConstants.getVeloMeasurementDepth());
        
        azimuthMotor.setSmartCurrentLimit(AZIMUTH_CURRENT_LIMIT);

        azimuthMotor.changeStatusFrame(StatusFrame.APPLIED_FAULTS_FOLLOWER, 65535);
        azimuthMotor.changeStatusFrame(StatusFrame.VELO_TEMP_VOLTAGE_CURRENT, 65535);
        azimuthMotor.resetStatusFrame(StatusFrame.POSITION);
        azimuthMotor.changeStatusFrame(StatusFrame.ANALOG_VOLTAGE_VELO_POS, 65535);
        azimuthMotor.changeStatusFrame(StatusFrame.ALTERNATE_VELO_POS, 65535);
        azimuthMotor.changeStatusFrame(StatusFrame.ABSOLUTE_ENCODER_POS, 65535);
        azimuthMotor.changeStatusFrame(StatusFrame.ABSOLUTE_ENCODER_VELO, 65535);

        azimuthMotor.pidController.setP(swerveModuleConstants.getAzimuthKP());
        azimuthMotor.pidController.setI(swerveModuleConstants.getAzimuthKI());
        azimuthMotor.pidController.setD(swerveModuleConstants.getAzimuthKD());
        azimuthMotor.pidController.setPositionPIDWrappingEnabled(true);
        azimuthMotor.pidController.setPositionPIDWrappingMinInput(-180);
        azimuthMotor.pidController.setPositionPIDWrappingMaxInput(180);

        Commands.waitSeconds(2).andThen(Commands.runOnce(() -> {
            initialized = true;
            azimuthEncoder = new CANCoder(swerveModuleConstants.getCanCoderID());
            azimuthEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180);
            azimuthEncoder.setStatusFramePeriod(CANCoderStatusFrame.VbatAndFaults, 255);
            seedAzimuthEncoder();
        })).ignoringDisable(true).schedule();

        // TODO: Get rid of this, this is just to test my proto code
        //SpyderRobot.tuneDouble("Swerve/Drive kP", this.swerveModuleConstants::getDriveKP, this.swerveModuleConstants::setDriveKP);
    }

    /**
     * Sets the swerve modules target state.
     * @param state Target state for the swerve module
     * @param openLoop Whether or not to run the module in open loop
     */
    public void setState(SwerveModuleState state, boolean openLoop) {
        state = SwerveModuleState.optimize(state, Rotation2d.fromDegrees(getAzimuth()));

        double targetSpeed = state.speedMetersPerSecond;
        double targetAngle = state.angle.getDegrees();

        if (!initialized) return;

        openLoop = false;
        
        if (targetSpeed != driveMotor.getTargetVelocity()) {
            driveMotor.setTargetVelocity(targetSpeed);
        }

       /*if (targetSpeed != driveMotor.getTargetVelocity() && !openLoop) {
            driveMotor.setTargetVelocity(targetSpeed);
        } else {
            driveMotor.set(targetSpeed * swerveModuleConstants.getDriveKFF());
        }*/

        if (targetAngle != azimuthMotor.getTargetPosition()) {
            azimuthMotor.setTargetPosition(targetAngle);
        }
    }

    public void setState(SwerveModuleState state) {
        setState(state, false);
    }

    /**
     * Turns off the swerve module. Turns off motors
     */
    public void turnOffModule() {
        driveMotor.setVoltage(0);
        azimuthMotor.setVoltage(0);
    }

    public void limpModule() {
        driveMotor.setIdleMode(IdleMode.kCoast);
        azimuthMotor.setIdleMode(IdleMode.kCoast);
    }

    public void unlimpModule() {
        driveMotor.setIdleMode(IdleMode.kBrake);
        azimuthMotor.setIdleMode(IdleMode.kBrake);
    }

    /**
     * Gets the current drive speed of the module
     * @return The current drive speed of the module in meters per second
     */
    public double getSpeed() {
        return driveMotor.getVelocity();
    }

    /**
     * Gets the current azimuth of the module using the integrated NEO encoder
     * @return The current azimuth of the module
     */
    public double getAzimuth() {
        return azimuthMotor.getPosition();
    }

    /**
     * Gets the current actual state of the module
     * @return The current actual state of the module
     */
    public SwerveModuleState getState() {
        return new SwerveModuleState(getSpeed(), Rotation2d.fromDegrees(getAzimuth()));
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(driveMotor.getPosition(), Rotation2d.fromDegrees(getAzimuth()));
    }

    public void seedAzimuthEncoder() {
        if (Robot.isReal()) {
            azimuthMotor.setPosition(azimuthEncoder.getAbsolutePosition());
        } else {
            azimuthMotor.setPosition(0);
        }
    }
}