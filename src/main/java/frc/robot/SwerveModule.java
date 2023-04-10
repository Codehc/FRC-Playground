// Developed by Reza from Team Spyder 1622

package frc.robot;

import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderStatusFrame;
import com.revrobotics.CANSparkMax.ControlType;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.NEO.StatusFrame;

import javax.naming.ldap.Control;

public class SwerveModule {
    private static final double WHEEL_DIAMETER = Units.inchesToMeters(4);

    private static final double DRIVE_GEAR_RATIO = 6.75; // L2 modules
    private static final double AZIMUTH_GEAR_RATIO = 150.0/7.0;

    private final SwerveModuleConstants swerveModuleConstants;

    private static final int DRIVE_FREE_CURRENT_LIMIT = 60;
    private static final int DRIVE_STALL_CURRENT_LIMIT = 40;
    private static final int AZIMUTH_CURRENT_LIMIT = 35;

    private final PIDController azimuthController;

    private SwerveModuleState desiredState;

    private final NEO azimuthMotor;
    private final NEO driveMotor;

    private CANCoder azimuthEncoder;

    private boolean initialized = false;

    /**
     * Instantiates a SwerveModule
     * @param swerveModuleConstants The constants to configure module with
     */
    public SwerveModule(SwerveModuleConstants swerveModuleConstants) {
        this.azimuthMotor = new NEO(swerveModuleConstants.getAzimuthID(), swerveModuleConstants.getAzimuthInverted());
        this.driveMotor = new NEO(swerveModuleConstants.getDriveID(), swerveModuleConstants.getDriveInverted());
        this.swerveModuleConstants = swerveModuleConstants;

        this.azimuthController = new PIDController(swerveModuleConstants.getAzimuthKP(), swerveModuleConstants.getAzimuthKI(), swerveModuleConstants.getAzimuthKD());

        this.driveMotor.getEncoder().setPositionConversionFactor((WHEEL_DIAMETER * Math.PI) / (DRIVE_GEAR_RATIO));
        this.driveMotor.getEncoder().setVelocityConversionFactor((WHEEL_DIAMETER * Math.PI) / (60 * DRIVE_GEAR_RATIO));

        this.driveMotor.getEncoder().setMeasurementPeriod(swerveModuleConstants.getVeloMeasurementPeriod());
        this.driveMotor.getEncoder().setAverageDepth(swerveModuleConstants.getVeloMeasurementDepth());

        this.driveMotor.setOpenLoopRampRate(0);
        this.driveMotor.setClosedLoopRampRate(0);

        this.driveMotor.disableVoltageComp();

        this.azimuthMotor.getEncoder().setPositionConversionFactor(360.0 / AZIMUTH_GEAR_RATIO);
        this.azimuthMotor.getEncoder().setVelocityConversionFactor(60 * 360.0 / AZIMUTH_GEAR_RATIO); // maybe wont work

        this.azimuthMotor.getEncoder().setMeasurementPeriod(swerveModuleConstants.getVeloMeasurementPeriod());
        this.azimuthMotor.getEncoder().setAverageDepth(swerveModuleConstants.getVeloMeasurementDepth());

        this.azimuthMotor.setClosedLoopRampRate(0);
        this.azimuthMotor.setOpenLoopRampRate(0);

        this.azimuthMotor.disableVoltageComp();

        // Smax uses normalized units not volts
        this.driveMotor.getPID().setP(swerveModuleConstants.getDriveKP());
        this.driveMotor.getPID().setI(swerveModuleConstants.getDriveKI());
        this.driveMotor.getPID().setD(swerveModuleConstants.getDriveKD());
        this.driveMotor.getPID().setFF(swerveModuleConstants.getDriveKFF());

        this.azimuthMotor.getPID().setP(swerveModuleConstants.getAzimuthKP());
        this.azimuthMotor.getPID().setI(swerveModuleConstants.getAzimuthKI());
        this.azimuthMotor.getPID().setD(swerveModuleConstants.getAzimuthKD());

        azimuthController.setTolerance(0.5);

        driveMotor.getMotor().setSmartCurrentLimit(DRIVE_STALL_CURRENT_LIMIT, DRIVE_FREE_CURRENT_LIMIT);
        azimuthMotor.setCurrentLimit(AZIMUTH_CURRENT_LIMIT);

        desiredState = new SwerveModuleState(0, Rotation2d.fromDegrees(getAzimuth()));

        azimuthController.enableContinuousInput(-180, 180);

        this.driveMotor.burn();
        this.azimuthMotor.burn();

        this.driveMotor.changeStatusFrame(StatusFrame.APPLIED_FAULTS_FOLLOWER, 65535);
        this.driveMotor.resetStatusFrame(StatusFrame.VELO_TEMP_VOLTAGE_CURRENT);
        this.driveMotor.changeStatusFrame(StatusFrame.POSITION, 65535);
        this.driveMotor.changeStatusFrame(StatusFrame.ANALOG_VOLTAGE_VELO_POS, 65535);
        this.driveMotor.changeStatusFrame(StatusFrame.ALTERNATE_VELO_POS, 65535);
        this.driveMotor.changeStatusFrame(StatusFrame.ABSOLUTE_ENCODER_POS, 65535);
        this.driveMotor.changeStatusFrame(StatusFrame.ABSOLUTE_ENCODER_VELO, 65535);

        this.azimuthMotor.changeStatusFrame(StatusFrame.APPLIED_FAULTS_FOLLOWER, 65535);
        this.azimuthMotor.changeStatusFrame(StatusFrame.VELO_TEMP_VOLTAGE_CURRENT, 65535);
        this.azimuthMotor.resetStatusFrame(StatusFrame.POSITION);
        this.azimuthMotor.changeStatusFrame(StatusFrame.ANALOG_VOLTAGE_VELO_POS, 65535);
        this.azimuthMotor.changeStatusFrame(StatusFrame.ALTERNATE_VELO_POS, 65535);
        this.azimuthMotor.changeStatusFrame(StatusFrame.ABSOLUTE_ENCODER_POS, 65535);
        this.azimuthMotor.changeStatusFrame(StatusFrame.ABSOLUTE_ENCODER_VELO, 65535);

        SmartDashboard.putBoolean("Init", false);
        Commands.waitSeconds(5).andThen(Commands.runOnce(() -> {
            SmartDashboard.putBoolean("Init", true);
            initialized = true;
            azimuthEncoder = new CANCoder(swerveModuleConstants.getCanCoderID());
            azimuthEncoder.setStatusFramePeriod(CANCoderStatusFrame.VbatAndFaults, 255);
            seedAzimuthEncoder();
        })).ignoringDisable(true).schedule();
        
        this.azimuthMotor.getPID().setPositionPIDWrappingEnabled(true);
        this.azimuthMotor.getPID().setPositionPIDWrappingMinInput(-180);
        this.azimuthMotor.getPID().setPositionPIDWrappingMaxInput(180);
    }

    public void tick() {
        this.driveMotor.tick();
        this.azimuthMotor.tick();
    }

    /**
     * Sets the swerve modules target state. Needs to be called periodically
     * @param state Target state for the swerve module
     */
    public void setState(SwerveModuleState state) {
        /*if (SmartDashboard.getNumber("Azimuth kP", swervePIDConstants.getAzimuthKP()) != swervePIDConstants.getAzimuthKP()) {
            swervePIDConstants.setAzimuthKP(SmartDashboard.getNumber("Azimuth kP", swervePIDConstants.getAzimuthKP()));
            azimuthController.setP(swervePIDConstants.getAzimuthKP());
        }
        if (SmartDashboard.getNumber("Azimuth kI", swervePIDConstants.getAzimuthKI()) != swervePIDConstants.getAzimuthKI()) {
            swervePIDConstants.setAzimuthKI(SmartDashboard.getNumber("Azimuth kI", swervePIDConstants.getAzimuthKI()));
            azimuthController.setI(swervePIDConstants.getAzimuthKI());
        }
        if (SmartDashboard.getNumber("Azimuth kD", swervePIDConstants.getAzimuthKD()) != swervePIDConstants.getAzimuthKD()) {
            swervePIDConstants.setAzimuthKD(SmartDashboard.getNumber("Azimuth kD", swervePIDConstants.getAzimuthKD()));
            azimuthController.setD(swervePIDConstants.getAzimuthKD());
        }
        if (SmartDashboard.getNumber("Drive kP", swervePIDConstants.getDriveKP()) != swervePIDConstants.getDriveKP()) {
            swervePIDConstants.setDriveKP(SmartDashboard.getNumber("Drive kP", swervePIDConstants.getDriveKP()));
            driveController.setP(swervePIDConstants.getDriveKP());
            driveMotor.getPID().setP(swervePIDConstants.getDriveKP());
        }
        if (SmartDashboard.getNumber("Drive kI", swervePIDConstants.getDriveKI()) != swervePIDConstants.getDriveKI()) {
            swervePIDConstants.setDriveKI(SmartDashboard.getNumber("Drive kI", swervePIDConstants.getDriveKI()));
            driveController.setI(swervePIDConstants.getDriveKI());
            driveMotor.getPID().setI(swervePIDConstants.getDriveKI());
        }
        if (SmartDashboard.getNumber("Drive kD", swervePIDConstants.getDriveKD()) != swervePIDConstants.getDriveKD()) {
            swervePIDConstants.setDriveKD(SmartDashboard.getNumber("Drive kD", swervePIDConstants.getDriveKD()));
            driveController.setD(swervePIDConstants.getDriveKD());
            driveMotor.getPID().setD(swervePIDConstants.getDriveKD());
        }
        if (SmartDashboard.getNumber("Drive kV", swervePIDConstants.getDriveKV()) != swervePIDConstants.getDriveKV()) {
            swervePIDConstants.setDriveKV(SmartDashboard.getNumber("Drive kV", swervePIDConstants.getDriveKV()));
            driveMotor.getPID().setFF(swervePIDConstants.getDriveKV());
        }*/

        if (!initialized) return;

        state = SwerveModuleState.optimize(state, Rotation2d.fromDegrees(getAzimuth()));

        this.desiredState = state;

        double targetSpeed = state.speedMetersPerSecond;

        driveMotor.getPID().setReference(targetSpeed, ControlType.kVelocity);

        /*double targetAzimuth = state.angle.getDegrees();
        double azimuthPIDOutput = azimuthController.calculate(azimuthEncoder.getAbsolutePosition(), targetAzimuth);
        azimuthMotor.setVoltage(azimuthPIDOutput);*/
        azimuthMotor.getPID().setReference(state.angle.getDegrees(), ControlType.kPosition);
    }

    public double getDriveCurrentDraw() {
        return driveMotor.getCurrent();
    }

    public double getAzimuthCurrentDraw() {
        return azimuthMotor.getCurrent();
    }

    public double getDriveVoltageDraw() {
        return driveMotor.getMotor().getAppliedOutput();
    }

    public double getAzimuthVoltageDraw() {
        return azimuthMotor.getMotor().getAppliedOutput();
    }

    /**
     * Turns off the swerve module. Turns off motors
     */
    public void turnOffModule() {
        driveMotor.setVoltage(0);
        azimuthMotor.set(0);
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


    public double getAzimuthAbsolute() {
        if (azimuthEncoder == null) {
            return 0;
        }
        return azimuthEncoder.getAbsolutePosition();
    }

    /**
     * Gets the current actual state of the module
     * @return The current actual state of the module
     */
    public SwerveModuleState getState() {
        return new SwerveModuleState(getSpeed(), Rotation2d.fromDegrees(getAzimuth()));
    }

    /**
     * Gets the desired state of the module
     * @return The current desired state of the module
     */
    public SwerveModuleState getDesiredState() {
        return desiredState;
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(driveMotor.getPosition(), Rotation2d.fromDegrees(getAzimuth()));
    }

    public void seedAzimuthEncoder() {
        azimuthMotor.getMotor().getEncoder().setPosition(azimuthEncoder.getAbsolutePosition());
    }

    public boolean isInitialized() {
        return initialized;
    }
}