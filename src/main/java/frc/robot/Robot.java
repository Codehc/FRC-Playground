// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private RobotContainer m_robotContainer;
  private Command m_autonomousCommand;

  // Test a
  private SwerveModuleConstants c = new SwerveModuleConstants(5, 6, 7, false, true, 16, 4, 0.25, 0, 0, 0, 0.01, 0, 0);
  private SwerveModuleConstants c2 = new SwerveModuleConstants(9, 8, 10, false, true, 16, 4, 0.25, 0, 0, 0, 0.01, 0, 0);
  private SwerveModuleConstants c3 = new SwerveModuleConstants(12, 11, 13, false, true, 16, 4, 0.25, 0, 0, 0, 0.01, 0, 0);
  private SwerveModuleConstants c4 = new SwerveModuleConstants(14, 15, 16, false, true, 16, 4, 0.25, 0, 0, 0, 0.01, 0, 0);

  // Test b
  private SwerveModule fl;
  private SwerveModule fr;
  private SwerveModule bl;
  private SwerveModule br;

  private CommandXboxController controller;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    m_robotContainer = new RobotContainer();

    DataLogManager.start();

    CommandScheduler.getInstance().onCommandInitialize((Command command) -> {
      DataLogManager.log(command.getName() + " is initializing and requires: " + command.getRequirements().toString());
    });

    CommandScheduler.getInstance().onCommandInterrupt((Command command) -> {
      DataLogManager.log(command.getName() + " has been interrupted.");
    });

    CommandScheduler.getInstance().onCommandFinish((Command command) -> {
      DataLogManager.log(command.getName() + " is finishing on its own.");
    });

    controller = new CommandXboxController(0);

    // Test b
    fl = new SwerveModule(c);
    fr = new SwerveModule(c2);
    bl = new SwerveModule(c3);
    br = new SwerveModule(c4);
  }

  @Override 
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();

    fl.tick();
    fr.tick();
    bl.tick();
    br.tick();

    SmartDashboard.putNumber("LiveFl", fl.getAzimuth());
    SmartDashboard.putNumber("LiveFr", fr.getAzimuth());
    SmartDashboard.putNumber("LiveBl", bl.getAzimuth());
    SmartDashboard.putNumber("LiveBr", br.getAzimuth());
    SmartDashboard.putNumber("LiveFlA", fl.getAzimuthAbsolute());
    SmartDashboard.putNumber("LiveFrA", fr.getAzimuthAbsolute());
    SmartDashboard.putNumber("LiveBlA", bl.getAzimuthAbsolute());
    SmartDashboard.putNumber("LiveBrA", br.getAzimuthAbsolute());
  }

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
    DataLogManager.log("ENABLED AUTO");
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    DataLogManager.log("ENABLED TELEOP");
  }

  @Override
  public void teleopPeriodic() {
    SwerveModuleState state = new SwerveModuleState();
    double x = controller.getLeftX();
    double y = controller.getLeftY();
    state.speedMetersPerSecond = Math.sqrt(y * y + x * x) * 3;
    state.angle = Rotation2d.fromDegrees(x * 90);
    fl.setState(state);
    fr.setState(state);
    bl.setState(state);
    br.setState(state);
  }

  @Override
  public void disabledInit() {
    DataLogManager.log("DISABLED");
  }

  @Override
  public void disabledPeriodic() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
    DataLogManager.log("ENABLED SELF TEST");
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {}
}
