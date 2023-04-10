package frc.robot;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
  private RobotContainer m_robotContainer;
  private Command m_autonomousCommand;

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
  }

  @Override 
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    
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
