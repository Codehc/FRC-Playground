package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.REVPhysicsSim;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
  private RobotContainer m_robotContainer;
  private Command m_autonomousCommand;

  private NEO neo = new NEO(0);

  private SwerveSubsystem swerveSubsystem = new SwerveSubsystem();

  private PS4Controller controller = new PS4Controller(0);
  private PS4Controller controller2 = new PS4Controller(1);

  @Override
  public void robotInit() {
    m_robotContainer = new RobotContainer();

    DataLogManager.start("./sim-logs");

    CommandScheduler.getInstance().onCommandInitialize((Command command) -> {
      DataLogManager.log(command.getName() + " is initializing and requires: " + command.getRequirements().toString());
    });

    CommandScheduler.getInstance().onCommandInterrupt((Command command) -> {
      DataLogManager.log(command.getName() + " has been interrupted.");
    });

    CommandScheduler.getInstance().onCommandFinish((Command command) -> {
      DataLogManager.log(command.getName() + " is finishing on its own.");
    });

    SmartDashboard.putNumber("Input Voltage", 0);
    SmartDashboard.putNumber("Target Position", 0);
    SmartDashboard.putNumber("Times updated", 0);

    //neo.encoder.getVelocity()

    neo.pidController.setP(0.01);
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
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    DataLogManager.log("ENABLED TELEOP");
  }

  @Override
  public void teleopPeriodic() {
    /*if (neo.getTargetVelocity() != SmartDashboard.getNumber("Target Position", 0)) {
      SmartDashboard.putNumber("Times updated", SmartDashboard.getNumber("Times updated", 0) + 1);
      neo.setTargetVelocity(SmartDashboard.getNumber("Target Position", 0));
    }

    SpyderRobot.tick();

    SmartDashboard.putNumber("Rotations", neo.getPosition());
    SmartDashboard.putNumber("Velocity", neo.getVelocity());
    SmartDashboard.putNumber("Applied Output", neo.getAppliedOutput());*/

    double leftY = -3 * controller.getLeftY();
    double leftX = -3 * controller.getLeftX();
    double rightX = -1.4 * controller.getRightX();

    leftY = Utils.deadband(leftY, 0.2);
    leftX = Utils.deadband(leftX, 0.2);
    rightX = Utils.deadband(rightX, 0.1);

    swerveSubsystem.driveFieldRelative(leftY, leftX, rightX);

    swerveSubsystem.periodic();

    SpyderRobot.tick();
  }

  @Override
  public void disabledInit() {
    DataLogManager.log("DISABLED");
  }

  @Override
  public void disabledPeriodic() {
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
    DataLogManager.log("ENABLED SELF TEST");
  }

  @Override
  public void testPeriodic() {
  }

  @Override
  public void simulationInit() {
  }

  @Override
  public void simulationPeriodic() {
    SpyderRobot.simulationTick();
  }
}
