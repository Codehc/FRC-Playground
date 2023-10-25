// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.REVPhysicsSim;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;

import java.util.*;

/*
 * Some of this is adapted from 3005's 2022 Code
 * Original source published at https://github.com/FRC3005/Rapid-React-2022-Public/tree/d499655448ed592c85f9cfbbd78336d8841f46e2
 */

public class SpyderRobot {
  private static REVPhysicsSim revPhysicsSim = REVPhysicsSim.getInstance();
  private static List<NEO> neos = new ArrayList<>();

  public static void registerNEO(NEO neo) {
    neos.add(neo);

    revPhysicsSim.addSparkMax(neo, DCMotor.getNEO(1));
  }

  private static Alliance lastAlliance = null;

  public static void tick() {
    lastAlliance = DriverStation.getAlliance();

    for (NEO neo : neos) {
      neo.tick();
    }
  }

  public static void simulationTick() {
    revPhysicsSim.run();
  }

  /**
   * Run burnFlash() for all controllers initialized. burnFlash() stops comms w/ device for 200ms or more.
   * Might include calls from before method was called or calls from after. Too risky so we do this and burn everything in sync
   * to avoid accidentally stopping messages we send from getting to the device.
   */
  public static void burnSparkMaxFlashes() {
    Timer.delay(0.25);
    for (NEO neo : neos) {
      neo.burnFlash();
      Timer.delay(0.005);
    }
    Timer.delay(0.25);
  }

  public static Alliance getAlliance() {
    return lastAlliance;
  }
}
