// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.HashMap;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.libs.NetworkTables;
import frc.robot.tests.Test;
import frc.robot.tests.TestClimber;
import frc.robot.tests.TestSwerve;
import frc.robot.tests.TestTurret;

public class TestRunner extends SubsystemBase {
  private static TestRunner instance = null;

  public enum TestType {
    SWERVE,
    TURRET,
    CLIMBER,
  };

  private final HashMap<TestType, Test> tests = new HashMap<TestType, Test>();

  public static TestRunner getInstance() {
    if (instance == null) {
      instance = new TestRunner();
    }
    return instance;
  }

  private TestRunner() {
    // Subsystems
    tests.put(TestType.SWERVE, new TestSwerve(NetworkTables.swerveButton_b, TestType.SWERVE));
    tests.put(TestType.TURRET, new TestTurret(NetworkTables.turretButton_b, TestType.TURRET));
    tests.put(TestType.CLIMBER, new TestClimber(NetworkTables.climberButton_b, TestType.CLIMBER));
  }

  @Override
  public void periodic() {
    for (TestType type : tests.keySet()) {
      tests.get(type).QueryNetworkTable();

      if (!tests.get(type).running)
        continue;

      tests.get(type).Periodic();
    }
  }

  public boolean isRunning(TestType type) {
    return tests.get(type).running;
  }

  public void setState(TestType type, boolean run) {
    if (tests.get(type).running == run)
      return;

    if (run) {
      tests.get(type).Start();
    } else {
      tests.get(type).Stop();
    }
  }

  public void updateStates() {
    for (TestType type : tests.keySet()) {
      setState(type, tests.get(type).ntEntry.getBoolean(false));
    }
  }

  public void stopAll() {
    for (TestType type : tests.keySet()) {
      setState(type, false);
    }
  }
}
