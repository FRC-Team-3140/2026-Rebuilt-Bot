// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Feeder extends SubsystemBase {


  private static Feeder m_instance = null;
  public static Feeder getInstance() {
    if (m_instance == null) {
      m_instance = new Feeder();
    }
    return m_instance;
  }

  /** Creates a new Feeder. */
  public Feeder() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
