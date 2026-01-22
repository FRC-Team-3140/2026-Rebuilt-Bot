// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climbers extends SubsystemBase {
  private static Climbers m_Instance = null;

  private final SparkMax lClimber = new SparkMax(Constants.MotorIDs.climberLeftMotor, MotorType.kBrushless);
  private final SparkMax rClimber = new SparkMax(Constants.MotorIDs.climberRightMotor, MotorType.kBrushless);

  private final DigitalInput climberLimitSwitchL = new DigitalInput(Constants.SensorIDs.climberLimitSwitchLeft);
  private final DigitalInput climberLimitSwitchR = new DigitalInput(Constants.SensorIDs.cllimberLimitSwitchRight);

  public static Climbers getInstance() {
    if (m_Instance == null) {
      m_Instance = new Climbers();
    }

    return m_Instance;
  }

  public static class climber {
    public static final climber LEFT = new climber();
    public static final climber RIGHT = new climber();
  }

  /** Creates a new Climber. */
  public Climbers() {

    lClimber.setInverted(true);
    rClimber.setInverted(false);
  }

  // Helpers
  public boolean getClimberLimitSwitchL() {
    return climberLimitSwitchL.get();
  }

  public boolean getClimberLimitSwitchR() {
    return climberLimitSwitchR.get();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
