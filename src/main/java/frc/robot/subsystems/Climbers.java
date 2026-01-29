// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climbers extends SubsystemBase {
  private static Climbers m_Instance = null;

  private final SparkMax lClimber = new SparkMax(Constants.MotorIDs.climberLeftMotor, MotorType.kBrushless);
  private final SparkMax rClimber = new SparkMax(Constants.MotorIDs.climberRightMotor, MotorType.kBrushless);

  private final DigitalInput climberLimitSwitchL = new DigitalInput(Constants.SensorIDs.Digital.climberLimitSwitchLeft);
  private final DigitalInput climberLimitSwitchR = new DigitalInput(
      Constants.SensorIDs.Digital.climberLimitSwitchRight);

  private final SparkMaxConfig lConfig = new SparkMaxConfig();
  private final SparkMaxConfig rConfig = new SparkMaxConfig();

  // Climber objects
  public class climber {
    public final SparkMax motor;
    public final SparkMaxConfig config;
    public final DigitalInput limitSwitch;

    private climber(SparkMax motor, SparkMaxConfig config, DigitalInput limitSwitch) {
      this.motor = motor;
      this.config = config;
      this.limitSwitch = limitSwitch;
    }

    public void setSpeed(double speed) {
      motor.set(speed);
    }

    public boolean getLimitSwitch() {
      return !limitSwitch.get(); // Inverted because limit switch returns false when pressed
    }
  }

  public final climber LEFT = new climber(lClimber, lConfig, climberLimitSwitchL);
  public final climber RIGHT = new climber(rClimber, rConfig, climberLimitSwitchR);

  public static Climbers getInstance() {
    if (m_Instance == null) {
      m_Instance = new Climbers();
    }

    return m_Instance;
  }

  /** Creates a new Climber. */
  public Climbers() {
    // Configure Motors
    lConfig.inverted(true).smartCurrentLimit(20).idleMode(IdleMode.kBrake);
    rConfig.inverted(true).smartCurrentLimit(20).idleMode(IdleMode.kBrake);

    lClimber.configure(lConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    rClimber.configure(rConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
