// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.Turret.TurretMain;

public class Feeder extends SubsystemBase {
  private static Feeder m_instance = null;

  private boolean feederActive = false;
  private boolean feederInverted = false;

  private final SparkMax feederMotor = new SparkMax(frc.robot.Constants.MotorIDs.feederMotor,
      SparkMax.MotorType.kBrushless);
  private final SparkMax rollerMotor = new SparkMax(frc.robot.Constants.MotorIDs.rollerMotor, 
      SparkMax.MotorType.kBrushless);

  public static Feeder getInstance() {
    if (m_instance == null) {
      m_instance = new Feeder();
    }
    return m_instance;
  }

  public void setFeederActive(boolean active) {
    feederActive = active;
  }
  public void setFeederInverted(boolean inverted) {
    feederInverted = inverted;
  }

  /** Creates a new Feeder. */
  public Feeder() {
    SparkMaxConfig config = new SparkMaxConfig();
    config.smartCurrentLimit(40);
    feederMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    config.inverted(true);
    rollerMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (feederActive) {
      feederMotor.set(Constants.MotorSpeeds.Feeder.rollerSpeed * (feederInverted ? -1 : 1));
      rollerMotor.set(Constants.MotorSpeeds.Feeder.feederSpeed * (feederInverted ? -1 : 1));

      if (Robot.isSimulation()) {
        TurretMain.getInstance().shootSimFuel();
      }
    } else {
      feederMotor.set(0);
      rollerMotor.set(0);
    }

    
  }
}
