// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Turret extends SubsystemBase {
  SparkMax turretRotationMotor = new SparkMax(10, SparkMax.MotorType.kBrushless);
  SparkMax flywheelMotor = new SparkMax(11, SparkMax.MotorType.kBrushless);

  public PIDController flywheelPID;
  public ProfiledPIDController rotationProfiledPID;

  public TrapezoidProfile.Constraints rotationConstraints = new TrapezoidProfile.Constraints(50, 25);
  /** Creates a new Turret. */
  public Turret() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
