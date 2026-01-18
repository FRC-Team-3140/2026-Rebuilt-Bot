// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;

public class Turret extends SubsystemBase {
  SparkMax turretRotationMotor = new SparkMax(Constants.MotorIDs.turretRotation, SparkMax.MotorType.kBrushless); // Neo
  SparkFlex flywheelMotor = new SparkFlex(Constants.MotorIDs.flywheelMotor, SparkMax.MotorType.kBrushless); // Vortex
  SparkMax hoodMotor = new SparkMax(Constants.MotorIDs.hoodMotor, SparkMax.MotorType.kBrushless); // MiniNeo

  AnalogEncoder hoodEncoder = new AnalogEncoder(4);
  AnalogEncoder turretEncoder = new AnalogEncoder(5);

  double hoodSetpoint = 0; // degrees
  double flywheelSetpoint = 0; // RPM
  double turretSetpoint = 0; // degrees

  public PIDController flywheelPID;
  public PIDController hoodPID;
  public ProfiledPIDController rotationProfiledPID;

  public TrapezoidProfile.Constraints rotationConstraints = new TrapezoidProfile.Constraints(50, 25);

  Mechanism2d hood = new Mechanism2d(3, 3);
  MechanismRoot2d hoodRoot = hood.getRoot("Hood", 1, 1);
  MechanismLigament2d hoodBody = new MechanismLigament2d("Hood Body", 1, 0);

  Mechanism2d turret = new Mechanism2d(26, 18);

  private static Turret m_instance = null;
  public static Turret getInstance() {
    if (m_instance == null) {
      m_instance = new Turret();
    }
    return m_instance;
  }


  /** Creates a new Turret. */
  public Turret() {
    flywheelPID = new PIDController(Constants.PID.Turret.flywheelP,
                                    Constants.PID.Turret.flywheelI, 
                                    Constants.PID.Turret.flywheelD);
    hoodPID = new PIDController(Constants.PID.Turret.hoodP,
                                Constants.PID.Turret.hoodI,
                                Constants.PID.Turret.hoodD);
    rotationProfiledPID = new ProfiledPIDController(Constants.PID.Turret.flywheelP, 
                                                    Constants.PID.Turret.flywheelI, 
                                                    Constants.PID.Turret.flywheelD, rotationConstraints);

    SparkMaxConfig turretConfig = new SparkMaxConfig();
    SparkFlexConfig flywheelConfig = new SparkFlexConfig();
    SparkMaxConfig hoodConfig = new SparkMaxConfig();

    turretConfig.idleMode(IdleMode.kBrake);
    hoodConfig.idleMode(IdleMode.kBrake);
    flywheelConfig.idleMode(IdleMode.kCoast);

    turretConfig.smartCurrentLimit(Constants.CurrentLimits.Turret.turretLimit);

    turretRotationMotor.configure(turretConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    hoodMotor.configure(hoodConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);


    hoodRoot.append(hoodBody);
    SmartDashboard.putData("Shooter Hood", hood);

    hoodPID.setSetpoint(hoodSetpoint);
    
  }

  @Override
  public void periodic() {
    hoodBody.setAngle(hoodSetpoint);
    hoodSetpoint = hoodSetpoint + 1 % 360;

    hoodMotor.set(hoodPID.calculate(hoodEncoder.get()));
    turretRotationMotor.set(rotationProfiledPID.calculate(turretEncoder.get()));
    flywheelMotor.set(flywheelPID.calculate(flywheelMotor.getEncoder().getVelocity()));
  }
}
