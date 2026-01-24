// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Turret;

import java.util.HashMap;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.libs.AbsoluteEncoder;

public class TurretMain extends SubsystemBase {
  SparkMax turretRotationMotor = new SparkMax(Constants.MotorIDs.turretRotation, SparkMax.MotorType.kBrushless); // Neo
  SparkFlex flywheelMotor = new SparkFlex(Constants.MotorIDs.flywheelMotor, SparkMax.MotorType.kBrushless); // Vortex
  SparkMax hoodMotor = new SparkMax(Constants.MotorIDs.hoodMotor, SparkMax.MotorType.kBrushless); // MiniNeo

  AbsoluteEncoder hoodEncoder = new AbsoluteEncoder(Constants.SensorIDs.hoodEncoder, 0);
  AbsoluteEncoder turretEncoder = new AbsoluteEncoder(Constants.SensorIDs.turretEncoder, 0);

  double lastUpdateTimestamp = 0;

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

  private boolean spinup = false;

  private static final double flywheelSpeedTolerance = 10; // RPM
  private static final double RPMSpeedConversion = 0.0762 * 2 * Math.PI; // convert from m/s to RPM

  public enum AimOpt {
    AUTO,
    MANUAL
  }

  private HashMap<AimOpt, AimType> aimTypes = new HashMap<AimOpt, AimType>();

  private AimOpt currentMode = AimOpt.AUTO;

  private static TurretMain m_instance = null;

  public static TurretMain getInstance() {
    if (m_instance == null) {
      m_instance = new TurretMain();
    }
    return m_instance;
  }

  /** Creates a new Turret. */
  public TurretMain() {
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

    turretRotationMotor.configure(turretConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    hoodMotor.configure(hoodConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    hoodRoot.append(hoodBody);
    SmartDashboard.putData("Shooter Hood", hood);

    hoodPID.setSetpoint(hoodSetpoint);

    aimTypes.put(AimOpt.AUTO, new AutoAim());
    aimTypes.put(AimOpt.MANUAL, new ManualAim());

    turretSetpoint = turretEncoder.getAbsolutePosition();
    hoodSetpoint = hoodEncoder.getAbsolutePosition();
    flywheelSetpoint = 0;

    aimTypes.get(currentMode).activate(turretSetpoint, hoodSetpoint, flywheelSetpoint * RPMSpeedConversion);
  }

  public void setAimMode(AimOpt mode) {
    if (mode != currentMode) {
      aimTypes.get(currentMode).deactivate();
      currentMode = mode;
      aimTypes.get(currentMode).activate(turretSetpoint, hoodSetpoint, flywheelSetpoint * RPMSpeedConversion);
    }
  }

  public void setFlywheelActive(boolean active) {
    spinup = active;
  }

  public boolean getFlywheelActive() {
    return spinup;
  }

  public boolean shouldShoot() {
    return aimTypes.get(currentMode).shouldShoot
        && Math.abs(flywheelSetpoint - flywheelMotor.getEncoder().getVelocity()) <= flywheelSpeedTolerance;
  }

  @Override
  public void periodic() {
    // TODO: Default Hood Angle DN, Manual Mode, Limiting Angle
    if (lastUpdateTimestamp == 0) {
      lastUpdateTimestamp = Timer.getFPGATimestamp();
      // first update, setup
      // TODO: read hood setpoint from encoder so that predict can work properly
    } else {
      double t = Timer.getFPGATimestamp();
      double deltaTime = t - lastUpdateTimestamp;
      lastUpdateTimestamp = t;

      AimType type = aimTypes.get(currentMode);

      type.periodic(deltaTime);

      flywheelSetpoint = type.flywheelSpeed / RPMSpeedConversion; // convert from m/s to RPM
      hoodSetpoint = type.hoodAngle;
      turretSetpoint = type.rotationAngle;
    }

    hoodBody.setAngle(hoodSetpoint);
    hoodSetpoint = hoodSetpoint + 1 % 360;

    hoodMotor.set(hoodPID.calculate(hoodEncoder.get()));
    turretRotationMotor.set(rotationProfiledPID.calculate(turretEncoder.get()));

    if (spinup) {
      flywheelMotor.set(flywheelPID.calculate(flywheelMotor.getEncoder().getVelocity()));
    } else {
      flywheelMotor.set(0);
    }
  }
}
