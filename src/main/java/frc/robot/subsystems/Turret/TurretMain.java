// Copyright (c) FIRST and other WPILib contributors.
//
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Turret;

import java.util.HashMap;

import org.littletonrobotics.junction.AutoLogOutput;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.sim.SparkFlexSim;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.libs.AbsoluteEncoder;

public class TurretMain extends SubsystemBase {

  private Pose3d turretPose = Constants.SIM.turretMechOffset; 

  private Pose3d hoodPose = Constants.SIM.hoodMechOffset;

  @AutoLogOutput
  private double flywheelSpeed = 0; 


  SparkMax turretRotationMotor = new SparkMax(Constants.MotorIDs.turretRotation, SparkMax.MotorType.kBrushless); // Neo
  SparkFlex flywheelMotor = new SparkFlex(Constants.MotorIDs.flywheelMotor, SparkMax.MotorType.kBrushless); // Vortex
  SparkMax hoodMotor = new SparkMax(Constants.MotorIDs.hoodMotor, SparkMax.MotorType.kBrushless); // MiniNeo

  public SparkMaxSim turretRotationMotorSim;
  public SparkFlexSim flywheelMotorSim;
  public SparkMaxSim hoodMotorSim;

  public AbsoluteEncoder hoodEncoder = new AbsoluteEncoder(Constants.SensorIDs.hoodEncoder, 0);
  public AbsoluteEncoder turretEncoder = new AbsoluteEncoder(Constants.SensorIDs.turretEncoder, 0);

  double lastUpdateTimestamp = 0;

  @AutoLogOutput
  double hoodSetpoint = 0; // degrees
  @AutoLogOutput
  double flywheelSetpoint = 0; // RPM
  @AutoLogOutput
  double turretSetpoint = 0; // degrees

  public PIDController flywheelPID;
  public PIDController hoodPID;
  public ProfiledPIDController rotationProfiledPID;

  public TrapezoidProfile.Constraints rotationConstraints = new TrapezoidProfile.Constraints(1000, 500);



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

    hoodPID.enableContinuousInput(0, 360);
    rotationProfiledPID.enableContinuousInput(0, 360);

    SparkMaxConfig turretConfig = new SparkMaxConfig();
    SparkFlexConfig flywheelConfig = new SparkFlexConfig();
    SparkMaxConfig hoodConfig = new SparkMaxConfig();

    turretConfig.idleMode(IdleMode.kBrake);
    hoodConfig.idleMode(IdleMode.kBrake);
    flywheelConfig.idleMode(IdleMode.kCoast);

    turretConfig.smartCurrentLimit(Constants.CurrentLimits.Turret.turretLimit);

    turretRotationMotor.configure(turretConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    hoodMotor.configure(hoodConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);


    hoodPID.setSetpoint(hoodSetpoint);

    aimTypes.put(AimOpt.AUTO, new AutoAim());
    aimTypes.put(AimOpt.MANUAL, new ManualAim());

    turretSetpoint = turretEncoder.getAbsolutePosition();
    hoodSetpoint = hoodEncoder.getAbsolutePosition();
    flywheelSetpoint = 0;

    aimTypes.get(currentMode).activate(turretSetpoint, hoodSetpoint, flywheelSetpoint * RPMSpeedConversion);

    if (RobotBase.isSimulation()) {
      turretRotationMotorSim = new SparkMaxSim(turretRotationMotor, DCMotor.getNEO(1));
      flywheelMotorSim = new SparkFlexSim(flywheelMotor, DCMotor.getNEO(1));
      hoodMotorSim = new SparkMaxSim(hoodMotor, DCMotor.getNEO(1));
    }
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
    hoodPID.setSetpoint(hoodSetpoint);
    rotationProfiledPID.setGoal(turretSetpoint);
    flywheelPID.setSetpoint(flywheelSetpoint);

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


    hoodMotor.set(hoodPID.calculate(hoodEncoder.getAbsolutePosition()));
    turretRotationMotor.set(rotationProfiledPID.calculate(turretEncoder.getAbsolutePosition()));

    if (spinup) {
      flywheelMotor.set(flywheelPID.calculate(flywheelMotor.getEncoder().getVelocity()));
    } else {
      flywheelMotor.set(0);
    }

    hoodPose = new Pose3d(
        Constants.SIM.hoodMechOffset.getX(),
        Constants.SIM.hoodMechOffset.getY(),
        Constants.SIM.hoodMechOffset.getZ(),
        new Rotation3d(0, Math.toRadians(90 - hoodEncoder.getAbsolutePosition()), Math.toRadians(turretEncoder.getAbsolutePosition())));    
    turretPose = new Pose3d(
        Constants.SIM.turretMechOffset.getX(),
        Constants.SIM.turretMechOffset.getY(),
        Constants.SIM.turretMechOffset.getZ(),
        new Rotation3d(0, 0, Math.toRadians(turretEncoder.getAbsolutePosition())));

    Robot.mecanismPoses[1] = turretPose;
    Robot.mecanismPoses[2] = hoodPose;

  }
}
