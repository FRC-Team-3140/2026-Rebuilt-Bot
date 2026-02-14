// Copyright (c) FIRST and other WPILib contributors.
//
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Turret;

import java.util.ArrayList;
import java.util.HashMap;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

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
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.libs.AbsoluteEncoder;
import frc.robot.libs.Vector2;
import frc.robot.subsystems.TestRunner;
import frc.robot.subsystems.TestRunner.TestType;
import frc.robot.subsystems.odometry.Odometry;

public class TurretMain extends SubsystemBase {

  private Pose3d turretPose = Constants.SIM.turretMechOffset;

  private Pose3d hoodPose = Constants.SIM.hoodMechOffset;

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
  @AutoLogOutput
  boolean shouldShoot = false;

  boolean shouldShootMode = false;

  public PIDController hoodPID;
  public PIDController rotationProfiledPID;
  public SimpleMotorFeedforward flywheelFeedforward;

  private boolean spinup = false;

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

  private ArrayList<Fuel> gamePieces = new ArrayList<Fuel>();
  private ArrayList<Pose3d> publishedGamePieces = new ArrayList<Pose3d>();

  private class Fuel {
    private static final double GRAVITY = -9.81; // m/s^2, downward

    public Pose3d position;
    public double vx, vy, vz; // velocity components

    public Fuel(Pose3d pos, double velocity) {
      this.position = pos;

      // Calculate launch direction from pose rotation (assume launch along +X axis)
      Rotation3d rot = pos.getRotation();
      double pitch = rot.getY(); // radians
      double yaw = rot.getZ(); // radians

      double dx = Math.cos(pitch) * Math.cos(yaw);
      double dy = Math.cos(pitch) * Math.sin(yaw);
      double dz = Math.sin(pitch);

      // Set velocity vector in facing direction
      vx = velocity * dx;
      vy = velocity * dy;
      vz = velocity * dz;
    }

    // Step simulation by dt seconds
    public void step(double dt) {
      // Update velocity with gravity (only z)
      vz += GRAVITY * dt;

      // Update position
      double newX = position.getX() + vx * dt;
      double newY = position.getY() + vy * dt;
      double newZ = position.getZ() + vz * dt;

      // Update pose with new position, keep rotation
      position = new Pose3d(newX, newY, newZ, position.getRotation());

      // Optionally, update rotation to match velocity direction
      // If you want the pose's rotation to follow the velocity vector:
      double speed = Math.sqrt(vx * vx + vy * vy + vz * vz);
      if (speed > 1e-6) {
        double pitch = Math.asin(vz / speed);
        double yaw = Math.atan2(vy, vx);
        position = new Pose3d(newX, newY, newZ, new Rotation3d(0, pitch, yaw));
      }
    }
  }

  /** Creates a new Turret. */
  public TurretMain() {
    flywheelFeedforward = new SimpleMotorFeedforward(
        Constants.FeedFoward.Turret.flywheelS,
        Constants.FeedFoward.Turret.flywheelV,
        Constants.FeedFoward.Turret.flywheelA);

    hoodPID = new PIDController(Constants.PID.Turret.hoodP,
        Constants.PID.Turret.hoodI,
        Constants.PID.Turret.hoodD);
    rotationProfiledPID = new PIDController(Constants.PID.Turret.rotationP,
        Constants.PID.Turret.rotationI,
        Constants.PID.Turret.rotationD);

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
    clampTurretSetpoint();

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
    return shouldShoot;
    // && Math.abs(flywheelSetpoint - flywheelMotor.getEncoder().getVelocity()) <=
    // flywheelSpeedTolerance;
  }

  public void setHoodAngle(double angle) {
    hoodSetpoint = angle;
  }

  public void setRotationAngle(double angle) {
    turretSetpoint = angle;
    clampTurretSetpoint();
  }

  private boolean shouldStow() {
    Vector2 botPosition = Odometry.getInstance().getPosition();
    Vector2 turretDirection = Constants.PathplannerConstants.botTurretOffset.rotate(Odometry.getInstance().getAngle());
    Vector2 turretPosition = botPosition.add(turretDirection);

    // TODO: Logic to check if it needs to stow based on turretPosition
    boolean shouldStow = false;

    return shouldStow;
  }

  @Override
  public void periodic() {
    hoodPID.setSetpoint(hoodSetpoint);
    rotationProfiledPID.setSetpoint(turretSetpoint);

    if (!TestRunner.getInstance().isRunning(TestType.TURRET)) {
      // TODO: Default Hood Angle DN, Manual Mode, Limiting Angle
      if (lastUpdateTimestamp == 0) {
        lastUpdateTimestamp = Timer.getFPGATimestamp();
        // first update, setup
        // TODO: read hood setpoint from encoder so that predict can work properly
        shouldShootMode = false;
      } else {
        double t = Timer.getFPGATimestamp();
        double deltaTime = t - lastUpdateTimestamp;
        lastUpdateTimestamp = t;

        AimType type = aimTypes.get(currentMode);

        type.periodic(deltaTime);

        flywheelSetpoint = type.flywheelSpeed / RPMSpeedConversion; // convert from m/s to RPM
        hoodSetpoint = type.hoodAngle;
        turretSetpoint = type.rotationAngle;
        shouldShootMode = type.shouldShoot && clampTurretSetpoint();
      }
    }

    boolean stow = shouldStow();
    shouldShoot = shouldShootMode && !stow;

    hoodPID.setSetpoint(stow ? hoodSetpoint : 90);  // TODO: Is this right? I think it is but make sure. Maybe its more like 80? Maybe use the Constants.Limits.Turret.MaxPitch
    rotationProfiledPID.setSetpoint(turretSetpoint);

    hoodMotor.set(hoodPID.calculate(hoodEncoder.getAbsolutePosition()));
    turretRotationMotor.set(rotationProfiledPID.calculate(turretEncoder.getAbsolutePosition()));

    if (spinup) {
      flywheelMotor.set(flywheelFeedforward.calculate(flywheelSetpoint) / 12);
    } else {
      flywheelMotor.setVoltage(0);
    }
    Logger.recordOutput("TurretMain/Flywheel/Setpoint", flywheelSetpoint);
    Logger.recordOutput("TurretMain/Flywheel/Speed", flywheelMotor.getEncoder().getVelocity());
    Logger.recordOutput("TurretMain/Flywheel/FeedFowardOutput", flywheelFeedforward.calculate(flywheelSetpoint));

    hoodPose = new Pose3d(
        Constants.SIM.hoodMechOffset.getX(),
        Constants.SIM.hoodMechOffset.getY(),
        Constants.SIM.hoodMechOffset.getZ(),
        new Rotation3d(0, Math.toRadians(90 - hoodEncoder.getAbsolutePosition()),
            Math.toRadians(turretEncoder.getAbsolutePosition())));
    turretPose = new Pose3d(
        Constants.SIM.turretMechOffset.getX(),
        Constants.SIM.turretMechOffset.getY(),
        Constants.SIM.turretMechOffset.getZ(),
        new Rotation3d(0, 0, Math.toRadians(turretEncoder.getAbsolutePosition())));

    Robot.mecanismPoses[1] = turretPose;
    Robot.mecanismPoses[2] = hoodPose;
  }

  private boolean clampTurretSetpoint() {
    while (turretSetpoint <= -180) {
      turretSetpoint += 360;
    }
    while (turretSetpoint > 180) {
      turretSetpoint -= 360;
    }

    if (turretSetpoint < Constants.Limits.Turret.minYaw) {
      turretSetpoint = Constants.Limits.Turret.minYaw;
      return false;
    } else if (turretSetpoint > Constants.Limits.Turret.maxYaw) {
      turretSetpoint = Constants.Limits.Turret.maxYaw;
      return false;
    }

    return true;
  }

  public double getLookDirection() {
    return aimTypes.get(currentMode).getLookDirection() + 180;
  }

  public void simFuel(double dt) {

    for (int i = 0; i < gamePieces.size(); i++) {
      publishedGamePieces.set(i, gamePieces.get(i).position);
      if (gamePieces.get(i).position.getZ() > 0) {
        gamePieces.get(i).step(dt);
      } else {
        publishedGamePieces.remove(i);
        gamePieces.remove(i);
      }
    }

    Logger.recordOutput("GamePieces", publishedGamePieces.toArray(new Pose3d[0]));
  }

  public void shootSimFuel() {
    if (!shouldShoot())
      return;

    // Get robot's field pose (x, y, rotation)
    Pose2d robotFieldPose = Odometry.getInstance().getRealSimPose();
    Pose3d turretOffset = Constants.SIM.hoodMechOffset;

    // Rotate turret offset by robot heading (about Z axis)
    double headingRad = robotFieldPose.getRotation().getRadians();
    double cosH = Math.cos(headingRad);
    double sinH = Math.sin(headingRad);
    double offsetX = turretOffset.getX() * cosH - turretOffset.getY() * sinH;
    double offsetY = turretOffset.getX() * sinH + turretOffset.getY() * cosH;
    double offsetZ = turretOffset.getZ();

    // Add to robot's field position
    double fieldX = robotFieldPose.getX() + offsetX;
    double fieldY = robotFieldPose.getY() + offsetY;
    double fieldZ = offsetZ;

    // Build the shooter pose at the correct field position, with turret/hood
    // rotation
    Rotation3d shooterRot = new Rotation3d(
        0,
        Math.toRadians(/* hoodEncoder.getAbsolutePosition() */hoodSetpoint),
        Math.toRadians(
            turretSetpoint/* turretEncoder.getAbsolutePosition() */ + robotFieldPose.getRotation().getDegrees()));

    Pose3d shooterPose = new Pose3d(fieldX, fieldY, fieldZ, shooterRot);

    // Calculate robot's velocity direction (field-relative)
    double robotVelX = Odometry.getInstance().getBotVelocity().X; // implement or replace with your method
    double robotVelY = Odometry.getInstance().getBotVelocity().Y; // implement or replace with your method
    double robotVelZ = 0; // usually 0 unless you have a swerve module that can jump :)

    // Calculate projectile speed (magnitude)
    double projectileSpeed = flywheelSetpoint * RPMSpeedConversion; // flywheelMotor.getEncoder().getVelocity()/*flywheelSetpoint*/
                                                                    // * RPMSpeedConversion /
                                                                    // Constants.Bot.flywheelGearRatio;

    // Calculate launch direction from shooter pose
    Rotation3d rot = shooterPose.getRotation();
    double pitch = rot.getY(); // radians
    double yaw = rot.getZ(); // radians
    double dx = Math.cos(pitch) * Math.cos(yaw);
    double dy = Math.cos(pitch) * Math.sin(yaw);
    double dz = Math.sin(pitch);

    // Add tangential velocity
    Vector2 turretPosition = new Vector2(turretPose.toPose2d().getX(), turretPose.toPose2d().getY())
        .rotate(Odometry.getInstance().getRotation().getRadians() + (Math.PI / 2))
        .mult(Odometry.getInstance().getAngularVelocity());

    // Add robot velocity to projectile velocity
    double vx = projectileSpeed * dx + robotVelX + turretPosition.X;
    double vy = projectileSpeed * dy + robotVelY + turretPosition.Y;
    double vz = projectileSpeed * dz + robotVelZ;

    Fuel fuel = new Fuel(shooterPose, 0);
    fuel.vx = vx;
    fuel.vy = vy;
    fuel.vz = vz;
    gamePieces.add(fuel);
    publishedGamePieces.add(shooterPose);
  }
}
