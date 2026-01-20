// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import java.util.Optional;
import java.util.Vector;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;
import frc.robot.libs.ShotPredictor;
import frc.robot.libs.Vector2;
import frc.robot.libs.ShotPredictor.Result;
import frc.robot.subsystems.odometry.CustomOdometry;
import frc.robot.subsystems.odometry.Odometry;

public class Turret extends SubsystemBase {
  SparkMax turretRotationMotor = new SparkMax(Constants.MotorIDs.turretRotation, SparkMax.MotorType.kBrushless); // Neo
  SparkFlex flywheelMotor = new SparkFlex(Constants.MotorIDs.flywheelMotor, SparkMax.MotorType.kBrushless); // Vortex
  SparkMax hoodMotor = new SparkMax(Constants.MotorIDs.hoodMotor, SparkMax.MotorType.kBrushless); // MiniNeo

  AnalogEncoder hoodEncoder = new AnalogEncoder(4);
  AnalogEncoder turretEncoder = new AnalogEncoder(5);

  private static double predictForwardTime = 0.1;

  double desiredVelocity = 10;
  double lastUpdateTimestamp = 0;

  boolean shouldShoot = false;

  double hoodSetpoint = 0; // degrees
  double flywheelSetpoint = 0; // RPM
  double turretSetpoint = 0; // degrees
  ShotPredictor shotPredictor = new ShotPredictor(
    Constants.Limits.Turret.minAngle,
    Constants.Limits.Turret.maxAngle,
    Constants.Limits.Turret.maxAngularVelocity,
    desiredVelocity
  );

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

  private Pair<Vector2, Vector2> getFutureState(double dt) {
    CustomOdometry odometry = Odometry.getInstance();
    Vector2 futureBotVelocity = odometry.getBotVelocity().add(odometry.getBotAcceleration().mult(dt));

    // position = velocity*time + acceleration*0.5*time^2
    Vector2 predictedPositionChange = odometry.getBotVelocity().mult(dt).add(
      odometry.getBotAcceleration().mult(0.5*dt*dt)
    );
    Vector2 futureBotPosition = odometry.getPosition().add(predictedPositionChange);
    

    //TODO: make this the position of the turret relative to the axis of rotation of the bot
    // this is used to get centripetal velocity
    // calculate for when the bot is facing in the positive x direction
    // might want to put this in constants too
    Vector2 turretPosition = new Vector2(-0.25, 0); // this is a guess

    double futureRotation = odometry.getAngle() + odometry.getAngularVelocity()*dt; // in rads
    Vector2 rotatedTurretPosition = turretPosition.rotate(futureRotation);

    Vector2 centripetalVelocity = new Vector2(-rotatedTurretPosition.Y, rotatedTurretPosition.X)
      .mult(odometry.getAngularVelocity());
    Vector2 futureTurretVelocity = futureBotVelocity.add(centripetalVelocity);
    Vector2 shotOrigin = futureBotPosition.add(rotatedTurretPosition);
    
    return new Pair<Vector2,Vector2>(shotOrigin, futureTurretVelocity);
  }

  private void predict(double deltaTime) {
    shotPredictor.DesiredShotVelocity = desiredVelocity; // incase we want to change desired velocity

    double targetHeight = 2; //TODO: make this correct
    Vector2 goalPosition = new Vector2(); //TODO: this too

    Pair<Vector2, Vector2> futureStatePair = getFutureState(predictForwardTime);
    Vector2 futureShotOrigin = futureStatePair.getFirst();
    Vector2 futureTurretVelocity = futureStatePair.getSecond();

    Vector2 relativeTargetPosition = goalPosition.sub(futureShotOrigin);
    double targetDistance = relativeTargetPosition.magnitude();

    shotPredictor.MinAngle = calculateMinimumAngle(targetDistance);

    boolean isShooting = false; //TODO: make this work
    double verticalVelocity = 0; //TODO: (optional) make this work
    Optional<Result> result = shotPredictor.Update(
      (!isShooting) || hoodSetpoint == 0,
      hoodSetpoint,
      deltaTime,
      futureTurretVelocity,
      verticalVelocity,
      relativeTargetPosition,
      targetHeight
    );

    shouldShoot = result.isPresent(); //TODO: make this matter (shouldn't let you shoot if this is false)

    if (result.isPresent()) {
      //TODO: Do we need to convert this? This is DEGREES ABOVE HORIZONTAL
      // If we do need to convert this, then we need to fix the above code that passes in the hood setpoint
      // If anything it'd be easier to convert the setpoint before feeding it to the motor.
      hoodSetpoint = result.get().ShotAngle;
      flywheelSetpoint = result.get().ShotSpeed; //TODO: convert this to RPM
      
    }
  }

  private static double calculateMinimumAngle(double distance) {
    if (distance >= Constants.Limits.Turret.nearInterpRange) {
      return Constants.Limits.Turret.minAngle;
    } else if (distance <= Constants.Limits.Turret.nearRange) {
      return Constants.Limits.Turret.nearMinAngle;
    } else {
      double alpha = (distance - Constants.Limits.Turret.nearRange)
        /(Constants.Limits.Turret.nearInterpRange - Constants.Limits.Turret.nearRange);
      double interpolatedAngle = Constants.Limits.Turret.nearMinAngle +
        (Constants.Limits.Turret.minAngle - Constants.Limits.Turret.nearMinAngle)*alpha;

      return interpolatedAngle;
    }
  }

  @Override
  public void periodic() {
    if (lastUpdateTimestamp == 0) {
      lastUpdateTimestamp = Timer.getFPGATimestamp();
      // first update, setup
    } else {
      double t = Timer.getFPGATimestamp();
      double deltaTime = t - lastUpdateTimestamp;
      lastUpdateTimestamp = t;

      predict(deltaTime);
    }
    

    hoodBody.setAngle(hoodSetpoint);
    hoodSetpoint = hoodSetpoint + 1 % 360;

    hoodMotor.set(hoodPID.calculate(hoodEncoder.get()));
    turretRotationMotor.set(rotationProfiledPID.calculate(turretEncoder.get()));
    flywheelMotor.set(flywheelPID.calculate(flywheelMotor.getEncoder().getVelocity()));
  }
}
