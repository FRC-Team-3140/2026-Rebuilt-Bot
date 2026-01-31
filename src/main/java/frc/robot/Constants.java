// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final Mode simMode = Mode.SIM; // Mode.REPLAY to replay

  public static class SIM {
    public static final double odometryDrift = 0.025;

    public static final Pose3d intakeMechOffset = new Pose3d(Units.inchesToMeters(11.120000), 0,
        Units.inchesToMeters(7.254931), new Rotation3d(0, 45, 0));
    public static final Pose3d turretMechOffset = new Pose3d(Units.inchesToMeters(-5.379), 0,
        Units.inchesToMeters(16.112162), new Rotation3d(0, 0, 45));
    public static final Pose3d hoodMechOffset = new Pose3d(Units.inchesToMeters(-6.067574), 0,
        Units.inchesToMeters(19.112162), new Rotation3d(0, 45, 45));
  }

  public static class MotorIDs {
    /* Swerve Drive Motors: */
    // FL
    public static final int FLNeo = 1; // NEO 1
    public static final int FLVortex = 2; // VORTEX 1

    // FR
    public static final int FRNeo = 3; // NEO 2
    public static final int FRVortex = 4; // VORTEX 2

    // BL
    public static final int BLNeo = 5; // NEO 3
    public static final int BLVortex = 6; // VORTEX 3

    // BR
    public static final int BRNeo = 7; // NEO 4
    public static final int BRVortex = 8; // VORTEX 4

    public static final int turretRotation = 9; // NEO 5
    public static final int flywheelMotor = 10; // VORTEX 5
    public static final int hoodMotor = 11; // MINI 1

    public static final int intakeMotor = 12; // MINI 2
    public static final int intakeArmMotor = 13; // NEO 6
    public static final int intakeArmMotorFollower = 14; // NEO 7

    public static final int feederMotor = 15; // NEO 8
    public static final int rollerMotor = 16; // NEO 9

    public static final int climberLeftMotor = 17; // NEO 10
    public static final int climberRightMotor = 18; // NEO 10
  }

  public static class SensorIDs {
    // Swerve Modules
    public static final int FL = 0;

    public static final int FR = 1;

    public static final int BL = 2;

    public static final int BR = 3;

    // Intake Absolute Encoder
    public static final int intakeEncoder = 4;

    // Turret Sensors
    public static final int turretEncoder = 5;

    public static final int hoodEncoder = 6;

    public static class Digital {
      // Climber Limit Switches
      public static final int climberLimitSwitchLeft = 0;
      public static final int climberLimitSwitchRight = 1;
    }
  }

  public static class MotorSpeeds {
    public static class Intake {
      public static final double intakeSpeed = 0.7;
      public static final double outtakeSpeed = -0.7;

      public static final double agitateSpeed = 0.1;
    }

    public static class Feeder {
      public static final double feederSpeed = 0.6;
    }
  }

  public static class PID {
    public static class Turret {
      public static final double hoodP = 0.009;
      public static final double hoodI = 0.0;
      public static final double hoodD = 0.0;

      public static final double rotationP = 0.004;
      public static final double rotationI = 0.0;
      public static final double rotationD = 0.0;

    }

    public static class Intake {
      public static final double intakeP = 0.05;
      public static final double intakeI = 0.0;
      public static final double intakeD = 0.0;
    }
  }

  public static class FeedFoward {
    public static class Turret {
      public static final double flywheelS = 0.02;
      public static final double flywheelV = 0.0167;
      public static final double flywheelA = 0.0;
    }

  }

  public static class CurrentLimits {
    public static class Turret {
      public static final int flywheelLimit = 40;
      public static final int hoodLimit = 15;
      public static final int turretLimit = 20;
    }

    public static class Intake {
    }

    public static class Feeder {
    }
  }

  public static class Bot {
    public static final double gearRatio = 6.75;
    public static final double steerGearRatio = 150 / 7;
    public static final double botMass = 48.988;
    public static final double wheelDiameter = Units.inchesToMeters(4);
    public static final double botLength = Units.inchesToMeters(26);

    // In meters per second, determined from the free speed of the bot via
    // SwerveDriveSpecialties
    public static final double maxChassisSpeed = 5.36448;
    public static final double maxModuleSpeed = maxChassisSpeed / (Math.PI * wheelDiameter);
    public static final double maxTurnSpeed = Double.MAX_VALUE; // These are basically infinite for our purposes
    public static final double maxAcceleration = 2500;
    public static final double botRadius = Math.hypot(botLength, botLength);
    // Max Speed divided by the circumference a circle determined by the distance of
    // the module from the center, divided by 2 pi to convert to radians
    public static final double maxChassisTurnSpeed = maxChassisSpeed / botRadius;
    public static final double encoderRotationToMeters = Math.PI * wheelDiameter / gearRatio;

    public static final double turnP = 0.01;

    /////// AI CODE ///////
    // Simulation-only momentum constants for realistic coast-down behavior
    public static final double simMaxDeceleration = 100; // m/s² for translation coast-down
    public static final double simMaxRotationalDeceleration = Math.toRadians(1200); // rad/s² for rotation
    public static final double simDragCoefficient = 0.05; // Friction/drag coefficient (0.01-0.1)

    // Simulation-only PID tuning (much softer to reduce oscillation in fast sim
    // updates)
    public static final double simDriveP = 0.0005; // Very low P - rely mostly on feedforward
    public static final double simDriveI = 0.0; // No integral term
    public static final double simDriveD = 0.00001; // Minimal D for damping only
    public static final double simTurnP = 0.003; // Reduced turn P for smoother rotation
                                                 /////// END AI CODE ///////

    // Swerve Module Base Angles
    // TODO: Update for this configuration
    public static final double FLZeroOffset = 216.997730;// 217.720;

    public static final double FRZeroOffset = 136.602900;// 228.319;

    public static final double BLZeroOffset = 200.317267;// 197.621;

    public static final double BRZeroOffset = 310.841840;// 312.425;

    public static final double[] lockedAngles = {
        45,
        315,
        315,
        45
    };

    // Default swerve state
    // new SwerveModuleState initializes states with 0s for angle and velocity
    public static final SwerveModuleState[] defaultSwerveStates = {
        new SwerveModuleState(0, new Rotation2d(0)),
        new SwerveModuleState(0, new Rotation2d(0)),
        new SwerveModuleState(0, new Rotation2d(0)),
        new SwerveModuleState(0, new Rotation2d(0))
    };

    public static final double intakeGearRatio = 50;
    public static final double turretGearRatio = 10;
    public static final double hoodGearRatio = 10;
    public static final double flywheelGearRatio = 10;

  }

  public static class Limits {
    public static class Turret {
      public static final double minAngle = 45; // degrees above horizontal
      public static final double maxAngle = 80;
      public static final double maxAngularVelocity = 30; // degrees per second

      public static final double nearMinAngle = 75; // minimum angle when near the goal. Makes sure the shot arcs
      public static final double nearInterpRange = 4; // the range where the min angle starts to interpolate to the
                                                      // nearMinAngle
      public static final double nearRange = 2; // the range where the min angle is the nearMinAngle
    }

    public static class Intake {
      public static final double deployedPosition = 90; // degrees
      public static final double stowedPosition = 0; // degrees
    }
  }

  public static class Controller {
    public static final int DriverControllerPort = 0;
    public static final int SecondaryDriverControllerPort = 1;

    public static final double triggerThreshold = 0.3;
  }

  public static class Constraints {
  }

  public static class CameraConstants {
    // public static final double maxTimeBeteweenFrames = 0.1;
    public static final double leftOffsetToCenter = -Units.inchesToMeters(13);
    public static final double rightOffsetToCenter = Units.inchesToMeters(13);
    public static final double offsetToCenterVert = Units.inchesToMeters(10);
    public static final double pitch = Math.toRadians(-30);
    public static final double maxReprojectionError = 0.3;
  }

  public static class Odometry {
    public static final double TagCorrectionSpeed = 8;
    public static final double maxCorrectionDistance = 0.5;
    public static final int startingCameraPasses = 50;
  }

  public static class PathplannerConstants {
    public static RobotConfig config;

    // Field Dimensions - Based on PathPlanner Rebuilt Field
    public static final double FieldLength = 16.540;
    public static final double FieldWidth = 8.070;

    // Translation PID Values
    public static final double TransP = 12;
    public static final double TransI = 0;
    public static final double TransD = 0;

    // Rotation PID Values
    public static final double RotP = 5;
    public static final double RotI = 0;
    public static final double RotD = 0;

    public static final PathConstraints pathplannerConstraints = new PathConstraints(
        Constants.Bot.maxChassisSpeed,
        4.0,
        Units.degreesToRadians(540),
        Units.degreesToRadians(720));

    public static final Pose2d shootPoseL = new Pose2d(1, 7, new Rotation2d(Units.degreesToRadians(0)));
    public static final Pose2d shootPoseR = new Pose2d(1, 1, new Rotation2d(Units.degreesToRadians(0)));
  }

  public static class NetworktablePaths {
    public static final String Dashboard = "Dashboard";

    public static final String Sensors = "sensors3140";

    // Subtables of Dashboard
    public static final String DS = "DS";
    public static final String Voltage = "Voltage";
    public static final String Pose = "Pose";
    public static final String Test = "Dev";
    public static final String Debug = "Debug";
    public static final String Misc = "Misc";
  }

  public static class LED {
    public static final int Port = 0;
    public static final int LEDCount = 76;
    public static final int RainbowSpan = 5;
  }

  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }
}
