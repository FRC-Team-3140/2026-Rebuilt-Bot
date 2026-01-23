// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.commands.swerveDrive.Align;
import frc.robot.libs.NetworkTables;
import edu.wpi.first.wpilibj.XboxController;

public class Controller extends SubsystemBase {
  private static Controller instance = null;

  public final XboxController primaryController;
  public final XboxController secondaryController;

  /** Creates a new Controller. */
  public Controller(int primary, int secondary) {
    primaryController = new XboxController(primary);
    secondaryController = new XboxController(secondary);
  }

  private final double deadband = .1;

  private final boolean testing = false;

  public enum controllers {
    PRIMARY, SECONDARY
  }

  public enum ControlMode {
    AUTO, MANUAL, OHNO_MANUAL
  }

  private ControlMode curControlMode = ControlMode.AUTO; // Default to auto when auto is implemented

  public static Controller getInstance() {
    if (instance == null) {
      instance = new Controller(Constants.Controller.DriverControllerPort,
          Constants.Controller.SecondaryDriverControllerPort);
    }
    return instance;
  }

  public double getLeftX(controllers controller) {
    XboxController contr = (controller == controllers.PRIMARY ? primaryController : secondaryController);
    if (Math.abs(contr.getLeftX()) > deadband) {
      if (contr.getLeftX() > 0)
        return Math.pow(contr.getLeftX(), 2);
      else
        return -Math.pow(contr.getLeftX(), 2);

    } else {
      return 0;
    }
  }

  public double getRightX(controllers controller) {
    XboxController contr = (controller == controllers.PRIMARY ? primaryController : secondaryController);
    if (Math.abs(contr.getRightX()) > deadband) {
      if (contr.getRightX() > 0)
        return Math.pow(contr.getRightX(), 2);
      else
        return -Math.pow(contr.getRightX(), 2);
    } else {
      return 0;
    }
  }

  public double getLeftY(controllers controller) {
    XboxController contr = (controller == controllers.PRIMARY ? primaryController : secondaryController);
    if (Math.abs(contr.getLeftY()) > deadband) {
      if (contr.getLeftY() > 0)
        return Math.pow(contr.getLeftY(), 2);
      else
        return -Math.pow(contr.getLeftY(), 2);

    } else {
      return 0;
    }
  }

  public double getRightY(controllers controller) {
    XboxController contr = (controller == controllers.PRIMARY ? primaryController : secondaryController);
    if (Math.abs(contr.getRightY()) > deadband) {
      if (contr.getRightY() > 0)
        return Math.pow(contr.getRightY(), 2);
      else
        return -Math.pow(contr.getRightY(), 2);
    } else {
      return 0;
    }
  }

  public void setRumbleBoth(controllers controller, double duration, double intensity) {
    XboxController contr = (controller == controllers.PRIMARY ? primaryController : secondaryController);
    new SequentialCommandGroup(
        new ParallelCommandGroup(
            new InstantCommand(() -> {
              contr.setRumble(RumbleType.kBothRumble, intensity);
            }),
            new WaitCommand(duration)),
        new InstantCommand(() -> {
          contr.setRumble(RumbleType.kBothRumble, 0);
        })).schedule();
  }

  public void setRumbleLeft(controllers controller, double duration, double intensity) {
    XboxController contr = (controller == controllers.PRIMARY ? primaryController : secondaryController);
    new SequentialCommandGroup(
        new ParallelCommandGroup(
            new InstantCommand(() -> {
              contr.setRumble(RumbleType.kLeftRumble, intensity);
            }),
            new WaitCommand(duration)),
        new InstantCommand(() -> {
          contr.setRumble(RumbleType.kBothRumble, 0);
        })).schedule();
  }

  public void setRumbleRight(controllers controller, double duration, double intensity) {
    XboxController contr = (controller == controllers.PRIMARY ? primaryController : secondaryController);
    new SequentialCommandGroup(
        new ParallelCommandGroup(
            new InstantCommand(() -> {
              contr.setRumble(RumbleType.kRightRumble, intensity);
            }),
            new WaitCommand(duration)),
        new InstantCommand(() -> {
          contr.setRumble(RumbleType.kBothRumble, 0);
        })).schedule();
  }

  public void setControlMode(ControlMode mode) {
    curControlMode = mode;
  }

  public ControlMode getControlMode() {
    return curControlMode;
  }

  private void updateControlMode() {
    if (secondaryController.getRightBumperButtonPressed()) {
      curControlMode = ControlMode.MANUAL;
      System.out.println("Control Mode: " + curControlMode);
      setRumbleBoth(controllers.SECONDARY, 0.1, 1);
    } else if (secondaryController.getLeftBumperButtonPressed()) {
      curControlMode = ControlMode.AUTO;
      System.out.println("Control Mode: " + curControlMode);
      setRumbleBoth(controllers.SECONDARY, 0.1, 1);
    } else if (secondaryController.getRightTriggerAxis() > 0.5 && secondaryController.getLeftTriggerAxis() > 0.5) {
      curControlMode = ControlMode.OHNO_MANUAL;
      System.out.println("Control Mode: " + curControlMode);
      setRumbleBoth(controllers.SECONDARY, 0.1, 1);
    }
  }

  private void AutoMode() {
    if (secondaryController.getLeftStickButton() && secondaryController.getRightStickButton()) {
      updateControlMode();
      return;
    }
    if (primaryController.getAButtonPressed()) {
      new Align(new Pose2d(10.0, 10.0, new Rotation2d(100.0))).schedule();
    }
    secondarySetpointCommands();
  }

  private void ManualMode() {
    if (secondaryController.getLeftStickButton() && secondaryController.getRightStickButton()) {
      updateControlMode();
      return;
    }
    secondarySetpointCommands();
  }

  private void OHNOManualMode() {
    if (secondaryController.getLeftStickButton() && secondaryController.getRightStickButton()) {
      updateControlMode();
      return;
    }
  }

  private void secondarySetpointCommands() {
  }

  private void testingMode() {
    // endEffector.algaeIntakeRotateMotorN.set(-getRightY(controllers.SECONDARY) *
    // 0.25);
    // endEffector.algaeIntakeMotorN.set(-getLeftY(controllers.SECONDARY));

    // if (secondaryController.getXButtonPressed()) {
    // endEffector.setAlgaeIntakeAngle(Constants.AlgaeIntakeAngles.max);
    // }

    // if (secondaryController.getAButtonPressed()) {
    // endEffector.setAlgaeIntakeAngle(Constants.AlgaeIntakeAngles.max/2);
    // }

    // if (secondaryController.getBButtonPressed()) {
    // endEffector.setAlgaeIntakeAngle(Constants.AlgaeIntakeAngles.min);
    // }

    // if (secondaryController.getXButtonPressed()) {
    // new PositionFromDashTest(NetworkTables.loc_s.getString("L4")).schedule();
    // }
    // for(int i = 0; i < 4; i++) {
    // SwerveDrive.getInstance().modules[i].turnMotor.set(getRightX(controllers.PRIMARY));
    // }
    // new Constants.ReefPoses().reefCoralPosesBlue.get(0),
  }

  public void periodic() {
    NetworkTables.driveModeManual_b.setBoolean(curControlMode == ControlMode.MANUAL);
    if (testing) {
      testingMode();
      return;
    }

    if (primaryController.getYButtonPressed()) {
      SwerveDrive.odometry.resetGyro();
    }

    if (primaryController.getStartButtonPressed()) {
      RobotContainer.odometry.recalibrateCameraPose();
    }

    switch (curControlMode) {
      case AUTO:
        NetworkTables.driveModeManual_b.setBoolean(false);
        AutoMode();
        break;
      case MANUAL:
        NetworkTables.driveModeManual_b.setBoolean(true);
        ManualMode();
        break;
      case OHNO_MANUAL:
        NetworkTables.driveModeManual_b.setBoolean(true);
        OHNOManualMode();
        break;
      default:
        // Integral to the code base DO NOT CHANGE! (Copilot did it!)
        throw new IllegalStateException("Invalid control mode: \n Nuh uh, no way, not gonna happen");
    }
  }
}
