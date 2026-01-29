// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.libs.NetworkTables;
import frc.robot.subsystems.Turret.TurretMain;

public class Controller extends SubsystemBase {
  private static Controller instance = null;

  public final XboxController primaryController;

  private boolean rightTriggerTriggeredPrimary = false;
  private boolean leftTriggerTriggeredPrimary = false;

  public final XboxController secondaryController;

  private boolean rightTriggerTriggeredSecondary = false;
  private boolean leftTriggerTriggeredSecondary = false;

  /** Creates a new Controller. */
  public Controller(int primary, int secondary) {
    primaryController = new XboxController(primary);
    secondaryController = new XboxController(secondary);
  }

  private final double deadband = 0.1;
  private final double triggerThreshold = 0.5;

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

  public boolean getLeftTriggerTriggered(controllers controller) {
    XboxController contr = (controller == controllers.PRIMARY ? primaryController : secondaryController);

    boolean trigger = contr.getLeftTriggerAxis() > triggerThreshold;

    if (trigger) {
      if (controller == controllers.PRIMARY) {
        leftTriggerTriggeredPrimary = true;
      } else {
        leftTriggerTriggeredSecondary = true;
      }
      return trigger;
    } else {
      return false;
    }
  }

  public boolean getRightTriggerTriggered(controllers controller) {
    XboxController contr = (controller == controllers.PRIMARY ? primaryController : secondaryController);

    boolean trigger = contr.getRightTriggerAxis() > triggerThreshold;

    if (trigger) {
      if (controller == controllers.PRIMARY) {
        rightTriggerTriggeredPrimary = true;
      } else {
        rightTriggerTriggeredSecondary = true;
      }
      return trigger;
    } else {
      return false;
    }
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

  /**
   * Reusable controls for all modes go here.
   *
   * <p>
   * 
   * Mapped Buttons:
   * <ol>
   * <li>Y - Reset Gyro</li>
   * <li>A - Deploy Intake</li>
   * <li>B - Stow Intake</li>
   * <li>Left Bumper - Intake</li>
   * <li>Right Bumper - Spinup Flywheel</li>
   * <li>Start - Recalibrate Camera Pose</li>
   * </ol>
   */
  private void reusableDefaultControls() {
    if (primaryController.getYButtonPressed()) {
      SwerveDrive.odometry.resetGyro();
    }

    if (primaryController.getStartButtonPressed()) {
      RobotContainer.odometry.recalibrateCameraPose();
    }

    if (primaryController.getLeftBumperButtonPressed()) {
      Intake.getInstance().intake(Constants.MotorSpeeds.Intake.intakeSpeed);
    }

    if (primaryController.getXButtonPressed()) {
      TurretMain.getInstance().shootSimFuel();
    }

    if (primaryController.getRightBumperButtonPressed()) {
      TurretMain.getInstance().setFlywheelActive(true);
    }

    if (primaryController.getRightBumperButtonReleased()) {
      TurretMain.getInstance().setFlywheelActive(false);
    }

    if (primaryController.getBButtonPressed()) {
      Intake.getInstance().stow();
    }

    if (primaryController.getAButtonPressed()) {
      Intake.getInstance().deploy();
    }
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
    } else if (getLeftTriggerTriggered(controllers.SECONDARY) && getRightTriggerTriggered(controllers.SECONDARY)) {
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

    reusableDefaultControls();

    if (primaryController.getRightTriggerAxis() > triggerThreshold && TurretMain.getInstance().getFlywheelActive()) {
      Feeder.getInstance().setFeederActive(true);
    }

    if (primaryController.getLeftTriggerAxis() > triggerThreshold) {
      // TODO: Climb
    }
  }

  private void ManualMode() {
    if (secondaryController.getLeftStickButton() && secondaryController.getRightStickButton()) {
      updateControlMode();
      return;
    }

    reusableDefaultControls();

    if (primaryController.getLeftTriggerAxis() > triggerThreshold) {
      // TODO: Climb with primary && Climb Manually With Secondary
    }

    // TODO: Turn Turret manually without camera
  }

  private void OHNOManualMode() {
    if (secondaryController.getLeftStickButton() && secondaryController.getRightStickButton()) {
      updateControlMode();
      return;
    }

    Feeder.getInstance().setFeederActive(
        primaryController.getRightTriggerAxis() > triggerThreshold && primaryController.getRightBumperButton());

    if (primaryController.getLeftTriggerAxis() > triggerThreshold) {
      // TODO: Climb Manually With Secondary
    }

    // TODO: Turn Turret manually without camera
  }

  private void testingMode() {
    System.out.println("Testing Mode Active - Controller Periodic Function");
  }

  public void periodic() {
    NetworkTables.driveModeManual_b.setBoolean(curControlMode == ControlMode.MANUAL);
    if (testing) {
      testingMode();
      return;
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

    if (leftTriggerTriggeredPrimary && primaryController.getLeftTriggerAxis() < triggerThreshold) {
      leftTriggerTriggeredPrimary = false;
    }

    if (rightTriggerTriggeredPrimary && primaryController.getRightTriggerAxis() < triggerThreshold) {
      rightTriggerTriggeredPrimary = false;
    }

    if (leftTriggerTriggeredSecondary && secondaryController.getLeftTriggerAxis() < triggerThreshold) {
      leftTriggerTriggeredSecondary = false;
    }

    if (rightTriggerTriggeredSecondary && secondaryController.getRightTriggerAxis() < triggerThreshold) {
      rightTriggerTriggeredSecondary = false;
    }
  }
}
