// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.swerveDrive.Drive;
import frc.robot.libs.FieldAprilTags;
import frc.robot.sensors.Camera;
import frc.robot.subsystems.Climbers;
import frc.robot.subsystems.Controller;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.TestRunner;
import frc.robot.subsystems.Turret.TurretMain;
import frc.robot.subsystems.odometry.Odometry;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  private static RobotContainer container = null;

  // The robot's subsystems and commands are defined here...
  public static SwerveDrive swerveDrive = SwerveDrive.getInstance();
  public static FieldAprilTags fieldAprilTags = FieldAprilTags.getInstance();
  public static Camera camera = Camera.getInstance();
  public static Odometry odometry = Odometry.getInstance();
  public static Controller controller = Controller.getInstance();
  //  public static TurretMain turret = TurretMain.getInstance();
  public static Feeder feeder = Feeder.getInstance();
  public static Intake intake = Intake.getInstance();
  public static Climbers climber = Climbers.getInstance();

  public static TestRunner testRunner = TestRunner.getInstance();

  private SendableChooser<Command> PathPlanner = new SendableChooser<>();

  // Get the singleton instance or create it if it doesn't exist
  public static RobotContainer getInstance() {
    if (container == null) {
      container = new RobotContainer();
    }
    return container;
  }

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  private RobotContainer() {
    PathPlanner.setDefaultOption("Normal - No PathPlanner", null);
    PathPlanner.addOption("Mobility", AutoBuilder.buildAuto("Simple Mobility"));
    SmartDashboard.putData("PathPlanner", PathPlanner);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // if (pushAutoMode)
    return new Drive(10000000, false, Constants.Bot.maxChassisSpeed / 2, 0, 0);
  }
}
