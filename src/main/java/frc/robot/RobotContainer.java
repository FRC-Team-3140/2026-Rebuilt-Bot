// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.swerveDrive.Align;
import frc.robot.commands.swerveDrive.Drive;
import frc.robot.sensors.Camera;
import frc.robot.subsystems.*;
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
  public static TestRunner testRunner = TestRunner.getInstance();
  public static Controller controller = Controller.getInstance();
  public static Odometry odometry = Odometry.getInstance();
  public static Camera camera = Camera.getInstance();

  private SendableChooser<Command> PathPlanner = new SendableChooser<>();

  // WARNING: These booleans will override auto selectiosn!
  private boolean pushAutoMode = false;

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
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    //if (pushAutoMode)
      return new Drive(10000000, false, Constants.Bot.maxChassisSpeed / 2, 0, 0);
  }
}