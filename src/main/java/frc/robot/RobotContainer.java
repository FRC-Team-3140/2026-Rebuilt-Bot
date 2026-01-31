// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.auto.L2R_Neutral;
import frc.robot.commands.auto.L2R_Neutral_Shoot;
import frc.robot.commands.auto.Pickup_Outpost;
import frc.robot.commands.auto.R2L_Neutral;
import frc.robot.commands.auto.R2L_Neutral_Shoot;
import frc.robot.commands.auto.SimpleShoot;
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
  public static TurretMain turret = TurretMain.getInstance();
  public static Feeder feeder = Feeder.getInstance();
  public static Intake intake = Intake.getInstance();
  public static Climbers climber = Climbers.getInstance();

  public static TestRunner testRunner = TestRunner.getInstance();

  private SendableChooser<Command> Path = new SendableChooser<>();

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
    Path.setDefaultOption("Normal - No PathPlanner", null);
    Path.addOption("Simple Mobility", new Drive(5, false, Constants.Bot.maxChassisSpeed / 2, 0, 0));
    Path.addOption("Simple Shoot", new SimpleShoot());
    Path.addOption("L2R Neutral", new L2R_Neutral());
    Path.addOption("L2R Neutral Shoot", new L2R_Neutral_Shoot());
    Path.addOption("R2L Neutral", new R2L_Neutral());
    Path.addOption("R2L Neutral Shoot", new R2L_Neutral_Shoot());
    Path.addOption("Outpost", new Pickup_Outpost());
    SmartDashboard.putData("Path", Path);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // if (pushAutoMode)
    // TODO: IF Mobiltiy checkbox is selected
    // return new Drive(10000000, false, Constants.Bot.maxChassisSpeed / 2, 0, 0);
    return Path.getSelected();
  }
}
