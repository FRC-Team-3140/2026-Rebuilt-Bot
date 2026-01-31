// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import java.io.IOException;

import org.json.simple.parser.ParseException;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FileVersionException;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.turret.Fire_Away;
import frc.robot.libs.FlipPose;
import frc.robot.subsystems.Turret.TurretMain;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class L2R_Neutral_Shoot extends SequentialCommandGroup {
  private Command pathCommand;

  /** Creates a new L2R_Neutral_Shoot. */
  public L2R_Neutral_Shoot() {
    try {
      pathCommand = AutoBuilder.pathfindThenFollowPath(PathPlannerPath.fromPathFile("L2R-Neutral"),
          Constants.PathplannerConstants.pathplannerConstraints);
    } catch (FileVersionException | IOException | ParseException e) {
      e.printStackTrace();
    }

    // TODO: ADD SHOOT LOGIC WITH CHECKBOX TO TACK ON CLIMBING (WHILE SHOOTING)
    this.addCommands(pathCommand,
        AutoBuilder.pathfindToPose(FlipPose.flipIfRed(Constants.PathplannerConstants.shootPoseR),
            Constants.PathplannerConstants.pathplannerConstraints).alongWith(new Fire_Away(TurretMain.getInstance()))
            .alongWith(new PrintCommand(
                DriverStation.getAlliance().isPresent() ? DriverStation.getAlliance().get().toString() : "NONE :(")));
  }
}
