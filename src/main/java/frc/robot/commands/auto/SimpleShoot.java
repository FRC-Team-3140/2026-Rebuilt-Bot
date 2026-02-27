// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import java.io.IOException;

import org.json.simple.parser.ParseException;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FileVersionException;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.turret.FireAway;
import frc.robot.subsystems.Turret.TurretMain;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SimpleShoot extends SequentialCommandGroup {
  private Command pathCommand;

  /** Creates a new SimpleShoot. */
  public SimpleShoot() {
    try {
      pathCommand = AutoBuilder.pathfindThenFollowPath(PathPlannerPath.fromPathFile("Simple Shoot Approach"),
          Constants.PathplannerConstants.pathplannerConstraints);
    } catch (FileVersionException | IOException | ParseException e) {
      e.printStackTrace();
    }

    // TODO: ADD SHOOT LOGIC WITH CHECKBOX TO TACK ON CLIMBING (WHILE SHOOTING)
    this.addCommands(
        pathCommand,
        new InstantCommand(() -> TurretMain.getInstance().setHoodAngle(Constants.Limits.Turret.minPitch)),
        new FireAway(TurretMain.getInstance()));
  }
}