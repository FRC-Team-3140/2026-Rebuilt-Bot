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
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.subsystems.Intake;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Depot extends SequentialCommandGroup {
  private Command pathCommand;

  /** Creates a new Depot. */
  public Depot() {
    try {
      pathCommand = AutoBuilder.pathfindThenFollowPath(PathPlannerPath.fromPathFile("Depot Approach"),
          Constants.PathplannerConstants.pathplannerConstraints);
    } catch (FileVersionException | IOException | ParseException e) {
      // TODO Auto-generated catch block
      e.printStackTrace();
    }

    // TODO: ADD SHOOT LOGIC WITH CHECKBOX TO TACK ON CLIMBING (WHILE SHOOTING)
    this.addCommands(pathCommand.andThen(new InstantCommand(() -> Intake.getInstance().deploy())),
        new WaitCommand(4) /* Then GO CLIMB */);
  }
}
