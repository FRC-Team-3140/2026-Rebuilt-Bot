// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.Climbers;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Home extends Command {
  private final Climbers climbers;

  private ParallelCommandGroup homeCommands;

  /** Creates a new home. */
  public Home(Climbers climbers) {
    this.climbers = climbers;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.climbers);

    this.withTimeout(4);

    homeCommands = new ParallelCommandGroup(
        new InstantCommand(() -> {
          climbers.LEFT.setSpeed(0.2);

          while (!climbers.LEFT.getLimitSwitch()) {
            // Wait until the left climber limit switch is pressed
          }

          climbers.LEFT.setSpeed(0.0);
        }),
        new InstantCommand(() -> {
          climbers.RIGHT.setSpeed(0.2);

          while (!climbers.RIGHT.getLimitSwitch()) {
            // Wait until the left climber limit switch is pressed
          }

          climbers.RIGHT.setSpeed(0.0);
        }));

    this.alongWith(homeCommands);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return homeCommands.isFinished();
  }
}
