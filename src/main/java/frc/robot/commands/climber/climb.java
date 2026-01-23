// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climber;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climbers;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class climb extends Command {
  private final Climbers climbers;
  private final int level;

  private int currentLevel = 0;

  private final int deployTime = 4; // seconds

  public class LEVELS {
    public static final int LEVEL_1 = 1;
    public static final int LEVEL_2 = 2;
    public static final int LEVEL_3 = 3;
  }

  /** Creates a new climb. */
  public climb(Climbers climbers, int level) {
    this.climbers = climbers;
    this.level = level;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.climbers);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    while (currentLevel < level) {
      climbers.LEFT.setSpeed(1);
      climbers.RIGHT.setSpeed(1);

      Timer.delay(deployTime);

      climbers.LEFT.setSpeed(-1);
      climbers.RIGHT.setSpeed(-1);

      currentLevel++;
    }

    currentLevel = level;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return currentLevel == level;
  }
}
