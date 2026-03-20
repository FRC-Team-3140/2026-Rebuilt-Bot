// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.turret.FireAway;
import frc.robot.subsystems.Controller;
import frc.robot.subsystems.Controller.ControlMode;
import frc.robot.subsystems.Turret.TurretMain;
import frc.robot.subsystems.Turret.TurretMain.AimOpt;
import frc.robot.libs.NetworkTables;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SuperSimpleShoot extends SequentialCommandGroup {
  /** Creates a new SimpleShoot. */
  public SuperSimpleShoot() {
    this.addCommands(
        new InstantCommand(() -> TurretMain.getInstance().setAimMode(AimOpt.MANUAL)),
        new InstantCommand(() -> Controller.getInstance().setControlMode(ControlMode.MANUAL)),
        new InstantCommand(() -> TurretMain.getInstance().setHoodAngle(25)),
        new InstantCommand(() -> NetworkTables.flywheelRPMOverride_d.setDouble(5000)),
        new InstantCommand(() -> TurretMain.flywheelRPMOverride = true),
        new FireAway(TurretMain.getInstance(), true));
  }
}
