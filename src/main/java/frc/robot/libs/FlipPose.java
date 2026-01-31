// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.libs;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants.PathplannerConstants;

/** Used to flip poses accross the Reefscape field */
public class FlipPose {
    public static Pose2d flipIfRed(Pose2d pose) {
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red) {
            // X must be flipped, but Y stays the same.
            double flippedX = PathplannerConstants.FieldLength - pose.getX();
            double flippedY = PathplannerConstants.FieldWidth - pose.getY();

            // In Radians
            Rotation2d flippedRot = new Rotation2d(pose.getRotation().getRadians() + Math.PI);

            return new Pose2d(flippedX, flippedY, flippedRot);
        }

        return pose;
    }

    public static Vector2 flipVectorIfRed(Vector2 vector) {
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red) {
            // X must be flipped, but Y stays the same.
            double flippedX = PathplannerConstants.FieldLength - vector.X;
            double flippedY = PathplannerConstants.FieldWidth - vector.Y;

            return new Vector2(flippedX, flippedY);
        }

        return vector;
    }
}
