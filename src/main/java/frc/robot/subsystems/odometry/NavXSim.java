package frc.robot.subsystems.odometry;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.SwerveDrive;

public class NavXSim {
    private static NavXSim instance;
    private double yawRadians = 0.0;
    private double angularVelocity = 0.0;

    private NavXSim() {
    }

    public static NavXSim getInstance() {
        if (instance == null) {
            instance = new NavXSim();
        }
        return instance;
    }

    public void reset(double yawRadians) {
        this.yawRadians = yawRadians;
    }

    public Rotation2d getRotation2d() {
        return new Rotation2d(yawRadians);
    }

    /**
     * Integrate chassis angular velocity (rad/s) over dt seconds
     */

    public void update(double omegaRadiansPerSec, double dt) {
        angularVelocity = omegaRadiansPerSec;
        yawRadians += omegaRadiansPerSec * dt;
    }

    public boolean isMoving() {
        return SwerveDrive.getInstance().getRobotRelativeSpeeds().omegaRadiansPerSecond + 
               SwerveDrive.getInstance().getRobotRelativeSpeeds().vxMetersPerSecond + 
               SwerveDrive.getInstance().getRobotRelativeSpeeds().vyMetersPerSecond > 0.01;
    }
}
