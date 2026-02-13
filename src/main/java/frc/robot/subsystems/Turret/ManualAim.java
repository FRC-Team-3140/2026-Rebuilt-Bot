package frc.robot.subsystems.Turret;

import edu.wpi.first.wpilibj.XboxController;
import frc.robot.subsystems.Controller;
import frc.robot.subsystems.odometry.Odometry;

public class ManualAim extends AimType {
    private XboxController controller = Controller.getInstance().secondaryController;

    private final double manualAimRotationSpeed = 60; // degrees per second
    private final double manualAimHoodSpeed = 30; // degrees per second

    private double desiredRotationAngle = 0;

    public ManualAim() {
        desiredRotationAngle = 0;
        rotationAngle = 0;
        hoodAngle = 0;
        flywheelSpeed = 0;
    }

    @Override
    public void periodic(double deltaTime) {
        desiredRotationAngle += -controller.getRightX() * manualAimRotationSpeed * deltaTime;
        hoodAngle += -controller.getLeftY() * manualAimHoodSpeed * deltaTime;
        rotationAngle = desiredRotationAngle - Odometry.getInstance().getRotation().getDegrees();
    }

    @Override
    public void activate(double rotationAngle, double hoodAngle, double flywheelSpeed) {

    }

    @Override
    public void deactivate() {

    }

    @Override
    public double getLookDirection() {
        return desiredRotationAngle;
    }
}
