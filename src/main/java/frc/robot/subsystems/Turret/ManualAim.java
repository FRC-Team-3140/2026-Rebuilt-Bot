package frc.robot.subsystems.Turret;

import edu.wpi.first.wpilibj.XboxController;
import frc.robot.subsystems.Controller;

public class ManualAim extends AimType {
    private XboxController controller = Controller.getInstance().secondaryController;

    private final double manualAimRotationSpeed = 60; // degrees per second
    private final double manualAimHoodSpeed = 30; // degrees per second

    public ManualAim() {
        rotationAngle = 0;
        hoodAngle = 0;
        flywheelSpeed = 0;
    }

    @Override
    public void periodic(double deltaTime) {
        rotationAngle += -controller.getRightX() * manualAimRotationSpeed * deltaTime;
        hoodAngle += -controller.getLeftY() * manualAimHoodSpeed * deltaTime;
    }

    @Override
    public void activate(double rotationAngle, double hoodAngle, double flywheelSpeed) {

    }

    @Override
    public void deactivate() {

    }

}
