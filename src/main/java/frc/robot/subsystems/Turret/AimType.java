package frc.robot.subsystems.Turret;

public abstract class AimType {
    public double rotationAngle;
    public double hoodAngle;
    public double flywheelSpeed;

    public boolean shouldShoot = false;

    public void periodic(double deltaTime) {
    };

    public void activate(double rotationAngle, double hoodAngle, double flywheelSpeed) {
    };

    public void deactivate() {
    };

    public double getLookDirection() {
        return 0;
    };
}
