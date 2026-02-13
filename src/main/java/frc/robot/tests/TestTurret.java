package frc.robot.tests;

import edu.wpi.first.networktables.NetworkTableEntry;
import frc.robot.Constants;
import frc.robot.subsystems.TestRunner.TestType;
import frc.robot.subsystems.Turret.TurretMain;

public class TestTurret extends Test {
    private final TurretMain turret = TurretMain.getInstance();

    private long lastSwitchTime = System.currentTimeMillis();
    private int stage = 0;

    public TestTurret(NetworkTableEntry ntEntry, TestType type) {
        super(ntEntry, type);
    }

    @Override
    public void Start() {
        super.Start();
    }

    @Override
    public void Periodic() {
        // Toggles between straight driving, horizontal driving, and turning
        long currentTime = System.currentTimeMillis();
        if (currentTime - lastSwitchTime >= 2000) {
            stage = (stage + 1) % 3;
            lastSwitchTime = currentTime;
        }

        switch (stage) {
            case 0:
                // First section of code
                turret.setHoodAngle(Constants.Limits.Turret.maxPitch);
                break;
            case 1:
                // Second section of code
                turret.setHoodAngle(Constants.Limits.Turret.minPitch);
                break;
            case 2:
                // Third section of code
                turret.setRotationAngle(90);
                break;
            case 3:
                // TODO: Correct angles to fit final bounds
                turret.setRotationAngle(-90);
                break;
        }
    }

    @Override
    public void Stop() {
        super.Stop();
    }
}
