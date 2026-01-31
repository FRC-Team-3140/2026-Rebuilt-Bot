package frc.robot.tests;

import edu.wpi.first.networktables.NetworkTableEntry;
import frc.robot.subsystems.TestRunner.TestType;

public class TestClimber extends Test {
    // SequentialCommandGroup climbCommand = new Home(Climbers.getInstance())
    // .andThen(new Climb(Climbers.getInstance(), 3));

    public TestClimber(NetworkTableEntry entry, TestType type) {
        super(entry, type);
    }

    @Override
    public void Start() {
        super.Start();
        // CommandScheduler.getInstance()
        // .schedule(climbCommand);
    }

    @Override
    public void Periodic() {

    }

    @Override
    public void Stop() {
        // CommandScheduler.getInstance().cancel(climbCommand);
        super.Stop();
    }
}
