package frc.robot.subsystems.Turret;

import java.util.Optional;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import frc.robot.libs.FlipPose;
import frc.robot.libs.ShotPredictor;
import frc.robot.libs.ShotPredictor.Result;
import frc.robot.libs.Vector2;
import frc.robot.subsystems.odometry.Odometry;

public class AutoAim extends AimType {
    private static double predictForwardTime = 0.1;

    private double desiredVelocity = 8;
    private double maxVelocityError = 2;

    private ShotPredictor shotPredictor = new ShotPredictor(
            Constants.Limits.Turret.minAngle,
            Constants.Limits.Turret.maxAngle,
            Constants.Limits.Turret.maxAngularVelocity,
            maxVelocityError);

    public AutoAim() {
        rotationAngle = 0;
        hoodAngle = 0;
        flywheelSpeed = 0;
    }

    private Pair<Pose2d, Vector2> getFutureState(double dt) {
        Odometry odometry = Odometry.getInstance();
        Vector2 futureBotVelocity = odometry.getBotVelocity().add(odometry.getBotAcceleration().mult(dt));

        // position = velocity*time + acceleration*0.5*time^2
        Vector2 predictedPositionChange = odometry.getBotVelocity().mult(dt).add(
                odometry.getBotAcceleration().mult(0.5 * dt * dt));
        Vector2 futureBotPosition = odometry.getPosition().add(predictedPositionChange);

        // TODO: make this the position of the turret relative to the axis of rotation
        // of the bot
        // this is used to get centripetal velocity
        // calculate for when the bot is facing in the positive x direction
        // might want to put this in constants too
        Vector2 turretPosition = new Vector2(-0.1366, 0); // this is a guess

        double futureRotation = odometry.getAngle() + odometry.getAngularVelocity() * dt; // in rads
        Vector2 rotatedTurretPosition = turretPosition.rotate(futureRotation);

        Vector2 centripetalVelocity = new Vector2(-rotatedTurretPosition.Y, rotatedTurretPosition.X)
                .mult(odometry.getAngularVelocity());
        Vector2 futureTurretVelocity = futureBotVelocity.add(centripetalVelocity);
        Vector2 shotOrigin = futureBotPosition.add(rotatedTurretPosition);

        return new Pair<Pose2d, Vector2>(new Pose2d(shotOrigin.X, shotOrigin.Y, new Rotation2d(futureRotation)),
                futureTurretVelocity);
    }

    private void predict(double deltaTime) {
        shotPredictor.DesiredShotVelocity = desiredVelocity; // incase we want to change desired velocity

        double targetHeight = Units.inchesToMeters(72 - 18 - 6); // TODO: make this correct (relative to turret)
        Vector2 goalPosition = FlipPose.flipVectorIfRed(new Vector2(4.625, 4.025));

        Pair<Pose2d, Vector2> futureStatePair = getFutureState(predictForwardTime);
        Pose2d futurePose = futureStatePair.getFirst();
        Vector2 futureShotOrigin = new Vector2(futurePose.getX(), futurePose.getY());
        double futureRotation = futurePose.getRotation().getRadians();
        Vector2 futureTurretVelocity = futureStatePair.getSecond();

        Vector2 relativeTargetPosition = goalPosition.sub(futureShotOrigin);
        double targetDistance = relativeTargetPosition.magnitude();

        shotPredictor.MinAngle = calculateMinimumAngle(targetDistance);

        // boolean isShooting =
        // Controller.getInstance().primaryController.getRightBumperButton();
        double verticalVelocity = 0; // TODO: (optional) make this work
        Optional<Result> result = shotPredictor.Update(
                true, // !isShooting,
                hoodAngle,
                deltaTime,
                futureTurretVelocity,
                verticalVelocity,
                relativeTargetPosition,
                targetHeight);

        shouldShoot = result.isPresent();

        if (result.isPresent()) {
            hoodAngle = result.get().ShotAngle;
            flywheelSpeed = result.get().ShotSpeed; // Convereted to RPM in TurretMain
            rotationAngle = Math
                    .toDegrees(Math.atan2(result.get().AimPosition.Y, result.get().AimPosition.X) - futureRotation);
        }
    }

    private static double calculateMinimumAngle(double distance) {
        if (distance >= Constants.Limits.Turret.nearInterpRange) {
            return Constants.Limits.Turret.minAngle;
        } else if (distance <= Constants.Limits.Turret.nearRange) {
            return Constants.Limits.Turret.nearMinAngle;
        } else {
            double alpha = (distance - Constants.Limits.Turret.nearRange)
                    / (Constants.Limits.Turret.nearInterpRange - Constants.Limits.Turret.nearRange);
            double interpolatedAngle = Constants.Limits.Turret.nearMinAngle +
                    (Constants.Limits.Turret.minAngle - Constants.Limits.Turret.nearMinAngle) * alpha;

            return interpolatedAngle;
        }
    }

    @Override
    public void periodic(double deltaTime) {
        predict(deltaTime);
    }

    @Override
    public void activate(double rotationAngle, double hoodAngle, double flywheelSpeed) {
        this.rotationAngle = rotationAngle;
        this.hoodAngle = hoodAngle;
        this.flywheelSpeed = flywheelSpeed;
    }

    @Override
    public void deactivate() {

    }

}
