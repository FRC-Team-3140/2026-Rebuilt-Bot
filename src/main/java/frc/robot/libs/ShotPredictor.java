package frc.robot.libs;

import java.util.HashMap;
import java.util.Optional;

import edu.wpi.first.math.Pair;

public class ShotPredictor {
    public double MinAngle;
    public double MaxAngle;
    public double MaxAngleVelocity;
    //public double DesiredShotVelocity;
    public HeightBounds Bounds;
    //public double MaxVelocityError;

    private static final int NewtonRepetitions = 8;
    private static final double maxNewtonError = 0.1;
    private static final double gravity = -9.8;

    private static final int updateSteps = 5; // divides the possible angle range
    private static final int updateRepetitions = 5; // repeats the process like a binary search

    public static class Result {
        public double ShotSpeed;
        public double TravelTime;
        public Vector2 AimPosition;
        public double ShotAngle;

        public Result() {

        }

        public Vector2 GetAimDirection(Vector2 targetPosition, Vector2 botVelocity) {
            return targetPosition.sub(botVelocity.mult(TravelTime));
        }
    }

    public static class HeightBounds {
        public double DistanceFromTarget;
        public double DesiredHeight;
        public double MinHeight;
        public double MaxHeight;

        public HeightBounds(double distanceFromTarget, double desiredHeight, double minHeight, double maxHeight) {
            DistanceFromTarget = distanceFromTarget;
            DesiredHeight = desiredHeight;
            MinHeight = minHeight;
            MaxHeight = maxHeight;
        }

        public double GetError(double height) {
            return Math.abs(height - DesiredHeight);
        }

        public boolean IsInBounds(double height) {
            return height >= MinHeight && height <= MaxHeight;
        }

        public Pair<Double, Double> GetHeightAtSetDistance(Vector2 ballVelocity, double verticalVelocity, double travelTime) {
            double backTime = ballVelocity.magnitude()/DistanceFromTarget;
            double time = Math.max(travelTime - backTime, 0);
            double height = time * verticalVelocity + 0.5 * gravity * time * time;

            return new Pair<Double, Double>(height, time);
        }
    }

    public ShotPredictor(double minAngle, double maxAngle, double maxAngleVelocity, HeightBounds bounds) {
        MinAngle = minAngle;
        MaxAngle = maxAngle;
        MaxAngleVelocity = maxAngleVelocity;
        Bounds = bounds;
    }
    // public ShotPredictor(double minAngle, double maxAngle, double maxAngleVelocity, double desiredMaxHeight) {
    //   this(minAngle, maxAngle, maxAngleVelocity, desiredMaxHeight, 0, Double.MAX_VALUE);
    // }

    public static Optional<Result> Predict(double shotAngle, Vector2 botVelocity, double verticalVelocity,
            Vector2 relativeTargetPosition, double targetHeight) {
        // If you want to understand the math (kinda)
        // https://www.overleaf.com/read/dfcbwtvytpms#32640b
        double botSpeedSq = botVelocity.magSq();
        double targetDistance = relativeTargetPosition.magnitude();
        double dot = -relativeTargetPosition.dot(botVelocity);
        double tansq = Math.tan(Math.toRadians(shotAngle));
        tansq *= tansq;

        Quartic q = new Quartic(
                0.25 * gravity * gravity,
                verticalVelocity * gravity,
                verticalVelocity * verticalVelocity - targetHeight * gravity - tansq * botSpeedSq,
                -(2 * targetHeight * verticalVelocity + 2 * tansq * dot),
                targetHeight * targetHeight - tansq * targetDistance * targetDistance);

        double guess = (verticalVelocity
                + Math.sqrt(verticalVelocity * verticalVelocity - 2 * gravity * targetDistance * tansq)) / (-gravity)
                + 2;
        Optional<Double> quarticResult = q.FindRoot(guess, NewtonRepetitions, maxNewtonError);

        if (quarticResult.isEmpty()) {
            return Optional.empty();
        }

        double travelTime = quarticResult.get();

        if (travelTime <= 0) {
            return Optional.empty();
        }

        Result result = new Result();
        result.TravelTime = travelTime;
        result.ShotAngle = shotAngle;
        result.ShotSpeed = (Math
                .sqrt(botSpeedSq * travelTime * travelTime + targetDistance * targetDistance + 2 * travelTime * dot))
                / (travelTime * Math.cos(Math.toRadians(shotAngle)));
        result.AimPosition = relativeTargetPosition.add(botVelocity.mult(-travelTime));

        return Optional.of(result);
    }

    public Optional<Result> Update(boolean loose, double currentAngle, double deltaTime, Vector2 botVelocity,
            double verticalVelocity, Vector2 relativeTargetPosition, double targetHeight, double shooterHeight) {
        double min = loose ? MinAngle : Math.max(MinAngle, currentAngle - deltaTime * MaxAngleVelocity);
        double max = loose ? MaxAngle : Math.min(MaxAngle, currentAngle + deltaTime * MaxAngleVelocity);

        double finalAngle = currentAngle;
        HashMap<Double, Optional<Result>> cache = new HashMap<Double, Optional<Result>>();

        // repetitively close in on the best angle by repeating the process
        for (int i = 0; i < updateRepetitions; i++) {

            double bestAngle = 0;
            double bestErr = Double.MAX_VALUE;
            boolean foundOption = false;

            // divide the interval into sections and find the best one
            double halfStepSize = (max - min) / (updateSteps * 2);
            for (double angle = min + halfStepSize; angle < max; angle += 2 * halfStepSize) {
                Optional<Result> resultOpt = cache.containsKey(angle)
                        ? cache.get(angle)
                        : Predict(angle, botVelocity, verticalVelocity, relativeTargetPosition, targetHeight - shooterHeight);

                cache.put(angle, resultOpt);

                if (resultOpt.isEmpty()) {
                  
                    continue;
                }

                Result result = resultOpt.get();

                Vector2 aimPosition = result.GetAimDirection(relativeTargetPosition, botVelocity);
                Vector2 ballVelocity = aimPosition.div(result.TravelTime);
                double ballVerticalVelocity = verticalVelocity + result.ShotSpeed * Math.sin(Math.toRadians(result.ShotAngle));        

                Pair<Double, Double> heightInst = Bounds.GetHeightAtSetDistance(
                    ballVelocity,
                    ballVerticalVelocity,
                    result.TravelTime
                );

                double height = heightInst.getFirst() + shooterHeight;
                //double heightTime = heightInst.getSecond();

                double err = Bounds.GetError(height);
                if (err < bestErr && Bounds.IsInBounds(height)) {
                    bestErr = err;
                    bestAngle = angle;
                    foundOption = true;
                }
            }

            // no options, so fail
            if (!foundOption) {
                return Optional.empty();
            }

            // repeat with a new interval closed in around the best option
            min = bestAngle - halfStepSize;
            finalAngle = bestAngle;
            max = bestAngle + halfStepSize;
        }

        return cache.getOrDefault(finalAngle, Optional.empty());
    }
}
