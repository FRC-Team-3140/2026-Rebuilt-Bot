package frc.robot.libs;

import java.util.Optional;

public class Quartic {
    public double a;
    public double b;
    public double c;
    public double d;
    public double e;

    public Quartic(double a, double b, double c, double d, double e) {
        this.a = a;
        this.b = b;
        this.c = c;
        this.d = d;
        this.e = e;
    }

    public double Evaluate(double x) {
        return this.a * Math.pow(x, 4) + this.b * Math.pow(x, 3) + this.c * Math.pow(x, 2) + this.d * (x) + this.e;
    }

    public double EvaluateDerivative(double x) {
        return 4 * this.a * Math.pow(x, 3) + 3 * this.b * Math.pow(x, 2) + 2 * this.c * x + this.d;
    }

    public Optional<Double> FindRoot(double guess, int maxIterations, double maxError) {
        // newton approximation
        for (int i = 0; i < maxIterations; i++) {
            double height = Evaluate(guess);
            double slope = EvaluateDerivative(guess);

            if (slope == 0) {
                guess += 0.001;
            } else {
                guess -= height / slope;
            }
        }

        double error = Math.abs(Evaluate(guess));
        if (error > maxError) {
            return Optional.empty();
        }

        return Optional.of(guess);
    }
}
