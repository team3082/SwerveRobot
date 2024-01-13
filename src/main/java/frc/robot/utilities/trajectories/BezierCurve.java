package frc.robot.utilities.trajectories;

import frc.robot.subsystems.swerve.SwerveState;
import frc.robot.utilities.Vector2;

public class BezierCurve implements SwerveTrajectory {

    Vector2 a, b, c, d;

    double length;
    double speed; // Inches / second

    public BezierCurve(Vector2 a, Vector2 b, Vector2 c, Vector2 d) {

        this.a = a;
        this.b = b;
        this.c = c;
        this.d = d;

        length = approxLength();
    }

    public Vector2 getPoint(double t) {
        double x = (Math.pow(1 - t,3) * a.x) +
        (3 * Math.pow(1 - t,2) * t * b.x) +
        (3 * (1 - t) * Math.pow(t,2) * c.x) +
        (Math.pow(t,3) * d.x);

        double y = (Math.pow(1 - t,3) * a.y) +
        (3 * Math.pow(1 - t,2) * t * b.y) +
        (3 * (1 - t) * Math.pow(t,2) * c.y) +
        (Math.pow(t,3) * d.y);

        Vector2 r = new Vector2(x, y);

        return r;
    }

    public Vector2 getTangent(double t) {
        // get derivative of the curve
        double x = -3 * a.x * Math.pow((1 - t), 2) + 3 * b.x * (3 * Math.pow(t, 2) - 4 * t + 1) + 3 * c.x * (2 * t - 3 * Math.pow(t, 2)) + 3 * d.x * Math.pow(t, 2);
        double y = -3 * a.y * Math.pow((1 - t), 2) + 3 * b.y * (3 * Math.pow(t, 2) - 4 * t + 1) + 3 * c.y * (2 * t - 3 * Math.pow(t, 2)) + 3 * d.y * Math.pow(t, 2);
        Vector2 vectorTangent = new Vector2(x, y);
        vectorTangent.norm();
        return vectorTangent;
    }

    public double approxLength() {
        int n = 100;
        double l = 0;
        Vector2 pPoint = getPoint(0);
        for(int i = 0; i <= n; i++){
            double t = (double) i / (double) n;
            l += getPoint(t).sub(pPoint).mag();
            pPoint = getPoint(t);
        }
        return l;
    }

    public double approxRemainingLength(int startingI) {
        int n = 100;
        double l = 0;
        Vector2 pPoint = getPoint(0);
        for(int i = startingI; i <= n; i++){
            double t = (double) i / (double) n;
            l += getPoint(t).sub(pPoint).mag();
            pPoint = getPoint(t);
        }
        return l;
    }

    public double getClosestT(Vector2 robotPos) {
        int n = 100;
        double t = 0;
        double distance;
        Vector2 distanceVector;
        double smallestDistance;
        Vector2 point;
        point = getPoint(0);
        distanceVector = point.sub(robotPos);
        distance = distanceVector.mag();
        t = 0;
        smallestDistance = distance;
        for (int i = 1; i <= n; i++) {
            point = getPoint(i / n);
            distanceVector = point.sub(robotPos);
            distance = distanceVector.mag();
            if (distance < smallestDistance) {
                t = i / n;
                smallestDistance = distance;
            }
        }
        System.out.println("distance from t: " + smallestDistance + " closest value t: " + t);
        return t;
    }

    @Override
    public SwerveState get(double t) {
        return null;
    }

    @Override
    public double length() {
        return 0;
    }

    @Override
    public SwerveState startState() {
        return null;
    }

    @Override
    public SwerveState endState() {
        Vector2 pos = new Vector2(0, 0);
        Vector2 dpos = new Vector2(0, 0);
        // placeholder
        double theta = 0;
        double dtheta = 0;
        SwerveState state = new SwerveState(pos, theta, dpos, dtheta);
        return null;
    }
}