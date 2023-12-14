package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import frc.robot.subsystems.swerve.SwervePosition;
import frc.robot.utilities.Vector2;

// Things to think about:
// Do we want a NT instance per camera?
// (i.e. we have a NetworkTables per Camera)?
// Or an object of values for each camera?
// (i.e. CameraOneData as an enum for data from Camera 1)?

/**
 * Revamped Telemetry file, containing robot tracking
 * and monitoring for the Vision subsystem.
 * Also enables for PID tuning alongside the dashboard.
 */
public class NetworkTables {
    static final double IN_TO_M = 1 / 39.37;

    private static final ShuffleboardTab robotTab = Shuffleboard.getTab("SmartDashboard");

    // Field positions
    private static final Field2d field = new Field2d();
    private static Vector2 prevSimPos = new Vector2();
    private static Rotation2d prevSimRot = new Rotation2d();

    // Vision NetworkTables
    public static NetworkTableEntry targetDetected;
    public static NetworkTableEntry PLACEHOLDER_X;
    public static NetworkTableEntry PLACEHOLDER_Y;
    public static NetworkTableEntry PLACEHOLDER_ROT;

    public static void init() {

        // Find our NetworkTable
        NetworkTableInstance NT = NetworkTableInstance.getDefault();
        NetworkTable table = NT.getTable("ChickenVision");

        // Fetch values from vision and input them into the table.
        targetDetected = table.getEntry("target");
        PLACEHOLDER_X = table.getEntry("x");
        PLACEHOLDER_Y = table.getEntry("y");
        PLACEHOLDER_ROT = table.getEntry("rot");
        
        robotTab.add("Field View", field);
    }

    public static double getX() {
        return PLACEHOLDER_X.getDouble(Double.MAX_VALUE);
    }

    public static double getY() {
        return PLACEHOLDER_Y.getDouble(Double.MAX_VALUE);
    }

    public static double getRot() {
        return PLACEHOLDER_ROT.getDouble(Double.MAX_VALUE);
    }

    public static void update() {

        // -1 if we're on the red alliance, 1 if we're on the blue alliance
        int allianceMultiplier = (DriverStation.getAlliance() == DriverStation.Alliance.Red) ? -1 : 1;

        if (RobotBase.isSimulation()) {
            // Allow the user to drag the robot around if we're in simulation mode
            Vector2 modifiedSimPos = new Vector2(field.getRobotPose().getX(), field.getRobotPose().getY());
            if (prevSimPos.sub(modifiedSimPos).mag() > 0.001) {
                Vector2 modifiedSwervePos = modifiedSimPos
                        .div(IN_TO_M)
                        .sub(new Vector2(325.62, 157.75));
                SwervePosition.setPosition(new Vector2(-modifiedSwervePos.y, modifiedSwervePos.x * allianceMultiplier));
            }

            Rotation2d modifiedSimRot = field.getRobotPose().getRotation();
            if (Math.abs(prevSimRot.minus(modifiedSimRot).getRadians()) > 0.001)
                Pigeon.setSimulatedRot(modifiedSimRot.getRadians() + Math.PI / 2 * allianceMultiplier);
        }

        // Update field position and trajectory
        Vector2 fieldPosMeters = new Vector2(SwervePosition.getPosition().y * allianceMultiplier, -SwervePosition.getPosition().x)
                .add(new Vector2(325.62, 157.75))
                .mul(IN_TO_M);
        Rotation2d rotation = Rotation2d.fromRadians(Pigeon.getRotationRad() - Math.PI / 2 * allianceMultiplier);
        field.setRobotPose(fieldPosMeters.x, fieldPosMeters.y, rotation);

        // Store the field position for the next frame to check if it has been manually changed
        prevSimPos = fieldPosMeters;
        prevSimRot = rotation;
        
    }
}
