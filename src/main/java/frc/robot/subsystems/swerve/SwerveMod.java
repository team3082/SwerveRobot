package frc.robot.subsystems.swerve;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import frc.robot.utils.Vector2;

import static frc.robot.utils.Constants.TAU;
import static frc.robot.utils.Constants.*;

public class SwerveMod {

    private final CANSparkMax steer;
    private final CANSparkMax drive;
    private final CANCoder absEncoder;

    private final Vector2 pos;

    private boolean inverted;

    private final double cancoderOffset;

    private SparkMaxPIDController drivePID, steerPID;
    private final RelativeEncoder steerEncoder, driveEncoder;


    public SwerveMod(int steerID, int driveID, double x, double y, double cancoderOffset) {

        steer = new CANSparkMax(steerID, MotorType.kBrushless);
        drive = new CANSparkMax(driveID, MotorType.kBrushless);
        absEncoder = new CANCoder(steerID);

        pos = new Vector2(x, y);

        drivePID = drive.getPIDController();
        steerPID = steer.getPIDController();

        // TODO TUNE PID

        // drivePID.setP();
        // drivePID.setI();
        // drivePID.setD();

        // steerPID.setP();
        // steerPID.setI();
        // steerPID.setD();

        steerEncoder = steer.getEncoder();
        driveEncoder = drive.getEncoder();

        // Ensure the intergrated encoders report back in radians
        steerEncoder.setPositionConversionFactor(steerEncoder.getCountsPerRevolution() * STEERRATIO * TAU);
        driveEncoder.setPositionConversionFactor(driveEncoder.getCountsPerRevolution() * DRIVERATIO * TAU);
        steerEncoder.setVelocityConversionFactor(steerEncoder.getCountsPerRevolution() * STEERRATIO * TAU);
        driveEncoder.setVelocityConversionFactor(driveEncoder.getCountsPerRevolution() * DRIVERATIO * TAU);
        
        drive.setInverted(true);
        steer.setInverted(false);

        drive.setIdleMode(IdleMode.kBrake);
        steer.setIdleMode(IdleMode.kBrake);

        absEncoder.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition);
        absEncoder.configMagnetOffset(0);
        absEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360);

        // Cap off our current at 39 amps. If we go above 40 amps, the breaker will flip
        drive.setSmartCurrentLimit(39);
        steer.setSmartCurrentLimit(39);

        // Enable voltage compensation to prevent variable behavior when the battery gets low/poor 
        drive.enableVoltageCompensation(12.5);
        steer.enableVoltageCompensation(12.5);

        this.cancoderOffset = cancoderOffset;

        inverted = false;

        resetSteerSensor();
    }

    private void resetSteerSensor() {
        double pos = absEncoder.getAbsolutePosition() - cancoderOffset;
        pos = (pos / 360.0) * TAU;
        steerEncoder.setPosition(pos);
    }
    
    /**
     * Drive the drive motor
     * @param power Power to send to the motor, represented -1 to 1.
     */
    public void drive(double power) {
        drivePID.setReference(power, ControlType.kDutyCycle);
    }

    /**
     * Rotates to an angle given in radians
     * @param destPos desired angle in radians,
     * clamped at (0, 2pi)
     */
    public void rotate(double destPos) {
        
        double steerPosUnclamped = steerEncoder.getPosition();

        // Clamping steerPos to (0, 2pi);
        double steerPos = (steerPosUnclamped % TAU + 1) % TAU;
        double diff = destPos - steerPos;

        //The minimum angular displacement between pos and destination pos
        double minDisp;

        // Determine the closest viable position for the motor to go to
        // Whether that be we make a full rotation,
        // or we invert the motor to reach the destination faster.
        if (Math.abs(diff) > 1.5 * Math.PI) {
            minDisp = diff - Math.signum(diff) * TAU;
        } else if(Math.abs(diff) > 0.5 * Math.PI) {
            inverted = !inverted;
            minDisp = diff - Math.signum(diff) * Math.PI;
        } else {
            minDisp = diff;
        }

        // The true destination the motor should rotate to
        double trueDest = steerPosUnclamped + minDisp;

        steerPID.setReference(trueDest, ControlType.kPosition);
    }

    public void setState(Vector2 command) {
        rotate(command.atan2());
        drive(command.mag());
    }

    /** Returns steer angle with reference to the front of the robot in radians */
    double getSteerAngle() {
        return steerEncoder.getPosition();
    }
    
    /** Returns drive velocity in ft/s */
    double getDriveVelocity() {
        return driveEncoder.getVelocity() / 60 * (WHEELDIAMETER / 2.0);
    }

    Vector2 getPosition(){
        return pos.clone();
    }
}