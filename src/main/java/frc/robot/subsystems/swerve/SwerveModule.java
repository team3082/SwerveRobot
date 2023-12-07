package frc.robot.subsystems.swerve;

import com.ctre.phoenix.sensors.*;
import com.revrobotics.*;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.utilities.*;

public class SwerveModule {

    // NEO's ticks per 1 rotation is 42
    // Much smaller than the Falcon's 2048... /shrug
    private static final double ticksPerRotationSteer = 42 * (150 / 7);
    private static final double ticksPerRotationDrive = 42 * 6.12;

    public CANSparkMax driveMotor;
    public CANSparkMax steerMotor;
    public SparkMaxPIDController drivePID, steerPID;

    public CANCoder absEncoder;
    private RelativeEncoder driveEncoder, steerEncoder;

    public Vector2 pos;
    private double offset;
    private boolean inverted;

    private static final double simMaxTicksPerSecond = 40000;
    private double simSteerAng;
    private double simDriveVel;

    public SwerveModule(int dID, int sID, double x, double y, double cancoderOffset) {

        this.driveMotor = new CANSparkMax(dID, MotorType.kBrushless);
        this.steerMotor = new CANSparkMax(sID, MotorType.kBrushless);
        this.absEncoder = new CANCoder(sID);

        this.pos = new Vector2(x, y);
        this.offset = cancoderOffset;

        this.driveMotor.restoreFactoryDefaults();
        this.driveEncoder = this.driveMotor.getEncoder();
        this.drivePID = this.driveMotor.getPIDController();
        this.drivePID.setP(0.02);
        this.drivePID.setI(0.002);
        this.drivePID.setD(0);

        this.steerMotor.restoreFactoryDefaults();
        this.steerEncoder = this.steerMotor.getEncoder();
        this.steerPID = this.steerMotor.getPIDController();
        this.steerPID.setP(0.25);
        this.steerPID.setI(0.01);
        this.steerPID.setD(0.1);

        this.driveMotor.setIdleMode(IdleMode.kBrake);
        this.steerMotor.setIdleMode(IdleMode.kBrake);

        this.absEncoder.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition);
        this.absEncoder.configMagnetOffset(0);
        this.absEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360);

        // We need to convert to motor ticks. It's easier than refactoring to rotations.
        this.steerEncoder.setPositionConversionFactor(42);
        this.driveEncoder.setPositionConversionFactor(42);

        this.resetSteerEncoder();
    }

    /**
     * Resets the steer motor's encoder to align with the CANcoder.
     */
    public void resetSteerEncoder() {
        double pos = this.absEncoder.getAbsolutePosition() - this.offset;
        pos = pos / 360.0 * ticksPerRotationSteer;
        this.steerEncoder.setPosition(pos);
        this.steerPID.setReference(pos, ControlType.kPosition);
    }

    /**
     * Apply power to the drive motor.
     * @param power Percent output desired, represented as a double.
     */
    public void drive(double power) {
        this.drivePID.setReference(power * (inverted ? -1.0 : 1.0), ControlType.kDutyCycle);

        simDriveVel = simMaxTicksPerSecond * power * (inverted ? -1.0 : 1.0);
    }

     /**
      * Rotate to an angle given radians.
      * @param angle Angle to rotate to, in radians.
      */
     public void rotateToRad(double angle) {
        rotate((angle - Math.PI * 0.5) / (2 * Math.PI) * ticksPerRotationSteer);
    }

    /**
     * Rotate the steer motor to a position, given in motor ticks.
     * @param toAngle Double representing motor ticks.
     */
    public void rotate(double toAngle) {
        double motorPos;
        if (RobotBase.isSimulation())
            motorPos = simSteerAng;
        else 
            motorPos = this.steerEncoder.getPosition();

        // The number of full rotations the motor has made
        int numRot = (int) Math.floor(motorPos / ticksPerRotationSteer);

        // The target motor position dictated by the joystick, in motor ticks
        double joystickTarget = numRot * ticksPerRotationSteer + toAngle;
        double joystickTargetPlus = joystickTarget + ticksPerRotationSteer;
        double joystickTargetMinus = joystickTarget - ticksPerRotationSteer;

        // The true destination for the motor to rotate to
        double destination;

        // Determine if, based on the current motor position, it should stay in the same
        // rotation, enter the next, or return to the previous.
        if (Math.abs(joystickTarget - motorPos) < Math.abs(joystickTargetPlus - motorPos)
                && Math.abs(joystickTarget - motorPos) < Math.abs(joystickTargetMinus - motorPos)) {
            destination = joystickTarget;
        } else if (Math.abs(joystickTargetPlus - motorPos) < Math.abs(joystickTargetMinus - motorPos)) {
            destination = joystickTargetPlus;
        } else {
            destination = joystickTargetMinus;
        }

        // If the target position is farther than a quarter rotation away from the
        // current position, invert its direction instead of rotating it the full
        // distance
        if (Math.abs(destination - motorPos) > ticksPerRotationSteer / 4.0) {
            inverted = true;
            if (destination > motorPos)
                destination -= ticksPerRotationSteer / 2.0;
            else
                destination += ticksPerRotationSteer / 2.0;
        } else {
            inverted = false;
        }

        this.steerPID.setReference(destination, ControlType.kPosition);

        simSteerAng = destination;
    }

    public double getSteerAngle() {
        
        if (RobotBase.isSimulation()) {
            return simSteerAng / ticksPerRotationSteer * Math.PI * 2 + Math.PI / 2;
        }

        return this.steerEncoder.getPosition() / ticksPerRotationSteer * Math.PI * 2;
    }

    public double getDrivePosition() {
        return this.driveEncoder.getPosition() / ticksPerRotationSteer * (3*Math.PI);
    }

    public double getDriveVelocity() {

        if (RobotBase.isSimulation()) {
            return simDriveVel * 10 / ticksPerRotationDrive * (4 * Math.PI);
        }

        return this.driveEncoder.getVelocity() * 10 / ticksPerRotationDrive * (4 * Math.PI);
    }
}