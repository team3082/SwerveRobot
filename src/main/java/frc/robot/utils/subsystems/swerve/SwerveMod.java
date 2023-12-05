package frc.robot.utils.subsystems.swerve;

import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.RobotConfig;
import frc.robot.utils.Vector2;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.motorcontrol.Spark;


public class SwerveMod {
    
    private static final double ticksPerRotationSteer = RobotConfig.ticksPerRotationSteer;
    private static final double ticksPerRotationDrive = RobotConfig.ticksPerRotationDrive;

    public CANSparkMax steer;
    public CANSparkMax drive;
    public CANCoder absEncoder;

    public Vector2 pos;

    private boolean inverted;

    private final double cancoderOffset;

    private static final double simMaxTicksPerSecond = 40000;
    private double simSteerAng;
    private double simDriveVel;

    public SparkMaxPIDController drivePID, steerPID;
    private RelativeEncoder steerEncoder, driveEncoder;


    public SwerveMod(int steerID, int driveID, double x, double y, double cancoderOffset, double falconOffset) {
        absEncoder = new CANCoder(steerID);
        steer = new CANSparkMax(steerID, MotorType.kBrushless);
        drive = new CANSparkMax(driveID, MotorType.kBrushless);

        pos = new Vector2(x, y);

        drivePID = drive.getPIDController();
        steerPID = steer.getPIDController();

        //TODO TUNE PID

        // drivePID.setP();
        // drivePID.setI();
        // drivePID.setD();

        // steerPID.setP();
        // steerPID.setI();
        // steerPID.setD();

        steerEncoder = steer.getEncoder();
        driveEncoder = drive.getEncoder();

        steerEncoder.setPositionConversionFactor(42);
        driveEncoder.setPositionConversionFactor(42);
        
        drive.setInverted(true);
        steer.setInverted(false);
        drive.setIdleMode(IdleMode.kBrake);
        steer.setIdleMode(IdleMode.kBrake);

        absEncoder.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition);
        absEncoder.configMagnetOffset(0);
        absEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360);


        // Power Management
        // Cap off our current at 39 amps. If we go above 40 amps, the breaker will flip
        drive.setSmartCurrentLimit(39);
        // Enable voltage compensation to prevent variable behavior when the battery gets low/poor 
        drive.enableVoltageCompensation(12.5);

//        steer.configVoltageCompSaturation(12.0);
//        steer.enableVoltageCompensation(true);

        this.cancoderOffset = cancoderOffset;

        inverted = false;

        resetSteerSensor();
    }

    public void resetSteerSensor() {
        // Align the falcon to the cancoder
        double pos = absEncoder.getAbsolutePosition() - cancoderOffset;
        pos = pos / 360.0 * ticksPerRotationSteer;
        steerEncoder.setPosition(pos);
        steerPID.setReference(pos, ControlType.kPosition);
    }
    

    public void drive(double power) {
        drivePID.setReference(power, ControlType.kDutyCycle);
        simDriveVel = simMaxTicksPerSecond * power * (inverted ? -1.0 : 1.0);
    }

    // Rotates to angle given in radians
    public void rotateToRad(double angle) {
        rotate((angle - Math.PI / 2) / (2 * Math.PI) * ticksPerRotationSteer);
    }

    // Rotates to a position given in ticks
    public void rotate(double toAngle) {
        double motorPos;
        if (RobotBase.isSimulation())
            motorPos = simSteerAng;
        else 
            motorPos = steerEncoder.getPosition();

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

        //steer.set(TalonFXControlMode.MotionMagic, destination);
        steerPID.setReference(destination,ControlType.kPosition);

        simSteerAng = destination;
        
    }


    // Returns an angle in radians
    public double getSteerAngle() {
        if (RobotBase.isSimulation()) {
            return simSteerAng / ticksPerRotationSteer * Math.PI * 2 + Math.PI / 2;
        }
        return steerEncoder.getPosition() / ticksPerRotationSteer * Math.PI * 2 + Math.PI / 2;
        }

    

    public double getDriveVelocity() {
        if (RobotBase.isSimulation()) {
            return simDriveVel * 10 / ticksPerRotationDrive * (4 * Math.PI);
        }
        return driveEncoder.getVelocity() * 10 / ticksPerRotationDrive * (4 * Math.PI);
    }
}