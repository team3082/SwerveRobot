package frc.robot.subsystems.swerve;

import com.ctre.phoenix.sensors.*;
import com.revrobotics.*;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.subsystems.swerve.states.SwerveState;
import frc.robot.utils.Vector2;

public class SwerveModule {
    
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

    private final SimpleMotorFeedforward feedforward =
      new SimpleMotorFeedforward(
          0, 
          0, 
          0
        );

    public SwerveModule(int dID, int sID, double x, double y, double cancoderOffset) {

        driveMotor = new CANSparkMax(dID, MotorType.kBrushless);
        steerMotor = new CANSparkMax(sID, MotorType.kBrushless);
        absEncoder = new CANCoder(sID);

        driveEncoder = driveMotor.getEncoder();
        steerEncoder = steerMotor.getEncoder();

        steerPID = steerMotor.getPIDController();
        drivePID = driveMotor.getPIDController();

        this.pos = new Vector2(x, y);

        driveMotor.setIdleMode(IdleMode.kCoast);
        steerMotor.setIdleMode(IdleMode.kCoast);

        steerMotor.restoreFactoryDefaults();
        steerPID.setP(0);
        steerPID.setI(0);
        steerPID.setD(0);

        driveMotor.restoreFactoryDefaults();
        drivePID.setP(0);
        drivePID.setI(0);
        drivePID.setD(0);

        steerMotor.burnFlash();
        driveMotor.burnFlash();

        absEncoder.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition);
        absEncoder.configMagnetOffset(0);
        absEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360);

        resetEncoder();
    }

    /**
     * Align the NEO's encoder to that of the CANCoder.
     * Only done once during initial bring-up.
     */
    public void resetEncoder() {
        double cancoderPos = absEncoder.getAbsolutePosition() - offset;
        cancoderPos = cancoderPos / 360.0 * ticksPerRotationSteer;
        steerEncoder.setPosition(cancoderPos);
    }

    public void drive(double power) {
        double truePower = power * (inverted ? -1.0 : 1.0);
        this.drivePID.setReference(truePower, ControlType.kVelocity, 0, feedforward.calculate(truePower));

        simDriveVel = simMaxTicksPerSecond * power * (inverted ? -1.0 : 1.0);
    }

    /**
     * Rotate to an angle defined in radians.
     * @param angle The desired angle in radians
     */
    public void rotateToRad(double angle) {
        rotate((angle - Math.PI * 0.5) / (2 * Math.PI) * ticksPerRotationSteer);
    }

    /**
     * Rotate to an angle defined in motor ticks.
     * @param toAngle The desired angle in motor ticks
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

    public void setModuleState(SwerveState state) {
        drive(state.velocity);
        rotateToRad(state.rotation);
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
