package frc.robot.subsystems.swerve;

import static frc.robot.utils.Constants.*;

import frc.robot.subsystems.Pigeon;
import frc.robot.utils.RTime;
import frc.robot.utils.Vector2;

public class SwerveManager {
    private static SwerveMod[] mods;
    private static SwerveKinematics kinematics;
    private static SwerveState state;
    private static SwerveState lastState;

    public static void init() {
        double xdist = WHEELBASEWIDTH / 2.0;
        double ydist = WHEELBASELENGTH / 2.0;

        mods = new SwerveMod[] {
            new SwerveMod(FLSTEERID, FLDRIVEID, -xdist, ydist, FLOFFSET),
            new SwerveMod(FRSTEERID, FRDRIVEID, xdist, ydist, FROFFSET),
            new SwerveMod(BRSTEERID, BRDRIVEID, xdist, -ydist, BROFFSET),
            new SwerveMod(BLSTEERID, BLDRIVEID, -xdist, -ydist, BLOFFSET)
        };

        kinematics = new SwerveKinematics(mods);
        state = new SwerveState(new Vector2(0,0), Math.PI/2);// This is temporary, if we add vision, it will be updated with vision
        lastState = state;
    }

    public static void update(){
        for(SwerveMod mod : mods){
            mod.update();
        }

        //updating odometry
        lastState = state;
        state = new SwerveState(state.pos.add(kinematics.getInnovationEulers()), Pigeon.getRotationRad());
    }

    public static Vector2 getVelocity(){
        return state.pos.sub(lastState.pos).div(RTime.deltaTime());
    }

    public static double getAngularVelocity(){
        return (state.heading - lastState.heading) / RTime.deltaTime();
    }

    /**
     * Rotate and drive the robot's drivetrain.
     * @param translation desired translation wrt the field
     * @param rotation desired rotation of the robot
     */
    public static void rotateAndDrive(Vector2 translation, double rotation) {

        // Fetch the robot's current heading in radians
        double heading = Pigeon.getRotationRad();

        // Multiply the translation vector against a rotation matrix
        // to compensate for the Pigeon's heading.
        Vector2 translated = translation.rotate(RA - heading); //wrt robot

        // The desired states of the modules
        Vector2[] moduleStates = new Vector2[4];

        // The maximum output this one module can have.
        // If any module is over, all the outputs are scaled down.
        double max = 1.0;

        for (int i = 0; i < 4; i++) {
            Vector2 rot = mods[i].getPosition().rotate(RA).mul(rotation); //wrt robot 
            moduleStates[i] = rot.add(translated);
            max = Math.max(max, moduleStates[i].mag());
        }

        // Scale the movement down to ensure we don't 
        // attempt a Duty Cycle over 1.
        for (int i = 0; i < 4; i++) {
            mods[i].setState(moduleStates[i].div(max));
        }
    }

    /**
     * Set the drivetrain in an "X" pattern,
     * clamping the wheels down to prevent movement.
     */
    public static void brake() {
        for (SwerveMod mod : mods) {
            mod.rotate(mod.getPosition().atan2());
            mod.drive(0);
        }
    }
}
