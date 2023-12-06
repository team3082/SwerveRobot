package frc.robot.subsystems.swerve;

import static frc.robot.utils.Constants.*;

import frc.robot.subsystems.Pigeon;
import frc.robot.utils.Vector2;

public class SwerveManager {
    private static SwerveMod[] mods;

    public static void init(){
        double xdist = WHEELBASEWIDTH / 2.0;
        double ydist = WHEELBASELENGTH / 2.0;
        mods = new SwerveMod[] {
            new SwerveMod(FLSTEERID, FLDRIVEID, -xdist, ydist, FLCANCODEROFFSET),
            new SwerveMod(FRSTEERID, FRDRIVEID, xdist, ydist, FRCANCODEROFFSET),
            new SwerveMod(BRSTEERID, BRDRIVEID, xdist, -ydist, BRCANCODEROFFSET),
            new SwerveMod(BLSTEERID, BLDRIVEID, -xdist, -ydist, BLCANCODEROFFSET)
        };
    }

    /**
     * 
     * @param translation desired translation wrt the field
     * @param rotation desired rotation
     */
    public void rotateAndDrive(Vector2 translation, double rotation){
        double heading = Pigeon.getRotationRad();
        Vector2 trans = translation.rotate(RA - heading);//wrt robot
        Vector2[] commands = new Vector2[4];
        double max = 1.0;
        for(int i = 0; i < 4; i++){
            Vector2 rot = mods[i].getPosition().rotate(RA).mul(rotation);//wrt robot 
            commands[i] = rot.add(trans);
            max = Math.max(max, commands[i].mag());
        }

        //scaling movement down so we don't attempt use a duty cycle over 1
        for(int i = 0; i < 4; i++){
            mods[i].set(commands[i].div(max));
        }
    }

    public void brake(){
        for(SwerveMod mod : mods){
            mod.rotate(mod.getPosition().atan2());
            mod.drive(0);
        }
    }
}
