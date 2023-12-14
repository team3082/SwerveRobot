package frc.robot.subsystems.swerve;

import frc.robot.subsystems.Pigeon;
import frc.robot.utils.Vector2;
import static frc.robot.utils.Constants.RA;

public class SwerveKinematics {
    private final SwerveMod[] modules;

    public SwerveKinematics(SwerveMod[] modules){
        this.modules = modules;
    }

    /**calculates the displacement of the robot since the last frame using euler's method*/
    Vector2 getInnovationEulers(){
        Vector2[] modDisplacements = new Vector2[modules.length];//wrt robot
        for(int i = 0; i < modules.length; i++){
            modDisplacements[i] = Vector2.fromPolar(modules[i].getSteerAngle(), modules[i].getDeltaDrive());
        }

        Vector2 averageDisplacement = Vector2.average(modDisplacements);
        return averageDisplacement.rotate(Pigeon.getRotationRad() - RA);//rotating to field frame of reference
    }

    /**calculates the displacement of the robot since the last frame using pose exponentiation assuming constant angular velocity of both the drive and steer motors*/
    Vector2 getInnovationExp(){
        Vector2[] modDisplacements = new Vector2[modules.length];//wrt robot
        for(int i = 0; i < modules.length; i++){
            SwerveMod mod = modules[i];
            double xDisplacement = mod.getDeltaDrive() * (Math.sin(mod.getSteerAngle()) - Math.sin(mod.getLastSteerAngle())) / mod.getDeltaSteer();
            double yDisplacement = mod.getDeltaDrive() * (Math.cos(mod.getLastSteerAngle() - Math.cos(mod.getSteerAngle()))) / mod.getDeltaSteer();
            modDisplacements[i] = new Vector2(xDisplacement, yDisplacement);
        }
        
        Vector2 averageDisplacement = Vector2.average(modDisplacements);
        return averageDisplacement.rotate(Pigeon.getRotationRad() - RA);//rotating to field frame of reference
    }
}
