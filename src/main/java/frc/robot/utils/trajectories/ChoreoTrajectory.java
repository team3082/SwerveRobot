package frc.robot.utils.trajectories;

import java.io.File;
import java.util.List;

import com.fasterxml.jackson.core.type.TypeReference;
import com.fasterxml.jackson.databind.ObjectMapper;

public class ChoreoTrajectory extends DiscreteTraj{
    
    private static DiscreteSwerveState toSwerveState(ChoreoState cs){
        return new DiscreteSwerveState(cs.x,cs.y,cs.heading,cs.velocityX,cs.velocityY,cs.angularVelocity, cs.timestamp);
    }

    public ChoreoTrajectory(File f){
        ObjectMapper om = new ObjectMapper();
        List<ChoreoState> choreoStates = null;
        try{
            choreoStates = om.convertValue(om.readTree(f).get("samples"), new TypeReference<List<ChoreoState>>(){});
        }catch(Exception e){
            e.printStackTrace();
        }
        
        path = choreoStates.stream().map((s) -> toSwerveState(s)).toList();
    }

    static class ChoreoState {
        double x,y,heading,angularVelocity,velocityX,velocityY, timestamp;

        public ChoreoState(){
            x=0.0;
            y=0;
            heading=0;
            angularVelocity=0;
            velocityX=0;
            velocityY=0;
        }

        public ChoreoState(double x, double y, double heading, double angularVelocity, double velocityX,
                double velocityY, double timestamp) {
            this.x = x;
            this.y = y;
            this.heading = heading;
            this.angularVelocity = angularVelocity;
            this.velocityX = velocityX;
            this.velocityY = velocityY;
            this.timestamp = timestamp;
        }

        public double getX() {
            return x;
        }

        public double getY() {
            return y;
        }

        public double getHeading() {
            return heading;
        }

        public double getAngularVelocity() {
            return angularVelocity;
        }

        public double getVelocityX() {
            return velocityX;
        }

        public double getVelocityY() {
            return velocityY;
        }

        public double getTimestamp() {
            return timestamp;
        }
        
    }
}
