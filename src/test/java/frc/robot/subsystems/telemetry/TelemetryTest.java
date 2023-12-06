package frc.robot.subsystems.telemetry;

import org.junit.jupiter.api.Test;
import static org.junit.jupiter.api.Assertions.*;
import frc.robot.subsystems.OI;

public class TelemetryTest {
    @Test
    public void test(){
        OI.init();
        try{
            Telemetry.init();
        }catch(Exception e){
            e.printStackTrace();
        }
        Telemetry.printAll();
    }
}
