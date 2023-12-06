package frc.robot.subsystems.telemetry;

import org.junit.jupiter.api.Test;
import static org.junit.jupiter.api.Assertions.*;
import frc.robot.subsystems.OI;

public class TelemetryTest {
    @Test
    public void test(){
        OI.init();
        Telemetry.init();

        assertEquals(Telemetry.numFields(), 1);
        Telemetry.printValues();
    }
}
