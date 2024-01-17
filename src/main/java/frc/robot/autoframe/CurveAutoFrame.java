package frc.robot.autoframe;
import frc.robot.autoframe.Autoframe;

public class CurveAutoFrame {
    Autoframe autoFrame;
    double tFrameStart;

    public CurveAutoFrame(Autoframe autoFrame, double tFrameStart) {

    }

    public CurveAutoFrame() {

    }

    public void start() {
        this.autoFrame.start();
    }

    public void update() {
        this.autoFrame.update();
    }
}
