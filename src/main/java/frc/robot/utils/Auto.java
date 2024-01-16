package frc.robot.utils;

import java.util.Arrays;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.Queue;

import frc.robot.autoframe.Autoframe;
import frc.robot.subsystems.swerve.SwerveManager;

public class Auto {
    // AUTO SYSTEM =================================================
    public static Queue<Autoframe> queuedFrames;
    public static HashSet<Autoframe> activeFrames = new HashSet<>();

    public static double rotSpeed;
    public static Vector2 movement;

    public static double startTimestamp = 0;

    public static double moveScale = 1;
    public static double rotScale = 1;

    private static void queueFrames(Autoframe[] frames) {
        activeFrames.clear();
        queuedFrames = new LinkedList<>(Arrays.asList(frames));
        startTimestamp = RTime.now();
    }

    public static void update() {
        rotSpeed = 0;
        movement = new Vector2();

        boolean advanceFrame = true;
        HashSet<Autoframe> finishedFrames = new HashSet<>();

        for (Autoframe frame : activeFrames) {

            // For debugging, print the classes of all of the active frames
            System.out.print(frame.getClass().getSimpleName() + " ");

            // Update and/or finish the frame according to whether or not its "done" boolean has been set to true
            frame.update();
            if (frame.done){
                finishedFrames.add(frame);
                frame.finish();
            }

            // If there's any blocking frame active that didn't just end, don't advance to the next frame
            else if (frame.blocking)
                advanceFrame = false;
        }

        // Remove all of the finished frames from the set of active frames
        activeFrames.removeAll(finishedFrames);

        System.out.println(); // Newline for debugging readability

        // Conditions for advancing the frame:
        // AdvanceFrame must be true, meaning there are no active, blocking frames
        // QueuedFrames must have items remaining
        if (advanceFrame && !queuedFrames.isEmpty()) {
            Autoframe newFrame = queuedFrames.remove();
            newFrame.start();
            activeFrames.add(newFrame);
        }

        // Print "Done" to signify Auto has finished
        if (queuedFrames.isEmpty() && activeFrames.isEmpty()) {
            System.out.println("Done");
        }

        // Rotate and drive the robot according to the output of the active AutoFrames
        SwerveManager.rotateAndDrive(rotSpeed * rotScale, movement.mul(moveScale));
    }

}
