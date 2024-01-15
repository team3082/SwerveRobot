package frc.robot.utils;

import java.util.Arrays;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.Queue;

import frc.robot.autoframe.Autoframe;

public class Auto {
    
    public Queue<Autoframe> queuedFrames;
    public HashSet<Autoframe> activeFrames;

    public void update(){
        boolean blocked = false;
        HashSet<Autoframe> finishedFrames = new HashSet<>();

        for(Autoframe frame : activeFrames){
            frame.update();
            if(frame.done){
                frame.finish();
                finishedFrames.add(frame);
            }else if(frame.blocking){
                blocked = true;
            }
        }

        activeFrames.removeAll(finishedFrames);

        if(!blocked){
            boolean flag = false;
            while(!flag && !queuedFrames.isEmpty()){
                Autoframe newFrame = queuedFrames.poll();
                activeFrames.add(newFrame);
                newFrame.start();
                flag = newFrame.blocking;
            }
        }
    }

    public void queueFrames(Autoframe[] frames){
        queuedFrames = new LinkedList<>(Arrays.asList(frames));
    }
}
