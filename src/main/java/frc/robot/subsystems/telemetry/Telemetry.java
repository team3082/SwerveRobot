package frc.robot.subsystems.telemetry;

import java.lang.reflect.Field;
import java.util.ArrayList;

import frc.robot.subsystems.OI;

public class Telemetry {
    private static final Class[] classes = {OI.class};

    private static final ArrayList<Field> trackedFields = new ArrayList<Field>();

    //Initiallize this class after all the classes that it is logging
    public static void init(){
        for(Class c : classes){
            Field[] declaredFields = c.getDeclaredFields();
            for(Field f: declaredFields){
                if(f.isAnnotationPresent(Log.class)){
                    f.setAccessible(true);
                    trackedFields.add(f);
                }
            }
        }
    }

    public static void printValues(){
        for(Field f: trackedFields){
            try{
                System.out.println(f.getName() + " : " + f.get(f.getDeclaringClass()).toString());
            }catch(IllegalAccessException e){
                e.printStackTrace();
            }
        }
    }

    public static int numFields(){
        return trackedFields.size();
    }
}
