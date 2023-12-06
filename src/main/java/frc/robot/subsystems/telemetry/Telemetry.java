package frc.robot.subsystems.telemetry;

import java.lang.reflect.Field;
import java.util.ArrayList;
import frc.robot.subsystems.OI;

public class Telemetry {
    @SuppressWarnings("rawtypes")
    private static final Class[] classes = {OI.class};//TODO find a better way to find classes

    //A two dimensiona array of tracked fields, with each row(row major) consisting of fields from the same tracked class;
    private static ArrayList<Field>[] trackedFields;
    
    
    //Initiallize this class after all the classes that it is logging
    @SuppressWarnings({"rawtypes", "unchecked"})
    public static void init(){
        //This type cast is unchecked, but it should be chill
        trackedFields = (ArrayList<Field>[]) new ArrayList[classes.length];
        for(int i = 0; i < classes.length; i++){
            trackedFields[i] = new ArrayList<Field>();
        }

        //grabbing all fields with Log annotation to array
        for(int i = 0; i < classes.length; i++){
            Class c = classes[i];
            Field[] declaredFields = c.getDeclaredFields();
            for(Field f : declaredFields){
                if(f.isAnnotationPresent(Log.class)) {
                    f.setAccessible(true);
                    trackedFields[i].add(f);
                }
            }

        }
    }

    public static void printAll(){
        for(int i = 0; i < classes.length; i++){
            //Printing class name
            System.out.println("\u001B[33m" + classes[i].getSimpleName() + ":\u001B[0m");
            for(Field f : trackedFields[i]){
                try{
                    String name = f.getName();
                    Object data = f.get(f.getDeclaringClass());
                    String dataStr = data == null ? "null" : data.toString();
                    System.out.println(name + " : " + dataStr);
                }catch(IllegalAccessException e){
                    e.printStackTrace();
                }
            }
        }
    }

}
