package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Joystick;
import frc.controllermaps.XBoxOne;
import frc.robot.subsystems.swerve.SwerveManager;
import frc.robot.utils.Vector2;
import static frc.robot.utils.Constants.*;

public class OI {
    private static Joystick driverStick;
    private static final int moveX = XBoxOne.AXIS_LEFT_X;
    private static final int moveY = XBoxOne.AXIS_LEFT_Y;
    private static final int rotateX = XBoxOne.AXIS_RIGHT_X;
    private static final int boost = XBoxOne.AXIS_RIGHT_TRIGGER;
    private static final int pigeonZero = XBoxOne.BUTTON_Y;
    private static final int driveToNode = XBoxOne.BUTTON_A;
    private static final int parkingBrake = XBoxOne.BUTTON_B;
    private static final int balance = XBoxOne.BUTTON_X;
    private static final int crawl = XBoxOne.AXIS_LEFT_TRIGGER;

    //state fields
    private static boolean braking;

    public static void init(){
        driverStick = new Joystick(0);
        braking = false;
    }

    private static double smoothInput(double input){
        return Math.pow(input, 2);
    }

    private static void driveInput(){
        //swerve commands before boost/crawl
        Vector2 movementCommand = new Vector2(smoothInput(driverStick.getRawAxis(moveX)), smoothInput(driverStick.getRawAxis(moveY)));
        double rotationCommand = smoothInput(driverStick.getRawAxis(rotateX));
        
        //Checking deadbands
        if(movementCommand.mag() < DRIVETRANSDEAD) movementCommand = new Vector2();
        if(Math.abs(rotationCommand) < DRIVEROTDEAD) rotationCommand = 0;

        double movementScale = kNORMTRANS;
        double rotationScale = kNORMROT;

        //Boost
        double boostPercent = driverStick.getRawAxis(boost);
        if(boostPercent > BOOSTDEAD){
            movementScale = kNORMTRANS + (kBOOSTTRANS - kNORMTRANS) * boostPercent;
            rotationScale = kNORMTRANS + (kBOOSTROT - kNORMROT) * boostPercent;
        } 

        //Crawl, overrides boost
        double crawlPercent = driverStick.getRawAxis(crawl);
        if(crawlPercent > CRAWLDEAD){
            movementScale = kNORMTRANS + (kCRAWLTRANS - kNORMTRANS) * crawlPercent;
            rotationScale = kNORMTRANS + (kCRAWLROT - kNORMROT) * crawlPercent;
        } 

        SwerveManager.rotateAndDrive(movementCommand.mul(movementScale),rotationCommand * rotationScale);
    }

    private static void brake(){
        SwerveManager.brake();
    }

    private static void zeroPigeon(){
        Pigeon.zero();
    }

    public static void update(){
        if(driverStick.getRawButtonPressed(parkingBrake)) braking = !braking;
        if(driverStick.getRawButtonPressed(pigeonZero)) zeroPigeon();
        if(braking) brake();
        else driveInput();
    }
}
