package frc.robot.utils;

public final class Constants {
    //QOL
    public static final double TAU = 2 * Math.PI;
    public static final double RA = Math.PI / 2; // Right angle

    //OI
    public static final double BOOSTTRANSFACTOR = 1.0;
    public static final double BOODYROTFACTOR = 1.0;
    public static final double NORMTRANSFACTOR = 0.6;
    public static final double NORMROTFACTOR = 0.6;
    public static final double CRAWLTRANSFACTOR = 0.3;
    public static final double CRAWLROTFACTOR = 0.3;
    public static final double BOOSTDEAD = 0.05;
    public static final double CRAWLDEAD = 0.05;
    public static final double DRIVETRANSDEAD = 0.05;
    public static final double DRIVEROTDEAD = 0.05;

    // SWERVE DRIVE
    public static final double DRIVERATIO = 6.12;
    public static final double STEERRATIO = 150 / 7;
    public static final double WHEELDIAMETER = 4.0; // Inches
    public static final double FRAMEWIDTH = 25.0;
    public static final double FRAMELENGTH = 25.0;
    public static final double WHEELBASEWIDTH = FRAMEWIDTH - 2 * 2.65;
    public static final double WHEELBASELENGTH = FRAMELENGTH - 2 * 2.65;

    // CANCODER OFFSETS
    public static final double FLOFFSET = 0.0;
    public static final double FROFFSET = 0.0;
    public static final double BLOFFSET = 0.0;
    public static final double BROFFSET = 0.0;
    public static final double FLCANCODEROFFSET = 0.0;//Mods should be at 0 when pointed to the right
    public static final double FRCANCODEROFFSET = 0.0;
    public static final double BLCANCODEROFFSET = 0.0;
    public static final double BRCANCODEROFFSET = 0.0;

    // CAN IDs
    public static final int FLDRIVEID = 0;
    public static final int FRDRIVEID = 0;
    public static final int BLDRIVEID = 0;
    public static final int BRDRIVEID = 0;
    public static final int FLSTEERID = 0;
    public static final int FRSTEERID = 0;
    public static final int BLSTEERID = 0;
    public static final int BRSTEERID = 0;

    //PID
    public static final double PDRIVE = 0.0;
    public static final double IDRIVE = 0.0;
    public static final double DDRIVE = 0.0;
    public static final double PSTEER = 0.0;
    public static final double ISTEER = 0.0;
    public static final double DSTEER = 0.0;


}
