package frc.robot.utils;

public final class Constants {
    //QOL
    public static final double TAU = 2 * Math.PI;
    public static final double RA = Math.PI / 4;//RIght angle

    //DRIVE TRAIN
    public static final double DRIVERATIO = 6.12;
    public static final double STEERRATIO = 150/7;
    public static final double WHEELDIAMETER = 4.0;//inches
    public static final double FRAMEWIDTH = 25.0;
    public static final double FRAMELENGTH = 25.0;
    public static final double WHEELBASEWIDTH = FRAMEWIDTH - 2 * 2.65;
    public static final double WHEELBASELENGTH = FRAMELENGTH - 2 * 2.65;

    public static final double FLCANCODEROFFSET = 0.0;
    public static final double FRCANCODEROFFSET = 0.0;
    public static final double BLCANCODEROFFSET = 0.0;
    public static final double BRCANCODEROFFSET = 0.0;

    //CAN IDs
    public static final int FLDRIVEID = 0;
    public static final int FRDRIVEID = 0;
    public static final int BLDRIVEID = 0;
    public static final int BRDRIVEID = 0;
    public static final int FLSTEERID = 0;
    public static final int FRSTEERID = 0;
    public static final int BLSTEERID = 0;
    public static final int BRSTEERID = 0;

}
