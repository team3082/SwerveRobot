package frc.robot.utils;

public final class Constants {

    // QOL
    public static final double TAU = 2 * Math.PI;
    public static final double RA = Math.PI / 4; // Right angle

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

    // CAN IDs
    public static final int FLDRIVEID = 0;
    public static final int FRDRIVEID = 0;
    public static final int BLDRIVEID = 0;
    public static final int BRDRIVEID = 0;
    public static final int FLSTEERID = 0;
    public static final int FRSTEERID = 0;
    public static final int BLSTEERID = 0;
    public static final int BRSTEERID = 0;

}
