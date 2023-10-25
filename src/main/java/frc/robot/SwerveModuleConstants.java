package frc.robot;

public class SwerveModuleConstants {
    private double DRIVE_KFF;
    private double DRIVE_KS;
    private double DRIVE_KV;
    private double DRIVE_KA;

    private double DRIVE_KP;
    private double DRIVE_KI;
    private double DRIVE_KD;

    private double AZIMUTH_KP;
    private double AZIMUTH_KI;
    private double AZIMUTH_KD;

    private final boolean DRIVE_INVERTED;
    private final boolean AZIMUTH_INVERTED;

    private final int DRIVE_ID;
    private final int AZIMUTH_ID;
    private final int CANCODER_ID;

    private final int VELO_MEASUREMENT_PERIOD;
    private final int VELO_MEASUREMENT_DEPTH;

    public SwerveModuleConstants(int DRIVE_ID, int AZIMUTH_ID, int CANCODER_ID, boolean DRIVE_INVERTED, boolean AZIMUTH_INVERTED, int VELO_MEASUREMENT_PERIOD, int VELO_MEASUREMENT_DEPTH, double DRIVE_KFF, double DRIVE_KS, double DRIVE_KV, double DRIVE_KA, double DRIVE_KP, double DRIVE_KI, double DRIVE_KD, double AZIMUTH_KP, double AZIMUTH_KI, double AZIMUTH_KD) {
        this.DRIVE_ID = DRIVE_ID;
        this.AZIMUTH_ID = AZIMUTH_ID;
        this.CANCODER_ID = CANCODER_ID;

        this.DRIVE_INVERTED = DRIVE_INVERTED;
        this.AZIMUTH_INVERTED = AZIMUTH_INVERTED;

        this.VELO_MEASUREMENT_PERIOD = VELO_MEASUREMENT_PERIOD;
        this.VELO_MEASUREMENT_DEPTH = VELO_MEASUREMENT_DEPTH;

        this.DRIVE_KFF = DRIVE_KFF;
        this.DRIVE_KS = DRIVE_KS;
        this.DRIVE_KV = DRIVE_KV;
        this.DRIVE_KA = DRIVE_KA;
        this.DRIVE_KP = DRIVE_KP;
        this.DRIVE_KI = DRIVE_KI;
        this.DRIVE_KD = DRIVE_KD;
        this.AZIMUTH_KP = AZIMUTH_KP;
        this.AZIMUTH_KI = AZIMUTH_KI;
        this.AZIMUTH_KD = AZIMUTH_KD;
    }

    public double getDriveKFF() {
        return DRIVE_KFF;
    }

    public double getDriveKS() {
        return DRIVE_KS;
    }

    public double getDriveKV() {
        return DRIVE_KV;
    }

    public double getDriveKA() {
        return DRIVE_KA;
    }

    public double getDriveKP() {
        return DRIVE_KP;
    }

    public double getDriveKI() {
        return DRIVE_KI;
    }

    public double getDriveKD() {
        return DRIVE_KD;
    }

    public double getAzimuthKP() {
        return AZIMUTH_KP;
    }

    public double getAzimuthKI() {
        return AZIMUTH_KI;
    }

    public double getAzimuthKD() {
        return AZIMUTH_KD;
    }

    public boolean getDriveInverted() {
        return DRIVE_INVERTED;
    }

    public boolean getAzimuthInverted() {
        return AZIMUTH_INVERTED;
    }

    public int getDriveID() {
        return DRIVE_ID;
    }

    public int getAzimuthID() {
        return AZIMUTH_ID;
    }

    public int getCanCoderID() {
        return CANCODER_ID;
    }

    public int getVeloMeasurementPeriod() {
        return VELO_MEASUREMENT_PERIOD;
    }

    public int getVeloMeasurementDepth() {
        return VELO_MEASUREMENT_DEPTH;
    }

    public void setDriveKFF(double DRIVE_KFF) {
        this.DRIVE_KFF = DRIVE_KFF;
    }

    public void setDriveKP(double DRIVE_KP) {
        this.DRIVE_KP = DRIVE_KP;
    }

    public void setDriveKI(double DRIVE_KI) {
        this.DRIVE_KI = DRIVE_KI;
    }

    public void setDriveKD(double DRIVE_KD) {
        this.DRIVE_KD = DRIVE_KD;
    }

    public void setAzimuthKP(double AZIMUTH_KP) {
        this.AZIMUTH_KP = AZIMUTH_KP;
    }

    public void setAzimuthKI(double AZIMUTH_KI) {
        this.AZIMUTH_KI = AZIMUTH_KI;
    }

    public void setAzimuthKD(double AZIMUTH_KD) {
        this.AZIMUTH_KD = AZIMUTH_KD;
    }
}
