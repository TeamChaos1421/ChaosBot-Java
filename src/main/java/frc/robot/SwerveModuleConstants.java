package frc.robot;

  public class SwerveModuleConstants {
    public final int driveMotorID;
    public final int turnMotorID;
    public final double angleOffset;
    public final String label;
    public final int moduleNumber;

    public SwerveModuleConstants(int driveMotorID, int turnMotorID, double angleOffset, String newLabel, int newModuleNumber) {
        this.driveMotorID = driveMotorID;
        this.turnMotorID = turnMotorID;
        this.angleOffset = angleOffset;
        this.label = newLabel;
        this.moduleNumber = newModuleNumber;
    }
  }
