package frc.robot;

  public class SwerveModuleConstants {
    public final int driveMotorID;
    public final int turnMotorID;
    public final double angleOffset;
    public final String label;

    public SwerveModuleConstants(int driveMotorID, int turnMotorID, double angleOffset, String newLabel) {
        this.driveMotorID = driveMotorID;
        this.turnMotorID = turnMotorID;
        this.angleOffset = angleOffset;
        this.label = newLabel;
    }
  }
