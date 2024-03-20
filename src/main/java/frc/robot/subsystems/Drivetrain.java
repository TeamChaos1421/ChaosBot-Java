package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.BackLeftConstants;
import frc.robot.Constants.BackRightConstants;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Constants.FrontLeftConstants;
import frc.robot.Constants.FrontRightConstants;

public class Drivetrain extends SubsystemBase {
    public final SwerveModule frontLeftModule = new SwerveModule(FrontLeftConstants.constants);
    public final SwerveModule frontRightModule = new SwerveModule(FrontRightConstants.constants);
    public final SwerveModule backLeftModule = new SwerveModule(BackLeftConstants.constants);
    public final SwerveModule backRightModule = new SwerveModule(BackRightConstants.constants);
    public final AHRS m_gyro = new AHRS(Port.kMXP);

    public final Translation2d FrontLeftLocation = new Translation2d(
        DrivetrainConstants.kTrackWidth / 2, DrivetrainConstants.kTrackLength / 2);
    public final Translation2d FrontRightLocation = new Translation2d(
        DrivetrainConstants.kTrackWidth / 2, -DrivetrainConstants.kTrackLength / 2);
    public final Translation2d BackLeftLocation = new Translation2d(
        -DrivetrainConstants.kTrackWidth / 2, DrivetrainConstants.kTrackLength / 2);
    public final Translation2d BackRightLocation = new Translation2d(
        -DrivetrainConstants.kTrackWidth / 2, -DrivetrainConstants.kTrackLength / 2);
    
    public final SwerveDriveKinematics m_Kinematics = new SwerveDriveKinematics(
        FrontLeftLocation,
        FrontRightLocation,
        BackLeftLocation,
        BackRightLocation
    );
    public final SwerveDriveOdometry m_Odometry = new SwerveDriveOdometry(
        m_Kinematics, 
        new SwerveModulePosition[] {
            frontLeftModule.get
        }, 
        null
    );
}
