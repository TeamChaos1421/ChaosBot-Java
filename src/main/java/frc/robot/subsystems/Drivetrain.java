package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.proto.Kinematics;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.BackLeftConstants;
import frc.robot.Constants.BackRightConstants;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Constants.FrontLeftConstants;
import frc.robot.Constants.FrontRightConstants;
import edu.wpi.first.wpilibj2.command.Command;

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
        m_gyro.getRotation2d(),
        new SwerveModulePosition[] {
            frontLeftModule.GetPosition(), frontRightModule.GetPosition(),
            backLeftModule.GetPosition(), backRightModule.GetPosition()
        }, 
        null
    );

    private void drive(double xSpeed, double ySpeed, double rSpeed, Boolean fieldRelative) {
        ChassisSpeeds speeds;
        if (fieldRelative == true) {
            speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                (DrivetrainConstants.kMaxSpeed.times(xSpeed)),
                (DrivetrainConstants.kMaxSpeed.times(ySpeed)),
                (DrivetrainConstants.kMaxRot.times(rSpeed)),
                m_gyro.getRotation2d()
            );
        } else {
            speeds = new ChassisSpeeds(
                (DrivetrainConstants.kMaxSpeed.times(xSpeed)),
                (DrivetrainConstants.kMaxSpeed.times(ySpeed)),
                (DrivetrainConstants.kMaxRot.times(rSpeed))
            );
        }

        SwerveModuleState[] moduleStates = m_Kinematics.toSwerveModuleStates(speeds);

        frontLeftModule.SetState(moduleStates[0]);
        frontRightModule.SetState(moduleStates[1]);
        backLeftModule.SetState(moduleStates[2]);
        backRightModule.SetState(moduleStates[3]);
    }

    private void zeroHeading() {
        m_gyro.zeroYaw();
    }

    // COMMANDS

    public Command Drive(double xSpeed, double ySpeed, double rSpeed, boolean fieldRelative) {
        return this.run(
            () -> {
                this.drive(xSpeed, ySpeed, rSpeed, fieldRelative);
            }
        );
    }

    public Command ZeroHeading() {
        return this.runOnce(
            () -> {
                this.zeroHeading();
            }
        );
    }
}
