package frc.robot.subsystems;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
        DrivetrainConstants.kTrackWidth / 2, -DrivetrainConstants.kTrackLength / 2);
    public final Translation2d FrontRightLocation = new Translation2d(
        DrivetrainConstants.kTrackWidth / 2, DrivetrainConstants.kTrackLength / 2);
    public final Translation2d BackLeftLocation = new Translation2d(
        -DrivetrainConstants.kTrackWidth / 2, -DrivetrainConstants.kTrackLength / 2);
    public final Translation2d BackRightLocation = new Translation2d(
        -DrivetrainConstants.kTrackWidth / 2, DrivetrainConstants.kTrackLength / 2);
    
    public final SwerveDriveKinematics m_Kinematics = new SwerveDriveKinematics(
        FrontLeftLocation,
        FrontRightLocation,
        BackLeftLocation,
        BackRightLocation
    );
    private final StructArrayPublisher<SwerveModuleState> desiredStatePublisher = NetworkTableInstance.getDefault()
        .getStructArrayTopic("/RealOutputs/desiredStates", SwerveModuleState.struct).publish();
    private final StructArrayPublisher<SwerveModuleState> actualStatePublisher = NetworkTableInstance.getDefault()
        .getStructArrayTopic("/RealOutputs/actualStates", SwerveModuleState.struct).publish();

    public final SwerveDriveOdometry m_Odometry = new SwerveDriveOdometry(
        m_Kinematics, 
        m_gyro.getRotation2d(),
        new SwerveModulePosition[] {
            frontLeftModule.GetPosition(), frontRightModule.GetPosition(),
            backLeftModule.GetPosition(), backRightModule.GetPosition()
        }, 
        new Pose2d()
    );

    public Drivetrain () {
        desiredStatePublisher.set(
            new SwerveModuleState[] {
                new SwerveModuleState(), new SwerveModuleState(),
                new SwerveModuleState(), new SwerveModuleState(),
            }
        );
        AutoBuilder.configureHolonomic(
            this::getPose, // Robot pose supplier
            this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
            this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            this::driveRobotRelative, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
            new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
                    new PIDConstants(.8, 0.0, 0.0), // Translation PID constants
                    new PIDConstants(0.35, 0.0, 0.0), // Rotation PID constants
                    4.65, // Max module speed, in m/s
                    0.4, // Drive base radius in meters. Distance from robot center to furthest module.
                    new ReplanningConfig() // Default path replanning config. See the API for the options here
            ),
            () -> {
              // Boolean supplier that controls when the path will be mirrored for the red alliance
              // This will flip the path being followed to the red side of the field.
              // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

              var alliance = DriverStation.getAlliance();
              if (alliance.isPresent()) {
                return alliance.get() == DriverStation.Alliance.Blue;
              }
              return false;
            },
            this // Reference to this subsystem to set requirements
        );
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("NavX_gyro", m_gyro.getRotation2d().getRadians());

        SwerveModulePosition[] currentSwervePositions = new SwerveModulePosition[] {
            frontLeftModule.GetPosition(), frontRightModule.GetPosition(),
            backLeftModule.GetPosition(), backRightModule.GetPosition()
        };
        SwerveModuleState[] currentSwerveStates = new SwerveModuleState[] {
            frontLeftModule.GetState(), frontRightModule.GetState(),
            backLeftModule.GetState(), backRightModule.GetState()
        };

        m_Odometry.update(
            m_gyro.getRotation2d(),
            currentSwervePositions
        );
        actualStatePublisher.set(currentSwerveStates);
    };

    public Pose2d getPose() {
        return m_Odometry.getPoseMeters();
    }

    public void resetPose(Pose2d newPose) {
        SwerveModulePosition[] swervePositions = {
            frontLeftModule.GetPosition(), frontRightModule.GetPosition(),
            backLeftModule.GetPosition(), backRightModule.GetPosition()
        };
        m_Odometry.resetPosition(m_gyro.getRotation2d(), swervePositions, newPose);
    }

    public ChassisSpeeds getRobotRelativeSpeeds() {
        return m_Kinematics.toChassisSpeeds(
            frontLeftModule.GetState(), frontRightModule.GetState(),
            backLeftModule.GetState(), backRightModule.GetState()
        );
    }

    public void driveRobotRelative(ChassisSpeeds speeds) {
        SwerveModuleState[] moduleStates = m_Kinematics.toSwerveModuleStates(speeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, DrivetrainConstants.kMaxSpeed);
        
        setModuleStates(moduleStates);

        desiredStatePublisher.set(moduleStates);
    }

    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds((desiredStates), DrivetrainConstants.kMaxSpeed);
        SwerveModule[] mSwerveModules = {frontLeftModule, frontRightModule, backLeftModule, backRightModule};

        for(SwerveModule module : mSwerveModules) {
            module.SetState(desiredStates[module.moduleNumber]);
        }
    }




    public void drive(double xSpeed, double ySpeed, double rSpeed, Boolean fieldRelative) {
        ChassisSpeeds speeds;
        if (fieldRelative == true) {
            speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                (DrivetrainConstants.kMaxSpeed.times(ySpeed)),
                (DrivetrainConstants.kMaxSpeed.times(xSpeed)),
                (DrivetrainConstants.kMaxRot.times(rSpeed)),
                new Rotation2d(m_gyro.getYaw())
            );
        } else {
            speeds = new ChassisSpeeds(
                (DrivetrainConstants.kMaxSpeed.times(ySpeed)),
                (DrivetrainConstants.kMaxSpeed.times(xSpeed)),
                (DrivetrainConstants.kMaxRot.times(rSpeed))
            );
        }

        SwerveModuleState[] moduleStates = m_Kinematics.toSwerveModuleStates(speeds);

        frontLeftModule.SetState(moduleStates[0]);
        frontRightModule.SetState(moduleStates[1]);
        backLeftModule.SetState(moduleStates[2]);
        backRightModule.SetState(moduleStates[3]);

        desiredStatePublisher.set(moduleStates);
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
