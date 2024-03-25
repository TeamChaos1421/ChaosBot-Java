package frc.robot.subsystems;

import frc.robot.SwerveModuleConstants;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.SparkRelativeEncoder;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwerveModule extends SubsystemBase{
    private CANSparkMax m_turningMotor;
    private CANSparkMax m_drivingMotor;
    private Rotation2d angleOffset;
    private String moduleLabel;
    private SparkAbsoluteEncoder m_turningEncoder;
    private SparkRelativeEncoder m_drivingEncoder;
    private SparkPIDController m_turningPidController;
    private SparkPIDController m_drivingPidController;
    private SwerveModuleState m_desiredState;
    public int moduleNumber;
    
    public SwerveModule(SwerveModuleConstants moduleConstants) {
        this.m_turningMotor = new CANSparkMax(moduleConstants.turnMotorID, MotorType.kBrushless);
        this.m_drivingMotor = new CANSparkMax(moduleConstants.driveMotorID, MotorType.kBrushless);
        this.angleOffset = new Rotation2d(moduleConstants.angleOffset);
        this.moduleLabel = new String(moduleConstants.label);
        this.moduleNumber = moduleConstants.moduleNumber;

        this.m_turningEncoder = m_turningMotor.getAbsoluteEncoder(Type.kDutyCycle);
        this.m_drivingEncoder = (SparkRelativeEncoder) m_drivingMotor.getEncoder();
        this.m_turningPidController = m_turningMotor.getPIDController();
        this.m_drivingPidController = m_drivingMotor.getPIDController();

        // Factory reset, so we get the SPARKS MAX to a known state before configuring
        // them. This is useful in case a SPARK MAX is swapped out.
        this.m_drivingMotor.restoreFactoryDefaults();
        this.m_turningMotor.restoreFactoryDefaults();

        // Apply position and velocity conversion factors for the driving encoder. The
        // native units for position and velocity are rotations and RPM, respectively,
        // but we want meters and meters per second to use with WPILib's swerve APIs.
        this.m_drivingEncoder.setPositionConversionFactor(0.049);
        this.m_drivingEncoder.setVelocityConversionFactor(0.049 / 60);

        // Apply position and velocity conversion factors for the turning encoder. We
        // want these in radians and radians per second to use with WPILib's swerve
        // APIs.
        this.m_turningEncoder.setPositionConversionFactor(2 * Math.PI);
        this.m_turningEncoder.setVelocityConversionFactor((2 * Math.PI) / 60);

        // Invert the turning encoder, since the output shaft rotates in the opposite
        // direction of the steering motor in the MAXSwerve Module.
        this.m_turningEncoder.setInverted(false);

        // Enable PID wrap around for the turning motor. This will allow the PID
        // controller to go through 0 to get to the setpoint i.e. going from 350
        // degrees to 10 degrees will go through 0 rather than the other direction
        // which is a longer route.
        this.m_turningPidController.setPositionPIDWrappingEnabled(true);
        this.m_turningPidController.setPositionPIDWrappingMinInput(0);
        this.m_turningPidController.setPositionPIDWrappingMaxInput(2 * Math.PI);
                
        // Set the PID Controller to use the duty cycle encoder on the swerve
        // module instead of the built in NEO550 encoder.
        this.m_turningPidController.setFeedbackDevice(m_turningEncoder);

        // Set the PID gains for the driving motor. Note these are example gains, and
        // you may need to tune them for your own robot!
        this.m_drivingPidController.setP(0.04);
        this.m_drivingPidController.setI(0.0);
        this.m_drivingPidController.setD(0.0);
        this.m_drivingPidController.setFF(1 / 4.65);
        this.m_drivingPidController.setOutputRange(-1.0, 1.0);

        // Set the PID gains for the turning motor. Note these are example gains, and
        // you may need to tune them for your own robot!
        this.m_turningPidController.setP(.35);
        this.m_turningPidController.setI(0.0);
        this.m_turningPidController.setD(0.0);
        this.m_turningPidController.setFF(0.0);
        this.m_turningPidController.setOutputRange(-1.0, 1.0);

        this.m_drivingMotor.setIdleMode(IdleMode.kBrake);
        this.m_turningMotor.setIdleMode(IdleMode.kBrake);
        this.m_drivingMotor.setSmartCurrentLimit(40);
        this.m_turningMotor.setSmartCurrentLimit(20);

        // Save the SPARK MAX configurations. If a SPARK MAX browns out during
        // operation, it will maintain the above configurations.
        this.m_drivingMotor.burnFlash();
        this.m_turningMotor.burnFlash();

        this.m_desiredState = new SwerveModuleState();
        this.m_desiredState.angle = new Rotation2d(m_turningEncoder.getPosition());
        this.m_drivingEncoder.setPosition(0);
    }

    public SwerveModuleState GetState() {
        return new SwerveModuleState(
            m_drivingEncoder.getVelocity(),
            new Rotation2d(m_turningEncoder.getPosition())
        );
    }

    public SwerveModulePosition GetPosition() {
        return new SwerveModulePosition(
            m_drivingEncoder.getPosition(),
            new Rotation2d(m_turningEncoder.getPosition())
        );
    }

    public void SetState(SwerveModuleState desiredState) {
        SwerveModuleState correctedState  = new SwerveModuleState(
            desiredState.speedMetersPerSecond,
            desiredState.angle.plus(angleOffset)
        );
        desiredState = SwerveModuleState.optimize(correctedState, this.GetState().angle);

        SmartDashboard.putNumber(moduleLabel, desiredState.angle.getDegrees());
        // SmartDashboard.putNumber(moduleLabel, (this.m_turningEncoder.getPosition() * (180 / Math.PI)));

        m_turningPidController.setReference(desiredState.angle.getRadians(), ControlType.kPosition);
        m_drivingPidController.setReference(desiredState.speedMetersPerSecond, ControlType.kVelocity);
    }
}

