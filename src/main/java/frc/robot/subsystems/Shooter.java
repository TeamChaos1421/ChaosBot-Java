package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MotorConstants;
import frc.robot.Constants.PneumaticConstants;

public class Shooter extends SubsystemBase {
    // VARIABLES
    private final TalonFX m_shooterHigh = 
        new TalonFX(MotorConstants.kShooterHighCANId);
    private final TalonFX m_shooterMid = 
        new TalonFX(MotorConstants.kShooterMidCANId);
    private final TalonFX m_shooterLow = 
        new TalonFX(MotorConstants.kShooterLowCANId);
    private final Compressor m_compressor = new Compressor(
        PneumaticConstants.kPCMCANId, 
        PneumaticsModuleType.CTREPCM
    );
    private final DoubleSolenoid m_Solenoid = new DoubleSolenoid(
        PneumaticConstants.kPCMCANId,
        PneumaticsModuleType.CTREPCM,
        PneumaticConstants.kDumpForward, 
        PneumaticConstants.kDumpReverse
    );
    private final Timer m_dumpTimer = new Timer();

    
    private void init() {
        m_compressor.enableDigital();
        m_Solenoid.set(Value.kReverse);
    }

    private void set(double speedHigh, double speedMid, double speedLow) {
        m_shooterHigh.set(speedHigh);
        m_shooterMid.set(speedMid);
        m_shooterLow.set(speedLow);
    }


    // COMMANDS
    public Command Init() {
        return this.runOnce(
            () -> {
                this.init();
            }
        );
    }

    public Command Shoot() {
        return this.runOnce(
            () -> {
                this.set(-1, -1, -0.2);
            }
        );
    }

    public Command ResetTimer() {
        return runOnce(
            () -> {
                this.m_dumpTimer.reset();
                this.m_dumpTimer.start();
            }
        );
    }

    public Command Reset() {
        return this.runOnce(
            () -> {
                this.m_Solenoid.set(Value.kReverse);
                this.set(0, 0, 0);
            }
        );
    }

    public Command Load() {
        return new FunctionalCommand(
            () -> {},
            () -> {
                this.set(-0.035, -0.2, -0.4);
                if (this.m_dumpTimer.get() > 0.25) {
                    this.m_Solenoid.set(Value.kForward);
                }
            }, 
            interrupted -> {
                this.m_Solenoid.set(Value.kForward);
            }, 
            () -> this.m_dumpTimer.get() > 0.5
        );
    }

    public Command Toggle() {
        return this.runOnce(
            () -> {
                this.m_Solenoid.toggle();
            }
        );
    }

    public Command Reverse() {
        return this.run(
            () -> {
                this.set(0.1, 0.1, 0.1);
            }
        );
    }

    public Command Zero() {
        return this.runOnce(
            () -> {
                this.set(0, 0, 0);
            }
        );
    }
}
