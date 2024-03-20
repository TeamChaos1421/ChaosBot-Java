package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MotorConstants;

public class Shooter extends SubsystemBase {
    // VARIABLES
    private final TalonFX m_shooterHigh = 
        new TalonFX(MotorConstants.kShooterHighCANId);
    private final TalonFX m_shooterMid = 
        new TalonFX(MotorConstants.kShooterMidCANId);
    private final TalonFX m_shooterLow = 
        new TalonFX(MotorConstants.kShooterLowCANId);

    private void set(double speedHigh, double speedMid, double speedLow) {
        m_shooterHigh.set(speedHigh);
        m_shooterMid.set(speedMid);
        m_shooterLow.set(speedLow);
    }


    // COMMANDS
    public Command Shoot() {
        return this.run(
            () -> {
                this.set(-1, -1, -0.2);
            }
        );
    }

    public Command Load() {
        return this.run(
            () -> {
                this.set(-0.2, -0.2, -0.2);
            }
        );
    }

    public Command Zero() {
        return this.run(
            () -> {
                this.set(0, 0, 0);
            }
        );
    }
}
