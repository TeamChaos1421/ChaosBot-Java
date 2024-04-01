package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MotorConstants;

public class Intake extends SubsystemBase {
    // VARIABLES
    private final CANSparkMax m_topIntake = 
        new CANSparkMax(
            MotorConstants.kTopCANId,
            MotorType.kBrushless);
    private final TalonFX m_botIntake = 
        new TalonFX(MotorConstants.kBotCANId);

    private void set(double speed) {
        m_topIntake.set(-speed);
        m_botIntake.set(-speed);
    }


    // COMMANDS
    public Command On() { 
        return this.runOnce(
            () -> {
                this.set(1.0);
            }
        );
    }

    public Command Off() {
        return this.runOnce(
            () -> {
                this.set(0.0);
            }
        );
    }
}
