package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MotorConstants;

public class Intake extends SubsystemBase {
    // VARIABLES
    private final CANSparkMax m_intakeMotor = 
        new CANSparkMax(
            MotorConstants.kIntakeCANId,
            MotorType.kBrushed);

    private void set(double speed) {
        m_intakeMotor.set(speed);
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
