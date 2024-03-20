package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MotorConstants;

public class Climber extends SubsystemBase{
    // VARIABLES
    private final CANSparkMax m_climberMotor = 
        new CANSparkMax(
            MotorConstants.kClimberCANId,
            MotorType.kBrushless);

    private void set(double speed) {
        m_climberMotor.set(speed);
    }


    // COMMANDS
    public Command Set(double speed) { 
        return this.run(
            () -> {
                this.set(speed);
            }
        );
    }
}
