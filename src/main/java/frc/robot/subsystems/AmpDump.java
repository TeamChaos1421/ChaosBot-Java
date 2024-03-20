package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PneumaticConstants;

public class AmpDump extends SubsystemBase {
    // VARIABLES
    private final Compressor m_compressor = new Compressor(
        PneumaticConstants.kPCMCANId, 
        PneumaticsModuleType.CTREPCM
    );
    private final DoubleSolenoid m_Solenoid = new DoubleSolenoid(
        PneumaticsModuleType.CTREPCM,
        PneumaticConstants.kDumpForward, 
        PneumaticConstants.kDumpReverse
    );

    private void init() {
        m_compressor.enableDigital();
        m_Solenoid.set(Value.kReverse);
    }

    private void toggle() {
        m_Solenoid.toggle();
    }

    // COMMANDS
    public Command Init() {
        return this.runOnce(
            () -> {
                this.init();
            }
        );
    }

    public Command Toggle() {
        return this.runOnce(
            () -> {
                this.toggle();
            }
        );
    }
}
