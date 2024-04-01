package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;

public class Leds extends SubsystemBase{
    private final PowerDistribution m_pdh = new PowerDistribution(3, ModuleType.kRev);
    private final Timer m_ledTimer = new Timer();
    private double lastTime;

    private void ToggleLEDs() {
        if (m_pdh.getSwitchableChannel() == true) {
            m_pdh.setSwitchableChannel(false);
        } else {
            m_pdh.setSwitchableChannel(true);
        }
    };

    public Command On() {
        return this.runOnce(
            () -> {
                this.m_pdh.setSwitchableChannel(true);
            }
        );
    }

    public Command Off() {
        return this.runOnce(
            () -> {
                this.m_pdh.setSwitchableChannel(false);
            }
        );
    }

    public Command ResetTimer() {
        return this.runOnce(
            () -> {
                m_ledTimer.reset();
                m_ledTimer.start();
                this.lastTime = 0;
            }
        );
    }

    public Command Flash() {
        return this.run(
            () -> {
                double timer = m_ledTimer.get();
                double interval = timer - this.lastTime;
                if (interval > 0.2) {
                    this.ToggleLEDs();
                    this.lastTime = timer;
                }
            }
        );
    }
}
