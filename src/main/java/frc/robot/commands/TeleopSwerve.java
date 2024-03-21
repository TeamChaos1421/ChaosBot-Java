package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;

public class TeleopSwerve extends Command {
    private Drivetrain m_drivetrain;
    private DoubleSupplier xSupplier;
    private DoubleSupplier ySupplier;
    private DoubleSupplier rSupplier;
    private BooleanSupplier fOSupplier;

    public TeleopSwerve(
        Drivetrain m_drivetrain, 
        DoubleSupplier xSupplier, 
        DoubleSupplier ySupplier, 
        DoubleSupplier rSupplier, 
        BooleanSupplier fOSupplier
    ) {
        this.m_drivetrain = m_drivetrain;

        this.xSupplier = xSupplier;
        this.ySupplier = ySupplier;
        this.rSupplier = rSupplier;
        this.fOSupplier = fOSupplier;

        addRequirements(m_drivetrain);
    }

    @Override
    public void execute() {
        double xSpeed = MathUtil.applyDeadband(xSupplier.getAsDouble(), 0.1);
        double ySpeed = MathUtil.applyDeadband(ySupplier.getAsDouble(), 0.1);
        double rSpeed = MathUtil.applyDeadband(rSupplier.getAsDouble(), 0.1);

        SmartDashboard.putNumber("input", xSpeed);

        m_drivetrain.drive(xSpeed, ySpeed, rSpeed, fOSupplier.getAsBoolean());
    }
}
