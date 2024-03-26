package frc.robot.commands;

import java.util.function.Supplier;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

/**
 * Class that extends the Phoenix SwerveDrivetrain class and implements subsystem
 * so it can be used in command-based projects easily.
 */
public class CommandSwerveDrivetrain extends SwerveDrivetrain implements Subsystem {
    private static final double kSimLoopPeriod = 0.005; // 5 ms
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;

    public CommandSwerveDrivetrain(SwerveDrivetrainConstants driveTrainConstants, double OdometryUpdateFrequency, SwerveModuleConstants... modules) {
        super(driveTrainConstants, OdometryUpdateFrequency, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
        setSwerveDriveCustomCurrentLimits();
    }
    public CommandSwerveDrivetrain(SwerveDrivetrainConstants driveTrainConstants, SwerveModuleConstants... modules) {
        super(driveTrainConstants, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
        setSwerveDriveCustomCurrentLimits();

    }

    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get()));
    }

    private void startSimThread() {
        m_lastSimTime = Utils.getCurrentTimeSeconds();

        /* Run simulation at a faster rate so PID gains behave more reasonably */
        m_simNotifier = new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - m_lastSimTime;
            m_lastSimTime = currentTime;

            /* use the measured time delta, get battery voltage from WPILib */
            updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });
        m_simNotifier.startPeriodic(kSimLoopPeriod);
    }

    public void setSwerveDriveCustomCurrentLimits() {
        //Create a current configuration to use for the drive motor of each swerve module.
        var customCurrentLimitConfigs = new CurrentLimitsConfigs();

        //Iterate through each module.
        for (var module : Modules) {
            //Get the Configurator for the current drive motor.
            var drivecurrentConfigurator = module.getDriveMotor().getConfigurator();
            var steercurrentConfigurator = module.getSteerMotor().getConfigurator();

            //Refresh the current configuration, since the stator current limit has already been set.
            drivecurrentConfigurator.refresh(customCurrentLimitConfigs);
            steercurrentConfigurator.refresh(customCurrentLimitConfigs);
            //Set all of the parameters related to the supply current.  The values should come from Constants.
            customCurrentLimitConfigs.SupplyCurrentLimit = 30;
            customCurrentLimitConfigs.SupplyCurrentLimitEnable = true;
            customCurrentLimitConfigs.SupplyCurrentThreshold = 80;
            customCurrentLimitConfigs.SupplyTimeThreshold = 0.01;

            //Apply the new current limit configuration.
            drivecurrentConfigurator.apply(customCurrentLimitConfigs);
            steercurrentConfigurator.apply(customCurrentLimitConfigs);
        }
    }
}
