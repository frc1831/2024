package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.SwerveModuleConstants;
import frc.robot.Constants.Drivebase;
import swervelib.encoders.CANCoderSwerve;


public class SwerveModule {
    public int moduleNumber;
    private double angleOffset, lastAngle;
    private CANSparkMax angleMotor, driveMotor;
    private CANCoderSwerve absoluteEncoder;
    private RelativeEncoder angleEncoder, driveEncoder;
    private SparkPIDController angleController, driveController;

    SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(Drivebase.KS, Drivebase.KV, Drivebase.KA);

    public SwerveModule(int moduleNumber, SwerveModuleConstants moduleConstants) {
        this.moduleNumber = moduleNumber;
        angleOffset = moduleConstants.angleOffset;

        angleMotor = new CANSparkMax(moduleConstants.angleMotorID, MotorType.kBrushless);
        driveMotor = new CANSparkMax(moduleConstants.driveMotorID, MotorType.kBrushless);
        angleMotor.restoreFactoryDefaults();
        //driveMotor.restoreFactoryDefaults();

        absoluteEncoder = new CANCoderSwerve(moduleConstants.cancoderID);
        //absoluteEncoder.factoryDefault();
        /*
        CANcoderConfiguration canCoderConfiguration = new CANCoderConfiguration();
        canCoderConfiguration.
        canCoderConfiguration.sensorDirection = Drivebase.CANCODER_INVERT;
        canCoderConfiguration.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
        canCoderConfiguration.sensorTimeBase = SensorTimeBase.PerSecond;
        absoluteEncoder.configAllSettings(canCoderConfiguration);
*/
        angleEncoder = angleMotor.getEncoder();
        angleEncoder.setPositionConversionFactor(Drivebase.DEGREES_PER_STEERING_ROTATION);
        angleEncoder.setVelocityConversionFactor(Drivebase.DEGREES_PER_STEERING_ROTATION / 60);
        angleEncoder.setPosition(absoluteEncoder.getAbsolutePosition() - angleOffset);

        angleController = angleMotor.getPIDController();
        angleController.setP(Drivebase.MODULE_KP);
        angleController.setI(Drivebase.MODULE_KI);
        angleController.setD(Drivebase.MODULE_KD);
        angleController.setFF(Drivebase.MODULE_KF);
        angleController.setIZone(Drivebase.MODULE_IZ);
        angleController.setPositionPIDWrappingEnabled(true);
        angleController.setPositionPIDWrappingMaxInput(180);
        angleController.setPositionPIDWrappingMinInput(-180);
        angleMotor.setIdleMode(CANSparkMax.IdleMode.kCoast);

        driveController = driveMotor.getPIDController();
        driveEncoder = driveMotor.getEncoder();
        driveEncoder.setPositionConversionFactor(Drivebase.METERS_PER_MOTOR_ROTATION);
        driveEncoder.setVelocityConversionFactor(Drivebase.METERS_PER_MOTOR_ROTATION / 60);
        driveController.setP(Drivebase.VELOCITY_KP);
        driveController.setI(Drivebase.VELOCITY_KI);
        driveController.setD(Drivebase.VELOCITY_KD);
        driveController.setFF(Drivebase.VELOCITY_KF);
        driveController.setIZone(Drivebase.VELOCITY_IZ);
        driveMotor.setInverted(Drivebase.DRIVE_MOTOR_INVERT);
        driveMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);

        lastAngle = getState().angle.getDegrees();
    }

    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {
        desiredState = SwerveModuleState.optimize(desiredState, getState().angle);

        if (isOpenLoop) {
            double percentOutput = desiredState.speedMetersPerSecond / Drivebase.MAX_SPEED;
            driveMotor.set(percentOutput);
        } else {
            double velocity = desiredState.speedMetersPerSecond;
            driveController.setReference(velocity, ControlType.kVelocity, 0, feedforward.calculate(velocity));
        }

        double angle = (Math.abs(desiredState.speedMetersPerSecond) <= (Drivebase.MAX_SPEED * 0.01) ? 
            lastAngle :
            desiredState.angle.getDegrees()); // Prevents module rotation if speed is less than 1%
        angleController.setReference(angle, ControlType.kPosition);
        lastAngle = angle; 
    }

    public SwerveModuleState getState() {
        double velocity = driveEncoder.getVelocity();
        Rotation2d azimuth = Rotation2d.fromDegrees(angleEncoder.getPosition());
        return new SwerveModuleState(velocity, azimuth);
    }

    public SwerveModulePosition getPosition() {
        double position = driveEncoder.getPosition();
        Rotation2d azimuth = Rotation2d.fromDegrees(angleEncoder.getPosition());
        return new SwerveModulePosition(position, azimuth);
    }

    public double getCANCoder() {
        return absoluteEncoder.getAbsolutePosition();
    }

    public double getRelativeEncoder() {
        return angleEncoder.getPosition();
    }

    public void setMotorBrake(boolean brake) {
        driveMotor.setIdleMode(brake ? CANSparkMax.IdleMode.kBrake : CANSparkMax.IdleMode.kCoast);
    }
}
