package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SpinMotor extends SubsystemBase {

    private CANSparkMax motor;
    private boolean invert = false;

    private SpinMotor(CANSparkMax motor, boolean inverted) {
        this.motor = motor;
        this.invert = inverted;
    }

    public SpinMotor(int canId) {
        this.motor = new CANSparkMax(canId, MotorType.kBrushless);
    }

    public SpinMotor inverted() {
        return new SpinMotor(this.motor, true);
    }

    public void periodic() {
        motor.setIdleMode(IdleMode.kBrake);
    }

    public void setSpeed(double speed) {
        motor.set(invert ? -speed : speed);
    }
    
    public CANSparkMax getSparkMax() {
        return motor;
    }
}
