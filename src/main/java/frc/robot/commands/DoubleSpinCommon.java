package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SpinMotor;

public class DoubleSpinCommon extends Command {

    private SpinMotor spinMotor1;
    private SpinMotor spinMotor2;

    private double speed1;
    private double speed2;

    private boolean finished = false;

    public DoubleSpinCommon(SpinMotor spinMotor1, SpinMotor spinMotor2, double speed1, double speed2) {
        this.spinMotor1 = spinMotor1;
        this.spinMotor2 = spinMotor2;
        this.speed1 = speed1;
        this.speed2 = speed2;
    }

    @Override
    public void execute() {
        this.spinMotor1.setSpeed(this.speed1);
        this.spinMotor2.setSpeed(this.speed2);
        this.finished = false;
    }

    @Override
    public void end(boolean interrupted) {
        this.spinMotor1.setSpeed(0);
        this.spinMotor2.setSpeed(0);
        this.finished = true;
    }

    @Override
    public boolean isFinished() {
        return this.finished;
    }
}
