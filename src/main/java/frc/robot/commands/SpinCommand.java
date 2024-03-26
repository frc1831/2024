package frc.robot.commands;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.stream.IntStream;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SpinMotor;

public class SpinCommand extends Command {

    private SpinAction[] actions;
    private boolean finished = false;

    private Timer timer = new Timer();

    public SpinCommand(SpinAction[] actions) {
        this.actions = actions;
    }

    public SpinCommand(SpinMotor motor, double speed) {
        this(new SpinAction[] { new SpinAction(motor, speed, 0) });
    }

    @Override
    public void initialize() {
        this.timer.reset();
        this.timer.start();
    }

    @Override
    public void execute() {
        final double timerValue = this.timer.get();
        Arrays.stream(this.actions).forEach(a -> a.run(timerValue));
        this.finished = false;
    }

    @Override
    public void end(boolean interrupted) {
        Arrays.stream(this.actions).forEach(SpinAction::stop);
        this.timer.stop();

        this.finished = true;
    }

    @Override
    public boolean isFinished() {
        return this.finished;
    }

    public SpinCommand toInverted() {
        return new SpinCommand(Arrays.stream(this.actions).map(SpinAction::inverted).toArray(SpinAction[]::new));
    }

    @Override
    public String toString() {
        return Arrays.toString(this.actions);
    }

    private static class SpinAction {

        private SpinMotor motor;
        private double speed;
        private double delay;

        private SpinAction(SpinMotor motor, double speed, double delay) {
            this.motor = motor;
            this.speed = speed;
            this.delay = delay;
        }

        private void run(double timerValue) {
            if (timerValue >= delay)
                this.motor.setSpeed(speed);
        }

        private void stop() {
            this.motor.setSpeed(0);
        }

        private SpinAction inverted() {
            return new SpinAction(this.motor.inverted(), this.speed, this.delay);
        }

        @Override
        public String toString() {
            int canId = this.motor.getSparkMax().getDeviceId();
            return "[CAN_ID_" + canId + ",SPEED_" + speed + ",DELAY_" + delay + "]";
        }

    }

    public static class Builder {

        private List<SpinAction> actions = new ArrayList<SpinAction>();

        public Builder motor(SpinMotor motor, double speed) {
            this.actions.add(new SpinAction(motor, speed, 0));
            return this;
        }

        public Builder motors(double speed, SpinMotor... motors) {
            Arrays.stream(motors).map(m -> new SpinAction(m, speed, 0)).forEach(this.actions::add);
            return this;
        }

        public DelayChain delayChain() {
            return new DelayChain(this);
        }

        private Builder add(SpinAction action) {
            this.actions.add(action);
            return this;
        }

        public SpinCommand build() {
            return new SpinCommand(this.actions.toArray(new SpinAction[this.actions.size()]));
        }
    }

    public static class DelayChain {

        private List<SpinAction[]> actions = new ArrayList<SpinAction[]>();
        private Builder builder;

        private DelayChain(Builder builder) {
            this.builder = builder;
        }

        public Link link(double delay, SpinMotor... motors) {
            if (motors.length == 0)
                throw new IllegalArgumentException("Motors must have at least 1 element");

            SpinAction[] actions = Arrays.stream(motors).map(m -> new SpinAction(m, 0, delay))
                    .toArray(SpinAction[]::new);
            this.actions.add(actions);

            return new Link(actions, this);
        }

        public Builder endChain() {
            double accumulator = 0;

            for (int i = 0; i < this.actions.size(); i++) {
                SpinAction[] link = this.actions.get(i);
                for (SpinAction action : link) {
                    action.speed += accumulator;
                    builder.add(action);
                }
                accumulator += link[0].delay;
            }

            return builder;
        }
    }

    public static class Link {

        private SpinAction[] actions;
        private DelayChain chain;

        private Link(SpinAction[] actions, DelayChain chain) {
            this.actions = actions;
            this.chain = chain;
        }

        public DelayChain withSpeeds(double... speeds) {
            if (speeds.length < 1)
                throw new IllegalArgumentException("Speeds must have at least 1 element");

            final double[] finalSpeeds;
            if (this.actions.length > speeds.length) {
                double[] newSpeeds = Arrays.copyOf(speeds, this.actions.length);
                Arrays.fill(newSpeeds, speeds.length, this.actions.length, speeds[speeds.length - 1]);
                finalSpeeds = newSpeeds;
            } else
                finalSpeeds = speeds;

            IntStream.range(0, speeds.length).forEach(i -> this.actions[i].speed = finalSpeeds[i]);
            return chain;
        }

    }

}