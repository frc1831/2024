package frc.robot.drivebase;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Drivebase;
import frc.robot.subsystems.SwerveBase;

public class TeleopDrive extends Command {
    
    private SwerveBase swerve;
    private DoubleSupplier vX, vY, omega;
    private BooleanSupplier driveMode;
    private boolean isOpenLoop;

    /**
     * Creates a new command
     * 
     * @param swerve
     */
    public TeleopDrive(SwerveBase swerve, DoubleSupplier vX, DoubleSupplier vY, DoubleSupplier omega, BooleanSupplier driveMode, boolean isOpenLoop) {
        this.swerve = swerve;
        this.vX = vX;
        this.vY = vY;
        this.omega = omega;
        this.driveMode = driveMode;
        this.isOpenLoop = isOpenLoop;
        addRequirements(swerve);
    }

    @Override
    public void initialize() {}

    @Override
  public void execute() {
    double xVelocity = Math.pow(vX.getAsDouble(), 3) * Drivebase.MAX_SPEED;
    double yVelocity = Math.pow(vY.getAsDouble(), 3) * Drivebase.MAX_SPEED;
    double angVelocity = Math.pow(omega.getAsDouble(), 3) * Drivebase.MAX_ANGULAR_VELOCITY;
    SmartDashboard.putNumber("vX", xVelocity);
    SmartDashboard.putNumber("vY", yVelocity);
    SmartDashboard.putNumber("omega", angVelocity);
    swerve.drive(
      new Translation2d(
        xVelocity,
        yVelocity),
      angVelocity,
      driveMode.getAsBoolean(),
      isOpenLoop);
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }

}
