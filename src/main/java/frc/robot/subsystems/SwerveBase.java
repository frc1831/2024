package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.SwerveModule;
import frc.robot.Constants.Drivebase;

public class SwerveBase extends SubsystemBase {
    
    public static SwerveModule[] swerveModules;

    public static Pigeon2 pigeon;
    private SwerveDriveOdometry odometry;
    public Field2d field = new Field2d();

    private boolean wasGyroReset;

    public SwerveBase() {
        pigeon = new Pigeon2(50);

        swerveModules = new SwerveModule[] {
            new SwerveModule(0, Drivebase.Mod0.CONSTANTS),
            new SwerveModule(1, Drivebase.Mod1.CONSTANTS),
            new SwerveModule(2, Drivebase.Mod2.CONSTANTS),
            new SwerveModule(3, Drivebase.Mod3.CONSTANTS)
        };

        odometry = new SwerveDriveOdometry(Drivebase.KINEMATICS, getYaw(), getModulePositions());
        zeroGyro();
    }
    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
    // Creates a robot-relative ChassisSpeeds object, converting from field-relative speeds if necessary.
    ChassisSpeeds velocity = fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
      translation.getX(), 
      translation.getY(), 
      rotation, 
      getYaw()
    )
    : new ChassisSpeeds(
      translation.getX(),
      translation.getY(),
      rotation
    );

    // Display commanded speed for testing
    SmartDashboard.putString("RobotVelocity", velocity.toString());

    // Calculate required module states via kinematics
    SwerveModuleState[] swerveModuleStates = 
      Drivebase.KINEMATICS.toSwerveModuleStates(
        velocity
      );

    // Desaturate calculated speeds
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Drivebase.MAX_SPEED);

    // Command and display desired states
    for (SwerveModule module : swerveModules) {
      SmartDashboard.putString("Module" + module.toString(), swerveModuleStates[module.moduleNumber].toString());
      module.setDesiredState(swerveModuleStates[module.moduleNumber], isOpenLoop);
    }
  }

  /**
   * Set the module states (azimuth and velocity) directly.  Used primarily for auto
   * pathing.
   * @param desiredStates  A list of SwerveModuleStates to send to the modules.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    
    // Desaturates wheel speeds
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Drivebase.MAX_SPEED);

    // Sets states
    for (SwerveModule module : swerveModules) {
      module.setDesiredState(desiredStates[module.moduleNumber], false);
    }
  }

  
  /**
   * Gets the current pose (position and rotation) of the robot, as reported by odometry.
   * @return The robot's pose
   */
  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  /**
   * Gets the current field-relative velocity (x, y and omega) of the robot
   * @return A ChassisSpeeds object of the current field-relative velocity
   */
  public ChassisSpeeds getFieldVelocity() {
    // ChassisSpeeds has a method to convert from field-relative to robot-relative speeds,
    // but not the reverse.  However, because this transform is a simple rotation, negating the angle
    // given as the robot angle reverses the direction of rotation, and the conversion is reversed.
    return ChassisSpeeds.fromFieldRelativeSpeeds(Drivebase.KINEMATICS.toChassisSpeeds(getStates()), getYaw().unaryMinus());
  }

  /**
   * Gets the current robot-relative velocity (x, y and omega) of the robot
   * @return A ChassisSpeeds object of the current robot-relative velocity
   */
  public ChassisSpeeds getRobotVelocity() {
    return Drivebase.KINEMATICS.toChassisSpeeds(getStates());
  }

  
  /**
   * Resets odometry to the given pose. Gyro angle and module positions do not need to 
   * be reset when calling this method.  However, if either gyro angle or module position
   * is reset, this must be called in order for odometry to keep working.
   * @param pose The pose to set the odometry to
   */
  public void resetOdometry(Pose2d pose) {
    odometry.resetPosition(getYaw(), getModulePositions(), pose);
  }

  /**
   * Gets the current module states (azimuth and velocity)
   * @return A list of SwerveModuleStates containing the current module states
   */
  public SwerveModuleState[] getStates() {
    SwerveModuleState[] states = new SwerveModuleState[Drivebase.NUM_MODULES];
    for (SwerveModule module : swerveModules) {
      states[module.moduleNumber] = module.getState();
    }
    return states;
  }

  /**
   * Gets the current module positions (azimuth and wheel position (meters))
   * @return A list of SwerveModulePositions cointaing the current module positions
   */
  public static SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] positions = new SwerveModulePosition[Drivebase.NUM_MODULES];
    for (SwerveModule module : swerveModules) {
      positions[module.moduleNumber] = module.getPosition();
    }
    return positions;
  }

  /**
   * A public method to allow other systems to determine if the gyro was reset by accessing
   * the wasGyroReset flag.
   * @return The boolean value of wasGyroReset
   */
  public boolean wasGyroReset() {
    return wasGyroReset;
  }

  /**
   * Sets wasGyroReset to false.  Should be called after all systems that need to know have called
   * wasGyroReset.
   */
  public void clearGyroReset() {
    wasGyroReset = false;
  }

  /**
   * Resets the gyro angle to zero and resets odometry to the same position, but facing toward 0.
   * Also sets the wasGyroReset flag to true.
   */
  public void zeroGyro() {
    // Resets the real gyro or the angle accumulator, depending on whether the robot is being simulated
    // imu.setYaw(0);
    pigeon.reset();
    wasGyroReset = true;
    resetOdometry(new Pose2d(getPose().getTranslation(), new Rotation2d()));
  }

  /**
   * Gets the current yaw angle of the robot, as reported by the imu.  CCW positive, not wrapped.
   * @return The yaw angle
   */
  public static Rotation2d getYaw() {
    // double[] ypr = new double[3];
    // imu.getYawPitchRoll(ypr);
    return (Drivebase.INVERT_GYRO) ? Rotation2d.fromDegrees(360-pigeon.getAngle()) : Rotation2d.fromDegrees(pigeon.getAngle());
  }

  /**
   * Sets the drive motors to brake/coast mode.
   * @param brake True to set motors to brake mode, false for coast.
   */
  public void setMotorBrake(boolean brake) {
    for (SwerveModule swerveModule : swerveModules) {
      swerveModule.setMotorBrake(brake);
    }
  }

  /**
   * Point all modules toward the robot center, thus making the robot very difficult to move.
   */
  public void setDriveBrake() {
    for (SwerveModule swerveModule : swerveModules) {
      swerveModule.setDesiredState(
        new SwerveModuleState(
          0,
          Drivebase.MODULE_LOCATIONS[swerveModule.moduleNumber].getAngle()),
        true);
    }
  }

  /* public void setAlliance(Alliance alliance) {
    this.alliance = alliance;
    wasOdometrySeeded = false;
  } */

  @Override
  public void periodic() {
    // Update odometry
    odometry.update(getYaw(), getModulePositions());

    for (SwerveModule module : swerveModules) {
      SmartDashboard.putNumber("Module" + module.moduleNumber + "CANCoder", module.getCANCoder());
      SmartDashboard.putNumber("Module" + module.moduleNumber + "Relative Encoder", module.getRelativeEncoder());
    }
  }

  @Override
  public void simulationPeriodic() {
  }
}

