package frc.robot.subsystems.Arm;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface ArmMotorIO {

  @AutoLog
  public static class ArmMotorIOInputs {
    public boolean masterConnected;
    public double masterPositionRad = 0.0;
    public double masterVelocityRadPerSec = 0.0;
    public double masterAppliedVolts = 0.0;
    public double masterCurrentAmps = 0.0;

    public boolean followerConnected = false;
    public double followerPositionRad = 0.0;
    public double followerVelocityRadPerSec = 0.0;
    public double followerAppliedVolts = 0.0;
    public double followerCurrentAmps = 0.0;

    public double extentionAbsPos = 0.0;

    public Rotation2d extentionPos = new Rotation2d();
  }
  /** Updates the set of loggable inputs. */
  public default void updateInputs(ArmMotorIOInputs inputs) {}

  /** Run the drive motor at the specified open loop value. */
  public default void setArmOpenLoop(double output) {}

  /** Run the drive motor at the specified velocity. */
  public default void setArmVelocity(double velocityRadPerSec) {}

  /** Run the turn motor to the specified rotation. */
  public default void setArmPosition(double rotation) {}

  public default void setPercentOutput(double percentDecimal) {}

  public default void setSetpoint(double setpoint) {}

  public default void setVoltage(double voltage) {}

  public default void resetEncoder() {}

  public default void stop() {
    setArmVelocity(0);
  }
}
