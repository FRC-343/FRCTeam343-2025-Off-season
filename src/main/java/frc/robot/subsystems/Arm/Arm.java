package frc.robot.subsystems.Arm;

import com.pathplanner.lib.config.PIDConstants;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.bobot_state2.BobotState;
import frc.robot.util.Constant;
import frc.robot.util.Constant.ArmConstants;
import org.littletonrobotics.junction.Logger;

/*
 * self explanatory
 * Do note that out Visulizer is not currently working as of 2/6/2025
 */

public class Arm extends SubsystemBase {
  private final ArmMotorIO io;

  boolean test = false;

  private final ArmMotorIOInputsAutoLogged inputs = new ArmMotorIOInputsAutoLogged();

  private final PIDController pidController =
      new PIDController(
          0.5, // Replace with actual PID values when on the bot
          0, 0);

  private final ArmVisualizer measuredVisualizer = new ArmVisualizer("Measured", Color.kBlack);
  private final ArmVisualizer setpointVisualizer = new ArmVisualizer("Setpoint", Color.kGreen);

  private double setpointInches = 0.0;

  public Arm() {
    switch (Constants.currentMode) {
      case REAL:
        io = new ArmMotorTalonFX(21);

        break;
      case SIM:
        io = new ArmMotorSim(DCMotor.getKrakenX60(1), 3, 1, new PIDConstants(1, 0, 0));

        break;
      case REPLAY:
      default:
        io = new ArmMotorIO() {};

        break;
    }
  }

  @Override
  public void periodic() {
    this.io.updateInputs(this.inputs);

    Logger.processInputs("Arm", this.inputs);

    if (DriverStation.isDisabled()) {
      this.setSetpoint(0.0);
      this.io.stop();
    }

    Logger.recordOutput("Arm/SetpointInches", setpointInches);

    // Log Mechanisms
    // measuredVisualizer.update(this.inputs.masterPositionRad);
    // setpointVisualizer.update(this.setpointInches);
    // // I'm not quite sure how this works, it is semi working in sim.

    // resetEncoder();

    // BobotState.setArmUp(this.inputs.masterPositionRad >= 1.0);

    BobotState.updateArmPose(this.inputs.masterPositionRad);
    // limitIsTriggered().onTrue(resetEncoder());
    // BackupLimitIsTriggerd().onTrue(resetEncoder());
    // ArmIsDown().onFalse(resetEncoder());
  }

  // These needs to be reorganized

  private void setSetpoint(double setpoint) {
    setpointInches = MathUtil.clamp(setpoint, 0, 56); // not real value
    this.pidController.setSetpoint(this.setpointInches);
  }

  public Command setSetpointCommand(double positionInches) {
    return new InstantCommand(() -> this.setSetpoint(positionInches));
  }

  public Command setSetpointCurrentCommand() {
    return new InstantCommand(() -> this.setSetpoint(this.inputs.extentionAbsPos));
  }

  public Command pidCommand() {
    return new RunCommand(
        () -> {
          double output = this.pidController.calculate(this.inputs.extentionAbsPos);
          setVoltage(output);
        },
        this);
  }

  public Command setArmPosition(double position) {
    return new RunCommand(() -> this.io.setArmPosition(position));
  }

  public Command setArmPositionL4() {
    System.out.println(ArmNearL4().getAsBoolean());

    return new RunCommand(() -> this.io.setArmPosition(Constant.ArmConstants.L4Level), this)
        .until(ArmNearL4());
  }

  public Command setArmPositionL3() {
    System.out.println(ArmNearL3().getAsBoolean());

    return new RunCommand(() -> this.io.setArmPosition(Constant.ArmConstants.L3Level), this)
        .until(ArmNearL3());
  }

  public Command setArmPositionL2() {
    System.out.println(ArmNearL2().getAsBoolean());

    return new RunCommand(() -> this.io.setArmPosition(Constant.ArmConstants.L2Level), this)
        .until(ArmNearL2());
  }

  public Command setArmPositionFeed() {
    System.out.println(ArmIsDown().getAsBoolean());

    return new RunCommand(() -> this.io.setArmPosition(Constant.ArmConstants.FEED), this)
        .until(ArmIsDown());
  }

  public Trigger ArmNearL4() {
    return new Trigger(
        () -> MathUtil.isNear(Constant.ArmConstants.L4Level, this.inputs.masterPositionRad, 1));
  }

  public Trigger ArmNearL3() {
    return new Trigger(
        () -> MathUtil.isNear(Constant.ArmConstants.L3Level, this.inputs.masterPositionRad, 1));
  }

  public Trigger ArmNearL2() {
    return new Trigger(
        () -> MathUtil.isNear(Constant.ArmConstants.L2Level, this.inputs.masterPositionRad, 1));
  }

  public Trigger higherThanL4() {
    return new Trigger(() -> (this.inputs.masterPositionRad > 20));
  }

  public Trigger higherThanL3() {
    return new Trigger(() -> (this.inputs.masterPositionRad > ArmConstants.L3Level));
  }

  public Command stopCommand() {
    return new InstantCommand(this.io::stop, this);
  }

  public void setVoltage(double voltage) {
    this.io.setArmVelocity(MathUtil.clamp(voltage, -12.0, 12.0));
  }

  public Command setVolatageCommand(double voltage) {
    return new RunCommand(() -> this.io.setArmVelocity(voltage), this);
  }

  public Command resetEncoder() {
    return new InstantCommand(this.io::resetEncoder, this);
  }

  public Trigger ArmIsDown() {
    return new Trigger(() -> MathUtil.isNear(0, this.inputs.masterPositionRad, 1));
  }

  public Command setPercentOutputCommand(double velocityRotPerSecond) {
    setpointInches = velocityRotPerSecond * 1000;
    return new RunCommand(() -> this.io.setPercentOutput(velocityRotPerSecond), this);
  }

  /* Adding new commands down here to ease readability,
   * at some point the above commands will be reorganized.
   * Triggers might also be separated at a later date, potentially added to BobotState
   */

}
