package frc.robot.subsystems.Arm;

import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.subsystems.Arm.ArmMotorIO.ArmMotorIOInputs;

public class ArmMotorTalonFX implements ArmMotorIO {
  private final TalonFX talon;

  // private final SparkBase encoder = new SparkMax(25, null);
  // private final AbsoluteEncoder absEnc;

  private final StatusSignal<Voltage> voltage;
  private final StatusSignal<Double> dutyCycle;
  private final StatusSignal<AngularVelocity> velocity;
  private final StatusSignal<Angle> position;
  private final StatusSignal<Current> current;

  private final VelocityVoltage velocityVoltage = new VelocityVoltage(0);
  private final DutyCycleOut dutyCycleOut = new DutyCycleOut(0);

  private final MotionMagicVoltage Vrequest = new MotionMagicVoltage(0);

  private final Orchestra m_orchestra = new Orchestra();

  public ArmMotorTalonFX(int deviceId) {
    talon = new TalonFX(deviceId);
    voltage = talon.getMotorVoltage();
    dutyCycle = talon.getDutyCycle();
    velocity = talon.getVelocity();
    position = talon.getPosition();
    current = talon.getStatorCurrent();

    // absEnc = encoder.getAbsoluteEncoder();

    this.m_orchestra.addInstrument(talon);
    this.m_orchestra.loadMusic("output.chrp");

    talon
        .getConfigurator()
        .apply(
            new TalonFXConfiguration()
                .withMotorOutput(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake))
                .withSlot0(
                    new Slot0Configs().withKV(0.12).withKA(.01).withKP(5).withKI(0).withKD(0))
                .withMotionMagic(
                    new MotionMagicConfigs()
                        .withMotionMagicAcceleration(100)
                        .withMotionMagicCruiseVelocity(100)
                        .withMotionMagicJerk(600)));
    velocityVoltage.Slot = 0;
  }

  public void updateInputs(ArmMotorIOInputs inputs) {
    StatusSignal.refreshAll(velocity, dutyCycle, voltage, position);
    inputs.masterAppliedVolts = voltage.getValueAsDouble();
    inputs.masterVelocityRadPerSec = velocity.getValueAsDouble();
    inputs.masterPositionRad = position.getValueAsDouble();
    inputs.masterCurrentAmps = current.getValueAsDouble();

    // inputs.extentionAbsPos = absEnc.getPosition();
  }

  @Override
  public void setArmVelocity(double velocityRotPerSecond) {
    talon.setControl(dutyCycleOut.withOutput(velocityRotPerSecond));
    // this.follower.setControl(dutyCycleOut.withOutput(velocityRotPerSecond));
  }

  @Override
  public void setPercentOutput(double percentDecimal) {
    talon.setControl(dutyCycleOut.withOutput(percentDecimal));
    // this.follower.setControl(dutyCycleOut.withOutput(percentDecimal));
  }

  @Override
  public void setSetpoint(double setpoint) {
    talon.setControl(dutyCycleOut.withOutput(setpoint));
    // this.follower.setControl(dutyCycleOut.withOutput(setpoint));
  }

  @Override
  public void setVoltage(double voltage) {
    talon.setControl(Vrequest.withPosition(voltage));
  }

  @Override
  public void setArmPosition(double rotation) {
    talon.setControl(Vrequest.withPosition(rotation));
  }

  @Override
  public void resetEncoder() {
    talon.setPosition(0);
  }
}
