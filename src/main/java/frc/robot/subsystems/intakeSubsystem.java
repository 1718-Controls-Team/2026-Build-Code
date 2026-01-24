// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;

public class intakeSubsystem extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
    
    TalonFX intakeSpinMotor = new TalonFX(0);
    TalonFX intakeEletricSlideMotor = new TalonFX(1);

    PositionVoltage intakePosition = new PositionVoltage(0);

    private VelocityVoltage voltageRequest = new VelocityVoltage(0);

  public intakeSubsystem() {
    this.configureintakeSpinMotor(intakeSpinMotor);
    this.configureintakeElectricSlideMotor(intakeEletricSlideMotor);
  }

  

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public void setIntakeSpinSpeed(double intakeSpeed) {
    intakeSpinMotor.setControl(voltageRequest.withVelocity(intakeSpeed));
  }

  public void setIntakeElectricSlidePos(double intakePos){
    intakeEletricSlideMotor.setControl(intakePosition.withPosition(intakePos));
  }

  public double getIntakeElectricSlidePos(){
    return intakeEletricSlideMotor.getPosition().getValueAsDouble();
  }

  public boolean getIntakeSlideInPos(){
    if ((Math.abs(intakeEletricSlideMotor.getPosition().getValueAsDouble() - getIntakeElectricSlidePos()) < 0.1)){
      return true;
    } else {
      return false;
    }
  }

  public void configureintakeSpinMotor(TalonFX intakeSpinMotor){
    TalonFXConfiguration intakeSpinMotorConfig = new TalonFXConfiguration();

    intakeSpinMotorConfig.CurrentLimits.SupplyCurrentLimit = Constants.kintakeSpinMotorSupplyCurrentLimit;
    intakeSpinMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;    

    intakeSpinMotorConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = Constants.kintakeSpinMotorClosedLoopRampPeriod;
    intakeSpinMotorConfig.Voltage.PeakForwardVoltage = Constants.kintakeSpinMotorPeakForwardVoltage;
    intakeSpinMotorConfig.Voltage.PeakReverseVoltage = Constants.kintakeSpinMotorPeakReverseVoltage;

    intakeSpinMotorConfig.MotorOutput.Inverted = Constants.kintakeSpinMotorDirection;
    intakeSpinMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
  

    Slot0Configs slot0 = intakeSpinMotorConfig.Slot0;
    slot0.kP = Constants.kintakeSpinMotorProportional;
    slot0.kI = Constants.kintakeSpinMotorIntegral;
    slot0.kD = Constants.kintakeSpinMotorDerivative;
    slot0.kV = Constants.kintakeSpinMotorVelocityFeedForward;
    
    slot0.GravityType = GravityTypeValue.Arm_Cosine;
    slot0.kG = Constants.kintakeSpinMotorGravityFeedForward;
    slot0.kS = Constants.kintakeSpinMotorStaticFeedForward;
    

    
    StatusCode intakeSpinMotorStatus = StatusCode.StatusCodeNotInitialized;
    for(int i = 0; i < 5; ++i) {
      intakeSpinMotorStatus = intakeSpinMotor.getConfigurator().apply(intakeSpinMotorConfig);
      if (intakeSpinMotorStatus.isOK()) break;
    }
    if (!intakeSpinMotorStatus.isOK()) {
      System.out.println("Could not configure device. Error: " + intakeSpinMotorStatus.toString());
    }
    intakeSpinMotor.setPosition(0);
  }

  public void configureintakeElectricSlideMotor(TalonFX intakeElectricSlideMotor){
    TalonFXConfiguration intakeElectricSlideMotorConfig = new TalonFXConfiguration();

    intakeElectricSlideMotorConfig.MotorOutput.Inverted = Constants.kintakeElectricSlideMotorDirection;
    intakeElectricSlideMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;


    intakeElectricSlideMotorConfig.CurrentLimits.SupplyCurrentLimit = Constants.kintakeElectricSlideMotorSupplyCurrentLimit;
    intakeElectricSlideMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    intakeElectricSlideMotorConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = Constants.kintakeElectricSlideMotorVoltageClosedLoopRampPeriod;
    intakeElectricSlideMotorConfig.Voltage.PeakForwardVoltage = Constants.kintakeElectricSlideMotorMaxForwardVoltage;
    intakeElectricSlideMotorConfig.Voltage.PeakReverseVoltage = Constants.kintakeElectricSlideMotorMaxReverseVoltage;
    

    Slot0Configs slot0 = intakeElectricSlideMotorConfig.Slot0;
    slot0.kP = Constants.kintakeElectricSlideMotorProportional;
    slot0.kI = Constants.kintakeElectricSlideMotorIntegral;
    slot0.kD = Constants.kintakeElectricSlideMotorDerivative;

    slot0.GravityType = GravityTypeValue.Elevator_Static;
    slot0.kV = Constants.kintakeElectricSlideMotorVelocityFeedForward;
    slot0.kG = Constants.kintakeElectricSlideMotorGravityFeedForward;
    slot0.kS = Constants.kintakeElectricSlideMotorStaticFeedForward;
 


    StatusCode intakeElectricSlideMotorStatus = StatusCode.StatusCodeNotInitialized;
    for(int i = 0; i < 5; ++i) {
      intakeElectricSlideMotorStatus = intakeEletricSlideMotor.getConfigurator().apply(intakeElectricSlideMotorConfig);
      if (intakeElectricSlideMotorStatus.isOK()) break;
    }
    if (!intakeElectricSlideMotorStatus.isOK()) {
      System.out.println("Could not configure device. Error: " + intakeElectricSlideMotorStatus.toString());
    }
    intakeEletricSlideMotor.setPosition(0);
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
