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
    TalonFX intakeEletricSlideMotor = new TalonFX(0);

    PositionVoltage intakePosition = new PositionVoltage(0);

    private VelocityVoltage voltageRequest = new VelocityVoltage(0);
  public intakeSubsystem() {
    this.configureintakeSpinMotor(intakeSpinMotor);
    this.
  }

  

  /**
   * Example command factory method.
   *
   * @return a command
   */
 
  public void setEricaPower(double Erika) {
    
  }



  public void configureintakeSpinMotor(TalonFX algaeIntakeSpin){
    TalonFXConfiguration intakeSpinMotorConfig = new TalonFXConfiguration();

    intakeSpinMotorConfig.CurrentLimits.SupplyCurrentLimit = Constants.intakeSpinMotorSupplyCurrentLimit;
    intakeSpinMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;    

    intakeSpinMotorConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = Constants.intakeSpinMotorClosedLoopRampPeriod;
    intakeSpinMotorConfig.Voltage.PeakForwardVoltage = Constants.intakeSpinMotorPeakForwardVoltage;
    intakeSpinMotorConfig.Voltage.PeakReverseVoltage = Constants.intakeSpinMotorPeakReverseVoltage;

    intakeSpinMotorConfig.MotorOutput.Inverted = Constants.intakeSpinMotorDirection;
    intakeSpinMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
  

    Slot0Configs slot0 = intakeSpinMotorConfig.Slot0;
    slot0.kP = Constants.intakeSpinMotorProportional;
    slot0.kI = Constants.intakeSpinMotorIntegral;
    slot0.kD = Constants.intakeSpinMotorDerivative;
    
    slot0.GravityType = GravityTypeValue.Arm_Cosine;
    slot0.kG = Constants.intakeSpinMotorGravityFeedForward;
    slot0.kV = Constants.intakeSpinMotorVelocityFeedForward;
    slot0.kS = Constants.intakeSpinMotorStaticFeedForward;
    

    
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

  public void configureintakeElectricSlideMotor(TalonFX algaeIntakeSpin){
    TalonFXConfiguration intakeElectricSlideMotorConfig = new TalonFXConfiguration();

    intakeElectricSlideMotorConfig.MotorOutput.Inverted = Constants.intakeElectricSlideMotorDirection;
    intakeElectricSlideMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;


    intakeElectricSlideMotorConfig.CurrentLimits.SupplyCurrentLimit = Constants.intakeElectricSlideMotorSupplyCurrentLimit;
    intakeElectricSlideMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    intakeElectricSlideMotorConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = Constants.intakeElectricSlideMotorVoltageClosedLoopRampPeriod;
    intakeElectricSlideMotorConfig.Voltage.PeakForwardVoltage = Constants.intakeElectricSlideMotorMaxForwardVoltage;
    intakeElectricSlideMotorConfig.Voltage.PeakReverseVoltage = Constants.intakeElectricSlideMotorMaxReverseVoltage;
    

    Slot0Configs slot0 = intakeElectricSlideMotorConfig.Slot0;
    slot0.kP = Constants.intakeElectricSlideMotorProportional;
    slot0.kI = Constants.intakeElectricSlideMotorIntegral;
    slot0.kD = Constants.intakeElectricSlideMotorDerivative;

    slot0.GravityType = GravityTypeValue.Elevator_Static;
    slot0.kV = Constants.intakeElectricSlideMotorVelocityFeedForward;
    slot0.kG = Constants.intakeElectricSlideMotorGravityFeedForward;
    slot0.kS = Constants.intakeElectricSlideMotorStaticFeedForward;
 


    StatusCode intakeElectricSlideMotorStatus = StatusCode.StatusCodeNotInitialized;
    for(int i = 0; i < 5; ++i) {
      intakeElectricSlideMotorStatus = intakeElectricSlideMotor.getConfigurator().apply(intakeElectricSlideMotorConfig);
      if (intakeElectricSlideMotorStatus.isOK()) break;
    }
    if (!intakeElectricSlideMotorStatus.isOK()) {
      System.out.println("Could not configure device. Error: " + intakeElectricSlideMotorStatus.toString());
    }
    m_intakeElectricSlideMotor.setPosition(0);
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
