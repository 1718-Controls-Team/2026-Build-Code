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

public class climberSubsystem extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
    
    TalonFX climberSpinMotor = new TalonFX(0);
    TalonFX climbRotateMotor = new TalonFX(1);

    private VelocityVoltage voltageRequest = new VelocityVoltage(0);
    PositionVoltage climbPosition = new PositionVoltage(0);

  public climberSubsystem() {
    //this.configureclimberSpinMotor(climberSpinMotor);
    //this.configureclimberSpinMotor(climbRotateMotor);

  }

  

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public void setclimberSpinSpeed(double climberSpeed) {
    climberSpinMotor.setControl(voltageRequest.withVelocity(climberSpeed));
  }

  public void setclimbRotatePos(double climbPos){
    climbRotateMotor.setControl(climbPosition.withPosition(climbPos));
  }

  
  public void configureclimberSpinMotor(TalonFX climberSpinMotor){
    TalonFXConfiguration climberSpinMotorConfig = new TalonFXConfiguration();

    climberSpinMotorConfig.CurrentLimits.SupplyCurrentLimit = Constants.kClimberSpinMotorSupplyCurrentLimit;
    climberSpinMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;    

    climberSpinMotorConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = Constants.kClimberSpinMotorClosedLoopRampPeriod;
    climberSpinMotorConfig.Voltage.PeakForwardVoltage = Constants.kClimberSpinMotorPeakForwardVoltage;
    climberSpinMotorConfig.Voltage.PeakReverseVoltage = Constants.kClimberSpinMotorPeakReverseVoltage;

    climberSpinMotorConfig.MotorOutput.Inverted = Constants.kClimberSpinMotorDirection;
    climberSpinMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
  

    Slot0Configs slot0 = climberSpinMotorConfig.Slot0;
    slot0.kP = Constants.kClimberSpinMotorProportional;
    slot0.kI = Constants.kClimberSpinMotorIntegral;
    slot0.kD = Constants.kClimberSpinMotorDerivative;
    slot0.kV = Constants.kClimberSpinMotorVelocityFeedForward;
    
    slot0.GravityType = GravityTypeValue.Arm_Cosine;
    slot0.kG = Constants.kClimberSpinMotorGravityFeedForward;
    slot0.kS = Constants.kClimberSpinMotorStaticFeedForward;
    

    
    StatusCode climberSpinMotorStatus = StatusCode.StatusCodeNotInitialized;
    for(int i = 0; i < 5; ++i) {
      climberSpinMotorStatus = climberSpinMotor.getConfigurator().apply(climberSpinMotorConfig);
      if (climberSpinMotorStatus.isOK()) break;
    }
    if (!climberSpinMotorStatus.isOK()) {
      System.out.println("Could not configure device. Error: " + climberSpinMotorStatus.toString());
    }
    climberSpinMotor.setPosition(0);
  }

  public void configureclimbRotateMotor(TalonFX climbRotateMotor){
    TalonFXConfiguration climbRotateMotorConfig = new TalonFXConfiguration();

    climbRotateMotorConfig.CurrentLimits.SupplyCurrentLimit = Constants.kClimbRotateMotorSupplyCurrentLimit;
    climbRotateMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;    

    climbRotateMotorConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = Constants.kClimbRotateMotorClosedLoopRampPeriod;
    climbRotateMotorConfig.Voltage.PeakForwardVoltage = Constants.kClimbRotateMotorPeakForwardVoltage;
    climbRotateMotorConfig.Voltage.PeakReverseVoltage = Constants.kClimbRotateMotorPeakReverseVoltage;

    climbRotateMotorConfig.MotorOutput.Inverted = Constants.kClimbRotateMotorDirection;
    climbRotateMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
  

    Slot0Configs slot0 = climbRotateMotorConfig.Slot0;
    slot0.kP = Constants.kClimbRotateMotorProportional;
    slot0.kI = Constants.kClimbRotateMotorIntegral;
    slot0.kD = Constants.kClimbRotateMotorDerivative;
    slot0.kV = Constants.kClimbRotateMotorVelocityFeedForward;
    
    slot0.GravityType = GravityTypeValue.Arm_Cosine;
    slot0.kG = Constants.kClimbRotateMotorGravityFeedForward;
    slot0.kS = Constants.kClimbRotateMotorStaticFeedForward;
    

    
    StatusCode climbRotateMotorStatus = StatusCode.StatusCodeNotInitialized;
    for(int i = 0; i < 5; ++i) {
      climbRotateMotorStatus = climbRotateMotor.getConfigurator().apply(climbRotateMotorConfig);
      if (climbRotateMotorStatus.isOK()) break;
    }
    if (!climbRotateMotorStatus.isOK()) {
      System.out.println("Could not configure device. Error: " + climbRotateMotorStatus.toString());
    }
    climbRotateMotor.setPosition(0);
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
