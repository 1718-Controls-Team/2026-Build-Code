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

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class spiralRoller extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
    
    TalonFX rightRollerSpinMotor = new TalonFX(15, Constants.kCanivore);
    TalonFX leftRollerSpinMotor = new TalonFX(24, Constants.kCanivore);

    PositionVoltage intakePosition = new PositionVoltage(0);

    DutyCycleOut RollerVoltage = new DutyCycleOut(0);

    private VelocityVoltage voltageRequest = new VelocityVoltage(0);

  public spiralRoller() {
    this.configureLeftRollerSpinMotor(leftRollerSpinMotor);
    this.configureRightRollerSpinMotor(rightRollerSpinMotor);  
  }

  

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public void setSpiralRollerSpinSpeed(double rollerSpeed) {
    leftRollerSpinMotor.setControl(voltageRequest.withVelocity(rollerSpeed));
    rightRollerSpinMotor.setControl(voltageRequest.withVelocity(rollerSpeed));

  }

  public void setSpiralRollerOff(double output) {
    leftRollerSpinMotor.setControl(RollerVoltage.withOutput(output));
    rightRollerSpinMotor.setControl(RollerVoltage.withOutput(output));
  }

   public double getSpiralRollerSpinSpeed() {
    return leftRollerSpinMotor.getVelocity().getValueAsDouble();
  }
//######################################### Start OF ROLLER CONFIGURATION ######################################################
//######################################### Start OF ROLLER CONFIGURATION ######################################################
//######################################### Start OF ROLLER CONFIGURATION ###################################################### 

//######################################### ROLLER LEFT CONFIGURATION ###################################################### 

  public void configureLeftRollerSpinMotor(TalonFX LeftRollerSpinMotor){
    TalonFXConfiguration leftRollerSpinMotorConfig = new TalonFXConfiguration();

    leftRollerSpinMotorConfig.CurrentLimits.SupplyCurrentLimit = Constants.kLeftRollerSpinMotorSupplyCurrentLimit;
    leftRollerSpinMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;    

    leftRollerSpinMotorConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = Constants.kLeftRollerSpinMotorClosedLoopRampPeriod;
    leftRollerSpinMotorConfig.Voltage.PeakForwardVoltage = Constants.kLeftRollerSpinMotorPeakForwardVoltage;
    leftRollerSpinMotorConfig.Voltage.PeakReverseVoltage = Constants.kLeftRollerSpinMotorPeakReverseVoltage;

    leftRollerSpinMotorConfig.MotorOutput.Inverted = Constants.kLeftRollerSpinMotorDirection;
    leftRollerSpinMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
  

    Slot0Configs slot0 = leftRollerSpinMotorConfig.Slot0;
    slot0.kP = Constants.kLeftRollerSpinMotorProportional;
    slot0.kI = Constants.kLeftRollerSpinMotorIntegral;
    slot0.kD = Constants.kLeftRollerSpinMotorDerivative;
    slot0.kV = Constants.kLeftRollerSpinMotorVelocityFeedForward;
    
    slot0.GravityType = GravityTypeValue.Arm_Cosine;
    slot0.kG = Constants.kLeftRollerSpinMotorGravityFeedForward;
    slot0.kS = Constants.kLeftRollerSpinMotorStaticFeedForward;
    

    
    StatusCode LeftRollerSpinMotorStatus = StatusCode.StatusCodeNotInitialized;
    for(int i = 0; i < 5; ++i) {
      LeftRollerSpinMotorStatus = LeftRollerSpinMotor.getConfigurator().apply(leftRollerSpinMotorConfig);
      if (LeftRollerSpinMotorStatus.isOK()) break;
    }
    if (!LeftRollerSpinMotorStatus.isOK()) {
      System.out.println("Could not configure device. Error: " + LeftRollerSpinMotorStatus.toString());
    }
    LeftRollerSpinMotor.setPosition(0);
  }
//######################################### ROLLER RIGHT CONFIGURATION ###################################################### 

  public void configureRightRollerSpinMotor(TalonFX RightRollerSpinMotor){
    TalonFXConfiguration rightRollerSpinMotorConfig = new TalonFXConfiguration();

    rightRollerSpinMotorConfig.CurrentLimits.SupplyCurrentLimit = Constants.kRightRollerSpinMotorSupplyCurrentLimit;
    rightRollerSpinMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;    

    rightRollerSpinMotorConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = Constants.kRightRollerSpinMotorClosedLoopRampPeriod;
    rightRollerSpinMotorConfig.Voltage.PeakForwardVoltage = Constants.kRightRollerSpinMotorPeakForwardVoltage;
    rightRollerSpinMotorConfig.Voltage.PeakReverseVoltage = Constants.kRightRollerSpinMotorPeakReverseVoltage;

    rightRollerSpinMotorConfig.MotorOutput.Inverted = Constants.kRightRollerSpinMotorDirection;
    rightRollerSpinMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
  

    Slot0Configs slot0 = rightRollerSpinMotorConfig.Slot0;
    slot0.kP = Constants.kRightRollerSpinMotorProportional;
    slot0.kI = Constants.kRightRollerSpinMotorIntegral;
    slot0.kD = Constants.kRightRollerSpinMotorDerivative;
    slot0.kV = Constants.kRightRollerSpinMotorVelocityFeedForward;
    
    slot0.GravityType = GravityTypeValue.Arm_Cosine;
    slot0.kG = Constants.kRightRollerSpinMotorGravityFeedForward;
    slot0.kS = Constants.kRightRollerSpinMotorStaticFeedForward;
    

    
    StatusCode RightRollerSpinMotorStatus = StatusCode.StatusCodeNotInitialized;
    for(int i = 0; i < 5; ++i) {
      RightRollerSpinMotorStatus = RightRollerSpinMotor.getConfigurator().apply(rightRollerSpinMotorConfig);
      if (RightRollerSpinMotorStatus.isOK()) break;
    }
    if (!RightRollerSpinMotorStatus.isOK()) {
      System.out.println("Could not configure device. Error: " + RightRollerSpinMotorStatus.toString());
    }
    RightRollerSpinMotor.setPosition(0);
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
