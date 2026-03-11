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

public class shooterIndexer extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
    
    TalonFX leftShooterMotor = new TalonFX(20, Constants.kCanivore);
    TalonFX leftIndexerMotor = new TalonFX(19, Constants.kCanivore);
    TalonFX rightShooterMotor = new TalonFX(17, Constants.kCanivore);
    TalonFX rightIndexerMotor = new TalonFX(16, Constants.kCanivore);

    PositionVoltage shooterPosition = new PositionVoltage(0);

    DutyCycleOut ShooterVoltage = new DutyCycleOut(0);

    private VelocityVoltage voltageRequest = new VelocityVoltage(0);

  public shooterIndexer() {
    this.configureleftShooterMotor(leftShooterMotor);
    this.configurerightShooterMotor(rightShooterMotor);
    this.configureleftIndexerMotor(leftIndexerMotor);
    this.configurerightIndexerMotor(rightIndexerMotor);

  }

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public void setShooterSpinSpeed(double shooterSpeed) {
    leftShooterMotor.setControl(voltageRequest.withVelocity(shooterSpeed));
    rightShooterMotor.setControl(voltageRequest.withVelocity(shooterSpeed));

  }
  public void setShooterOff(double output){
    leftIndexerMotor.setControl(ShooterVoltage.withOutput(output));
    leftShooterMotor.setControl(ShooterVoltage.withOutput(output));
    rightIndexerMotor.setControl(ShooterVoltage.withOutput(output));
    rightShooterMotor.setControl(ShooterVoltage.withOutput(output));

  }
  
   public double getShooterSpeed(){
    return leftShooterMotor.getVelocity().getValueAsDouble();
  }

  public void setIndexerSpinSpeed(double indexerSpeed){
    leftIndexerMotor.setControl(voltageRequest.withVelocity(indexerSpeed));
    rightIndexerMotor.setControl(voltageRequest.withVelocity(indexerSpeed));

  }

  public double getIndexerSpeed(){
    return leftIndexerMotor.getVelocity().getValueAsDouble();
  }

  
//######################################### Start OF SHOOTER CONFIGURATION ######################################################
//######################################### Start OF SHOOTER CONFIGURATION ######################################################
//######################################### Start OF SHOOTER CONFIGURATION ###################################################### 

//######################################### SHOOTER LEFT CONFIGURATION ###################################################### 


  public void configureleftShooterMotor(TalonFX leftShooterMotor){
    TalonFXConfiguration leftShooterMotorConfig = new TalonFXConfiguration();

    leftShooterMotorConfig.CurrentLimits.SupplyCurrentLimit = Constants.kLeftShooterMotorSupplyCurrentLimit;
    leftShooterMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;    

    leftShooterMotorConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = Constants.kLeftShooterMotorClosedLoopRampPeriod;
    leftShooterMotorConfig.Voltage.PeakForwardVoltage = Constants.kLeftShooterMotorPeakForwardVoltage;
    leftShooterMotorConfig.Voltage.PeakReverseVoltage = Constants.kLeftShooterMotorPeakReverseVoltage;

    leftShooterMotorConfig.MotorOutput.Inverted = Constants.kLeftShooterMotorDirection;
    leftShooterMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
  

    Slot0Configs slot0 = leftShooterMotorConfig.Slot0;
    slot0.kP = Constants.kLeftShooterMotorProportional;
    slot0.kI = Constants.kLeftShooterMotorIntegral;
    slot0.kD = Constants.kLeftShooterMotorDerivative;
    slot0.kV = Constants.kLeftShooterMotorVelocityFeedForward;
    
    slot0.GravityType = GravityTypeValue.Arm_Cosine;
    slot0.kG = Constants.kLeftShooterMotorGravityFeedForward;
    slot0.kS = Constants.kLeftShooterMotorStaticFeedForward;
    

    
    StatusCode leftShooterMotorStatus = StatusCode.StatusCodeNotInitialized;
    for(int i = 0; i < 5; ++i) {
      leftShooterMotorStatus = leftShooterMotor.getConfigurator().apply(leftShooterMotorConfig);
      if (leftShooterMotorStatus.isOK()) break;
    }
    if (!leftShooterMotorStatus.isOK()) {
      System.out.println("Could not configure device. Error: " + leftShooterMotorStatus.toString());
    }
  }
  //######################################### SHOOTER RIGHT CONFIGURATION ###################################################### 


  public void configurerightShooterMotor(TalonFX rightShooterMotor){
    TalonFXConfiguration rightShooterMotorConfig = new TalonFXConfiguration();

    rightShooterMotorConfig.CurrentLimits.SupplyCurrentLimit = Constants.kRightShooterMotorSupplyCurrentLimit;
    rightShooterMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;    

    rightShooterMotorConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = Constants.kRightShooterMotorClosedLoopRampPeriod;
    rightShooterMotorConfig.Voltage.PeakForwardVoltage = Constants.kRightShooterMotorPeakForwardVoltage;
    rightShooterMotorConfig.Voltage.PeakReverseVoltage = Constants.kRightShooterMotorPeakReverseVoltage;

    rightShooterMotorConfig.MotorOutput.Inverted = Constants.kRightShooterMotorDirection;
    rightShooterMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
  

    Slot0Configs slot0 = rightShooterMotorConfig.Slot0;
    slot0.kP = Constants.kRightShooterMotorProportional;
    slot0.kI = Constants.kRightShooterMotorIntegral;
    slot0.kD = Constants.kRightShooterMotorDerivative;
    slot0.kV = Constants.kRightShooterMotorVelocityFeedForward;
    
    slot0.GravityType = GravityTypeValue.Arm_Cosine;
    slot0.kG = Constants.kRightShooterMotorGravityFeedForward;
    slot0.kS = Constants.kRightShooterMotorStaticFeedForward;
    

    
    StatusCode rightShooterMotorStatus = StatusCode.StatusCodeNotInitialized;
    for(int i = 0; i < 5; ++i) {
      rightShooterMotorStatus = rightShooterMotor.getConfigurator().apply(rightShooterMotorConfig);
      if (rightShooterMotorStatus.isOK()) break;
    }
    if (!rightShooterMotorStatus.isOK()) {
      System.out.println("Could not configure device. Error: " + rightShooterMotorStatus.toString());
    }
  }
//######################################### Start OF INDEXER CONFIGURATION ######################################################
//######################################### Start OF INDEXER CONFIGURATION ######################################################
//######################################### Start OF INDEXER CONFIGURATION ###################################################### 

//######################################### INDEXER LEFT CONFIGURATION ###################################################### 

  public void configureleftIndexerMotor(TalonFX leftIndexerMotor){
    TalonFXConfiguration leftIndexerMotorConfig = new TalonFXConfiguration();

    leftIndexerMotorConfig.MotorOutput.Inverted = Constants.kLeftIndexerMotorDirection;
    leftIndexerMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;


    leftIndexerMotorConfig.CurrentLimits.SupplyCurrentLimit = Constants.kLeftIndexerMotorSupplyCurrentLimit;
    leftIndexerMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    leftIndexerMotorConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = Constants.kLeftIndexerMotorClosedLoopRampPeriod;
    leftIndexerMotorConfig.Voltage.PeakForwardVoltage = Constants.kLeftIndexerMotorPeakForwardVoltage;
    leftIndexerMotorConfig.Voltage.PeakReverseVoltage = Constants.kLeftIndexerMotorPeakReverseVoltage;
    

    Slot0Configs slot0 = leftIndexerMotorConfig.Slot0;
    slot0.kP = Constants.kLeftIndexerMotorProportional;
    slot0.kI = Constants.kLeftIndexerMotorIntegral;
    slot0.kD = Constants.kLeftIndexerMotorDerivative;

    slot0.GravityType = GravityTypeValue.Elevator_Static;
    slot0.kV = Constants.kLeftIndexerMotorVelocityFeedForward;
    slot0.kG = Constants.kLeftIndexerMotorGravityFeedForward;
    slot0.kS = Constants.kLeftIndexerMotorStaticFeedForward;
 


    StatusCode leftIndexerMotorStatus = StatusCode.StatusCodeNotInitialized;
    for(int i = 0; i < 5; ++i) {
      leftIndexerMotorStatus = leftIndexerMotor.getConfigurator().apply(leftIndexerMotorConfig);
      if (leftIndexerMotorStatus.isOK()) break;
    }
    if (!leftIndexerMotorStatus.isOK()) {
      System.out.println("Could not configure device. Error: " + leftIndexerMotorStatus.toString());
    }
  }
 //######################################### INDEXER LEFT CONFIGURATION ###################################################### 

  public void configurerightIndexerMotor(TalonFX rightIndexerMotor){
    TalonFXConfiguration rightIndexerMotorConfig = new TalonFXConfiguration();

    rightIndexerMotorConfig.MotorOutput.Inverted = Constants.kRightIndexerMotorDirection;
    rightIndexerMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;


    rightIndexerMotorConfig.CurrentLimits.SupplyCurrentLimit = Constants.kRightIndexerMotorSupplyCurrentLimit;
    rightIndexerMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    rightIndexerMotorConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = Constants.kRightIndexerMotorClosedLoopRampPeriod;
    rightIndexerMotorConfig.Voltage.PeakForwardVoltage = Constants.kRightIndexerMotorPeakForwardVoltage;
    rightIndexerMotorConfig.Voltage.PeakReverseVoltage = Constants.kRightIndexerMotorPeakReverseVoltage;
    

    Slot0Configs slot0 = rightIndexerMotorConfig.Slot0;
    slot0.kP = Constants.kRightIndexerMotorProportional;
    slot0.kI = Constants.kRightIndexerMotorIntegral;
    slot0.kD = Constants.kRightIndexerMotorDerivative;

    slot0.GravityType = GravityTypeValue.Elevator_Static;
    slot0.kV = Constants.kRightIndexerMotorVelocityFeedForward;
    slot0.kG = Constants.kRightIndexerMotorGravityFeedForward;
    slot0.kS = Constants.kRightIndexerMotorStaticFeedForward;
 


    StatusCode rightIndexerMotorStatus = StatusCode.StatusCodeNotInitialized;
    for(int i = 0; i < 5; ++i) {
      rightIndexerMotorStatus = rightIndexerMotor.getConfigurator().apply(rightIndexerMotorConfig);
      if (rightIndexerMotorStatus.isOK()) break;
    }
    if (!rightIndexerMotorStatus.isOK()) {
      System.out.println("Could not configure device. Error: " + rightIndexerMotorStatus.toString());
    }
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
