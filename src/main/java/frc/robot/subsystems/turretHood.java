// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class turretHood extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
    
    TalonFX leftTurretMotor = new TalonFX(21, Constants.kCanivore);
    TalonFX rightTurretMotor = new TalonFX(18, Constants.kCanivore);


    PositionVoltage turretPosition = new PositionVoltage(0);

    private VelocityVoltage voltageRequest = new VelocityVoltage(0);

  public turretHood() {
    this.configureLeftTurretMotor(leftTurretMotor);
    this.configureRightTurretMotor(rightTurretMotor);

  }

  /**
   * Example command factory method.
   *
   * @return a command
   */
 


  public void setTurretMotorPos(double turretPos){
    if (turretPos > -.2 && turretPos < .35) {
    leftTurretMotor.setControl(turretPosition.withPosition(turretPos));
    rightTurretMotor.setControl(turretPosition.withPosition(turretPos));
     }
  } 

  public double getTurretMotorPos(){
    return leftTurretMotor.getPosition().getValueAsDouble();
  }


  /*public void setTurretMotor(double indexerSpeed){
    indexerSpinMotor.setControl(voltageRequest.withVelocity(indexerSpeed));
  } this wont be used bc the turret will auto adjust with april tags */

//######################################### Start OF TURRET CONFIGURATION ######################################################
//######################################### Start OF TURRET CONFIGURATION ######################################################
//######################################### Start OF TURRET CONFIGURATION ###################################################### 

//######################################### TURRET LEFT CONFIGURATION #######################################################

  public void configureLeftTurretMotor(TalonFX leftTurretMotor){
    TalonFXConfiguration leftTurretMotorConfig = new TalonFXConfiguration();

    leftTurretMotorConfig.CurrentLimits.SupplyCurrentLimit = Constants.kLeftTurretMotorSupplyCurrentLimit;
    leftTurretMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;    

    leftTurretMotorConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = Constants.kLeftTurretMotorClosedLoopRampPeriod;
    leftTurretMotorConfig.Voltage.PeakForwardVoltage = Constants.kLeftTurretMotorPeakForwardVoltage;
    leftTurretMotorConfig.Voltage.PeakReverseVoltage = Constants.kLeftTurretMotorPeakReverseVoltage;

    leftTurretMotorConfig.MotorOutput.Inverted = Constants.kLeftTurretMotorDirection;
    leftTurretMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
  

    Slot0Configs slot0 = leftTurretMotorConfig.Slot0;
    slot0.kP = Constants.kLeftTurretMotorProportional;
    slot0.kI = Constants.kLeftTurretMotorIntegral;
    slot0.kD = Constants.kLeftTurretMotorDerivative;
    
    slot0.GravityType = GravityTypeValue.Arm_Cosine;
    slot0.kG = Constants.kLeftTurretMotorGravityFeedForward;
    slot0.kV = Constants.kLeftTurretMotorVelocityFeedForward;

    
    StatusCode leftTurretMotorStatus = StatusCode.StatusCodeNotInitialized;
    for(int i = 0; i < 5; ++i) {
      leftTurretMotorStatus = leftTurretMotor.getConfigurator().apply(leftTurretMotorConfig);
      if (leftTurretMotorStatus.isOK()) break;
    }
    if (!leftTurretMotorStatus.isOK()) {
      System.out.println("Could not configure device. Error: " + leftTurretMotorStatus.toString());
    }
    leftTurretMotor.setPosition(0);
  }
  
//######################################### TURRET LEFT CONFIGURATION #######################################################

  public void configureRightTurretMotor(TalonFX rightTurretMotor){
    TalonFXConfiguration rightTurretMotorConfig = new TalonFXConfiguration();

    rightTurretMotorConfig.CurrentLimits.SupplyCurrentLimit = Constants.kRightTurretMotorSupplyCurrentLimit;
    rightTurretMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;    

    rightTurretMotorConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = Constants.kRightTurretMotorClosedLoopRampPeriod;
    rightTurretMotorConfig.Voltage.PeakForwardVoltage = Constants.kRightTurretMotorPeakForwardVoltage;
    rightTurretMotorConfig.Voltage.PeakReverseVoltage = Constants.kRightTurretMotorPeakReverseVoltage;

    rightTurretMotorConfig.MotorOutput.Inverted = Constants.kRightTurretMotorDirection;
    rightTurretMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
  

    Slot0Configs slot0 = rightTurretMotorConfig.Slot0;
    slot0.kP = Constants.kRightTurretMotorProportional;
    slot0.kI = Constants.kRightTurretMotorIntegral;
    slot0.kD = Constants.kRightTurretMotorDerivative;
    
    slot0.GravityType = GravityTypeValue.Arm_Cosine;
    slot0.kG = Constants.kRightTurretMotorGravityFeedForward;
    slot0.kV = Constants.kRightTurretMotorVelocityFeedForward;

    
    StatusCode rightTurretMotorStatus = StatusCode.StatusCodeNotInitialized;
    for(int i = 0; i < 5; ++i) {
      rightTurretMotorStatus = rightTurretMotor.getConfigurator().apply(rightTurretMotorConfig);
      if (rightTurretMotorStatus.isOK()) break;
    }
    if (!rightTurretMotorStatus.isOK()) {
      System.out.println("Could not configure device. Error: " + rightTurretMotorStatus.toString());
    }
    rightTurretMotor.setPosition(0);
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
