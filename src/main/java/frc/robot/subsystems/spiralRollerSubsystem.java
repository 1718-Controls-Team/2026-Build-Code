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

public class spiralRollerSubsystem extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
    
    TalonFX spiralRollerSpinMotor = new TalonFX(9);

    PositionVoltage intakePosition = new PositionVoltage(0);

    private VelocityVoltage voltageRequest = new VelocityVoltage(0);

  public spiralRollerSubsystem() {
    this.configureSpiralRollerSpinMotor(spiralRollerSpinMotor);
  }

  

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public void setSpiralRollerSpinSpeed(double SpiralRollerSpeed) {
    spiralRollerSpinMotor.setControl(voltageRequest.withVelocity(SpiralRollerSpeed));
  }

   public double getSpiralRollerSpinSpeed() {
    return spiralRollerSpinMotor.getVelocity().getValueAsDouble();
  }

  public void configureSpiralRollerSpinMotor(TalonFX SpiralRollerSpinMotor){
    TalonFXConfiguration SpiralRollerSpinMotorConfig = new TalonFXConfiguration();

    SpiralRollerSpinMotorConfig.CurrentLimits.SupplyCurrentLimit = Constants.kSpiralRollerSpinMotorSupplyCurrentLimit;
    SpiralRollerSpinMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;    

    SpiralRollerSpinMotorConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = Constants.kSpiralRollerSpinMotorClosedLoopRampPeriod;
    SpiralRollerSpinMotorConfig.Voltage.PeakForwardVoltage = Constants.kSpiralRollerSpinMotorPeakForwardVoltage;
    SpiralRollerSpinMotorConfig.Voltage.PeakReverseVoltage = Constants.kSpiralRollerSpinMotorPeakReverseVoltage;

    SpiralRollerSpinMotorConfig.MotorOutput.Inverted = Constants.kSpiralRollerSpinMotorDirection;
    SpiralRollerSpinMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
  

    Slot0Configs slot0 = SpiralRollerSpinMotorConfig.Slot0;
    slot0.kP = Constants.kSpiralRollerSpinMotorProportional;
    slot0.kI = Constants.kSpiralRollerSpinMotorIntegral;
    slot0.kD = Constants.kSpiralRollerSpinMotorDerivative;
    slot0.kV = Constants.kSpiralRollerSpinMotorVelocityFeedForward;
    
    slot0.GravityType = GravityTypeValue.Arm_Cosine;
    slot0.kG = Constants.kSpiralRollerSpinMotorGravityFeedForward;
    slot0.kS = Constants.kSpiralRollerSpinMotorStaticFeedForward;
    

    
    StatusCode SpiralRollerSpinMotorStatus = StatusCode.StatusCodeNotInitialized;
    for(int i = 0; i < 5; ++i) {
      SpiralRollerSpinMotorStatus = SpiralRollerSpinMotor.getConfigurator().apply(SpiralRollerSpinMotorConfig);
      if (SpiralRollerSpinMotorStatus.isOK()) break;
    }
    if (!SpiralRollerSpinMotorStatus.isOK()) {
      System.out.println("Could not configure device. Error: " + SpiralRollerSpinMotorStatus.toString());
    }
    SpiralRollerSpinMotor.setPosition(0);
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
