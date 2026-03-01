package frc.robot;

import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;


public class Constants {
    public static final double[] kBlueHubCoord = {4.627, 4.024};
    public static final double[] kRedHubCoord = {4.627, 4.024};
    public static final double[] kBluePassCoords = {1.905, 6.222, 2.059};
    
    // key - distance    value - hood angle
    public static final InterpolatingDoubleTreeMap kHoodTable = new InterpolatingDoubleTreeMap();
    static {
        kHoodTable.put(1.0, 2.0);
        kHoodTable.put(2.0, 3.0);
    }


    // key - distance    value - speed
    public static final InterpolatingDoubleTreeMap kSpeedTable = new InterpolatingDoubleTreeMap();
    static {
        kSpeedTable.put(1.0, 2.0);
        kSpeedTable.put(2.0, 3.0);
    }

    // This is only necessary for shooting on move
    // The value is the time from shooting to the time that the shot lands
    // key - distance    value - time till shot lands
    public static final InterpolatingDoubleTreeMap kShotTimeTable = new InterpolatingDoubleTreeMap();
    static {
        kShotTimeTable.put(1.0, 2.0);
        kShotTimeTable.put(2.0, 3.0);
    }

    // limelight forward pos according to intake position
    public static final InterpolatingDoubleTreeMap kLLForwardPos = new InterpolatingDoubleTreeMap();
    static {
        kLLForwardPos.put(2.0, 0.4);
    }
    public static final double kAccelCompFactor = 0.1;

/*
 * ####################################################################################################################################
 * ##################################################### Positions ###################################################################
 * ####################################################################################################################################
*/

    public static final double kIndexerMainSpeed = 20;
    public static final double kIndexerNoSpeed = 0;
    public static final double kShooterOutSpeed = 70;

    public static final double kIntakeInSpeed = 20;
    public static final double kIntakeIdleSpeed = 10;
    public static final double kIntakeSlideInPos = 12;
    public static final double kIntakeSlideOutPos = 0.5;

    public static final double kClimbRotateOutPos = 13;
    public static final double kClimbRotateDownPos = 0;
    public static final double kClimbUpSpeed = 30;

    public static final double kHoodServoDownPos = 0;



/*
 * ####################################################################################################################################
 * ####################################################### Intake Spin ###############################################################
 * ####################################################################################################################################
*/
    
    public static final double kIntakeSpinMotorProportional = 0;
    public static final double kIntakeSpinMotorIntegral = 0;
    public static final double kIntakeSpinMotorDerivative = 0;

    public static final double kIntakeSpinMotorVelocityFeedForward = 0.122;
    public static final double kIntakeSpinMotorStaticFeedForward = 0;
    public static final double kIntakeSpinMotorGravityFeedForward = 0;

    public static final double kIntakeSpinMotorPeakForwardVoltage = 11;
    public static final double kIntakeSpinMotorPeakReverseVoltage = -11;

    public static final InvertedValue kIntakeSpinMotorDirection = InvertedValue.Clockwise_Positive;
    public static final double kIntakeSpinMotorSupplyCurrentLimit = 40;

    public static final double kIntakeSpinMotorClosedLoopRampPeriod = 0;
    
  
/*
 * ####################################################################################################################################
 * ####################################################### Intake Slide ###############################################################
 * ####################################################################################################################################
*/

    public static final double kIntakeElectricSlideMotorProportional = 0;
    public static final double kIntakeElectricSlideMotorIntegral = 0;
    public static final double kIntakeElectricSlideMotorDerivative = 0;    

    public static final double kIntakeElectricSlideMotorVelocityFeedForward = 0.2;
    public static final double kIntakeElectricSlideMotorGravityFeedForward = 0;

    public static final double kIntakeElectricSlideMotorMaxForwardVoltage = 11;
    public static final double kIntakeElectricSlideMotorMaxReverseVoltage = -11;

    public static final double kIntakeElectricSlideMotorVoltageClosedLoopRampPeriod = 0;
    public static final InvertedValue kIntakeElectricSlideMotorDirection = InvertedValue.Clockwise_Positive;
    public static final double kIntakeElectricSlideMotorSupplyCurrentLimit = 10;
    


/*
 * ####################################################################################################################################
 * ####################################################### Spiral Roller ###############################################################
 * ####################################################################################################################################
*/
    
    public static final double kSpiralRollerSpinMotorProportional = 0;
    public static final double kSpiralRollerSpinMotorIntegral = 0;
    public static final double kSpiralRollerSpinMotorDerivative = 0;

    public static final double kSpiralRollerSpinMotorVelocityFeedForward = 0.122;
    public static final double kSpiralRollerSpinMotorStaticFeedForward = 0;
    public static final double kSpiralRollerSpinMotorGravityFeedForward = 0;

    public static final double kSpiralRollerSpinMotorPeakForwardVoltage = 11;
    public static final double kSpiralRollerSpinMotorPeakReverseVoltage = -11;

    public static final InvertedValue kSpiralRollerSpinMotorDirection = InvertedValue.Clockwise_Positive;
    public static final double kSpiralRollerSpinMotorSupplyCurrentLimit = 40;

    public static final double kSpiralRollerSpinMotorClosedLoopRampPeriod = 0;
    
  
/*
 * ####################################################################################################################################
 * ####################################################### Shooter Spin ###############################################################
 * ####################################################################################################################################
*/

    public static final double kShooterSpinMotorProportional = 1;
    public static final double kShooterSpinMotorIntegral = 0;
    public static final double kShooterSpinMotorDerivative = 0;

    public static final double kShooterSpinMotorGravityFeedForward = 0;
    public static final double kShooterSpinMotorVelocityFeedForward = 0.112;
    public static final double kShooterSpinMotorStaticFeedForward = 0;

    public static final double kShooterSpinMotorPeakForwardVoltage = 11;
    public static final double kShooterSpinMotorPeakReverseVoltage = -11;

    public static final InvertedValue kShooterSpinMotorDirection = InvertedValue.Clockwise_Positive;
    public static final double kShooterSpinMotorSupplyCurrentLimit = 40;

    public static final double kShooterSpinMotorClosedLoopRampPeriod = 0;
    
    

/*
 * ####################################################################################################################################
 * ####################################################### Indexer ###############################################################
 * ####################################################################################################################################
*/

    public static final double kIndexerMotorProportional = 0;
    public static final double kIndexerMotorIntegral = 0;
    public static final double kIndexerMotorDerivative = 0;

    public static final double kIndexerMotorGravityFeedForward = 0;
    public static final double kIndexerMotorVelocityFeedForward = 0.112;
    public static final double kIndexerMotorStaticFeedForward = 0;

    public static final double kIndexerMotorPeakForwardVoltage = 11;
    public static final double kIndexerMotorPeakReverseVoltage = -11;

    public static final InvertedValue kIndexerMotorDirection = InvertedValue.Clockwise_Positive;
    public static final double kIndexerMotorSupplyCurrentLimit = 40;

    public static final double kIndexerMotorClosedLoopRampPeriod = 0;
    

/*
 * ####################################################################################################################################
 * ####################################################### Turret ###############################################################
 * ####################################################################################################################################
*/

    public static final double kTurretMotorProportional = 0;
    public static final double kTurretMotorIntegral = 0;
    public static final double kTurretMotorDerivative = 0;

    public static final double kTurretMotorGravityFeedForward = 0;
    public static final double kTurretMotorVelocityFeedForward = 0.2;
    
    public static final double kTurretMotorPeakForwardVoltage = 11;
    public static final double kTurretMotorPeakReverseVoltage = -11;

    public static final InvertedValue kTurretMotorDirection = InvertedValue.Clockwise_Positive;
    public static final double kTurretMotorSupplyCurrentLimit = 40;

    public static final double kTurretMotorClosedLoopRampPeriod = 0;
    

/*
 * ####################################################################################################################################
 * ####################################################### Climber Spin ###############################################################
 * ####################################################################################################################################
*/

    public static final double kClimberSpinMotorProportional = 0;
    public static final double kClimberSpinMotorIntegral = 0;
    public static final double kClimberSpinMotorDerivative = 0;
    
    public static final double kClimberSpinMotorGravityFeedForward = 0;
    public static final double kClimberSpinMotorVelocityFeedForward = 0.112;
    public static final double kClimberSpinMotorStaticFeedForward = 0;

    public static final double kClimberSpinMotorPeakForwardVoltage = 11;
    public static final double kClimberSpinMotorPeakReverseVoltage = -11;

    public static final InvertedValue kClimberSpinMotorDirection = InvertedValue.Clockwise_Positive;
    public static final double kClimberSpinMotorSupplyCurrentLimit = 40;

    public static final double kClimberSpinMotorClosedLoopRampPeriod = 0;
    
    
    
/*
 * ####################################################################################################################################
 * ####################################################### Climb Rotate ###############################################################
 * ####################################################################################################################################
*/

    public static final double kClimbRotateMotorProportional = 0;
    public static final double kClimbRotateMotorIntegral = 0;
    public static final double kClimbRotateMotorDerivative = 0;

    public static final double kClimbRotateMotorGravityFeedForward = 0;
    public static final double kClimbRotateMotorVelocityFeedForward = 0.2;

    public static final double kClimbRotateMotorPeakForwardVoltage = 11;
    public static final double kClimbRotateMotorPeakReverseVoltage = -11;

    public static final InvertedValue kClimbRotateMotorDirection = InvertedValue.Clockwise_Positive;
    public static final double kClimbRotateMotorSupplyCurrentLimit = 40;

    public static final double kClimbRotateMotorClosedLoopRampPeriod = 0;
    
    
    
}
