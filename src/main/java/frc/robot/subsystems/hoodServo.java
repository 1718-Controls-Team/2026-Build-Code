package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Servo;


/**
 * The 2017 turret shooter hood subsystem
 * @author Peter Dentch
 */
public class hoodServo extends SubsystemBase {

    private static Servo hoodServo1;
    private static Servo hoodServo2;

    
    //SERVO Parameters from https://s3.amazonaws.com/actuonix/Actuonix+L16+Datasheet.pdf
    
    // pwm values in ms for the max and min angles of the shooter hood
    
    /**
     * Default constructor for the subsystem 
     */
    public hoodServo(){
    	hoodServo1 = new Servo(1);
    	hoodServo2 = new Servo(0);

    	
		
    }
	
    /**
     * Returns the shooter hood singleton object
     * @return is the current shooter hood object
     */
	
    /**
     * Takes a given angle and rotates the servo motor to that angle
     * @param degrees the angle limited by the min and max values defined in RobotMap
     */
    public void setPos(double degrees){
	hoodServo1.set(degrees);
    hoodServo2.set(degrees);
    }

    /**
     * Returns the current angle of the servo by taking the angle it was last set to
     * with the time before the movement begins and after that is called the current
     * time and angle the servo is moving to is taken and the current angle is estimated
     * @return the estimated current angle of the servo in degrees
     */
    public double getPos(){
		
	return hoodServo1.getAngle();
		
	
//	return currentAngle;
    }
	
    /**
     * Sets the default command of the subsystem
     */
    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
    	//setDefaultCommand(new DriveHoodWithJoystick());
    }
}
