package frc.robot.commands.Climb;



import frc.robot.subsystems.climber;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.Timer;


/** An example command that uses an example subsystem. */
public class releaseClimb extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final climber m_climberSubsystem;
  
    private boolean m_isFinished = false;
    private Timer climbTime;
  
  
    /**
     * Creates a new set-PowerCommand.
     *
     * @param subsystem The subsystem used by this command.
     */
    public releaseClimb(climber climberSubsystem) {
      m_climberSubsystem = climberSubsystem;
  
     
      // Use addRequirements() here to declare subsystem dependencies.
      addRequirements(climberSubsystem);
      
          
    }
   
    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
      m_climberSubsystem.setclimberSpinSpeed(0);
      climbTime.reset();
      climbTime.start();
    }


  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
   if(climbTime.get() >= 2){
      m_climberSubsystem.setclimbRotatePos(Constants.kClimbRotateDownPos);
      }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  
}
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_isFinished;
  }
}