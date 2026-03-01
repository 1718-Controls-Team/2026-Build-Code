package frc.robot.commands;



import frc.robot.subsystems.climber;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;


/** An example command that uses an example subsystem. */
public class releaseClimb extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final climber m_climberSubsystem;
  
    private boolean m_isFinished = false;
  
  
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
      m_climberSubsystem.setclimbRotatePos(Constants.kClimbRotateDownPos);
    }


  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
   
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