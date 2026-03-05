// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.Console;

import com.ctre.phoenix6.HootAutoReplay;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.ShiftConstants.shifts;
import frc.robot.generated.TunerConstants;
public class Robot extends TimedRobot {
    private double teleopstart = 0;
    private double lastteleopstart = 0;
    private double countdownTimer =0;
    private double currentcountdown = 0;
    private shifts currentShift = shifts.SHIFT_1;

    private Command m_autonomousCommand;
    private final CommandXboxController joystick = new CommandXboxController(0);
    private final RobotContainer m_robotContainer;
    private SwerveDrivetrainConstants drivetrainconstants = TunerConstants.DrivetrainConstants;
    private Pigeon2 gyro = new Pigeon2(drivetrainconstants.Pigeon2Id);
    /* log and replay timestamp and joystick data */
    private final HootAutoReplay m_timeAndJoystickReplay = new HootAutoReplay()
        .withTimestampReplay()
        .withJoystickReplay();

    public Robot() {
        m_robotContainer = new RobotContainer();
    }


    @Override
    public void robotPeriodic() {
        m_timeAndJoystickReplay.update();
        CommandScheduler.getInstance().run(); 
    }

    @Override
    public void disabledInit() {}

    @Override
    public void disabledPeriodic() {}

    @Override
    public void disabledExit() {}

    @Override
    public void autonomousInit() {
        
        m_autonomousCommand = m_robotContainer.getAutonomousCommand();

        if (m_autonomousCommand != null) {
            CommandScheduler.getInstance().schedule(m_autonomousCommand);
        }
    }

    @Override
    public void autonomousPeriodic() {}

    @Override
    public void autonomousExit() {}

    @Override
    public void teleopInit() {

        if (m_autonomousCommand != null) {
            CommandScheduler.getInstance().cancel(m_autonomousCommand);
        }
    }

    @Override
    public void teleopPeriodic() {
        lastteleopstart = teleopstart;
        teleopstart = System.currentTimeMillis();
        countdownTimer = (teleopstart - lastteleopstart) / 1000.0;
        
        if (countdownTimer > ShiftConstants.SHIFT_LENGTHS.get(currentShift)-ShiftConstants.SHIFT_WARNING_TIME){
            System.out.println("WARNING: "+ShiftConstants.SHIFT_NAMES.get(currentShift)+" to end!");
            if (Math.floor(countdownTimer) > currentcountdown){
                currentcountdown = Math.floor(countdownTimer);
                double timeleft = ShiftConstants.SHIFT_LENGTHS.get(currentShift)-countdownTimer;
                System.out.println(ShiftConstants.SHIFT_NAMES.get(currentShift)+" ends in:" + String.valueOf(timeleft) + " seconds!");
            
            }
            if (countdownTimer >= ShiftConstants.SHIFT_LENGTHS.get(currentShift)){
                System.out.println("WARNING: "+ShiftConstants.SHIFT_NAMES.get(currentShift)+" Ended!");
                currentShift = ShiftConstants.SHIFT_NEXT.get(currentShift);
                System.out.println("Starting new shift: "+ShiftConstants.SHIFT_NAMES.get(currentShift));
                System.out.println("    - time left"+ShiftConstants.SHIFT_NAMES.get(currentShift));
                // TODO: finish
            }
        }


        // joystick.setRumble(GenericHID.RumbleType.kBothRumble,1);
              

        // System.out.println(RobotContainer.drivetrain.getState().Pose.getRotation().getCos()+" "+RobotContainer.drivetrain.getState().Pose.getRotation().getSin());
    }

    @Override
    public void teleopExit() {}

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {}

    @Override
    public void testExit() {}

    @Override
    public void simulationPeriodic() {}
}
