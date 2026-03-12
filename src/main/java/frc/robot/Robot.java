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
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.ShiftConstants.shifts;
import frc.robot.generated.TunerConstants;
public class Robot extends TimedRobot {
    private double teleopstart = 0;
    private double lastteleopstart = 0;
    private static double countdownTimer =0;
    private int currentcountdown = 0;
    private static shifts currentShift = shifts.SHIFT_1;
    private boolean hasWarned = false;
    private Command m_autonomousCommand;
    private final CommandXboxController joystick = new CommandXboxController(0);
    private final RobotContainer m_robotContainer;
    private SwerveDrivetrainConstants drivetrainconstants = TunerConstants.DrivetrainConstants;
    private Pigeon2 gyro = new Pigeon2(drivetrainconstants.Pigeon2Id);
    private ShuffleboardTab gameTab;
    /* log and replay timestamp and joystick data */
    private final HootAutoReplay m_timeAndJoystickReplay = new HootAutoReplay()
        .withTimestampReplay()
        .withJoystickReplay();

    public Robot() {
        m_robotContainer = new RobotContainer();


        gameTab = Shuffleboard.getTab("game");
        Shuffleboard.selectTab("game");
        gameTab.add("Shift", ShiftConstants.SHIFT_NAMES.get(currentShift));
    }
    public static shifts getShift(){
        return currentShift;
    }
    public static double getShiftTimeLeft(){
        return countdownTimer;
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
        countdownTimer = 0;
        currentcountdown = 0;
        hasWarned = false;
        currentShift = shifts.SHIFT_1;
        teleopstart = System.currentTimeMillis();
    }

    @Override
    public void teleopPeriodic() {
        lastteleopstart = teleopstart;
        teleopstart = System.currentTimeMillis();
        countdownTimer += ((teleopstart - lastteleopstart) / 1000.0);
        if (countdownTimer > ShiftConstants.SHIFT_LENGTHS.get(currentShift)-ShiftConstants.SHIFT_WARNING_TIME){
            if (!hasWarned){
                System.out.println("WARNING: "+ShiftConstants.SHIFT_NAMES.get(currentShift)+" to end!");
                hasWarned=true;
            }
            if (Math.floor(countdownTimer) > currentcountdown){
                currentcountdown = (int) Math.floor(countdownTimer);
                int timeleft = ShiftConstants.SHIFT_LENGTHS.get(currentShift)-currentcountdown;
                System.out.println(ShiftConstants.SHIFT_NAMES.get(currentShift)+" ends in: " + String.valueOf(timeleft) + " seconds!");
            
            }   
            if (countdownTimer >= ShiftConstants.SHIFT_LENGTHS.get(currentShift)){
                System.out.println("Alert: "+ShiftConstants.SHIFT_NAMES.get(currentShift)+" Ended!");
                currentShift = ShiftConstants.SHIFT_NEXT.get(currentShift);
                System.out.println("Starting new shift: "+ShiftConstants.SHIFT_NAMES.get(currentShift));
                System.out.println("   - time in shift: "+ShiftConstants.SHIFT_LENGTHS.get(currentShift));
                countdownTimer = 0;
                currentcountdown = 0;
                hasWarned = false;
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
