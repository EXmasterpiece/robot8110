/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.concurrent.TimeUnit;


/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  
  CANSparkMax left_motor_1;
  CANSparkMax left_motor_2;
  CANSparkMax right_motor_1;
  CANSparkMax right_motor_2;
  CANSparkMax ball_collection_motor;
  CANSparkMax left_high_eject;
  CANSparkMax right_high_eject;
  
  TalonSRX left_belt;
  TalonSRX right_belt;
  TalonSRX left_low_eject;
  TalonSRX right_low_eject;
  DoubleSolenoid double_solenoid_1;
  XboxController xbox_controller;
  boolean clockwise;
  Servo servo;
  double eject_speed;
  double go_speed,turn_speed;
  boolean high_eject_run;
  boolean if_correted;
  private Command m_autonomousCommand;
  private RobotContainer m_robotContainer;
  double servo_angle;
  Joystick joystick;
  double currentServoAngle;
  double factor;
  boolean pushed;
  NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
  NetworkTableEntry tx = table.getEntry("tx");
  NetworkTableEntry tv = table.getEntry("tv");
  NetworkTableEntry ta= table.getEntry("ta");
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
    
    left_motor_1= new CANSparkMax(2,MotorType.kBrushless);
    left_motor_2=new CANSparkMax(3,MotorType.kBrushless);
    right_motor_1=new CANSparkMax(5,MotorType.kBrushless);
    right_motor_2=new CANSparkMax(6,MotorType.kBrushless);
    ball_collection_motor=new CANSparkMax(4,MotorType.kBrushless);
    left_high_eject=new CANSparkMax(1, MotorType.kBrushless);
    right_high_eject=new CANSparkMax(7,MotorType.kBrushless);
    left_belt=new TalonSRX(9);
    right_belt=new TalonSRX(11);
    left_low_eject=new TalonSRX(8);
    right_low_eject=new TalonSRX(12);
    double_solenoid_1=new DoubleSolenoid(0,1);
    joystick=new Joystick(1);
    servo=new Servo(0);
    if_correted=false;
    xbox_controller=new XboxController(0);
    servo_angle=1;
    eject_speed=0;
    go_speed=0;
    turn_speed=0;
    high_eject_run=false;
    clockwise=false;
    left_motor_1.restoreFactoryDefaults();
    left_motor_2.restoreFactoryDefaults();
    right_motor_1.restoreFactoryDefaults();
    right_motor_2.restoreFactoryDefaults();
    
    left_motor_2.follow(left_motor_1);
    right_motor_1.follow(right_motor_2);
    servo.set(0);
    currentServoAngle = 0;
    factor = 0.01;
    pushed = false;
    //right_high_eject.follow(left_high_eject);
    //right_low_eject.follow(left_low_eject);

  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before
   * LiveWindow and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /**
   * This function is called once each time the robot enters Disabled mode.
   */
  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
  }

  /**
   * This autonomous runs the autonomous command selected by your {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
    //left_belt.set(ControlMode.PercentOutput,0.5);
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
    go_speed=-xbox_controller.getRawAxis(1);
    turn_speed=xbox_controller.getRawAxis(4);
    right_motor_2.set(-go_speed+turn_speed);
    left_motor_1.set(go_speed+turn_speed);

    while(xbox_controller.getRawButton(2)){
      double x = tx.getDouble(0.0);
      double a = ta.getDouble(0.0);
      if(a >= 1 && a <= 1.7){
        servo.set(0.26);
        if (x > -6 && x < 6){
          right_motor_2.set(0);
          left_motor_1.set(0);
          if_correted=true;
          break;
        } 
        else if ( x >= 6 ){
          right_motor_2.set(0.15);
          left_motor_1.set(0.15);
        }
        else if ( x <= -6 ){
          right_motor_2.set(-0.15);
          left_motor_1.set(-0.15);
        }
      }
      else if(a>=0.8 && a<1){
        servo.set(0.35);
        if (x > -2.3 && x < 2.3){
          right_motor_2.set(0);
          left_motor_1.set(0);
          if_correted=true;
          break;
        } 
        else if ( x >= 2.3 ){
          right_motor_2.set(0.11);
          left_motor_1.set(0.11);
        }
        else if ( x <= -2.3 ){
          right_motor_2.set(-0.11);
          left_motor_1.set(-0.11);
        }
      }
      else if(a>=0.5 && a<0.8){
        servo.set(0.5);
        if (x > -1.7 && x < 1.7){
          right_motor_2.set(0);
          left_motor_1.set(0);
          if_correted=true;
          break;
        } 
        else if ( x >= 1.7 ){
          right_motor_2.set(0.08);
          left_motor_1.set(0.08);
        }
        else if ( x <= -1.7 ){
          right_motor_2.set(-0.08);
          left_motor_1.set(-0.08);
        }
      }
      else if(a>=0.3 && a<0.5){
        servo.set(0.32);
        if (x > -1.2 && x < 1.2){
          right_motor_2.set(0);
          left_motor_1.set(0);
          if_correted=true;
          break;
        } 
        else if ( x >= 1.2 ){
          right_motor_2.set(0.05);
          left_motor_1.set(0.05);
        }
        else if ( x <= -1.2 ){
          right_motor_2.set(-0.05);
          left_motor_1.set(-0.05);
        }
      }
      else{}
    }

    if(xbox_controller.getRawAxis(2)!=0){
      ball_collection_motor.set(1);
    }
    else{
      ball_collection_motor.set(0);
    }

    while(joystick.getRawButton(5)){
      double a=ta.getDouble(0.0);
      if(a>=1&&a<=1.7){
        servo.set(0.26);
      }
      else if(a>=0.8 && a<1){
        servo.set(0.35);
      }
      else if(a>=0.5 && a<0.8){
        servo.set(0.5);
      }
      else if(a>=0.3 && a<0.5){
        servo.set(0.32);
      }
      else{}
    }

    while (joystick.getRawButton(6)) {
      double x = tx.getDouble(0.0);
      if ( x>-1 && x<1 ){
        right_motor_2.set(0);
        left_motor_1.set(0);
        if_correted=true;
        break;
      } 
      else if ( x >= 1 ){
        right_motor_2.set(0.05);
        left_motor_1.set(0.05);
      }
      else if ( x <= -1 ){
        right_motor_2.set(-0.05);
        left_motor_1.set(-0.05);
      }
      else{}
    }

    if(xbox_controller.getRawButton(3)){
      System.out.println(currentServoAngle);
      servo.set(currentServoAngle + factor);
      currentServoAngle += factor;
      pushed = true;
      if(currentServoAngle >= 1 || (currentServoAngle <= 0 && pushed == true)){
        factor *= -1;
      }  
    }
    
    
    //ball_collection_motor.set(xbox_controller.getRawAxis(2));//xbox rt
    if(xbox_controller.getRawAxis(3)!=0){
      while(xbox_controller.getRawAxis(3)){
      left_low_eject.set(ControlMode.PercentOutput, 1);
      right_low_eject.set(ControlMode.PercentOutput,-1);
      TimeUnit.SECONDS.sleep(0.5);
      left_belt.set(ControlMode.PercentOutput,-0.5);
      right_belt.set(ControlMode.PercentOutput,0.5);
      }
    }
    else{
      left_low_eject.set(ControlMode.PercentOutput,0);
      right_low_eject.set(ControlMode.PercentOutput,0);
      left_belt.set(ControlMode.PercentOutput,0);
      right_belt.set(ControlMode.PercentOutput,0);
    }
    if(xbox_controller.getRawButton(1)){
      double_solenoid_1.set(DoubleSolenoid.Value.kForward);
    }
    else if(xbox_controller.getRawButton(4)){
      double_solenoid_1.set(DoubleSolenoid.Value.kReverse);
    }
    if(xbox_controller.getRawButton(5)){
      right_motor_2.set(-0.1);
      left_motor_1.set(-0.1);
    }
    else if(xbox_controller.getRawButton(6)){
      right_motor_2.set(0.1);
      left_motor_1.set(0.1);
    }
    if(xbox_controller.getRawButton(8)){
      /*
      for(double speed=0;speed<=0.85;speed+=0.000002){
        left_high_eject.set(speed+0.2*(joystick.getRawAxis(3)-control_speed_init));
      }    
      */
      if(high_eject_run==false){
        high_eject_run=true;
        eject_speed=0;
      }

    }

    if(xbox_controller.getRawButton(7)){
      left_high_eject.set(0);
      high_eject_run=false;
      eject_speed=0;
    }
    if(high_eject_run){
      if(eject_speed<=0.85){
        //System.out.println(123);
        eject_speed+=0.001;
        //System.out.println(eject_speed);
        left_high_eject.set(-eject_speed);
      }
    }

    
  }
}
