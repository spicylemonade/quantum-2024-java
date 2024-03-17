// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.SerialPort.Port;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalOutput;

import java.util.function.BooleanSupplier;

import com.fasterxml.jackson.databind.ser.std.StdKeySerializers.Default;
import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;


/** This is a demo program showing how to use Mecanum control with the MecanumDrive class. */
public class Robot extends TimedRobot {

    // private boolean DEFAULT= false;
    // private boolean FEED= false;
    // private boolean SHOOT= false;
    // private boolean SPEAKER= false;

    // private boolean UP = false;
    // private boolean DOWN = false;

    private double ultrasonicDistance;
    private boolean isRingCentered;
    private int state;


    private Joystick joystick;
    private AHRS navx;
    // private GenericHID controller;
    private double driveSpeed = 0.8;

    private CANSparkMax topLeft, bottomLeft, topRight, bottomRight, climberMotor;
    private CANSparkMax motorIntake, motorShooterUp,motorShooterDown,armMotor, armMotor2; 
    //private RelativeEncoder armMotorEncoder,climberMotorEncoder; // From REV library
    private MecanumDrive mecDrive;

    // private DigitalOutput sensor1Trig, sensor2Trig;
    // private DigitalInput sensor1Echo, sensor2Echo; 
    // private ConditionalCommand shootConditional;
    // private Command m_autonomousCommand;
    private AnalogInput ultrasonicSensor;

    //private RobotContainer m_robotContainer;


  @Override
  public void robotInit() {
    // Joysticks
    navx = new AHRS(Port.kUSB1);
    

    joystick = new Joystick(0);
    //controller = new GenericHID(1);

    //  Mecanum Drive Motors
    topLeft = new CANSparkMax(1, MotorType.kBrushless);
    bottomLeft = new CANSparkMax(2, MotorType.kBrushless);
    topRight = new CANSparkMax(4, MotorType.kBrushless);
    bottomRight = new CANSparkMax(3, MotorType.kBrushless);
    climberMotor = new CANSparkMax(10, MotorType.kBrushless);

    mecDrive = new MecanumDrive(topLeft, bottomLeft, topRight, bottomRight);

    // Invert motors if needed
    topRight.setInverted(false);
    bottomRight.setInverted(true);
    bottomLeft.setInverted(true);

    // Additional Talon Motors
    motorIntake = new CANSparkMax(9, MotorType.kBrushless);
     
    motorShooterUp = new CANSparkMax(7, MotorType.kBrushless);
    motorShooterDown = new CANSparkMax(8, MotorType.kBrushless);  
    // armMotorEncoder = armMotor.getEncoder();
    // climberMotorEncoder = climberMotor.getEncoder();

    armMotor = new CANSparkMax(6, MotorType.kBrushless); 
    armMotor2 = new CANSparkMax(5, MotorType.kBrushless); 
    

    // armMotorEncoder.setPosition(0);
    // climberMotorEncoder.setPosition(0);



    // DEFAULT = true;
    // DOWN = true;


    

    // // Ultrasonic Sensors
    // sensor1Trig = new DigitalOutput(0);
    // sensor1Echo = new DigitalInput(1);
    // sensor2Trig = new DigitalOutput(2);
    // sensor2Echo = new DigitalInput(3);
    //m_robotContainer = new RobotContainer();
  }

  //@Override
  //public void robotPeriodic(){
    // CommandScheduler.getInstance().run();
  //}

  BooleanSupplier seesRing = () -> {
  // Code to check if the robot sees a ring
     return true;
  };
  

  @Override
  public void teleopPeriodic() {
    // if (m_autonomousCommand != null) {
    //   m_autonomousCommand.cancel();
    // }
        // Mecanum Drive
    double x = joystick.getX();
    double y = joystick.getY();
    double rotation = joystick.getZ();
    if (Math.abs(y) < 0.3){
      y = 0;
    }
    if (Math.abs(x) < 0.3){
      x = 0;
    }
    if (Math.abs(rotation) < 0.3){
      rotation = 0;
    }
    mecDrive.driveCartesian(y * driveSpeed, x * driveSpeed, rotation * driveSpeed);

    // Additional Talon Motors

    if (joystick.getRawButton(5)) { 
        motorIntake.set(0.5); 
    } else if (joystick.getRawButton(3)) { 
        motorIntake.set(-0.5); 
    } else {
        motorIntake.set(0);
    }


    if (joystick.getRawButton(1)) { 
        motorShooterUp.set(0.5); 
        motorShooterDown.set(0.5); 
        
    } else if(joystick.getRawButton(8)) {
         motorShooterUp.set(-0.2); 
        motorShooterDown.set(-0.2); 
    }else{
      motorShooterUp.set(0);
        motorShooterDown.set(0);
    }

    //Arm Climber

    if (joystick.getRawButton(6)) { 
        armMotor.set(0.2); 
        armMotor2.set(0.2);

    } else if (joystick.getRawButton(4)) { 
        armMotor.set(-0.2); 
        armMotor2.set(-0.2);

    } else {
        armMotor.set(0);
        armMotor2.set(0);
    }

    //testing

    if (joystick.getRawButton(2)) { 
        climberMotor.set(0.5); 
    } else {
        climberMotor.set(0);
    }
    //SmartDashboard.putNumber("climber encoder value", armMotorEncoder.getPosition());
    //SmartDashboard.putNumber("arm encoder value", armMotorEncoder.getPosition());


    // Climber
    // climbControllers();
    // armControllers();

  }

  @Override
  public void autonomousInit() {

    ultrasonicSensor = new AnalogInput(1);
    state = 1;
  }

  /** This function is called periodically during autonomous. */
   public void autonomousPeriodic() {
        double ultrasonicDistance = ultrasonicSensor.getVoltage() / (5.0 / 512.0);

        switch (state) {
            case 1:
                // Turn 5 degrees, shoot, then turn back 5 degrees
                navx.reset();
                mecDrive.driveCartesian(0.0, 0.0, 0.2); // Rotate clockwise
                while (navx.getAngle() < 5.0) {
                    // Wait until rotation is complete
                }
                mecDrive.stopMotor();
                /* Shoot */

                navx.reset();
                mecDrive.driveCartesian(0.0, 0.0, -0.2); // Rotate counter-clockwise
                while (navx.getAngle() > -5.0) {
                    // Wait until rotation is complete
                }
                mecDrive.stopMotor();
                state = 2;
                break;

            case 2:
                // Move forward until ultrasonic sensor detects something 1 cm away, then turn and shoot
                mecDrive.driveCartesian(0.5, 0.0, 0.0); // Move forward
                if (ultrasonicDistance < 1.0) {
                    mecDrive.stopMotor();
                    navx.reset();
                    mecDrive.driveCartesian(0.0, 0.0, 0.2); // Rotate clockwise
                    while (navx.getAngle() < 180.0) {
                        // Wait until rotation is complete
                    }
                    mecDrive.stopMotor();
                    /* Shoot */
                    state = 3;
                }
                break;

            case 3:
                // Rotate 90 degrees, move forward until ultrasonic sensor detects something 1 cm away, then turn and shoot
                navx.reset();
                mecDrive.driveCartesian(0.0, 0.0, 0.2); // Rotate clockwise
                while (navx.getAngle() < 90.0) {
                    // Wait until rotation is complete
                }
                mecDrive.stopMotor();

                mecDrive.driveCartesian(0.5, 0.0, 0.0); // Move forward
                if (ultrasonicDistance < 1.0) {
                    mecDrive.stopMotor();
                    navx.reset();
                    mecDrive.driveCartesian(0.0, 0.0, 0.2); // Rotate clockwise
                    while (navx.getAngle() < 180.0) {
                        // Wait until rotation is complete
                    }
                    mecDrive.stopMotor();
                    /* Shoot */
                    state = 4;
                }
                break;

            case 4:
                // Rotate 40 degrees, move forward until ultrasonic sensor detects something 1 cm away, then turn and shoot
                navx.reset();
                mecDrive.driveCartesian(0.0, 0.0, 0.2); // Rotate clockwise
                while (navx.getAngle() < 40.0) {
                    // Wait until rotation is complete
                }
                mecDrive.stopMotor();

                mecDrive.driveCartesian(0.5, 0.0, 0.0); // Move forward
                if (ultrasonicDistance < 1.0) {
                    mecDrive.stopMotor();
                    navx.reset();
                    mecDrive.driveCartesian(0.0, 0.0, 0.2); // Rotate clockwise
                    while (navx.getAngle() < 180.0) {
                        // Wait until rotation is complete
                    }
                    mecDrive.stopMotor();
                    /* Shoot */
                }
                break;
        }
    }
}
