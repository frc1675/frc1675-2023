// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

public final class Constants {
      //auto constants
      public static final double AUTO_MAX_VELOCITY = 3;
      public static final double AUTO_MAX_ACCELERATION = 3;


      //drivetrain constants
      public static final double DRIVETRAIN_TRACKWIDTH_METERS = 0.5969; //Measured on robot
      public static final double DRIVETRAIN_WHEELBASE_METERS = 0.5969;
      public static final double DRIVE_ROTATE_TOLERANCE_DEGREES = 1.0;
      public static final double DRIVE_ROTATION_TARGET_DEGREES = 0.0;
  
      //PID constants
      public static final double PROPORTIONAL_COEFFICENT = 3.0;
      public static final double INTEGRAL_COEFFICENT = 0.5;
      public static final double DERIVATIVE_COEFFICENT = 0.5;
  
      //swerve module constants
      public static final int FRONT_LEFT_MODULE_DRIVE_MOTOR = 3;
      public static final int FRONT_LEFT_MODULE_STEER_MOTOR = 4;
      public static final int FRONT_LEFT_MODULE_STEER_ENCODER = 12; 
      public static final double FRONT_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(322.47);
  
      public static final int FRONT_RIGHT_MODULE_DRIVE_MOTOR = 1;
      public static final int FRONT_RIGHT_MODULE_STEER_MOTOR = 2; 
      public static final int FRONT_RIGHT_MODULE_STEER_ENCODER = 10; 
      public static final double FRONT_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(70.48);
  
      public static final int BACK_LEFT_MODULE_DRIVE_MOTOR = 7; 
      public static final int BACK_LEFT_MODULE_STEER_MOTOR = 8; 
      public static final int BACK_LEFT_MODULE_STEER_ENCODER = 11; 
      public static final double BACK_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(121.46);
  
      public static final int BACK_RIGHT_MODULE_DRIVE_MOTOR = 5; 
      public static final int BACK_RIGHT_MODULE_STEER_MOTOR = 6; 
      public static final int BACK_RIGHT_MODULE_STEER_ENCODER = 9; 
      public static final double BACK_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(134.73);
  
      //controller constants
      public static final int DRIVER_CONTROLLER = 0;
      public static final int OPERATOR_CONTROLLER = 1;
  
      public static final int LEFT_X_AXIS = 0;
      public static final int LEFT_Y_AXIS= 1;
      public static final int RIGHT_X_AXIS = 4;
      public static final int RIGHT_Y_AXIS = 5;
  
      public static final int A_BUTTON = 1;
      public static final int B_BUTTON = 2;
      public static final int X_BUTTON = 3;
      public static final int Y_BUTTON = 4;
  
      public static final int LEFT_BUMPER = 5;
      public static final int RIGHT_BUMPER = 6;
      public static final int BACK_BUTTON = 7;
      public static final int START_BUTTON = 8;
      public static final int LEFT_JOYSTICK_BUTTON = 9;
      public static final int RIGHT_JOYSTICK_BUTTON = 10;
      public static final double INTAKE_SPEED = 1;
      public static final int INTAKE_MOTOR = 9;
      public static final double FLOOR_INTAKE_SPEED = 1;
      public static final int FLOOR_INTAKE_MOTOR = 8;
}
