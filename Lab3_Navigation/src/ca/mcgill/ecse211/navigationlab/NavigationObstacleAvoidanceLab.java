package ca.mcgill.ecse211.navigationlab;

import lejos.hardware.Button;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.motor.EV3MediumRegulatedMotor;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;

/**
 * The NavigationObstacleAvoidanceLab allows the robot to drive to a specified set of coordinates,
 * known as waypoints collected in an 2 dimensional array, while potentially avoiding any obstacles
 * on its path
 * @author Marine Huynh
 * @author Sihui Shen
 * @since 09-28-2017
 *
 */
public class NavigationObstacleAvoidanceLab {

	//create the ports
	private static final EV3LargeRegulatedMotor leftMotor = 
			new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));
	private static final EV3LargeRegulatedMotor rightMotor =
			new EV3LargeRegulatedMotor(LocalEV3.get().getPort("D"));
	private static final Port usPort = LocalEV3.get().getPort("S4");
	private static final EV3MediumRegulatedMotor usMotor =
			new EV3MediumRegulatedMotor(LocalEV3.get().getPort("B"));
	
	public static boolean isObstacle;
	
	/**
	 * this is the main method that waits for a left button press for navigation
	 * or a right button press to navigate if there are obstacles
	 * this will call the Navigation class to run
	 * @param args
	 * @return nothing
	 */
	public static void main(String[] args) {
		int buttonChoice=0;

		//create the instances
		final TextLCD t=LocalEV3.get().getTextLCD();
		Odometer odometer=new Odometer(leftMotor,rightMotor);
		Navigation navigation;
		OdometryDisplay odometrydisplay=new OdometryDisplay(odometer,t);

		//created for the distance measured
		//@SuppressWarnings("resource") // Because we don't bother to close this resource
		SensorModes usSensor = new EV3UltrasonicSensor(usPort); // usSensor is the instance
		SampleProvider usDistance = usSensor.getMode("Distance"); // usDistance provides samples from
		// this instance
		float[] usData = new float[usDistance.sampleSize()]; // usData is the buffer in which data are
		// returned   

		do {
			// clear the display
			t.clear();

			// ask the user whether the motors should drive in a square or float
			t.drawString("< Left   | Right  ", 0, 0);
			t.drawString("         |        ", 0, 1);
			t.drawString("   no    | will   ", 0, 2);
			t.drawString("  obst.  | Avoid  ", 0, 3);
			t.drawString("         |        ", 0, 4);
			buttonChoice= Button.waitForAnyPress();
		}	
		while(buttonChoice!=Button.ID_LEFT && buttonChoice!=Button.ID_RIGHT);

		odometer.start();
		odometrydisplay.start();
		
		if(buttonChoice == Button.ID_LEFT) {
			isObstacle=false;
		}
		else if(buttonChoice == Button.ID_RIGHT)
		{
			isObstacle=true;
		}
        navigation = new Navigation(leftMotor, rightMotor,odometer,t, usSensor, usData, usMotor);
		navigation.start();

		while(Button.waitForAnyPress()!=Button.ID_ESCAPE);
		System.exit(0);
	}

}
