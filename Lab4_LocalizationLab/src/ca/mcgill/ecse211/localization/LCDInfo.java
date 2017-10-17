package ca.mcgill.ecse211.localization;

import lejos.utility.Timer;

import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.utility.TimerListener;

public class LCDInfo implements TimerListener {
	public static final int LCD_REFRESH=100;
	private Odometer odometer;
	private Timer lcdTimer;
	private TextLCD LCD= LocalEV3.get().getTextLCD();
	
	//start an array to display data
	private double [] position;
	private boolean[] booleanArray;
	
	public LCDInfo(Odometer odometer) {
		this.odometer=odometer;
		this.lcdTimer=new Timer(LCD_REFRESH,this);
		
		//initialize the array to display data
		position=new double[3];
		booleanArray=new boolean[3];
		
		//start the timer
		lcdTimer.start();
	}
	public void timedOut() {
		odometer.getPosition(position, booleanArray);
		LCD.clear();
		//x y stands for coordinates of the robot, H stands for heading
		LCD.drawString("X:", 0, 0);
		LCD.drawString("Y:", 0, 1);
		LCD.drawString("H:", 0, 2);
		LCD.drawInt((int)(position[0]*10), 3, 0);
		LCD.drawInt((int)(position[1]*10), 3, 1);
		LCD.drawInt((int)position[2], 3, 2);
	}
}
