package com.robot.et.core.hardware.wakeup;

import android.app.Activity;
import android.os.Bundle;
import android.util.Log;

public class MainActivity extends Activity {

	private int fd;
	private int wakeUpState;
	private int degree;

	@Override
	protected void onCreate(Bundle savedInstanceState) {
		super.onCreate(savedInstanceState);
		setContentView(R.layout.activity_main);
		fd = WakeUp.open("", 0);
		Log.i("I2C", "fd:" + fd);
		new Thread(new Runnable() {

			@Override
			public void run() {
				while (true) {
					if (fd > 0) {
						wakeUpState = WakeUp.getWakeUpState(fd);
						// Log.i("I2C", "wakeUpState:" + wakeUpState);
						if (wakeUpState == 1) {
							degree = WakeUp.getWakeUpDegree();
							Log.i("I2C", "degree:" + degree);
							int result=WakeUp.setGainDirection(0);
							Log.i("I2C", "result:" + result);
						} else {
							// Log.i("I2C", "no wakeUp");
						}
					} else {
//						Log.i("I2C", "未打开I2C");
					}
				}
			}
		}).start();
	}
}
