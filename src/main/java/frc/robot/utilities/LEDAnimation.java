/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.utilities;

import javax.swing.text.Utilities; // TODO check why this class extends Utilities unlike the other utilities

import edu.wpi.first.wpilibj.util.Color;

/**
 * Class used to control looping through patterns on an LED strip
 */
public class LEDAnimation extends Utilities {
    
    private int delay; // number of scheduler cycles before switching to the next pattern
    private int delayCounter; // increases every cycle until reaching "delay"
    private int currentIndex; // current pattern
    
    private Color[][] pattern; // 2D array of patterns to cycle through

    public LEDAnimation(int delay, Color[][] pattern) {
        this.pattern = pattern;
        this.delay = delay;
        delayCounter = 0;
        currentIndex = 0;
    }

    /**
     * @return next 1D array of Colors to be displayed
     */
    public Color[] getNextPattern() {
        if (delayCounter == delay){
            incrementCounterIndex();
            delayCounter = 0;   
        }
        return pattern[currentIndex];
    }

    /**
     * increments delayCounter
     * resets counter to 0 if it exceeds the max (delay)
     */
    public void setDelayCounter() {
        if (delayCounter == delay) {
            delayCounter = 0;
            incrementCounterIndex();
        } 
        delayCounter++;
    }

    /**
     * increments counterIndex (move on to next pattern)
     * resets counterIndex to 0 if it exceeds the max (# of patterns)
     */
    private void incrementCounterIndex() {
        currentIndex++;
        if(currentIndex == pattern.length) {
            currentIndex = 0;
        }
    }
}

