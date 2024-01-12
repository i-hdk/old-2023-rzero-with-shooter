package raidzero.robot.submodules;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.StrobeAnimation;

import edu.wpi.first.wpilibj.Timer;
import raidzero.robot.Constants.LightsConstants;

public class Lights extends Submodule {
    private enum ControlState {
        PLAIN, ANIMATION, CUSTOM_PIXELS
    };

    private Lights() {}

    private static Lights instance = null;

    public static Lights getInstance() {
        if (instance == null) {
            instance = new Lights();
        }
        return instance;
    }

    private final CANdle mCANdle = new CANdle(LightsConstants.CANDLE_ID);
    private final Timer mTimer = new Timer();

    private int mR, mG, mB, mPixelR, mPixelG, mPixelB;
    private Animation mAnimation = new StrobeAnimation(0, 0, 0);
    private double mBrightness = 1.0;
    private boolean mFirstTimeNotDetected = false;
    private ControlState mControlState = ControlState.PLAIN;
    private ControlState mPrevControlState = ControlState.PLAIN;
    private int prevNumber = 0;

    private int[] mPixels;

    @Override
    public void onInit() {
        mCANdle.configFactoryDefault();
        mCANdle.configLOSBehavior(LightsConstants.LOS_BEHAVIOR);
        mCANdle.configLEDType(LightsConstants.LED_STRIP_TYPE);
        mCANdle.configBrightnessScalar(LightsConstants.BRIGHTNESS_SCALAR);
        mCANdle.configStatusLedState(LightsConstants.STATUS_LED_CONFIG);
        mCANdle.configVBatOutput(LightsConstants.V_BAT_OUTPUT_MODE);
    }

    @Override
    public void onStart(double timestamp) {
        stop();
    }

    @Override
    public void update(double timestamp) {}

    @Override
    public void run() {
        if(mPrevControlState != mControlState) {
            stop();
        }
        mPrevControlState = mControlState;

        mCANdle.configBrightnessScalar(mBrightness);
        if(mControlState == ControlState.PLAIN) {
            mCANdle.clearAnimation(LightsConstants.PRIMARY_ANIMATION_SLOT);
            mCANdle.setLEDs(mR, mG, mB, 0, 8, 512);
        } else if(mControlState == ControlState.ANIMATION) {
            mCANdle.animate(mAnimation, LightsConstants.PRIMARY_ANIMATION_SLOT);
        } else if(mControlState == ControlState.CUSTOM_PIXELS) {
            for(int i : mPixels) {
                mCANdle.setLEDs(mPixelR, mPixelG, mPixelB, 0, i+8, 1);
            }
        }
    }

    @Override
    public void stop() {
        mCANdle.setLEDs(0, 0, 0);
    }

    @Override
    public void zero() {}

    public void setColor(int r, int g, int b) {
        mControlState = ControlState.PLAIN;
        mR = r;
        mG = g; 
        mB = b;
    }

    public void setAnimation(Animation animation) {
        mControlState = ControlState.ANIMATION;
        mAnimation = animation;
    }

    public void setBrightness(double num) {
        mBrightness = num; 
    }

    public void displayNumber(int num, int r, int g, int b) {
        switch(num) {
            case 0:
                enablePixels(r, g, b, 1, 2, 4, 7, 11, 8, 15, 12, 18, 17);
                break;
            case 1:
                enablePixels(r, g, b, 0, 7, 11, 10, 9, 8, 15, 12, 18, 16);
                break;
            case 2:
                enablePixels(r, g, b, 18, 12, 11, 5, 9, 15, 16, 8, 7, 0);
                break;
            case 3:
                enablePixels(r, g, b, 11, 4, 3, 10, 5, 2, 8, 7, 0);
                break;
            case 4:
                enablePixels(r, g, b, 12, 13, 17, 14, 9, 6, 1, 4, 5, 7);
                break;
            case 5:
                enablePixels(r, g, b, 11, 4, 3, 10, 9, 6, 1, 8, 7, 0);
                break;
            case 6:
                enablePixels(r, g, b, 3, 4, 11, 12, 19, 18, 17, 14, 9, 6, 1);
                break;
            case 7:
                enablePixels(r, g, b, 0, 7, 8, 15, 16);
                break;
            case 8:
                enablePixels(r, g, b, 16, 15, 11, 4, 3, 17, 14, 10, 5, 2, 18, 1, 19, 12, 11, 7, 0);
                break;
            case 9:
                enablePixels(r, g, b, 17, 11, 4, 15, 13, 9, 5, 6, 7);
                break;
            default:
                enablePixels(r, g, b, 2, 3, 4, 18, 19, 12, 8, 6, 14);
                break;
        }
    }

    public void enablePixels(int r, int g, int b, int ... pixel) {
        mControlState = ControlState.CUSTOM_PIXELS;
        mPixels = pixel;
        mPixelR = r;
        mPixelG = g;
        mPixelB = b;
    }

    public void intake(double intake, double deadband) {
        setBrightness(1.0);
        if(intake > deadband) {
            setColor(255, 255, 0);
        } else if(intake < -deadband) {
            setColor(255, 0, 255);
        } 
        else {
            // Animation animation = new RainbowAnimation(1, 0.5, -1, false, 8);
            // setAnimation(animation);
            // enablePixels(255, 0, 0, 5, 12, 15);
            // displayNumber(4, 255, 0, 255);
            if(!mFirstTimeNotDetected) {
                mFirstTimeNotDetected = true;
                mTimer.restart();
            }
            int num = ((int)mTimer.get()) % 5;
            if(prevNumber != num) {
                stop();
            }
            prevNumber = num;
            switch(num) {
                case 0:
                    displayNumber(4, 255, 0, 255);
                    break;
                case 1:
                    displayNumber(2, 255, 0, 255);
                    break;
                case 2:
                    displayNumber(5, 255, 0, 255);
                    break;
                case 3:
                    displayNumber(3, 255, 0, 255);
                    break;
                case 4:
                    displayNumber(10, 255, 0, 255);
                    break;
            }
        }
    }

    public void apples(boolean isDetected) {
        if(isDetected) {
            setBrightness(1.0);
            setColor(0, 255, 0);
            mFirstTimeNotDetected = false;
            return;
        } 
        if(!mFirstTimeNotDetected) {
            mFirstTimeNotDetected = true;
            mTimer.restart();
        }
        if(mTimer.get() > 1) {
            setBrightness(1.0);
            Animation animation = new StrobeAnimation(255, 0, 0, 0, 0.5, -1, 8);
            setAnimation(animation);
        } else {
            setBrightness(mTimer.get());
            setColor(0, 255, 0);
        }
    }

    public void coneCube(boolean cone) {
        setBrightness(1.0);
        if(cone) {
            setColor(255, 255, 0);
        } else {
            setColor(255, 0, 255);
        } 
    }
}
