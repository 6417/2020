package ch.team6417.lib.utils;

/**
 * An iterative boolean latch.
 * <p>
 * Returns true once if and only if the value of newValue changes from false to true.
 */
public class LatchedBoolean {
    private boolean mLast = false;
    private boolean risingEdge;
    private boolean fallingEdge;

    public enum EdgeDetection {
        RISING, FALLING, BOTH
    }

    public LatchedBoolean() {
        this(EdgeDetection.RISING);
    }

    public LatchedBoolean(final LatchedBoolean.EdgeDetection edge) {
        switch (edge) {
        case RISING:
            risingEdge = true;
            fallingEdge = false;
            break;
        case FALLING:
            risingEdge = false;
            fallingEdge = true;
            break;
        case BOTH:
            risingEdge = true;
            fallingEdge = true;
            break;
        }
    }

    public boolean update(final boolean newValue) {
        boolean ret = false;
        if (risingEdge && newValue && !mLast) {
            ret = true;
        }
        if (fallingEdge && !newValue && mLast) {
            ret = true;
        }
        mLast = newValue;
        return ret;
    }
}