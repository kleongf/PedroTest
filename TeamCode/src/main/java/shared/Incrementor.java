package shared;

public class Incrementor {
    private int currentIndex;
    private int numValues;
    private int[] values;

    public Incrementor(int numValues) {
        this.numValues = numValues;
        this.values = new int[numValues];
        this.currentIndex = 0;
    }

    public int getValue(int index) {
        return values[index];
    }

    public int getCurrentIndex() {
        return currentIndex;
    }

    public void incrementValue() {
        values[currentIndex] += 1;
    }

    public void decrementValue() {
        values[currentIndex] -= 1;
    }

    public void incrementIndex() {
        if (currentIndex < numValues-1) {
            currentIndex++;
        }
    }

    public void decrementIndex() {
        if (currentIndex > 0) {
            currentIndex--;
        }
    }

    public int[] getValues() {
        return values;
    }
}
