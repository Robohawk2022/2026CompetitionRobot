package frc.robot.util;

import java.util.function.ToDoubleFunction;

/**
 * A fixed-capacity circular buffer that overwrites the oldest elements
 * when full. Useful for rolling statistics and pose history.
 *
 * @param <T> the type of elements stored in the buffer
 */
public class CircularBuffer<T> {

    private final Object[] buffer;
    private final int capacity;
    private int head = 0;
    private int size = 0;

    /**
     * Creates a new circular buffer with the specified capacity.
     *
     * @param capacity the maximum number of elements (must be positive)
     */
    public CircularBuffer(int capacity) {
        if (capacity <= 0) {
            throw new IllegalArgumentException("Capacity must be positive: " + capacity);
        }
        this.capacity = capacity;
        this.buffer = new Object[capacity];
    }

    /**
     * Adds an element to the buffer, overwriting the oldest element if full.
     *
     * @param element the element to add
     */
    public void add(T element) {
        buffer[head] = element;
        head = (head + 1) % capacity;
        if (size < capacity) {
            size++;
        }
    }

    /**
     * Gets the element at the specified index, where 0 is the oldest element.
     *
     * @param index the index (0 = oldest, size-1 = newest)
     * @return the element at that index
     * @throws IndexOutOfBoundsException if index is out of range
     */
    @SuppressWarnings("unchecked")
    public T get(int index) {
        if (index < 0 || index >= size) {
            throw new IndexOutOfBoundsException("Index: " + index + ", Size: " + size);
        }
        int actualIndex = (head - size + index + capacity) % capacity;
        return (T) buffer[actualIndex];
    }

    /**
     * Gets the most recently added element.
     *
     * @return the newest element
     * @throws IllegalStateException if the buffer is empty
     */
    @SuppressWarnings("unchecked")
    public T getLast() {
        if (size == 0) {
            throw new IllegalStateException("Buffer is empty");
        }
        int lastIndex = (head - 1 + capacity) % capacity;
        return (T) buffer[lastIndex];
    }

    /**
     * Gets the oldest element in the buffer.
     *
     * @return the oldest element
     * @throws IllegalStateException if the buffer is empty
     */
    public T getFirst() {
        if (size == 0) {
            throw new IllegalStateException("Buffer is empty");
        }
        return get(0);
    }

    /**
     * @return the current number of elements in the buffer
     */
    public int size() {
        return size;
    }

    /**
     * @return true if the buffer has no elements
     */
    public boolean isEmpty() {
        return size == 0;
    }

    /**
     * @return true if the buffer is at capacity
     */
    public boolean isFull() {
        return size == capacity;
    }

    /**
     * @return the maximum capacity of the buffer
     */
    public int capacity() {
        return capacity;
    }

    /**
     * Clears all elements from the buffer.
     */
    public void clear() {
        for (int i = 0; i < capacity; i++) {
            buffer[i] = null;
        }
        head = 0;
        size = 0;
    }

    /**
     * Calculates the average of numeric values extracted from elements.
     *
     * @param extractor function to extract a double value from each element
     * @return the average of all extracted values, or 0.0 if empty
     */
    public double average(ToDoubleFunction<T> extractor) {
        if (size == 0) {
            return 0.0;
        }
        double sum = 0.0;
        for (int i = 0; i < size; i++) {
            sum += extractor.applyAsDouble(get(i));
        }
        return sum / size;
    }

    /**
     * Finds the maximum of numeric values extracted from elements.
     *
     * @param extractor function to extract a double value from each element
     * @return the maximum of all extracted values, or Double.NEGATIVE_INFINITY if empty
     */
    public double max(ToDoubleFunction<T> extractor) {
        if (size == 0) {
            return Double.NEGATIVE_INFINITY;
        }
        double max = Double.NEGATIVE_INFINITY;
        for (int i = 0; i < size; i++) {
            max = Math.max(max, extractor.applyAsDouble(get(i)));
        }
        return max;
    }
}
