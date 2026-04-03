package org.griffins1884.griffinsim.runtime;

import java.util.ArrayList;
import java.util.List;
import java.util.Objects;
import java.util.Optional;
import java.util.function.Consumer;

public final class DeterministicRingBuffer<T> {
  private final Object[] elements;
  private int head;
  private int tail;
  private int size;

  public DeterministicRingBuffer(int capacity) {
    if (capacity <= 0) {
      throw new IllegalArgumentException("capacity must be positive");
    }
    this.elements = new Object[capacity];
  }

  public synchronized boolean offer(T element) {
    Objects.requireNonNull(element, "element");
    if (size == elements.length) {
      return false;
    }
    elements[tail] = element;
    tail = (tail + 1) % elements.length;
    size++;
    return true;
  }

  public synchronized Optional<T> poll() {
    if (size == 0) {
      return Optional.empty();
    }
    @SuppressWarnings("unchecked")
    T element = (T) elements[head];
    elements[head] = null;
    head = (head + 1) % elements.length;
    size--;
    return Optional.of(element);
  }

  public synchronized int drainTo(Consumer<? super T> consumer) {
    Objects.requireNonNull(consumer, "consumer");
    int drained = 0;
    while (size > 0) {
      consumer.accept(poll().orElseThrow());
      drained++;
    }
    return drained;
  }

  public synchronized List<T> snapshot() {
    List<T> copy = new ArrayList<>(size);
    for (int i = 0; i < size; i++) {
      @SuppressWarnings("unchecked")
      T element = (T) elements[(head + i) % elements.length];
      copy.add(element);
    }
    return List.copyOf(copy);
  }

  public synchronized int size() {
    return size;
  }

  public synchronized boolean isEmpty() {
    return size == 0;
  }
}
