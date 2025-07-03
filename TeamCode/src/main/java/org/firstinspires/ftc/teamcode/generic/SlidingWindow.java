package org.firstinspires.ftc.teamcode.generic;

import java.util.ArrayList;
import java.util.Optional;

/// A type which holds the last n values pushed into it.
///
/// Pushing one to the end (when it is full) removes the first one and moves all other elements over.
///
/// Useful for averaging a reading or calculating a rate of change for a slightly larger delta t
public class SlidingWindow<T> {
	ArrayList<T> values;

	int size;

	/// Creates the window with the provided size
	public SlidingWindow(int window_size) {
		size = window_size;
		values = new ArrayList<T>(window_size);
	}

	/// Returned the number of elements pushed inside the window
	public int length() {
		return values.size();
	}

	/// Pushes an element to the end of the window
	public void push(T element) {
		if (length() == size) {
			values.remove(0);
		}

		values.add(element);
	}

	/// Returns the first element in the window, if any
	public Optional<T> first() {
		if (values.isEmpty()) {
			Optional.empty();
		}

		return Optional.of(values.get(0));
	}


	/// Returns the last element in the window, if there is at least one
	public Optional<T> last() {
		if (values.isEmpty()) {
			Optional.empty();
		}

		return Optional.of(values.get(values.size() - 1));
	}
}
