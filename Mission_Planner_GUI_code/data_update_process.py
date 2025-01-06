import multiprocessing
import time
from queue import Empty

def data_update(queue):
    while True:
        try:
            # Try to get data from the queue with a timeout
            data = queue.get(timeout=2)  # Blocking get with a 2-second timeout

            # Print the updated data after retrieving it from the queue
            print("Updated Data:", data)
            
            # Sleep for a short duration to prevent CPU overload
            time.sleep(0.5)
            
        except Empty:
            print("Queue is empty. No data to retrieve.")
            
        except Exception as e:
            print(f"Error: {e}")
            break  # Exit loop on error

if __name__ == "__main__":
    # Create a multiprocessing queue
    queue = multiprocessing.Queue()

    # Start the data update process
    process = multiprocessing.Process(target=data_update, args=(queue,))
    process.start()

    try:
        # Example code to simulate putting data into the queue
        for i in range(10):
            print(f"Putting data into queue: Data {i}")  # Print statement to show data being put into the queue
            queue.put(f"Data {i}")
            time.sleep(2)  # Simulate time delay between data updates

    except KeyboardInterrupt:
        print("Interrupted by user")

    finally:
        process.terminate()
        process.join()
        print("Process terminated")
