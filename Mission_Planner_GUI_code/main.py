import sys
from multiprocessing import Process, Queue
from dead_reckoning import dead_reckoning_simulation
from gui import MainWindow  # Ensure this imports the MainWindow class correctly
from PyQt5.QtWidgets import QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, QLabel, QDial

def start_data_simulation(queue):
    """Start the dead reckoning simulation process."""
    simulation_process = Process(target=dead_reckoning_simulation, args=(queue,))
    simulation_process.start()
    return simulation_process

def main():
    # Create a queue for communication between processes
    queue = Queue()
    
    print(f"Queue created: {queue}")  # Debugging queue creation

    try:
        # Start the dead reckoning simulation process
        simulation_process = start_data_simulation(queue)
        

        # Initialize and run the GUI in the main process
        app = QApplication(sys.argv)
        main_window = MainWindow(queue)
        main_window.show()
        sys.exit(app.exec_())

    except KeyboardInterrupt:
        print("Terminating processes...")
    
    finally:
        # Ensure the simulation process is terminated cleanly
        if simulation_process.is_alive():
            simulation_process.terminate()

        print("Processes terminated.")

if __name__ == '__main__':
    main()
