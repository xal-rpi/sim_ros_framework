"""
LoggerProcess implements a separate process that receives sensor data
via a multiprocessing.Queue. It extracts the vehicle name, sensor name,
and data (which is a list of dictionaries with a time field), and organizes
them into structured time series for each sensor field. Every flush_interval seconds,
it saves the structured data into a new pickle file inside the specified folder 
(save_location) with a unique filename.
"""

import os
import time
import pickle
from multiprocessing import Process, Event, Queue
from queue import Empty


class LoggerProcess(Process):
    def __init__(self, data_queue, save_location, flush_interval=5, poll_interval=0.5):
        """
        Initialize the LoggerProcess.

        Args:
            data_queue (Queue): Queue instance from which sensor data is read.
            save_location (str): Folder where flushed data files will be saved.
            flush_interval (int, optional): Interval (in seconds) to flush data.
            poll_interval (float, optional): Sleep time (in seconds) per loop iteration.
        """
        super().__init__(name="LoggerProcess")
        self.data_queue: Queue = data_queue
        # Ensure that the save_location folder exists.
        if not os.path.exists(save_location):
            os.makedirs(save_location)
        self.save_location = save_location
        self.flush_interval = flush_interval
        self.poll_interval = poll_interval
        self.stop_event = Event()
        self.buffer = []
        self.file_counter = 0

    def run(self):
        """
        Run the logger process. Continuously drain the queue and store items
        in the buffer. Sleep for poll_interval on each iteration. When flush_interval
        elapses, flush the buffer to a new file.
        """
        last_flush = time.time()

        while not self.stop_event.is_set():
            # Drain the queue completely.
            try:
                while True:
                    m_data = self.data_queue.get_nowait()
                    self.buffer.append(m_data)
            except Empty:
                pass

            if (time.time() - last_flush) >= self.flush_interval and self.buffer:
                self.flush_data()
                last_flush = time.time()

            time.sleep(self.poll_interval)

        # Final flush on stop
        self.flush_data()

    def flush_data(self):
        """
        Process the buffered items by grouping them according to
        vehicle_name and sensor_name. For each group, each field is aggregated
        into a time series (list of values). The resulting structured dictionary is
        saved to a new pickle file inside the save_location folder with a unique filename.
        """
        if not self.buffer:
            return

        full_data = {}
        # Group each message in the buffer.
        for item in self.buffer:
            vehicle = item["vehicle_name"]
            sensor = item["sensor_name"]
            records = item["data"]
            # Append to the group
            if (vehicle, sensor) not in full_data:
                full_data[(vehicle, sensor)] = {}
            curr_data = full_data[(vehicle, sensor)]
            for rec in records:
                for key, value in rec.items():
                    if isinstance(value, dict):
                        for sub_key, sub_value in value.items():
                            new_key = f"{key}_{sub_key}"
                            if new_key not in curr_data:
                                curr_data[new_key] = []
                            curr_data[new_key].append(sub_value)
                    elif isinstance(value, list):
                        assert len(value) <= 4, f"Invalid value: {value}"
                        list_name = [f"{key}_x", f"{key}_y", f"{key}_z", f"{key}_w"]
                        for i, sub_value in enumerate(value):
                            list_name_i = list_name[i]
                            if list_name_i not in curr_data:
                                curr_data[list_name_i] = []
                            curr_data[list_name_i].append(sub_value)
                    else:
                        if key not in curr_data:
                            curr_data[key] = []
                        curr_data[key].append(value)

        # Save the structured output to a new file.
        file_name = os.path.join(
            self.save_location, f"data_{self.file_counter:03d}.pkl"
        )
        try:
            with open(file_name, "wb") as f:
                pickle.dump(full_data, f)
            self.file_counter += 1
            self.buffer = []
        except Exception as e:
            print("Flush error:", e)

    def stop(self):
        """
        Signal the logger process to stop.
        """
        self.stop_event.set()
