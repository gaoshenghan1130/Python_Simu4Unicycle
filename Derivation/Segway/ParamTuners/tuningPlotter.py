from matplotlib import pyplot as plt
from typing import Dict, Tuple
from IPython import display 
import csv
import os

class tuningPlotter:
    def __init__(self, topic, ax_real_name, ax_try_name, ax_real_set, columnIndex):
        self.topic = topic
        self.ax_real_name = ax_real_name
        self.ax_try_name = ax_try_name
        self.ax_real_set = ax_real_set
        self.fig = None
        self.ax = None
        self.columnIndex = columnIndex

        print("real_set example:", self.ax_real_set)  # Debug print to check the structure of real_set

    def start(self):
        print(f"Initialized tuningPlotter with topic: {self.topic}, ax_real_name: {self.ax_real_name}, ax_try_name: {self.ax_try_name}")

        # plot the real data
        plt.ion()
        self.fig, self.ax = plt.subplots()
        self.plotRealData()  # Plot the real data first
        self.ax.set_title(self.topic)
        self.ax.set_xlabel("Time step")
        self.ax.set_ylabel("Value")
        self.ax.legend()
        self.last_try_data = None


    def update(self, ax_try_set):
            if self.fig is None or self.ax is None:
                raise ValueError("Plotter not initialized. Please call start() before update().")
            
            t_try = ax_try_set[0]  
            v_try = ax_try_set[self.columnIndex] 
            self.last_try_data = ax_try_set

            self.ax.clear() 
            self.ax.plot(t_try, v_try, label=self.ax_try_name)
            
            self.ax.set_title(self.topic)
            self.ax.set_xlabel("Time / Step")
            self.ax.set_ylabel("Value")
            
            
            self.plotRealData()  # Re-plot the real data to ensure it stays visible after clearing the axes

            self.ax.legend()
            plt.draw()
            plt.pause(0.01)

    def plotRealData(self):
        if isinstance(self.ax_real_set, list) and len(self.ax_real_set) > 0 and isinstance(self.ax_real_set[0], Tuple):
            for i, real_set in self.ax_real_set:
                if self.ax is not None:

                    real_x = [item[0] for item in real_set]
                    real_y = [item[1] for item in real_set]

                    self.ax.plot(real_x, real_y, label=(self.ax_real_name + f" {i}"))
        else:
            real_x = [item[0] for item in self.ax_real_set]
            real_y = [item[1] for item in self.ax_real_set]
            if self.ax is not None:
                self.ax.plot(real_x, real_y, label=self.ax_real_name)

    def export_data(self, filename="plot_data.csv"):
        os.makedirs("LOG", exist_ok=True)
        filepath = os.path.join("LOG", filename)

        with open(filepath, "w", newline="") as f:
            writer = csv.writer(f)

            writer.writerow(["type", "time", "x_c", "x_c_dot", "gamma", "gamma_dot"])

            if isinstance(self.ax_real_set, list) and len(self.ax_real_set) > 0 and isinstance(self.ax_real_set[0], Tuple):
                for i, real_set in self.ax_real_set:
                    for t, v in real_set:
                        writer.writerow([f"real_{i}", t, v, "", "", ""]) 
            else:
                for t, v in self.ax_real_set:
                    writer.writerow(["real", t, v, "", "", ""])

            if self.last_try_data is not None:
                for row in zip(*self.last_try_data):
                    writer.writerow(["try", *row])
            else:
                print("No try data to export.")

        print(f"Data exported to {filepath}")
