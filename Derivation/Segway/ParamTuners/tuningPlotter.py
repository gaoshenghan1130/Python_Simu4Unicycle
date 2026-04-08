from matplotlib import pyplot as plt

class tuningPlotter:
    def __init__(self, topic, ax_real_name, ax_try_name, ax_real_set, columnIndex):
        self.topic = topic
        self.ax_real_name = ax_real_name
        self.ax_try_name = ax_try_name
        self.ax_real_set = ax_real_set
        self.fig = None
        self.ax = None
        self.columnIndex = columnIndex

    def start(self):
        print(f"Initialized tuningPlotter with topic: {self.topic}, ax_real_name: {self.ax_real_name}, ax_try_name: {self.ax_try_name}")

        # plot the real data
        plt.ion()
        self.fig, self.ax = plt.subplots()
        self.ax.plot(self.ax_real_set, label=self.ax_real_name)
        self.ax.set_title(self.topic)
        self.ax.set_xlabel("Time step")
        self.ax.set_ylabel("Value")
        self.ax.legend()


    def update(self, ax_try_set, columnIndex=1):
            if self.fig is None or self.ax is None:
                raise ValueError("Plotter not initialized. Please call start() before update().")
            
            t_try = ax_try_set[0]  
            v_try = ax_try_set[columnIndex] 

            self.ax.clear()

            real_x = [item[0] for item in self.ax_real_set]
            real_y = [item[1] for item in self.ax_real_set]
            
            self.ax.plot(real_x, real_y, label=self.ax_real_name) 
            
            self.ax.plot(t_try, v_try, label=self.ax_try_name)
            
            self.ax.set_title(self.topic)
            self.ax.set_xlabel("Time / Step")
            self.ax.set_ylabel("Value")
            self.ax.legend()
            
            # 必须调用 draw 或 pause 才能在 ion 模式下刷新界面
            plt.draw()
            plt.pause(0.01)
