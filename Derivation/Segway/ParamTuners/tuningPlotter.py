from matplotlib import pyplot as plt

class tuningPlotter:
    REAL_COLOR = '#9ca3af'
    POSITION_COLOR = '#f59e0b'
    VELOCITY_COLOR = '#3860e0'
    TARGET_COLOR = '#dc2626'
    FIGSIZE = (10, 4.5)

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
        self.fig, self.ax = plt.subplots(figsize=self.FIGSIZE)
        self.plotRealData()  # Plot the real data first
        self.ax.set_xlabel("Time step")
        self.ax.set_ylabel("Value")
        self.ax.set_xlim(0,16)
        self._plot_target()
        self._style_axis()

    def _is_velocity_plot(self):
        text = f"{self.topic} {self.ax_real_name} {self.ax_try_name}".lower()
        return "velocity" in text

    def _try_color(self):
        return self.VELOCITY_COLOR if self._is_velocity_plot() else self.POSITION_COLOR

    def _target_value(self):
        return 0.5 if self._is_velocity_plot() else 1.0

    def _style_axis(self):
        if self.ax is None:
            return
        self.ax.grid(False)
        self.ax.spines['top'].set_visible(False)
        self.ax.spines['right'].set_visible(False)
        self.ax.tick_params(direction='out')
        self.ax.legend(frameon=False)

    def _plot_target(self):
        if self.ax is None:
            return
        target_value = self._target_value()
        self.ax.axhline(
            y=target_value,
            color=self.TARGET_COLOR,
            ls='--',
            linewidth=1.3,
            label=f'Target {target_value:g}',
        )

    def update(self, ax_try_set):
            if self.fig is None or self.ax is None:
                raise ValueError("Plotter not initialized. Please call start() before update().")
            
            t_try = ax_try_set[0]  
            v_try = ax_try_set[self.columnIndex] 

            self.ax.clear() 
            self.plotRealData()
            self.ax.plot(
                t_try,
                v_try,
                label=self.ax_try_name,
                color=self._try_color(),
                linewidth=3.0,
                zorder=3,
            )
            self.ax.set_xlim(0,16)
            self.ax.set_xlabel("Time(s)")
            self.ax.set_ylabel("Value")
            self._plot_target()

            self._style_axis()
            plt.draw()
            plt.pause(0.01)

    def plotRealData(self):
        if isinstance(self.ax_real_set, list) and len(self.ax_real_set) > 0 and isinstance(self.ax_real_set[0], tuple):
            for plot_index, (_, real_set) in enumerate(self.ax_real_set):
                if self.ax is None:
                    continue

                real_x = [item[0] for item in real_set]
                real_y = [item[1] for item in real_set]

                self.ax.plot(
                    real_x,
                    real_y,
                    label=self.ax_real_name if plot_index == 0 else None,
                    color=self.REAL_COLOR,
                    linewidth=1.4,
                    alpha=0.55,
                    zorder=2,
                )
        else:
            real_x = [item[0] for item in self.ax_real_set]
            real_y = [item[1] for item in self.ax_real_set]
            if self.ax is not None:
                self.ax.plot(
                    real_x,
                    real_y,
                    label=self.ax_real_name,
                    color=self.REAL_COLOR,
                    linewidth=2.0,
                    alpha=0.9,
                    zorder=2,
                )
