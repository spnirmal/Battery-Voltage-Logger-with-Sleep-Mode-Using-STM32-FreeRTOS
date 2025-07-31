# Battery Voltage Logger with Sleep Mode Using STM32 + FreeRTOS

This project is a **battery voltage logger** developed on an **STM32 Nucleo-F030R8** board using **FreeRTOS**.

I wanted to create a **real-world, production-style firmware** that does more than just blink LEDs. My goal was to demonstrate:

* **FreeRTOS proficiency** (tasks, queues, notifications, power control)
* **STM32 peripheral handling** (ADC, UART, GPIO)
* **Power-aware design** (sleep mode transitions)
* **Robust bare-metal + RTOS architecture**

---

## Hardware Setup

| Component        | Description                                  |
| ---------------- | -------------------------------------------- |
| STM32F030R8      | Cortex-M0 Nucleo board (64KB Flash, 8KB RAM) |
| Coin cell holder | Connected to ADC pin via voltage divider     |
| SD card module   | *(Future scope)* For on-board CSV logging    |
| UART             | For live data log (115200 baud via USART2)   |
| GPIO Button      | To trigger exit from sleep mode (EXTI based) |

A voltage divider is used in this project in case we have to read a cell voltage above the ADC’s input range (3.3V). This setup ensures voltage levels are always within the measurable range of the STM32 ADC.

### MCU Specifications and Peripheral Usage

The project is based on the **STM32F030R8** microcontroller:

* **Core**: ARM Cortex-M0 (48 MHz)
* **Flash**: 64KB
* **RAM**: 8KB
* **Peripherals Used**:

  * **ADC1**: For reading voltage from the divided coin cell signal
  * **USART2**: For serial terminal communication
  * **EXTI (External Interrupt)**: Configured on the user button pin for waking from sleep mode
  * **GPIOs**: For general signal interfacing and board button input

These peripherals are configured manually or using STM32CubeMX-generated code and initialized during system startup before the FreeRTOS scheduler begins execution.

---

## Memory Footprint and Queue Handling

The MCU has very limited SRAM (8KB), so memory usage had to be optimized carefully.

* `q_print` is a FreeRTOS queue created to hold **10 items** of type `char[20]`. This results in **200 bytes** allocated for the queue.
* Each task was allocated **128 words** (512 bytes) of stack size, totaling \~1.7KB RAM usage excluding heap.

Queue `q_print` acts as a communication medium between the `voltagetask` (producer) and `logtask` (consumer). By isolating UART transmission to only one task (`logtask`), we avoid concurrency issues and remove the need for heavier synchronization primitives like semaphores or mutexes.

---

## Task Design

We skip straight to task creation to focus on core logic. Each task performs an isolated responsibility within the RTOS ecosystem:
![task creation](https://github.com/spnirmal/Battery-Voltage-Logger-with-Sleep-Mode-Using-STM32-FreeRTOS/blob/main/.assets/image.png)

| Task Name      | Priority | Role                                                                 |
| -------------- | -------- | -------------------------------------------------------------------- |
| `voltagetask`  | 2        | Periodically samples ADC and formats voltage string                  |
| `logtask`      | 2        | Waits for formatted message and transmits it over UART               |
| `lowpowertask` | 3        | Waits for notification when voltage drops and puts MCU in sleep mode |

> Although `lowpowertask` has a higher priority, it is blocked by default and only runs when notified by `voltagetask`. This guarantees power-saving actions are taken immediately without affecting sampling behavior during normal operation.

### voltagetask
![volatge task](https://github.com/spnirmal/Battery-Voltage-Logger-with-Sleep-Mode-Using-STM32-FreeRTOS/blob/main/.assets/voltage.png)

* Reads ADC value from the analog pin (voltage divider connected to coin cell)
* Converts it to float voltage
* Originally used `snprintf()` but replaced with manual formatting to save memory
* Sends formatted voltage string to `q_print`
* If voltage < threshold, notifies `lowpowertask` and suspends itself
* Notifies `logtask` to log each reading

### logtask
![log task](https://github.com/spnirmal/Battery-Voltage-Logger-with-Sleep-Mode-Using-STM32-FreeRTOS/blob/main/.assets/log.png)
* Waits for notification using `ulTaskNotifyTake()`
* Once notified, receives a string from `q_print`
* Sends the message via UART (USART2)

Using task notifications instead of semaphores or event groups was a conscious choice to conserve stack and heap memory. The system only permits `logtask` to access UART, which reduces coupling and enables N number of producers to safely send UART data indirectly through the queue.

`pdTRUE` is a FreeRTOS-defined macro equivalent to `1`, used in task synchronization APIs such as `ulTaskNotifyTake()` to indicate that the task should reset the notification value before waiting again.

### lowpowertask
![lowpower task](https://github.com/spnirmal/Battery-Voltage-Logger-with-Sleep-Mode-Using-STM32-FreeRTOS/blob/main/.assets/lowpower.png)
* Triggered only when voltage falls below a set threshold (e.g., 0.50V)
* Sends a direct UART message saying "Entering sleep mode"
* Enables GPIO external interrupt on the user button
* MCU enters **WFI (Wait For Interrupt)** low-power mode
* Once button is pressed, the ISR resumes `voltagetask` and normal operation continues

---

## Working Demo & Testing

To simulate the low voltage condition, I manually placed a wire between the positive and negative terminals of the coin cell holder, which causes the ADC to read 0V. Since this is below the 0.50V threshold, `voltagetask` suspends itself, and `lowpowertask` takes over, putting the MCU to sleep.

![sleep](https://github.com/spnirmal/Battery-Voltage-Logger-with-Sleep-Mode-Using-STM32-FreeRTOS/blob/main/.assets/sleepop.png)

Upon pressing the user button, the external interrupt wakes the MCU, and `voltagetask` is resumed.

![interrupt](https://github.com/spnirmal/Battery-Voltage-Logger-with-Sleep-Mode-Using-STM32-FreeRTOS/blob/main/.assets/interruptop.png)

This interaction was verified using UART output. Two screenshots were captured to demonstrate the logging and transition into sleep mode.

---

## Why Memory Optimization Was Critical

This MCU offers only **8KB of RAM**, so every function and buffer matters:

* Standard functions like `printf()` and `snprintf()` can consume significant stack space, especially when called frequently.
* I implemented manual formatting logic to keep RAM usage minimal and deterministic.
* Stack size for all tasks was minimized while ensuring stability under execution.
* Queue length and buffer sizes were tuned to remain within acceptable limits for sampling and logging frequencies.

This tight memory constraint is also the reason SD card logging (planned as a feature) was deferred. A heavier file system stack like FatFs would require more RAM and heap, which currently can't be afforded on this MCU.

---

## Future Enhancements

* Integrate SD card logging in CSV format
* Add battery aging estimation with timestamp logging
* Use DMA for both ADC and UART to offload CPU
* Integrate RTC and timekeeping for timestamped logs

---

## Summary

This project balances **minimal memory use**, **real-time responsiveness**, and **power-aware operation** using the STM32 platform. Despite the limitations of the STM32F030R8 (especially its RAM), the firmware is robust and modular, serving as a strong foundation for scalable enhancements in future.

The architecture promotes separation of concerns and RTOS best practices:

* Only one task accesses UART
* All data is passed via queues
* Tasks are statically prioritized
* Notifications are used instead of semaphores for lightweight signaling

This design choice mirrors professional embedded systems development where memory, power, and reliability are critical.
