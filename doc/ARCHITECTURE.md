# FSAE Formula Electric — STM32 Firmware Documentation Template

> **Purpose:** Standardized, reusable documentation for all STM32-based firmware projects within the team. Designed for FreeRTOS-based systems with CAN communication and modular task-based architecture.

---

# 1. Project Overview

* **Project Name:** VCU 2.0
* **Module:** VCU
* **Microcontroller:** STM32F446RE
* **Firmware Version:** 2.0
* **Maintainers:** Mason Pronger, Caleb Pollreis, Ethan Alexander, Evan Mack 
* **Last Updated:** March 21st 2026

## 1.1 Description

The Vehicle Control Unit (VCU) serves as the primarlly supervisory controller, interpreting driver inputs to manage the vehicle's global state machine. The VCU issues state-change requests to the ACU; the transition is finalized only upon receipt of a positive acknowledgment (ACK). If the ACU cannot safely execute the request, it returns a negative acknowledgment (NACK), and the VCU maintains the current safe state.

## 1.2 Key Responsibilities

* Read TSA, RTD, and break pedal inputs for state changes
* Change dashboard LED to communicate to the driver the current state of the vehicle.
* Watch BMS and motor controller hearbeats for any faults in the system
* Perform state change requests to the ACU to handle global vehicle state.

## 1.3 Dependencies

* Hardware peripherals (ADC, CAN, UART)
* Middleware (FreeRTOS, HAL, etc.)

---

# 2. System Architecture

## 2.1 High-Level Diagram

*(Insert diagram here — block diagram of tasks, queues, interrupts, peripherals)*

## 2.2 Architectural Principles

* Deterministic timing using RTOS scheduling
* Separation of concerns via task modularization
* Non-blocking communication
* Fault tolerance and watchdog integration

---

# 3. FreeRTOS Overview

## 3.1 Scheduler Model

* Preemptive / Cooperative: Preeemptive
* Tick Rate (Hz): 1000
* Idle Hook Usage: N/a

## 3.2 Task States

* Running
* Ready
* Blocked
* Suspended

## 3.3 Priority Scheme

| Priority Level | Purpose            | Example Tasks  |
| -------------- | ------------------ | -------------- |
| High           | Critical real-time | Safety, CAN RX |
| Medium         | Control loops      | Sensor fusion  |
| Low            | Background         | Logging        |

## 3.4 Timing Considerations

* Worst-case execution time (WCET):
* Deadline requirements:
* Jitter tolerance:

---

# 4. Time-Multiplexed Threading Model

## 4.1 Concept

- Determinism: An RTOS guarantees tasks are executed within a predictable, fixed time frame, which is essential for time-sensitive applications.

- Multitasking: Multitasking in a RTOS involves scheduling and managing multiple tasks, determining the order in which tasks are executed, and switching between them rapidly, giving the impression of parallel execution. 

- Memory management: An RTOS prevents tasks from interfering with each other’s memory space, improving system stability and security.

- Minimal latency: An RTOS reduces response time to external events or interrupts, ensuring quick reactions in real-time environments and minimizing interrupt latency. Efficient context switching in an RTOS further minimizes task-switching latency, allowing tasks to be quickly swapped in and out of the CPU, which reduces delays between executions and enhances overall system responsiveness.

- Priority-based scheduling: An RTOS executes higher-priority tasks before lower-priority ones, making sure critical tasks are handled first.

- Resource allocation: An RTOS efficiently handles memory allocation, processing power and other system resources to support real-time performance.

- Interrupt handling: An RTOS quickly and efficiently responds to hardware or software interrupts using RTOS application programming interface (API) mechanisms. This capability minimizes the time spent handling interrupts and ensures real-time task completion.

- Task synchronization: An RTOS provides inter-task communication (ITC) with mechanisms like semaphores and message queues to synchronize tasks and ensure safe sharing of resources among multiple tasks.
## 4.2 Time Slicing Strategy

* Time slice duration:
* Round-robin usage: Yes / No
* Cooperative sections:

## 4.3 Deterministic Behavior

* Use of vTaskDelayUntil()
* Avoidance of blocking calls

## 4.4 Critical Sections

* Mutex usage:
* Interrupt masking:

---

# 5. Inter-Process Communication (IPC)

## 5.1 Message Queues

### 5.1.1 Design

* Queue length:
* Element size:
* Blocking behavior:

### 5.1.2 Example

```c
// Example queue send
xQueueSend(queueHandle, &data, portMAX_DELAY);
```

### 5.1.3 Use Cases

* Sensor data transfer
* Command dispatch

## 5.2 Semaphores

* Binary / Counting
* Use cases:

## 5.3 Mutexes

* Priority inheritance: Enabled / Disabled

## 5.4 Event Groups (Optional)

* Bitmask usage:

---

# 6. CAN Communication

## 6.1 Overview

Explain how CAN is used in this ECU.

## 6.2 Configuration

* Bitrate:
* Mode: Normal / Loopback / Silent

## 6.3 Message Structure

| Field | Description |
| ----- | ----------- |
| ID    |             |
| DLC   |             |
| Data  |             |

## 6.4 RX Handling

* Interrupt-driven / Polling
* Buffering strategy
* Error handling

## 6.5 TX Handling

* Queue-based transmission
* Priority handling

## 6.6 CAN Database (DBC Reference)

* File:
* Version:

---

# 7. Task Breakdown

> Each major function should be encapsulated in a dedicated RTOS task.

## 7.1 Task Template

### Task Name:

* **Purpose:**
* **Priority:**
* **Stack Size:**
* **Periodicity:** (e.g., 10ms, event-driven)

### Responsibilities

*
*

### Inputs

*

### Outputs

*

### Dependencies

*

### Failure Modes

*

---

## 7.2 Example Tasks

### CAN Receive Task

* Handles incoming CAN messages
* Pushes data to queues

### Sensor Task

* Reads ADC / sensors
* Applies filtering

### Control Task

* Executes control algorithms

### Logging Task

* Stores or transmits debug data

---

# 8. Interrupts and ISRs

## 8.1 Interrupt Sources

* CAN RX
* Timer
* ADC

## 8.2 ISR Guidelines

* Keep ISRs short
* Defer work to tasks
* Use FromISR APIs

## 8.3 Example

```c
BaseType_t xHigherPriorityTaskWoken = pdFALSE;
xQueueSendFromISR(queue, &data, &xHigherPriorityTaskWoken);
portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
```

---

# 9. Timing and Scheduling

## 9.1 Task Frequencies

| Task | Frequency | Deadline |
| ---- | --------- | -------- |
|      |           |          |

## 9.2 System Tick

* Tick frequency:

## 9.3 Watchdog Integration

* IWDG usage:
* Refresh strategy:

---

# 10. Error Handling and Fault Management

## 10.1 Fault Detection

* Sensor failures
* Communication loss

## 10.2 Recovery Strategies

* Retry logic
* Safe state fallback

## 10.3 Logging

* Error codes
* Debug output

---

# 11. Hardware Abstraction Layer (HAL)

## 11.1 Peripheral Usage

| Peripheral | Purpose |
| ---------- | ------- |
| ADC        |         |
| CAN        |         |
| UART       |         |

## 11.2 Driver Design

* HAL vs LL usage
* Custom drivers

---

# 12. Build and Deployment

## 12.1 Toolchain

* IDE:
* Compiler:
* Build system: (CMake, Make, etc.)

## 12.2 Flashing Procedure

* ST-Link / DFU / Bootloader

## 12.3 Configuration

* CubeMX settings

---

# 13. Testing and Validation

## 13.1 Unit Testing

* Framework: (e.g., Ceedling)

## 13.2 Integration Testing

* Hardware-in-the-loop (HIL)

## 13.3 Simulation

* Renode / custom simulation

---

# 14. Coding Standards

* Naming conventions: All struct should end with the _t postfix
* File structure
* Documentation requirements: Any system level changes or addiditions should be written and documented.

---

# 15. Future Improvements

*
*

---

# 16. Appendix

## 16.1 Glossary

| Term | Definition |
| ---- | ---------- |
| RTOS | Real time Operating System           |
| IPC  | Interprocess communication           |

## 16.2 References

* FreeRTOS Documentation
* STM32 Reference Manual
* Team Design Docs

---

# Notes

* Keep this document updated alongside firmware changes.
* Every task added must be documented.
* All IPC mechanisms must be explicitly described.
