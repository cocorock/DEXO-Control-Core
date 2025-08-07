# State Lock Documentation - Emergency Stop System

## Overview

This document explains the state lock mechanism used in the Emergency Stop Node to prevent race conditions and deadlocks in our multi-threaded ROS system.

## 🔒 What is a State Lock?

Think of a state lock like a **bathroom door lock**:

- Only **one person** can use the bathroom at a time
- When someone goes in, they **lock the door**
- Anyone else who tries to enter has to **wait outside** until the door is unlocked
- This prevents **chaos** (two people trying to use the bathroom simultaneously)

In our code:
- The "bathroom" = **shared data** (like `emergency_active`, `current_state`)
- "People" = **different threads** (parts of code running simultaneously)
- The lock ensures only **one thread** can modify the data at a time

```python
with self.state_lock:  # "Lock the bathroom door"
    self.emergency_active = True  # "Use the bathroom safely"
# "Automatically unlock when done"
```

## ☠️ What is a Deadlock?

A deadlock is like this scenario:

**Person A** is in Bathroom 1, but needs to use Bathroom 2
**Person B** is in Bathroom 2, but needs to use Bathroom 1

- Person A **waits** for Person B to finish
- Person B **waits** for Person A to finish  
- **Nobody can move!** They're stuck forever waiting for each other

## 🐛 Our Deadlock Problem (Before Fix)

Here's what was happening in our code:

```python
def crutch_command_callback(self, msg):
    with self.state_lock:  # 🔒 "Person A locks Bathroom 1"
        # ... do stuff ...
        self.trigger_emergency_stop()  # "Person A needs Bathroom 2"

def trigger_emergency_stop(self):
    with self.state_lock:  # 🔒 "Trying to lock Bathroom 1 again"
        # ☠️ DEADLOCK! Person A is waiting for Person A to unlock Bathroom 1
```

**The problem:** The same thread tried to lock the **same lock twice**!

## ✅ How We Fixed It

```python
def crutch_command_callback(self, msg):
    if msg.command == SHUTDOWN:
        with self.state_lock:  # 🔒 "Quick lock to set flag"
            self.shutdown_trig = True
        # 🔓 "Release lock immediately"
        
        # Now call emergency stop WITHOUT holding the lock
        self.trigger_emergency_stop()  # ✅ "No deadlock!"

def trigger_emergency_stop(self):
    with self.state_lock:  # 🔒 "Fresh lock acquisition - no problem!"
        self.emergency_active = True
```

## 🗂️ Shared Data in EmergencyStopNode

### 🚨 Emergency State Variables
```python
self.emergency_active = False      # Is emergency stop currently active?
self.current_state = "INIT"        # Current FSM state (INIT, READY, WALKING, etc.)
self.previous_state = "INIT"       # Previous FSM state for transitions
```

### ⚡ Trigger Flags (Set by external events)
```python
self.st_calibration_trig = False   # Calibration command received
self.st_walking_trig = False       # Walking command received  
self.stop_trig = False             # Stop command received
self.shutdown_trig = False         # Emergency shutdown command received
```

### 📊 System Monitoring Data
```python
self.last_motor_update = rospy.Time.now()  # When we last heard from motors
self.calibration_completed = False         # Has calibration finished?
self.calibration_failed = False           # Did calibration fail?
self.cycle_finished = False               # Has walking cycle finished?
```

### ⚙️ Configuration Data (Read-only after init)
```python
self.communication_timeout = 2.0    # How long before communication timeout
```

## 🧵 Who Accesses This Shared Data?

### Thread 1: ROS Callback Threads
- `crutch_command_callback()` - Sets trigger flags
- `motor_status_callback()` - Updates `last_motor_update`
- `calibration_complete_callback()` - Sets `calibration_completed`
- `cycle_finished_callback()` - Sets `cycle_finished`

### Thread 2: SMACH State Machine Thread
- Reads trigger flags to decide state transitions
- Updates `current_state` and `previous_state`
- Clears trigger flags after processing

### Thread 3: Monitor Timer Thread
- `monitor_callback()` - Checks `last_motor_update` for timeouts
- May call `trigger_emergency_stop()` on timeout

### Thread 4: FSM Publisher Timer
- `publish_fsm_state_periodically()` - Reads `current_state` to publish

## 🔒 Why State Lock is Critical

Without the lock, this could happen:

```python
# Thread 1 (callback): "Let me set emergency_active = True"
# Thread 2 (state machine): "Let me check if emergency_active == False"
# 💥 RACE CONDITION! They might interfere with each other
```

**Example Race Condition:**
1. Thread 1 reads `emergency_active = False`
2. Thread 2 reads `emergency_active = False` 
3. Thread 1 sets `emergency_active = True`
4. Thread 2 also sets `emergency_active = True`
5. **Both think they triggered the emergency stop first!**

## 🎯 What the State Lock Protects

```python
with self.state_lock:
    # 🔒 Only ONE thread can be in here at a time
    
    # Safe to read/write these variables:
    if not self.emergency_active:           # ✅ Safe read
        self.emergency_active = True        # ✅ Safe write
        self.current_state = "E_STOP"       # ✅ Safe write
        self.st_calibration_trig = False    # ✅ Safe write
        
    # 🔓 Lock automatically released when exiting this block
```

## 📋 Best Practices

### ✅ DO:
- Use `with self.state_lock:` when reading/writing shared variables
- Keep lock sections as **short as possible**
- Release locks before calling functions that might need the same lock
- Handle emergency shutdown outside the main state lock

### ❌ DON'T:
- Call functions that acquire the same lock while holding the lock
- Hold locks for long periods (blocks other threads)
- Access shared variables without proper locking
- Nest multiple locks (can cause deadlocks)

## 🔧 Implementation Details

### Current State Lock Usage:
```python
# File: emergency_stop_node.py
self.state_lock = threading.Lock()

# Safe pattern for emergency shutdown:
def crutch_command_callback(self, msg):
    if msg.command == CrutchCommand.SHUTDOWN:
        with self.state_lock:
            self.shutdown_trig = True  # Quick lock for flag setting
        # Release lock before calling trigger_emergency_stop
        self.trigger_emergency_stop("Shutdown command from crutches")

def trigger_emergency_stop(self, reason):
    with self.state_lock:  # Fresh lock acquisition
        if not self.emergency_active:
            self.emergency_active = True
            self.current_state = SystemStates.E_STOP
            self.publish_emergency_stop()
```

## 📚 Summary

**Shared Data** = Variables accessed by multiple threads:
- Emergency flags and states
- Trigger flags from user commands  
- Motor communication timestamps
- State machine status

**State Lock** = Bathroom door lock ensuring only one thread can modify shared data at a time

**Deadlock Prevention** = Don't hold locks when calling functions that need the same lock

This prevents **chaos** when multiple parts of the system try to update the same variables simultaneously!