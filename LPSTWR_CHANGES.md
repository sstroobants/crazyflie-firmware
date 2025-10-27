# LPS TWR Tag Communication Protocol Changes

## Overview
This document describes the changes made to the `lpsTwrTag.c` implementation to improve the robustness of the multi-crazyflie localization scheme.

## Problem Statement
The original implementation used a fixed ring protocol for communication between crazyflies in a swarm. This had several issues:
1. **Single Point of Failure**: If one crazyflie in the ring became disabled or unresponsive, the entire swarm communication would break
2. **Fixed Communication Pattern**: Each crazyflie only communicated with specific neighbors (previous and next in ring)
3. **Limited Resilience**: No mechanism to detect and work around non-responsive peers
4. **Difficult Swarm Management**: Hard to add or remove crazyflies dynamically

## Solution Implemented

### 1. Random Peer Selection
Instead of a fixed ring topology, each crazyflie now randomly selects which peer to communicate with. This is implemented through three helper functions:

- **`selectRandomPeer()`**: Selects a random peer from the swarm (excluding self) using the FreeRTOS tick count as a pseudo-random seed
- **`isPeerActive(peerID)`**: Checks if a peer has responded successfully within the last 5 seconds
- **`selectNextPeer()`**: Intelligently selects the next peer to communicate with, preferring active peers but falling back to any peer if none are active

### 2. Peer Activity Tracking
Added tracking of when each peer last successfully completed a ranging operation:
- `lastSuccessfulRanging[]`: Array storing the timestamp of last successful ranging for each peer
- `PEER_TIMEOUT_MS`: Threshold (5000ms) after which a peer is considered potentially inactive

### 3. Timeout Handling
Enhanced timeout handling to automatically switch to different peers:
- `consecutiveTimeouts`: Counter tracking consecutive timeouts with current peer
- `MAX_CONSECUTIVE_TIMEOUTS`: After 3 consecutive timeouts, automatically switch to a different peer
- Reset timeout counter on successful ranging completion

### 4. Distributed Initiation
Instead of the fixed ring protocol where only specific crazyflies could initiate ranging, the new implementation uses a distributed approach:
- After receiving a ranging report, each crazyflie randomly decides whether to initiate the next ranging
- Decision is based on: `(tick % swarmSize) == selfID`
- This ensures roughly equal distribution of ranging initiation across the swarm

## Key Changes by Location

### Global Variables (lines 123-127)
```c
static uint32_t lastSuccessfulRanging[LOCODECK_NR_OF_TWR_ANCHORS + 1];
static uint32_t consecutiveTimeouts = 0;
#define MAX_CONSECUTIVE_TIMEOUTS 3
#define PEER_TIMEOUT_MS 5000
```

### Helper Functions (lines 156-212)
Three new static helper functions for peer selection and activity tracking.

### Ranging Success Tracking (lines 355-357, 497-499)
Both transmit-mode and receive-mode ranging completion now update:
- `lastSuccessfulRanging[peerID]` with current tick count
- Reset `consecutiveTimeouts` to 0

### Protocol Initiation (lines 501-530)
Replaced fixed ring logic (`if (selfID == fromID + 1 || selfID == 0)`) with random distributed initiation.

### Timeout Handling (lines 564-572, 592-596)
Added logic to switch peers after consecutive timeouts.

### Initialization (lines 640-648, 655-659)
- Initialize peer activity tracking arrays
- Use random peer selection for initial peer instead of fixed pattern

## Benefits

1. **Fault Tolerance**: Single crazyflie failure no longer breaks the entire swarm
2. **Full Connectivity**: Over time, all crazyflies communicate with all other crazyflies
3. **Automatic Recovery**: System automatically routes around non-responsive peers
4. **Dynamic Membership**: Easier to add/remove crazyflies without reconfiguration
5. **Load Distribution**: Communication load more evenly distributed across swarm

## Testing Recommendations

1. **Multi-Crazyflie Test**: Deploy 3+ crazyflies and verify they all successfully range to each other
2. **Failure Recovery**: Disable one crazyflie mid-flight and verify others continue operating
3. **Hot-Swap**: Add a new crazyflie to an active swarm and verify it integrates
4. **Timeout Behavior**: Test with weak radio conditions to verify timeout handling
5. **Long-Duration**: Run for extended periods to verify no deadlocks or edge cases

## Configuration Parameters

Users can tune the following parameters in `lpsTwrTag.c`:

- `MAX_CONSECUTIVE_TIMEOUTS` (default: 3): Number of timeouts before switching peers
- `PEER_TIMEOUT_MS` (default: 5000): Milliseconds before considering a peer inactive
- `swarmSize`: Total number of crazyflies (defined by LOCODECK_NR_OF_TWR_ANCHORS + 1)

## Backward Compatibility

The changes maintain the same external API and data structures. Existing logging and parameter interfaces remain unchanged. The only behavioral difference is the communication pattern, which is more robust than before.

## Future Enhancements

Potential future improvements:
1. Better random number generation (current implementation uses tick count modulo)
2. Adaptive timeout values based on observed network conditions
3. Priority-based peer selection (e.g., prefer peers with worse position estimates)
4. Metrics/logging for peer activity and timeout events
