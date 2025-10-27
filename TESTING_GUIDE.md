# Testing Guide for LPS TWR Tag Improvements

## Overview
This guide explains how to test and verify the improved lpsTwrTag implementation that replaces the fixed ring protocol with random peer selection.

## Prerequisites
- 3 or more Crazyflie drones with UWB decks
- Configured with LOCODECK_NR_OF_TWR_ANCHORS set appropriately
- Radio addresses configured consecutively (e.g., IDs 0, 1, 2)

## Test Scenarios

### Test 1: Basic Multi-Crazyflie Ranging
**Objective**: Verify all crazyflies can range to each other

**Steps**:
1. Power on 3+ crazyflies simultaneously
2. Monitor ranging log data: `ranging.distance0`, `ranging.distance1`, etc.
3. Verify each crazyflie successfully ranges to all other crazyflies
4. Check that distances are reasonable and stable

**Expected Result**: All crazyflies should successfully range to all peers within 10-15 seconds

**Success Criteria**:
- Each crazyflie shows non-zero distances to all peers
- Distance values are stable (variation < 50mm)
- No persistent zeros in distance logs

### Test 2: Single Crazyflie Failure Recovery
**Objective**: Verify swarm continues operating when one crazyflie fails

**Steps**:
1. Start with 3+ crazyflies all ranging successfully
2. Record baseline ranging performance
3. Power off or disconnect one crazyflie (the "failed" unit)
4. Monitor remaining crazyflies for 30+ seconds
5. Verify remaining crazyflies continue ranging to each other

**Expected Result**: Remaining crazyflies should continue ranging to each other without interruption

**Success Criteria**:
- Remaining crazyflies show continued non-zero distances to active peers
- No more than 5 seconds of ranging interruption when one crazyflie fails
- Timeout counter increases for the failed peer
- System automatically switches to other peers

### Test 3: Dynamic Swarm Membership (Hot-Swap)
**Objective**: Verify new crazyflies can join an active swarm

**Steps**:
1. Start with 2 crazyflies ranging successfully
2. After 10 seconds, power on a third crazyflie
3. Monitor all three units
4. Verify the new crazyflie integrates into the swarm

**Expected Result**: New crazyflie should begin ranging to existing peers within 5-10 seconds

**Success Criteria**:
- New crazyflie shows non-zero distances to peers within 10 seconds
- Existing crazyflies begin showing distance to new unit
- No disruption to existing ranging operations

### Test 4: Timeout Handling
**Objective**: Verify proper handling of consecutive timeouts

**Steps**:
1. Start with 2+ crazyflies
2. Create weak radio conditions (e.g., move crazyflies far apart or add obstacles)
3. Monitor timeout behavior in logs
4. Observe peer switching after MAX_CONSECUTIVE_TIMEOUTS (3)

**Expected Result**: System should automatically try different peers after 3 consecutive timeouts

**Success Criteria**:
- Timeouts are logged but don't cause permanent failures
- After 3 timeouts with one peer, system tries a different peer
- Eventually finds a peer that responds successfully

### Test 5: Long-Duration Stability
**Objective**: Verify no deadlocks or edge cases over extended operation

**Steps**:
1. Deploy 3+ crazyflies in ranging mode
2. Run for at least 10 minutes
3. Monitor for any degradation in performance
4. Check for any stuck states or communication failures

**Expected Result**: System should maintain stable ranging throughout

**Success Criteria**:
- Ranging continues consistently for entire duration
- No permanent failures or stuck states
- Distance measurements remain accurate
- All crazyflies continue ranging to all peers

### Test 6: Peer Activity Tracking
**Objective**: Verify peer activity tracking correctly identifies active/inactive peers

**Steps**:
1. Start with 3 crazyflies all active
2. Note which peers each crazyflie considers "active"
3. Power off one crazyflie
4. After 5+ seconds (PEER_TIMEOUT_MS), verify it's marked inactive
5. Verify `selectNextPeer()` prefers active peers

**Expected Result**: Inactive peers should be correctly identified after timeout

**Success Criteria**:
- Active peers have lastSuccessfulRanging timestamps < 5 seconds old
- Inactive peers marked after PEER_TIMEOUT_MS (5000ms)
- Peer selection prefers active over inactive peers

## Debugging Tips

### Check Ranging Logs
Monitor these log variables:
```
ranging.distance0  # Distance to peer 0
ranging.distance1  # Distance to peer 1
ranging.distance2  # Distance to peer 2
```

### Add Debug Prints
Uncomment debug prints in lpsTwrTag.c to see:
- Packet reception details (line 276-282)
- Distance measurements (line 361, 413)
- Peer selection decisions

### Monitor Parameters
Check these parameters:
```
swarm.size  # Number of crazyflies in swarm
```

### Common Issues

**Issue**: No ranging happening at all
- **Solution**: Check that LOCODECK_NR_OF_TWR_ANCHORS is set correctly
- **Solution**: Verify radio addresses are configured properly
- **Solution**: Check UWB antenna connections

**Issue**: Ranging to some peers works, others don't
- **Solution**: Check radio address configuration (should be consecutive)
- **Solution**: Verify swarmSize matches actual number of crazyflies
- **Solution**: Check for physical obstructions or radio interference

**Issue**: Ranging works initially but stops
- **Solution**: Check for power issues
- **Solution**: Monitor for increasing timeout counters
- **Solution**: Verify PEER_TIMEOUT_MS is appropriate for your environment

## Performance Metrics

Expected performance with 3 crazyflies:

| Metric | Target | Notes |
|--------|--------|-------|
| Time to first ranging | < 5 seconds | From power-on |
| Ranging rate per pair | ~2-5 Hz | Varies by swarm size |
| Timeout recovery time | < 500ms | After 3 consecutive timeouts |
| New peer integration | < 10 seconds | Hot-add to swarm |
| Distance accuracy | ±30-50mm | Typical UWB accuracy |

## Configuration Tuning

If you need to adjust behavior, modify these constants in lpsTwrTag.c:

```c
#define MAX_CONSECUTIVE_TIMEOUTS 3      // Timeouts before switching peers
#define PEER_TIMEOUT_MS 5000            // Ms before peer considered inactive
```

Increase MAX_CONSECUTIVE_TIMEOUTS in noisy environments.
Decrease PEER_TIMEOUT_MS for faster failure detection.

## Verification Checklist

- [ ] All crazyflies can range to all other crazyflies
- [ ] System continues when one crazyflie is disabled
- [ ] New crazyflies can be added to active swarm
- [ ] Timeouts cause automatic peer switching
- [ ] Long-duration operation is stable
- [ ] Inactive peers are correctly identified
- [ ] No deadlocks or stuck states observed
- [ ] Distance measurements are accurate and stable

## Reporting Issues

If you encounter issues, please provide:
1. Number of crazyflies in swarm
2. LOCODECK_NR_OF_TWR_ANCHORS value
3. Radio addresses of all crazyflies
4. Log output showing the issue
5. Environmental conditions (obstacles, range, etc.)
6. Steps to reproduce

This information will help diagnose and fix any problems with the implementation.
