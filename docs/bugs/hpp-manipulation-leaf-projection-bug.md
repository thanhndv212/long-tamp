# Bug Report: WaypointEdge::generateTargetConfig Returns Uninitialized Memory on Failure

**Package**: hpp-manipulation  
**Component**: `WaypointEdge::generateTargetConfig()`  
**Severity**: High  
**Affects**: hpp-python users, retry-based planning loops

---

## Summary

`WaypointEdge::generateTargetConfig()` modifies its output parameter `q` even when returning `false`, leaving it with uninitialized/partially-initialized memory. This corrupts retry loops that reuse the output buffer.

---

## Bug: WaypointEdge Returns Uninitialized Memory on Failure

### Problem

When `WaypointEdge::generateTargetConfig()` fails partway through waypoint generation, it returns `false` but leaves the output configuration `q` containing uninitialized memory from the internal buffer `configs_`.

### Observed Behavior

**Initial call** (returns false due to constraint failure):
```cpp
Configuration_t q = random_config;
bool success = edge->generateTargetConfig(q_start, q);  // Returns false
// q now contains partially-initialized data
```

**Retry using same buffer** (garbage in → garbage out):
```cpp
bool success = edge->generateTargetConfig(q_start, q);  // Reuses corrupted q
// q contains values like 6.455e-310, 1.581e-322 (uninitialized memory)
```

**Example corrupted configuration**:
```
(0.86466, 0, 0, 0, 0, 0,           ← Valid SE3 values
 6.455661986e-310,                 ← Uninitialized!
 1.581010066e-322,                 ← Uninitialized!
 5.549345334e-320,                 ← Uninitialized!
 6.455652917e-310, ...)            ← Uninitialized!
```

```

### Root Cause

**File**: `hpp-manipulation/src/graph/edge.cc`  
**Function**: `WaypointEdge::generateTargetConfig()`

```cpp
bool WaypointEdge::generateTargetConfig(ConfigurationIn_t qStart,
                                        ConfigurationOut_t q) const {
  assert(configs_.cols() == size_type(edges_.size() + 1));
  configs_.col(0) = qStart;
  for (std::size_t i = 0; i < edges_.size(); ++i) {
    configs_.col(i + 1) = q;  // ← Uses input 'q' as seed
    if (!edges_[i]->generateTargetConfig(configs_.col(i),
                                         configs_.col(i + 1))) {
      q = configs_.col(i + 1);  // ← BUG: Returns partially-initialized buffer
      lastSucceeded_ = false;
      return false;
    }
  }
  q = configs_.col(edges_.size());
  lastSucceeded_ = true;
  return true;
}
```

**Issue**: On failure at waypoint `i`, the function:
1. Returns `false` ✓ (correct)
2. **But also writes `configs_.col(i + 1)` to output `q`** ✗ (incorrect)
3. `configs_.col(i + 1)` contains uninitialized data from failed projection

### Impact

- **Retry loops**: Using `q` from failed attempt as seed corrupts subsequent attempts
- **hpp-python**: Python callers cannot distinguish valid vs. garbage output
- **Silent corruption**: No warning that output parameter contains invalid data

---

## Proposed Fixes

### Option 1: Don't Modify Output on Failure (Recommended)

```cpp
bool WaypointEdge::generateTargetConfig(ConfigurationIn_t qStart,
                                        ConfigurationOut_t q) const {
  Configuration_t q_temp = q;  // Work on temporary buffer
  
  configs_.col(0) = qStart;
  for (std::size_t i = 0; i < edges_.size(); ++i) {
    configs_.col(i + 1) = q_temp;
    if (!edges_[i]->generateTargetConfig(configs_.col(i),
                                         configs_.col(i + 1))) {
      // Don't modify output 'q' on failure
      lastSucceeded_ = false;
      return false;
    }
  }
  
  q = configs_.col(edges_.size());  // Only write on success
  lastSucceeded_ = true;
  return true;
}
```

### Option 2: Zero-Initialize on Failure

```cpp
    if (!edges_[i]->generateTargetConfig(configs_.col(i),
                                         configs_.col(i + 1))) {
      q.setZero();  // Clear garbage before returning
      lastSucceeded_ = false;
      return false;
    }
```

### Option 3: Document Behavior (Not Recommended)

Add warning to API documentation that output parameter is undefined on failure. This doesn't solve the retry-loop corruption issue.

---

## Reproduction

```cpp
// Setup
WaypointEdgePtr_t edge = /* ... */;
Configuration_t q_start = /* ... */;
Configuration_t q_rand = robot->randomConfig();

// First attempt may fail partway through
bool success = edge->generateTargetConfig(q_start, q_rand);
if (!success) {
  // q_rand now contains partial/garbage data
  
  // Retry reuses corrupted buffer
  success = edge->generateTargetConfig(q_start, q_rand);  // ← Likely to fail again
}
```

**Expected**: Each retry should have independent chance of success  
**Actual**: Corrupted seed causes cascade failures

---

## Workarounds

### For Callers (Temporary)

Generate fresh random configuration on each retry:

```python
# Python (hpp-python)
for attempt in range(max_attempts):
    q_rand = robot.shootRandomConfig()  # Fresh seed each time
    success, q_out, err = edge.generateTargetConfig(q_start, q_rand)
    if success:
        break
```

```cpp
// C++
for (int attempt = 0; attempt < maxAttempts; ++attempt) {
  Configuration_t q = robot->randomConfig();  // Fresh each time
  if (edge->generateTargetConfig(qStart, q)) {
    break;
  }
}
```

---

## Related Issues

This pattern may exist in other `generateTargetConfig()` implementations:
- `Edge::generateTargetConfig()`
- `LevelSetEdge::generateTargetConfig()`

**Recommendation**: Audit all implementations for output-parameter-on-failure contract.

---

## Testing

Add unit test verifying output parameter is not modified on failure:

```cpp
TEST(WaypointEdge, generateTargetConfig_doesNotModifyOutputOnFailure) {
  // Setup edge that will fail
  Configuration_t q_start = /* ... */;
  Configuration_t q_original = /* ... */;
  Configuration_t q = q_original;
  
  bool success = edge->generateTargetConfig(q_start, q);
  
  ASSERT_FALSE(success);
  ASSERT_TRUE(q.isApprox(q_original));  // Output unchanged on failure
}
```

---

## Files Affected

- `hpp-manipulation/src/graph/edge.cc` (lines 484-504)
- `hpp-manipulation/include/hpp/manipulation/graph/edge.hh`

---

## References

**Similar Issue**: `TransitionPlanner::planPath()` validates that goal configs satisfy leaf constraints (lines 109-120 of `transition-planner.cc`). This validation catches corrupted configs but doesn't prevent the corruption.

---

**Recommendation**: Apply **Option 1** (don't modify output on failure) for consistency with standard C++ practice where output parameters of failed functions are undefined but not actively corrupted.
