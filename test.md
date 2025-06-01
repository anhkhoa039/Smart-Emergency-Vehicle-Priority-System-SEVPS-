# SEVPS System Testing Documentation

## Overview
This document outlines the testing procedures and expected results for the Smart Emergency Vehicle Priority System (SEVPS). The system is tested through a comprehensive test suite that verifies all major components and their interactions.

## Test Components

### 1. Emergency Vehicle Detection
- **Test Objective**: Verify that the system correctly identifies emergency vehicles
- **Test Method**: Monitor vehicle types in the simulation
- **Expected Results**:
  - System should detect ambulances, fire trucks, and police cars
  - Correct vehicle attributes should be captured (position, speed, route)
  - Detection should be real-time and continuous
- **Success Criteria**: All emergency vehicles are detected and properly classified

### 2. Route Optimization
- **Test Objective**: Verify dynamic route optimization for emergency vehicles
- **Test Method**: Compare original and optimized routes
- **Expected Results**:
  - Routes should be optimized based on current traffic conditions
  - Optimized routes should be different from original routes
  - Routes should avoid congested areas
- **Success Criteria**: Routes are successfully optimized with reduced travel time

### 3. Traffic Light Control
- **Test Objective**: Verify traffic light response to emergency vehicles
- **Test Method**: Monitor traffic light phase changes
- **Expected Results**:
  - Traffic lights should change phases when emergency vehicles approach
  - Priority should be given to emergency vehicle directions
  - Smooth transitions between phases
- **Success Criteria**: Traffic lights respond appropriately to emergency vehicles

### 4. Congestion Prediction
- **Test Objective**: Verify congestion prediction accuracy
- **Test Method**: Monitor prediction outputs and actual traffic conditions
- **Expected Results**:
  - Predictions should be generated for all edges
  - Predictions should reflect current traffic patterns
  - Updates should occur in real-time
- **Success Criteria**: Predictions are generated and reasonably accurate

### 5. V2X Communication
- **Test Objective**: Verify vehicle-to-vehicle and vehicle-to-infrastructure communication
- **Test Method**: Monitor communication events
- **Expected Results**:
  - Emergency vehicles should communicate with nearby vehicles
  - Infrastructure should receive and process priority requests
  - Regular vehicles should respond to warnings
- **Success Criteria**: Communication is established and effective

### 6. Metrics Collection
- **Test Objective**: Verify comprehensive data collection
- **Test Method**: Monitor collected metrics
- **Expected Results**:
  - Emergency vehicle metrics (response time, speed, etc.)
  - Traffic light metrics (phase changes, efficiency)
  - General traffic metrics (flow, congestion)
- **Success Criteria**: All relevant metrics are collected and stored

### 7. Visualization
- **Test Objective**: Verify real-time visualization of system metrics
- **Test Method**: Monitor plot updates
- **Expected Results**:
  - Real-time updates of all metrics
  - Clear and informative plots
  - Both static (matplotlib) and interactive (plotly) visualizations
- **Success Criteria**: Visualizations are updated and informative

## Running the Tests

1. **Prerequisites**:
   ```bash
   pip install -r requirements.txt
   ```

2. **Execute Tests**:
   ```bash
   python src/test.py
   ```

3. **Monitor Progress**:
   - Test progress is displayed in real-time
   - Status updates every 100 simulation steps
   - Component-specific messages show active features

4. **Review Results**:
   - Results are saved in `test_results/[timestamp]/`
   - Includes:
     - Test status report
     - Collected metrics
     - Visualization plots

## Expected Output

### Console Output
```
Starting SEVPS System Tests...
Detected emergency vehicles: 3
Route optimized for amb1
Traffic light 1234 phase changed: 0 -> 1
Congestion prediction active
V2X communication active
Metrics collection active
Visualization active
...
```

### Test Results File
```
SEVPS System Test Results
=======================

emergency_detection: PASS
route_optimization: PASS
traffic_light_control: PASS
congestion_prediction: PASS
v2x_communication: PASS
metrics_collection: PASS
visualization: PASS
```

## Performance Metrics

The system should achieve:
- Emergency vehicle response time reduction: >20%
- Traffic light efficiency improvement: >15%
- Congestion prediction accuracy: >80%
- V2X communication success rate: >90%

## Troubleshooting

### Common Issues
1. **Emergency Vehicles Not Detected**
   - Check vehicle type definitions in `emergency.rou.xml`
   - Verify SUMO configuration includes emergency vehicle types

2. **Route Optimization Failures**
   - Check network connectivity
   - Verify edge weights are being updated correctly

3. **Traffic Light Control Issues**
   - Check traffic light program definitions
   - Verify phase timing parameters

4. **Visualization Problems**
   - Check matplotlib and plotly installations
   - Verify display settings

### Debugging Tips
- Use SUMO-GUI to visually inspect vehicle behavior
- Check log files for detailed error messages
- Monitor system resources during test execution

## Test Environment

- **Operating System**: macOS 23.6.0
- **Python Version**: 3.8+
- **SUMO Version**: 1.22.0
- **Required Packages**: See requirements.txt

## Maintenance

- Regular updates to test cases as system evolves
- Periodic review of success criteria
- Update of performance benchmarks based on real-world data 