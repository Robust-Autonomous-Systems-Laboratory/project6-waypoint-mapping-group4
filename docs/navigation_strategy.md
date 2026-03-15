# Navigation and Measurement Strategy

## Team Members
- Member 1: Malcolm Benedict
- Member 2: Ian Q. Mattson

---

## 1. Environment Description

### Location
The room where the experiment will be preformed is EERC 722, the Academic Robotics Lab. This environment presents a unique challenge: the north wall is a good landmark, however on the other three sides, there are tables and chairs which will not return a clean LiDAR reading. Because of this, we have elected to add an additional landmark in the form of a recycle bin placed in the center of the room.

### Environment Sketch
![A drawing of EERC 722](../figures/roomSketch.png "Sketch of the room with waypoints")


### Identified Landmarks
| ID | Landmark | Location | Notes |
|----|----------|----------|-------|
| A  | North wall | North side | Flat surface, good for distance measurement. |
| B  | Recycle Bin | Center | Should be clearly visible from all waypoints and has good corners. |

---

## 2. Waypoint Plan

### Waypoint Layout

| Waypoint | Position (x, y) | Landmark to Measure | Measurement Direction |
|----------|-----------------|---------------------|----------------------|
| 1 (Start)| (0.0, 0.0) | North wall | South |
| 2 | (0.0, 0.0) | Recycle Bin | South |
| 3 | (0.0, 0.0) | Recycle Bin | East |
| 4 | (0.0, 0.0)| Recycle Bin | North |
| 5 | (0.0, 0.0) | North wall | North |
| 6 (Loop Closure)| (0.0, 0.0)| North wall | South |

### Path Statistics
- Number of waypoints: 6
- Total path length: 
- Loop closure planned: Yes 
- Estimated navigation time: 

---

## 3. Orientation Strategy

### Heading Reference System
Orientation will be defined in cardinal directions, using the north wall as a reference. For the sake of convenience, tape markings will be added to the floor. Angle measurements will be taken 
at the point alone the surface closest to the robot. (The ray from the robot to the surface should be normal to the surface.)

Options:
- [ ] Align robot parallel to a specific wall at each waypoint
- [ ] Use floor tape lines to define heading
- [x] Use compass directions (if walls align N/S/E/W)
- [ ] Other: _______________

### At Each Waypoint
| Waypoint | Orientation Reference | Expected Landmark Direction |
|----------|----------------------|----------------------------|
| 1 | North Wall | North wall at 180° (Behind) |
| 2 | West side of trash can | Trash can at 90° (Left) |
| 3 | South side of trash can | Trash can at 90° (Left) |
| 4 | East side of trash can | Trash can at 90° (Left) |
| 5 | North Wall | North wall at 0° (Aead) |
| 6 | North Wall | North wall at 180° (Behind) |
---

## 4. Measurement Protocol

### Before Data Collection
- [ ] Mark all waypoints with tape
- [ ] Measure waypoint positions from start
- [ ] Identify measurement landmark at each waypoint
- [ ] Measure distance to each landmark
- [ ] Record all measurements in `config/measurements.yaml`

### At Each Waypoint During Collection
1. [ ] Stop robot completely at tape mark
2. [ ] Align robot to planned orientation
3. [ ] Wait 2 seconds for settling
4. [ ] Verify landmark visible in RViz scan
5. [ ] Capture scan with waypoint ID
6. [ ] Take RViz screenshot
7. [ ] Note any observations

### Measurement Technique
- Measuring from: Tape mark
- Measuring to: Wall surface
- Tool: Laser measure
- Estimated measurement uncertainty: ± 10 mm

---

## 5. Expected Challenges

### Localization Error Sources
1. **Odometry drift**: Significant impact expected. Unfortunately very little can be done about this besides minimizing the number of turns.
2. **IMU bias**: Moderate impact expected. While IMU data is typically not great, it is not the only data being used and the EKF should help mitigate the impact.
3. **Wheel slip**: Minimal. The floor appears relatively clean. Picking up obvious debris and minimizing turns should further mitigate this.

### Mapping Challenges
1. **Scan alignment**: This is a significant issue. Due to human error it is very likely the scans will not align perfectly.
2. **Landmark visibility**: Should be pretty good, the robot is always relatively close to the landmarks.
3. **Environmental factors**: The biggest issues here will be people and inconsistent readings from table legs.

### Mitigation Strategies
- Minimize unnecessary turning.
- Take extra care when recording measurements and piloting the robot.
- Use the EKF to filter IMU data.

---

## 6. Roles During Data Collection

| Task | Team Member |
|------|-------------|
| Robot pilot | |
| RViz monitoring / screenshots | |
| Scan capture triggering | |
| Observation notes | |

---

## 7. Post-Collection Analysis Plan

### Map Evaluation Steps
1. [ ] Load all point clouds in RViz
2. [ ] Verify scans appear in `odom` frame
3. [ ] For each waypoint, measure distance to landmark using RViz Measure tool
4. [ ] Record RViz measurements in `measurements.yaml`
5. [ ] Compute errors (measured - RViz)
6. [ ] Take screenshots showing measurements
7. [ ] Assess orientation consistency at each waypoint

### Quality Checks
- [ ] Do walls from different scans align?
- [ ] Is loop closure consistent (if applicable)?
- [ ] Are there any obvious mapping artifacts?

---

## 8. Pre-Run Checklist

- [ ] Robot battery charged
- [ ] All waypoints marked with tape
- [ ] All positions measured and recorded
- [ ] All landmark distances measured and recorded
- [ ] `measurements.yaml` filled with ground truth
- [ ] Localization node tested
- [ ] Scan capture service tested
- [ ] RViz configured
- [ ] Bag recording tested
- [ ] Camera/screenshot tool ready
- [ ] This strategy document complete
