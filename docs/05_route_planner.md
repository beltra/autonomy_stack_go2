# Route Planner: `far_planner`, `graph_decoder`, `boundary_handler`, `visibility_graph_msg`

---

## `far_planner` (FAR Planner)

**Location:** `src/route_planner/far_planner/`
**When Used:** Route-planner launch modes only (`sim_route.sh`, `real_route.sh`).

### Overview

**FAR Planner** (Fast, Attemptable Route Planner) is a global navigation planner based on a **visibility graph** approach. It builds and maintains a dynamic visibility graph as the robot explores, then uses this graph to plan routes from the current position to a user-specified goal point. The planner supports both navigation through known (free) space and exploration through unknown space.

### Algorithm Description

1. **Contour Detection** (`contour_detector.cpp`): Detects obstacle contours from the terrain cloud by analyzing point distributions. Obstacle boundaries are extracted as polygonal contours. Optionally uses OpenCV for image-based contour detection.

2. **Contour Graph** (`contour_graph.cpp`): Processes detected contours into corner/vertex nodes. Analyzes the convexity of each vertex to determine its free direction (convex corners can "see around" obstacles).

3. **Dynamic Graph** (`dynamic_graph.cpp`): Maintains the visibility graph between detected nodes:
   - Nodes are added as new contour vertices are detected
   - Edges connect nodes that have line-of-sight visibility (checked against obstacle contours)
   - Nodes are finalized after they've been consistently observed for `node_finalize_thred` cycles
   - Dead nodes are cleaned up after `clear_dumper_thred` missed observations

4. **Map Handler** (`map_handler.cpp`): Manages the global 3D voxel grid:
   - Maintains obstacle and free-space clouds at a configurable resolution
   - Handles multi-layer terrain (if `is_multi_layer=true`)
   - Grid dimensions: `cell_length × map_grid_max_length`

5. **Scan Handler** (`scan_handler.cpp`): Processes incoming terrain scans:
   - Separates new (recently observed) vs. established terrain points using intensity thresholds
   - Identifies surround free and obstacle regions
   - Detects terrain height variations

6. **Graph Planner** (`graph_planner.cpp`): Plans routes on the visibility graph:
   - **Free-space path** (green in RVIZ): Path through known free space only
   - **Attemptable path** (blue in RVIZ): Path through both free and unknown space
   - Uses Dijkstra-like shortest path on the graph
   - Goal reaching detection with voting (`reach_goal_vote_size`)
   - Path momentum to avoid oscillation (`path_momentum_thred`)

7. **Terrain Planner** (`terrain_planner.cpp`): Handles terrain-aware path adjustments.

8. **Graph Messager** (`graph_msger.cpp`): Serializes/deserializes the visibility graph to ROS messages for persistence and multi-robot sharing.

9. **Planner Visualizer** (`planner_visualizer.cpp`): Publishes visualization markers for RVIZ (graph nodes, edges, paths, etc.)

### Subscribed Topics

| Topic | Type | Remapped From | Description |
|-------|------|---------------|-------------|
| `/odom_world` | `Odometry` | `/state_estimation` | Vehicle odometry |
| `/terrain_cloud` | `PointCloud2` | `/terrain_map_ext` | Extended terrain map |
| `/scan_cloud` | `PointCloud2` | `/terrain_map` | Local terrain map |
| `/terrain_local_cloud` | `PointCloud2` | `/registered_scan` | Raw registered scan |
| `/goal_point` | `PointStamped` | — | Goal point from RVIZ |
| `/joy` | `Joy` | — | Joystick (button 4 = reset graph) |
| `/update_visibility_graph` | `Bool` | — | Enable/disable graph updates |
| `/reset_visibility_graph` | `Empty` | — | Reset the visibility graph |
| `/read_file_dir` | `String` | — | Load graph from file |
| `/save_file_dir` | `String` | — | Save graph to file |

### Published Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/way_point` | `PointStamped` | Next navigation waypoint (consumed by local_planner) |
| `/navigation_boundary` | `PolygonStamped` | Local boundary for obstacle avoidance |
| `/runtime` | `Float32` | Planner computation time |
| `/planning_time` | `Float32` | Planning elapsed time |
| `/traverse_time` | `Float32` | Traverse time metric |
| `/planning_attemptable` | `Bool` | Whether the goal has been reached |
| `/dynamic_obs` | `PointCloud2` | Detected dynamic obstacles |
| Various viz topics | `MarkerArray` | Visualization markers (graph, paths, etc.) |

### Configuration Parameters (`config/default.yaml`)

#### Master Parameters
| Parameter | Default | Description |
|-----------|---------|-------------|
| `main_run_freq` | `3.0` | Main loop frequency (Hz). Overridden to `4.0` in launch. |
| `voxel_dim` | `0.15` | Voxel resolution for map (m) |
| `robot_dim` | `0.6` | Robot diameter for inflation (m) |
| `vehicle_height` | `0.5` | Vehicle height (m) |
| `sensor_range` | `8.0` | Sensor effective range (m) |
| `terrain_range` | `6.0` | Terrain observation range (m) |
| `local_planner_range` | `2.0` | Local planner interaction range (m) |
| `visualize_ratio` | `0.2` | Visualization scale |
| `is_viewpoint_extend` | `true` | Extend viewpoints on obstacle clouds |
| `is_multi_layer` | `false` | Multi-layer terrain handling |
| `is_opencv_visual` | `false` | Show obstacle contour images |
| `is_viz_freespace` | `false` | Visualize free-space V-graph edges |
| `is_static_env` | `false` | Static environment assumption |
| `is_pub_boundary` | `false` | Publish local boundary |
| `is_debug_output` | `false` | Debug terminal output |
| `is_attempt_autoswitch` | `true` | Auto-switch to attemptable navigation |
| `world_frame` | `map` | World frame ID |
| `suppress_nonessential_topics` | `true` | Disable debug visualization topics |

#### Map Handler
| Parameter | Default | Description |
|-----------|---------|-------------|
| `floor_height` | `2.0` | Floor height for multi-layer (m) |
| `cell_length` | `2.5` | Grid cell length (m) |
| `map_grid_max_length` | `200.0` | Maximum map extent (m) |
| `map_grad_max_height` | `10.0` | Maximum map height (m) |

#### Utility
| Parameter | Default | Description |
|-----------|---------|-------------|
| `angle_noise` | `16.0` | Angular noise tolerance (deg) |
| `accept_max_align_angle` | `4.0` | Max alignment angle for acceptance (deg) |
| `obs_inflate_size` | `1` | Obstacle inflation size (voxels) |
| `new_intensity_thred` | `2.0` | Intensity threshold for new points |
| `terrain_free_Z` | `0.15` | Z tolerance for free terrain (m) |
| `dynamic_obs_decay_time` | `1.0` | Dynamic obstacle decay time (s) |
| `new_points_decay_time` | `1.0` | New points decay time (s) |

#### Dynamic Graph
| Parameter | Default | Description |
|-----------|---------|-------------|
| `connect_votes_size` | `5` | Votes to confirm an edge |
| `clear_dumper_thred` | `3` | Missed observations to remove a node |
| `node_finalize_thred` | `5` | Observations to finalize a node |
| `filter_pool_size` | `4` | Filtering pool size |

#### Corner Detector
| Parameter | Default | Description |
|-----------|---------|-------------|
| `resize_ratio` | `2.0` | Image resize ratio for detection |
| `filter_count_value` | `4` | Filter count for corner detection |

#### Graph Planner
| Parameter | Default | Description |
|-----------|---------|-------------|
| `converge_distance` | `0.25` | Convergence distance (m) |
| `goal_adjust_radius` | `1.0` | Goal position adjustment radius (m) |
| `free_counter_thred` | `7` | Counter threshold for free space |
| `reach_goal_vote_size` | `3` | Votes to confirm goal reached |
| `path_momentum_thred` | `3` | Momentum threshold for path stability |

---

## `graph_decoder`

**Location:** `src/route_planner/graph_decoder/`
**When Used:** Launched automatically with `far_planner`.

### Overview

Decodes and re-encodes visibility graph messages. Acts as a relay and format converter for the visibility graph, supporting multi-robot graph sharing and file-based graph loading/saving.

### Subscribed Topics
| Topic | Type | Description |
|-------|------|-------------|
| `/decoded_vgraph` | `Graph` | Decoded visibility graph |
| `/encoded_vgraph` | `Graph` | Encoded visibility graph |
| `/save_file_dir` | `String` | Save graph to file path |
| `/read_file_dir` | `String` | Load graph from file path |

### Published Topics
| Topic | Type | Description |
|-------|------|-------------|
| `/decoded_vgraph` | `Graph` | Published decoded graph |
| `/graph_decoder_viz` | `MarkerArray` | Graph visualization |

### Key Parameters (from `config/default.yaml`)
| Parameter | Default | Description |
|-----------|---------|-------------|
| `world_frame` | `map` | TF frame for visualization |
| `visual_scale_ratio` | `1.0` | Visualization scale |

---

## `boundary_handler`

**Location:** `src/route_planner/boundary_handler/`
**When Used:** Offline tool for pre-processing boundary data into visibility graphs.

### Overview

Reads a boundary polygon point cloud from a PCD file, constructs a visibility graph from the boundary vertices, and saves it to a file. This is used to pre-compute visibility graphs from known environment boundaries.

### Algorithm

1. **Read Boundary:** Loads a PCD file containing boundary polygon vertices (with intensity encoding polygon IDs).
2. **Polygon Extraction:** Groups points by polygon ID, creates polygon structures.
3. **Convexity Analysis:** For each vertex, determines if it's convex or concave relative to a known free point.
4. **V-Graph Construction:** Tests all pairs of vertices for visibility (no polygon edge intersections) and creates edges.
5. **Save:** Writes the graph to a text file and visualizes it continuously.

### Published Topics
| Topic | Type | Description |
|-------|------|-------------|
| `/graph_decoder_viz` | `MarkerArray` | Graph visualization |
| `/free_p_viz` | `Marker` | Free point visualization |

### Key Parameters
| Parameter | Default | Description |
|-----------|---------|-------------|
| `world_frame` | `map` | Reference frame |
| `folder_path` | `/home/workspace/...` | Data directory |
| `boundary_file` | `boundary.pcd` | Boundary PCD filename |
| `traj_file` | `traj.txt` | Trajectory (free point) filename |
| `graph_file` | `graph.txt` | Output graph filename |
| `height_tolz` | `1.0` | Height tolerance for edge connections (m) |

---

## `visibility_graph_msg`

**Location:** `src/route_planner/visibility_graph_msg/`
**When Used:** Whenever the route planner is active.

### Overview

Defines custom ROS 2 message types for the visibility graph:

- **`Node.msg`**: Represents a graph vertex with position, ID, free type, coverage status, surface directions, and connection lists.
- **`Graph.msg`**: Contains a list of Nodes and metadata (size, robot_id, header).

These messages are used by `far_planner`, `graph_decoder`, and `boundary_handler` for graph serialization and inter-node communication.
