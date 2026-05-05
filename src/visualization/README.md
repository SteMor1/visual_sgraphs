### `graph_publisher.cc`

Class responsible for publishing the situational graph data in the S-Graphs format.

**Takes:**
- `pCurrentMap` — pointer to the current ORB-SLAM3 map, used to retrieve all detected planes and rooms
- `graph_type` — string flag (`"Prior"` or `"Online"`) that controls which room variants and filtering rules are applied
- `tfStampedSE` — transform used to convert room centroids from the SLAM frame into the ROS world frame
- `tfStampedBC` — transform used to project wall-plane coefficients from the SLAM frame into the ROS world frame

**Produces:**
A `situational_graphs_reasoning_msgs::msg::Graph` message containing:
- **Plane nodes** — vertical wall planes, each carrying the transformed 4-coefficient equation (`[a, b, c, d]`)
- **Room nodes** — typed as `Finite Room` or `Infinite Room` (Prior mode only), each carrying the 2D transformed centroid `[x, y, 0]`
- **Room–wall edges** — connectivity edges of type `EdgeRoom4Planes` linking each room to its associated wall planes, with plane IDs offset by `1 000 000` to avoid ID collisions with room IDs

**Key internal helpers:**
- `_extract_plane_nodes` — filters out bad and non-vertical planes, applies the `tfStampedBC` transform to plane coefficients
- `_extract_room_wall_edges` — iterates over a room's walls and emits one labeled edge per valid wall