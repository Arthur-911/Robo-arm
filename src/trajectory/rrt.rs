use nalgebra::Point3;
use serde::{Deserialize, Serialize};
use web_time::Instant;

use super::planner::Waypoint;
use crate::kinematics::{
    is_configuration_valid, solve_ik, IKSolverMode, IKSolverParams, IKSolverType, ObstacleBox,
    RobotArm,
};

/// Fast, deterministic 64-bit pseudo-random number generator (Xorshift64*).
/// Requires zero external crates and runs identically on desktop and wasm32.
pub struct SimpleRng {
    state: u64,
}

impl SimpleRng {
    pub fn new(seed: u64) -> Self {
        Self {
            state: if seed == 0 { 0x853c49e6748fea9b } else { seed },
        }
    }

    #[inline]
    pub fn next_u64(&mut self) -> u64 {
        let mut x = self.state;
        x ^= x >> 12;
        x ^= x << 25;
        x ^= x >> 27;
        self.state = x;
        x.wrapping_mul(0x2545F4914F6CDD1D)
    }

    #[inline]
    pub fn next_f64(&mut self) -> f64 {
        (self.next_u64() >> 11) as f64 * (1.0 / 9007199254740992.0)
    }

    #[inline]
    pub fn next_in_range(&mut self, min: f64, max: f64) -> f64 {
        min + self.next_f64() * (max - min)
    }
}

/// Selector for the active RRT search algorithm.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default, Serialize, Deserialize)]
pub enum RrtAlgorithm {
    /// Bi-directional RRT (RRT-Connect) grows two trees simultaneously (fastest convergence).
    #[default]
    RrtConnect,
    /// Single-tree Rapidly-exploring Random Tree with goal biasing.
    StandardRrt,
}

/// Hyperparameters for RRT obstacle-avoidance path planning.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct RrtParams {
    /// Maximum allowed iterations before declaring failure. Default: 3000.
    pub max_iterations: usize,
    /// Step size in joint space per expansion step (radians). Default: 0.15 rad (~8.6 deg).
    pub step_size: f64,
    /// Probability of sampling the goal directly [0.0, 1.0]. Default: 0.15.
    pub goal_bias: f64,
    /// Discretization resolution along edges for collision checking (radians). Default: 0.04 rad (~2.3 deg).
    pub collision_resolution: f64,
    /// Whether to apply post-planning path shortcutting to remove zig-zag waypoints. Default: true.
    pub shortcut_path: bool,
    /// Maximum iterations of shortcutting passes. Default: 60.
    pub max_shortcut_iterations: usize,
    /// Which RRT algorithm variant to employ. Default: RrtConnect.
    pub algorithm: RrtAlgorithm,
    /// Random seed for deterministic reproducibility. If 0, uses a clock-based or default seed.
    pub seed: u64,
}

impl Default for RrtParams {
    fn default() -> Self {
        Self {
            max_iterations: 3000,
            step_size: 0.15,
            goal_bias: 0.15,
            collision_resolution: 0.04,
            shortcut_path: true,
            max_shortcut_iterations: 60,
            algorithm: RrtAlgorithm::RrtConnect,
            seed: 42,
        }
    }
}

/// Node in a configuration-space search tree.
#[derive(Debug, Clone)]
struct RrtNode {
    q: Vec<f64>,
    parent_idx: Option<usize>,
}

/// Complete diagnostic result of an RRT motion planning query.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct RrtPlanResult {
    /// Whether a collision-free path connecting start to goal was successfully found.
    pub success: bool,
    /// Sequence of joint configurations along the collision-free path.
    pub joint_path: Vec<Vec<f64>>,
    /// Cartesian 3D Tool Center Point positions corresponding to each joint waypoint.
    pub cartesian_path: Vec<Point3<f64>>,
    /// Planning computation time in microseconds.
    pub planning_time_us: u128,
    /// Total iterations evaluated during the search.
    pub iterations: usize,
    /// Total number of tree nodes explored.
    pub total_nodes: usize,
    /// Diagnostic summary or failure explanation.
    pub message: String,
}

impl RrtPlanResult {
    /// Creates an empty failed plan result.
    pub fn failure(message: impl Into<String>, planning_time_us: u128, iterations: usize) -> Self {
        Self {
            success: false,
            joint_path: Vec::new(),
            cartesian_path: Vec::new(),
            planning_time_us,
            iterations,
            total_nodes: 0,
            message: message.into(),
        }
    }

    /// Converts the planned joint waypoints into timed `Waypoint` entries for `TrajectoryPlanner`.
    pub fn to_waypoints(&self, average_speed: f64) -> Vec<Waypoint> {
        if self.cartesian_path.is_empty() {
            return Vec::new();
        }

        let speed = average_speed.clamp(0.05, 2.0);
        let mut waypoints = Vec::with_capacity(self.cartesian_path.len());

        for (i, &pos) in self.cartesian_path.iter().enumerate() {
            let dur = if i == 0 {
                1.5
            } else {
                let dist = (pos - self.cartesian_path[i - 1]).norm();
                (dist / speed).clamp(0.3, 4.0)
            };

            let name = if i == 0 {
                "RRT Start".to_string()
            } else if i == self.cartesian_path.len() - 1 {
                "RRT Goal".to_string()
            } else {
                format!("RRT WP {}", i)
            };

            waypoints.push(Waypoint::new(name, pos, dur));
        }

        waypoints
    }

    /// Calculates total arc-length of the planned Cartesian path in meters.
    pub fn total_path_length(&self) -> f64 {
        let mut length = 0.0;
        for i in 0..self.cartesian_path.len().saturating_sub(1) {
            length += (self.cartesian_path[i + 1] - self.cartesian_path[i]).norm();
        }
        length
    }
}

/// Computes the Euclidean distance between two joint configurations.
#[inline]
pub fn joint_distance(q1: &[f64], q2: &[f64]) -> f64 {
    let mut sum_sq = 0.0;
    for (a, b) in q1.iter().zip(q2.iter()) {
        let d = a - b;
        sum_sq += d * d;
    }
    sum_sq.sqrt()
}

/// Steers from `from` towards `to` by at most `step_size`.
pub fn steer(from: &[f64], to: &[f64], step_size: f64) -> Vec<f64> {
    let dist = joint_distance(from, to);
    if dist <= step_size || dist < 1e-9 {
        return to.to_vec();
    }
    let ratio = step_size / dist;
    from.iter()
        .zip(to.iter())
        .map(|(&a, &b)| a + (b - a) * ratio)
        .collect()
}

/// Checks whether the straight line in C-space between `q_start` and `q_end` is collision-free.
pub fn is_edge_valid(
    robot: &RobotArm,
    q_start: &[f64],
    q_end: &[f64],
    obstacles: &[ObstacleBox],
    resolution: f64,
) -> bool {
    let dist = joint_distance(q_start, q_end);
    let steps = ((dist / resolution.max(1e-3)).ceil() as usize).max(1);

    for s in 1..=steps {
        let u = s as f64 / steps as f64;
        let mut q_interp = Vec::with_capacity(q_start.len());
        for (&a, &b) in q_start.iter().zip(q_end.iter()) {
            q_interp.push(a + (b - a) * u);
        }

        if !is_configuration_valid(robot, &q_interp, obstacles) {
            return false;
        }
    }

    true
}

/// Extracts the active joint limits for the robot: (min, max).
pub fn get_joint_limits(robot: &RobotArm) -> Vec<(f64, f64)> {
    robot
        .joints
        .iter()
        .filter(|j| j.is_actuated())
        .map(|j| {
            j.limits
                .unwrap_or((-std::f64::consts::PI, std::f64::consts::PI))
        })
        .collect()
}

/// Samples a random configuration in C-space within joint limits.
pub fn sample_random_configuration(
    limits: &[(f64, f64)],
    goal_q: &[f64],
    goal_bias: f64,
    rng: &mut SimpleRng,
) -> Vec<f64> {
    if rng.next_f64() < goal_bias {
        return goal_q.to_vec();
    }

    limits
        .iter()
        .map(|&(min, max)| rng.next_in_range(min, max))
        .collect()
}

/// Finds the index of the nearest node in `tree` to `target_q`.
fn nearest_node_idx(tree: &[RrtNode], target_q: &[f64]) -> usize {
    let mut min_dist = f64::MAX;
    let mut best_idx = 0;

    for (i, node) in tree.iter().enumerate() {
        let d = joint_distance(&node.q, target_q);
        if d < min_dist {
            min_dist = d;
            best_idx = i;
        }
    }

    best_idx
}

/// Reconstructs the path from tree root to the specified node index.
fn reconstruct_path(tree: &[RrtNode], mut curr_idx: usize) -> Vec<Vec<f64>> {
    let mut path = Vec::new();
    loop {
        path.push(tree[curr_idx].q.clone());
        if let Some(parent) = tree[curr_idx].parent_idx {
            curr_idx = parent;
        } else {
            break;
        }
    }
    path.reverse();
    path
}

/// Applies post-planning path shortcutting to remove zig-zag waypoints.
pub fn shortcut_path(
    robot: &RobotArm,
    mut path: Vec<Vec<f64>>,
    obstacles: &[ObstacleBox],
    max_iterations: usize,
    resolution: f64,
    rng: &mut SimpleRng,
) -> Vec<Vec<f64>> {
    if path.len() <= 2 {
        return path;
    }

    for _ in 0..max_iterations {
        let n = path.len();
        if n <= 2 {
            break;
        }

        let i = (rng.next_u64() as usize) % (n - 1);
        let j = (rng.next_u64() as usize) % n;

        let (idx1, idx2) = if i < j { (i, j) } else { (j, i) };
        if idx2 <= idx1 + 1 {
            continue;
        }

        // Check if direct line between path[idx1] and path[idx2] is collision-free
        if is_edge_valid(robot, &path[idx1], &path[idx2], obstacles, resolution) {
            // Remove intermediate nodes between idx1 and idx2
            let mut new_path = Vec::with_capacity(n - (idx2 - idx1 - 1));
            new_path.extend_from_slice(&path[..=idx1]);
            new_path.extend_from_slice(&path[idx2..]);
            path = new_path;
        }
    }

    path
}

/// Bi-directional RRT (RRT-Connect) path planner in Configuration Space.
pub fn plan_rrt_connect(
    robot: &RobotArm,
    start_q: &[f64],
    goal_q: &[f64],
    obstacles: &[ObstacleBox],
    params: &RrtParams,
) -> RrtPlanResult {
    let t0 = Instant::now();

    // 1. Initial configuration validity check
    if !is_configuration_valid(robot, start_q, obstacles) {
        return RrtPlanResult::failure(
            "Start configuration is in collision or violates joint limits",
            t0.elapsed().as_micros(),
            0,
        );
    }
    if !is_configuration_valid(robot, goal_q, obstacles) {
        return RrtPlanResult::failure(
            "Goal configuration is in collision or violates joint limits",
            t0.elapsed().as_micros(),
            0,
        );
    }

    // 2. Direct edge check (if direct line is valid, return immediately!)
    if is_edge_valid(
        robot,
        start_q,
        goal_q,
        obstacles,
        params.collision_resolution,
    ) {
        let raw_path = vec![start_q.to_vec(), goal_q.to_vec()];
        let cartesian_path = raw_path
            .iter()
            .map(|q| {
                let poses = robot.forward_kinematics_with_q(q);
                Point3::from(poses.last().unwrap().translation.vector)
            })
            .collect();

        return RrtPlanResult {
            success: true,
            joint_path: raw_path,
            cartesian_path,
            planning_time_us: t0.elapsed().as_micros(),
            iterations: 1,
            total_nodes: 2,
            message: "Direct collision-free path connected start to goal".to_string(),
        };
    }

    let limits = get_joint_limits(robot);
    let mut rng = SimpleRng::new(if params.seed == 0 {
        t0.elapsed().as_nanos() as u64 ^ 0x9e3779b97f4a7c15
    } else {
        params.seed
    });

    // Trees
    let mut tree_a = vec![RrtNode {
        q: start_q.to_vec(),
        parent_idx: None,
    }];
    let mut tree_b = vec![RrtNode {
        q: goal_q.to_vec(),
        parent_idx: None,
    }];

    let mut is_tree_a_start = true;
    let mut connected = false;
    let mut connect_idx_a = 0;
    let mut connect_idx_b = 0;
    let mut iter_count = 0;

    for it in 0..params.max_iterations {
        iter_count = it + 1;

        // Sample random configuration towards tree B's root
        let q_target =
            sample_random_configuration(&limits, &tree_b[0].q, params.goal_bias, &mut rng);

        // Extend Tree A
        let near_idx_a = nearest_node_idx(&tree_a, &q_target);
        let q_near_a = &tree_a[near_idx_a].q;
        let q_new_a = steer(q_near_a, &q_target, params.step_size);

        if is_edge_valid(
            robot,
            q_near_a,
            &q_new_a,
            obstacles,
            params.collision_resolution,
        ) {
            let new_idx_a = tree_a.len();
            tree_a.push(RrtNode {
                q: q_new_a.clone(),
                parent_idx: Some(near_idx_a),
            });

            // Connect Tree B towards q_new_a
            let curr_q_b_target = q_new_a;
            let mut near_idx_b = nearest_node_idx(&tree_b, &curr_q_b_target);

            loop {
                let q_near_b = &tree_b[near_idx_b].q;
                let q_new_b = steer(q_near_b, &curr_q_b_target, params.step_size);

                if !is_edge_valid(
                    robot,
                    q_near_b,
                    &q_new_b,
                    obstacles,
                    params.collision_resolution,
                ) {
                    break;
                }

                let new_idx_b = tree_b.len();
                tree_b.push(RrtNode {
                    q: q_new_b.clone(),
                    parent_idx: Some(near_idx_b),
                });
                near_idx_b = new_idx_b;

                // Did tree B connect to tree A's new node?
                if joint_distance(&q_new_b, &curr_q_b_target) < 1e-4 {
                    connected = true;
                    if is_tree_a_start {
                        connect_idx_a = new_idx_a;
                        connect_idx_b = near_idx_b;
                    } else {
                        connect_idx_a = near_idx_b;
                        connect_idx_b = new_idx_a;
                    }
                    break;
                }
            }

            if connected {
                break;
            }
        }

        // Swap trees for balanced exploration
        std::mem::swap(&mut tree_a, &mut tree_b);
        is_tree_a_start = !is_tree_a_start;
    }

    if !connected {
        return RrtPlanResult::failure(
            format!(
                "RRT-Connect reached max iterations ({}) without finding a collision-free path",
                params.max_iterations
            ),
            t0.elapsed().as_micros(),
            iter_count,
        );
    }

    // Reconstruct joint path
    let tree_start = if is_tree_a_start { &tree_a } else { &tree_b };
    let tree_goal = if is_tree_a_start { &tree_b } else { &tree_a };

    let mut path_start = reconstruct_path(tree_start, connect_idx_a);
    let path_goal = reconstruct_path(tree_goal, connect_idx_b);

    // Append reversed goal branch (excluding duplicate bridge node)
    for q in path_goal.into_iter().rev() {
        if path_start.is_empty() || joint_distance(path_start.last().unwrap(), &q) > 1e-4 {
            path_start.push(q);
        }
    }

    let mut joint_path = path_start;

    // Apply shortcutting
    if params.shortcut_path {
        joint_path = shortcut_path(
            robot,
            joint_path,
            obstacles,
            params.max_shortcut_iterations,
            params.collision_resolution,
            &mut rng,
        );
    }

    // Convert joint path to Cartesian poses
    let cartesian_path = joint_path
        .iter()
        .map(|q| {
            let poses = robot.forward_kinematics_with_q(q);
            Point3::from(poses.last().unwrap().translation.vector)
        })
        .collect();

    let total_nodes = tree_a.len() + tree_b.len();
    let planning_time_us = t0.elapsed().as_micros();

    RrtPlanResult {
        success: true,
        joint_path,
        cartesian_path,
        planning_time_us,
        iterations: iter_count,
        total_nodes,
        message: format!(
            "Found collision-free path in {:.2}ms ({} waypoints, {} tree nodes)",
            planning_time_us as f64 / 1000.0,
            iter_count,
            total_nodes
        ),
    }
}

/// Standard single-tree RRT path planner with goal biasing.
pub fn plan_standard_rrt(
    robot: &RobotArm,
    start_q: &[f64],
    goal_q: &[f64],
    obstacles: &[ObstacleBox],
    params: &RrtParams,
) -> RrtPlanResult {
    let t0 = Instant::now();

    if !is_configuration_valid(robot, start_q, obstacles) {
        return RrtPlanResult::failure(
            "Start configuration is in collision or violates joint limits",
            t0.elapsed().as_micros(),
            0,
        );
    }
    if !is_configuration_valid(robot, goal_q, obstacles) {
        return RrtPlanResult::failure(
            "Goal configuration is in collision or violates joint limits",
            t0.elapsed().as_micros(),
            0,
        );
    }

    if is_edge_valid(
        robot,
        start_q,
        goal_q,
        obstacles,
        params.collision_resolution,
    ) {
        let raw_path = vec![start_q.to_vec(), goal_q.to_vec()];
        let cartesian_path = raw_path
            .iter()
            .map(|q| {
                let poses = robot.forward_kinematics_with_q(q);
                Point3::from(poses.last().unwrap().translation.vector)
            })
            .collect();

        return RrtPlanResult {
            success: true,
            joint_path: raw_path,
            cartesian_path,
            planning_time_us: t0.elapsed().as_micros(),
            iterations: 1,
            total_nodes: 2,
            message: "Direct collision-free path found".to_string(),
        };
    }

    let limits = get_joint_limits(robot);
    let mut rng = SimpleRng::new(if params.seed == 0 {
        t0.elapsed().as_nanos() as u64 ^ 0xa0761d6478bd642f
    } else {
        params.seed
    });

    let mut tree = vec![RrtNode {
        q: start_q.to_vec(),
        parent_idx: None,
    }];

    let mut goal_node_idx = None;
    let mut iter_count = 0;

    for it in 0..params.max_iterations {
        iter_count = it + 1;
        let q_target = sample_random_configuration(&limits, goal_q, params.goal_bias, &mut rng);
        let near_idx = nearest_node_idx(&tree, &q_target);
        let q_near = &tree[near_idx].q;
        let q_new = steer(q_near, &q_target, params.step_size);

        if is_edge_valid(
            robot,
            q_near,
            &q_new,
            obstacles,
            params.collision_resolution,
        ) {
            let new_idx = tree.len();
            tree.push(RrtNode {
                q: q_new.clone(),
                parent_idx: Some(near_idx),
            });

            // Check if goal is reachable from this new node
            if joint_distance(&q_new, goal_q) <= params.step_size
                && is_edge_valid(
                    robot,
                    &q_new,
                    goal_q,
                    obstacles,
                    params.collision_resolution,
                )
            {
                tree.push(RrtNode {
                    q: goal_q.to_vec(),
                    parent_idx: Some(new_idx),
                });
                goal_node_idx = Some(tree.len() - 1);
                break;
            }
        }
    }

    let goal_idx = match goal_node_idx {
        Some(idx) => idx,
        None => {
            return RrtPlanResult::failure(
                format!(
                    "Standard RRT reached max iterations ({}) without reaching goal",
                    params.max_iterations
                ),
                t0.elapsed().as_micros(),
                iter_count,
            );
        }
    };

    let mut joint_path = reconstruct_path(&tree, goal_idx);

    if params.shortcut_path {
        joint_path = shortcut_path(
            robot,
            joint_path,
            obstacles,
            params.max_shortcut_iterations,
            params.collision_resolution,
            &mut rng,
        );
    }

    let cartesian_path = joint_path
        .iter()
        .map(|q| {
            let poses = robot.forward_kinematics_with_q(q);
            Point3::from(poses.last().unwrap().translation.vector)
        })
        .collect();

    let total_nodes = tree.len();
    let planning_time_us = t0.elapsed().as_micros();
    let path_len = joint_path.len();

    RrtPlanResult {
        success: true,
        joint_path,
        cartesian_path,
        planning_time_us,
        iterations: iter_count,
        total_nodes,
        message: format!(
            "Standard RRT found path in {:.2}ms ({} waypoints)",
            planning_time_us as f64 / 1000.0,
            path_len
        ),
    }
}

/// Unified entry point to plan a collision-free path between two joint configurations.
pub fn plan_rrt(
    robot: &RobotArm,
    start_q: &[f64],
    goal_q: &[f64],
    obstacles: &[ObstacleBox],
    params: &RrtParams,
) -> RrtPlanResult {
    match params.algorithm {
        RrtAlgorithm::RrtConnect => plan_rrt_connect(robot, start_q, goal_q, obstacles, params),
        RrtAlgorithm::StandardRrt => plan_standard_rrt(robot, start_q, goal_q, obstacles, params),
    }
}

/// High-level Cartesian Task-Space path planner.
/// Automatically solves IK to find a collision-free target joint posture,
/// then runs RRT path planning around all workcell obstacles.
pub fn plan_cartesian_rrt(
    robot: &RobotArm,
    target_pos: Point3<f64>,
    obstacles: &[ObstacleBox],
    params: &RrtParams,
) -> RrtPlanResult {
    let t0 = Instant::now();
    let start_q = robot.get_actuated_joint_positions();

    // 1. Solve IK for goal posture
    let ik_params = IKSolverParams {
        max_iterations: 80,
        tolerance: 2e-3,
        enable_null_space_centering: true,
        ..Default::default()
    };

    let ik_sol = solve_ik(
        robot,
        target_pos,
        None,
        IKSolverType::JacobianDLS,
        IKSolverMode::PositionOnly,
        &ik_params,
    );

    if !ik_sol.converged || ik_sol.joint_positions.is_empty() {
        return RrtPlanResult::failure(
            format!(
                "Inverse Kinematics could not reach target position ({:.2}, {:.2}, {:.2})",
                target_pos.x, target_pos.y, target_pos.z
            ),
            t0.elapsed().as_micros(),
            0,
        );
    }

    let goal_q = ik_sol.joint_positions;

    // 2. Check if goal configuration itself is collision-free
    if !is_configuration_valid(robot, &goal_q, obstacles) {
        return RrtPlanResult::failure(
            "Target position is inside or colliding with an obstacle",
            t0.elapsed().as_micros(),
            0,
        );
    }

    // 3. Plan collision-free joint trajectory
    plan_rrt(robot, &start_q, &goal_q, obstacles, params)
}
