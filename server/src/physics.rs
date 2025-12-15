use hellfire_crucible_shared::{
    PlayerId, Vec2, ARENA_SIZE, BLADE_LENGTH, BLADE_OFFSET, BLADE_WIDTH, PLAYER_SIZE,
};
use nalgebra::vector;
use rapier2d::parry::query;
use rapier2d::prelude::*;
use std::collections::HashMap;

// Physics constants
const WALL_THICKNESS: f32 = 1.0;
const PLAYER_LINEAR_DAMPING: f32 = 10.0;
const PLAYER_ANGULAR_DAMPING: f32 = 5.0;
const BLADE_LINEAR_DAMPING: f32 = 5.0;
const BLADE_ANGULAR_DAMPING: f32 = 100.0;
const BLADE_DENSITY: f32 = 0.1;

pub struct PhysicsWorld {
    rigid_body_set: RigidBodySet,
    collider_set: ColliderSet,
    integration_parameters: IntegrationParameters,
    physics_pipeline: PhysicsPipeline,
    island_manager: IslandManager,
    broad_phase: DefaultBroadPhase,
    narrow_phase: NarrowPhase,
    impulse_joint_set: ImpulseJointSet,
    multibody_joint_set: MultibodyJointSet,
    ccd_solver: CCDSolver,
    query_pipeline: QueryPipeline,

    player_bodies: HashMap<PlayerId, RigidBodyHandle>,
    blade_bodies: HashMap<PlayerId, RigidBodyHandle>,
    blade_colliders: HashMap<PlayerId, ColliderHandle>,
}

impl PhysicsWorld {
    pub fn new() -> Self {
        let mut physics = Self {
            rigid_body_set: RigidBodySet::new(),
            collider_set: ColliderSet::new(),
            integration_parameters: IntegrationParameters::default(),
            physics_pipeline: PhysicsPipeline::new(),
            island_manager: IslandManager::new(),
            broad_phase: DefaultBroadPhase::new(),
            narrow_phase: NarrowPhase::new(),
            impulse_joint_set: ImpulseJointSet::new(),
            multibody_joint_set: MultibodyJointSet::new(),
            ccd_solver: CCDSolver::new(),
            query_pipeline: QueryPipeline::new(),
            player_bodies: HashMap::new(),
            blade_bodies: HashMap::new(),
            blade_colliders: HashMap::new(),
        };

        physics.create_walls();
        physics
    }

    fn create_walls(&mut self) {
        let half_size = ARENA_SIZE / 2.0;
        let wall_thickness = WALL_THICKNESS;

        let walls = [
            (
                vector![0.0, half_size + wall_thickness / 2.0],
                half_size,
                wall_thickness / 2.0,
            ),
            (
                vector![0.0, -half_size - wall_thickness / 2.0],
                half_size,
                wall_thickness / 2.0,
            ),
            (
                vector![half_size + wall_thickness / 2.0, 0.0],
                wall_thickness / 2.0,
                half_size,
            ),
            (
                vector![-half_size - wall_thickness / 2.0, 0.0],
                wall_thickness / 2.0,
                half_size,
            ),
        ];

        for (position, half_width, half_height) in walls {
            let wall = RigidBodyBuilder::fixed().translation(position).build();
            let wall_handle = self.rigid_body_set.insert(wall);

            let collider = ColliderBuilder::cuboid(half_width, half_height).build();
            self.collider_set
                .insert_with_parent(collider, wall_handle, &mut self.rigid_body_set);
        }
    }

    pub fn add_player(&mut self, player_id: PlayerId, position: Vec2) {
        // Create player body
        let player = RigidBodyBuilder::dynamic()
            .translation(vector![position.x, position.y])
            .linear_damping(PLAYER_LINEAR_DAMPING)
            .angular_damping(PLAYER_ANGULAR_DAMPING)
            .lock_rotations()
            .ccd_enabled(true)
            .build();
        let player_handle = self.rigid_body_set.insert(player);

        let player_collider = ColliderBuilder::cuboid(PLAYER_SIZE / 2.0, PLAYER_SIZE / 2.0).build();
        self.collider_set.insert_with_parent(
            player_collider,
            player_handle,
            &mut self.rigid_body_set,
        );

        // Create blade body
        let blade_center = BLADE_OFFSET + BLADE_LENGTH / 2.0;
        let blade = RigidBodyBuilder::dynamic()
            .translation(vector![position.x, position.y + blade_center])
            .linear_damping(BLADE_LINEAR_DAMPING)
            .angular_damping(BLADE_ANGULAR_DAMPING)
            .ccd_enabled(true)
            .build();
        let blade_handle = self.rigid_body_set.insert(blade);

        let blade_collider = ColliderBuilder::cuboid(BLADE_WIDTH / 2.0, BLADE_LENGTH / 2.0)
            .density(BLADE_DENSITY)
            .build();
        let blade_collider_handle = self.collider_set.insert_with_parent(
            blade_collider,
            blade_handle,
            &mut self.rigid_body_set,
        );

        // Create revolute joint
        let joint = RevoluteJointBuilder::new()
            .local_anchor1(point![0.0, 0.0])
            .local_anchor2(point![0.0, -blade_center])
            .build();

        self.impulse_joint_set
            .insert(player_handle, blade_handle, joint, true);

        self.player_bodies.insert(player_id, player_handle);
        self.blade_bodies.insert(player_id, blade_handle);
        self.blade_colliders
            .insert(player_id, blade_collider_handle);
    }

    pub fn remove_player(&mut self, player_id: PlayerId) {
        // Remove blade first (this also removes any joints)
        if let Some(blade_handle) = self.blade_bodies.remove(&player_id) {
            self.rigid_body_set.remove(
                blade_handle,
                &mut self.island_manager,
                &mut self.collider_set,
                &mut self.impulse_joint_set,
                &mut self.multibody_joint_set,
                true, // wake_up = true to ensure proper cleanup
            );
        }

        // Then remove player
        if let Some(player_handle) = self.player_bodies.remove(&player_id) {
            self.rigid_body_set.remove(
                player_handle,
                &mut self.island_manager,
                &mut self.collider_set,
                &mut self.impulse_joint_set,
                &mut self.multibody_joint_set,
                true, // wake_up = true to ensure proper cleanup
            );
        }

        // Clean up blade collider reference
        self.blade_colliders.remove(&player_id);

        // Force update of spatial data structures
        self.query_pipeline.update(&self.collider_set);
    }

    pub fn apply_player_force(&mut self, player_id: PlayerId, force: Vec2) {
        if let Some(&handle) = self.player_bodies.get(&player_id) {
            if let Some(body) = self.rigid_body_set.get_mut(handle) {
                body.reset_forces(false);
                body.add_force(vector![force.x, force.y], true);
            }
        }
    }

    pub fn apply_blade_torque(&mut self, player_id: PlayerId, torque: f32) {
        if let Some(&handle) = self.blade_bodies.get(&player_id) {
            if let Some(body) = self.rigid_body_set.get_mut(handle) {
                body.reset_torques(true);
                if torque != 0.0 {
                    body.add_torque(torque, true);
                }
            }
        }
    }

    pub fn step(&mut self, dt: f32) {
        self.integration_parameters.dt = dt;

        self.physics_pipeline.step(
            &vector![0.0, 0.0],
            &self.integration_parameters,
            &mut self.island_manager,
            &mut self.broad_phase,
            &mut self.narrow_phase,
            &mut self.rigid_body_set,
            &mut self.collider_set,
            &mut self.impulse_joint_set,
            &mut self.multibody_joint_set,
            &mut self.ccd_solver,
            None,
            &(),
            &(),
        );

        self.query_pipeline.update(&self.collider_set);
    }

    pub fn get_player_transform(&self, player_id: PlayerId) -> Option<(Vec2, f32)> {
        self.player_bodies.get(&player_id).and_then(|&handle| {
            self.rigid_body_set.get(handle).map(|body| {
                let pos = body.translation();
                let rot = body.rotation().angle();
                (Vec2::new(pos.x, pos.y), rot)
            })
        })
    }

    pub fn get_player_velocity(&self, player_id: PlayerId) -> Option<Vec2> {
        self.player_bodies.get(&player_id).and_then(|&handle| {
            self.rigid_body_set.get(handle).map(|body| {
                let vel = body.linvel();
                Vec2::new(vel.x, vel.y)
            })
        })
    }

    pub fn get_blade_angle(&self, player_id: PlayerId) -> Option<f32> {
        self.blade_bodies.get(&player_id).and_then(|&handle| {
            self.rigid_body_set
                .get(handle)
                .map(|body| body.rotation().angle())
        })
    }

    pub fn get_blade_angular_velocity(&self, player_id: PlayerId) -> Option<f32> {
        self.blade_bodies
            .get(&player_id)
            .and_then(|&handle| self.rigid_body_set.get(handle).map(|body| body.angvel()))
    }

    pub fn get_blade_position(&self, player_id: PlayerId) -> Option<Vec2> {
        self.blade_bodies.get(&player_id).and_then(|&handle| {
            self.rigid_body_set.get(handle).map(|body| {
                let pos = body.translation();
                Vec2::new(pos.x, pos.y)
            })
        })
    }

    pub fn check_blade_hits(&self) -> Vec<(PlayerId, PlayerId)> {
        let mut hits = Vec::new();

        for (&attacker_id, &blade_collider) in &self.blade_colliders {
            // Verify the blade body still exists
            if !self.blade_bodies.contains_key(&attacker_id) {
                continue;
            }

            // Also verify the collider still exists in the collider set
            if !self.collider_set.contains(blade_collider) {
                continue;
            }

            for (&victim_id, &player_body) in &self.player_bodies {
                if attacker_id == victim_id {
                    continue;
                }

                // Check if blade collider intersects with player
                if let Some(blade_collider_ref) = self.collider_set.get(blade_collider) {
                    if let Some(player_body) = self.rigid_body_set.get(player_body) {
                        let player_pos = player_body.translation();
                        let shape_pos = Isometry::translation(player_pos.x, player_pos.y);
                        let shape = Cuboid::new(vector![PLAYER_SIZE / 2.0, PLAYER_SIZE / 2.0]);

                        // Use contact query instead of intersection_with_shape
                        let blade_pos = *blade_collider_ref.position();
                        let blade_shape = blade_collider_ref.shape();

                        if query::contact(&blade_pos, blade_shape, &shape_pos, &shape, 0.1)
                            .unwrap_or(None)
                            .is_some()
                        {
                            hits.push((attacker_id, victim_id));
                        }
                    }
                }
            }
        }

        hits
    }

    pub fn respawn_player(&mut self, player_id: PlayerId, position: Vec2) {
        if let Some(&player_handle) = self.player_bodies.get(&player_id) {
            if let Some(player_body) = self.rigid_body_set.get_mut(player_handle) {
                player_body.set_translation(vector![position.x, position.y], true);
                player_body.set_linvel(vector![0.0, 0.0], true);
                player_body.set_angvel(0.0, true);
                player_body.wake_up(true);
            }
        }
        // Blade is handled separately now - it's removed on death and recreated on respawn
    }

    pub fn remove_blade(&mut self, player_id: PlayerId) {
        // Remove blade body (this also removes associated colliders and joints)
        if let Some(blade_handle) = self.blade_bodies.remove(&player_id) {
            // Get all associated colliders before removal
            let colliders: Vec<_> = self
                .rigid_body_set
                .get(blade_handle)
                .map(|body| body.colliders().to_vec())
                .unwrap_or_default();

            self.rigid_body_set.remove(
                blade_handle,
                &mut self.island_manager,
                &mut self.collider_set,
                &mut self.impulse_joint_set,
                &mut self.multibody_joint_set,
                true,
            );

            // Verify colliders were removed
            for collider_handle in colliders {
                if self.collider_set.contains(collider_handle) {
                    eprintln!(
                        "Warning: Collider {:?} not removed with blade body",
                        collider_handle
                    );
                }
            }
        }

        // Clean up blade collider reference
        self.blade_colliders.remove(&player_id);

        // Force update of spatial data structures
        self.query_pipeline.update(&self.collider_set);
    }

    pub fn recreate_blade(&mut self, player_id: PlayerId, player_position: Vec2) {
        if let Some(&player_handle) = self.player_bodies.get(&player_id) {
            // Create blade body
            let blade_center = BLADE_OFFSET + BLADE_LENGTH / 2.0;
            let blade = RigidBodyBuilder::dynamic()
                .translation(vector![player_position.x, player_position.y + blade_center])
                .linear_damping(BLADE_LINEAR_DAMPING)
                .angular_damping(BLADE_ANGULAR_DAMPING)
                .ccd_enabled(true)
                .build();
            let blade_handle = self.rigid_body_set.insert(blade);

            let blade_collider = ColliderBuilder::cuboid(BLADE_WIDTH / 2.0, BLADE_LENGTH / 2.0)
                .density(BLADE_DENSITY)
                .build();
            let blade_collider_handle = self.collider_set.insert_with_parent(
                blade_collider,
                blade_handle,
                &mut self.rigid_body_set,
            );

            // Create revolute joint
            let joint = RevoluteJointBuilder::new()
                .local_anchor1(point![0.0, 0.0])
                .local_anchor2(point![0.0, -blade_center])
                .build();

            self.impulse_joint_set
                .insert(player_handle, blade_handle, joint, true);

            self.blade_bodies.insert(player_id, blade_handle);
            self.blade_colliders
                .insert(player_id, blade_collider_handle);
        }
    }
}
