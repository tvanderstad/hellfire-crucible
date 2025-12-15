use serde::{Deserialize, Serialize};

pub type PlayerId = u32;

// Physics constants (in meters)
pub const ARENA_SIZE: f32 = 20.0; // 20x20 meter arena
pub const PLAYER_SIZE: f32 = 0.5; // 0.5 meter square
pub const BLADE_LENGTH: f32 = 2.0; // 2 meter long blade
pub const BLADE_WIDTH: f32 = 0.1; // 0.1 meter wide blade
pub const BLADE_OFFSET: f32 = 1.0; // blade starts 1 meter from player center

// Forces and torques
pub const PLAYER_FORCE: f32 = 50.0; // Newtons
pub const PLAYER_MAX_VELOCITY: f32 = 10.0; // Max meters/second
pub const BLADE_TORQUE_MULTIPLIER: f32 = 10.0; // Multiplier for blade torque input
pub const BLADE_MAX_ANGULAR_VELOCITY: f32 = 2.0; // Max radians/second

// Game constants
pub const TICK_RATE: u64 = 60;
pub const TICK_DURATION: f32 = 1.0 / TICK_RATE as f32;

#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub struct Vec2 {
    pub x: f32,
    pub y: f32,
}

impl Vec2 {
    pub fn new(x: f32, y: f32) -> Self {
        Self { x, y }
    }

    pub fn zero() -> Self {
        Self { x: 0.0, y: 0.0 }
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PlayerState {
    pub id: PlayerId,
    pub position: Vec2,
    pub rotation: f32,
    pub blade_angle: f32,     // relative to player
    pub blade_position: Vec2, // actual position of blade center
    pub is_dead: bool,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct GameState {
    pub players: Vec<PlayerState>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum ClientMessage {
    Join,
    Input {
        movement: Vec2,     // normalized -1 to 1
        blade_target: Vec2, // target position in world space
        blade_torque: f32,  // -1 for CCW, 0 for none, 1 for CW
    },
    Respawn,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum ServerMessage {
    Welcome { your_id: PlayerId },
    GameState(GameState),
    PlayerJoined { id: PlayerId },
    PlayerLeft { id: PlayerId },
}
