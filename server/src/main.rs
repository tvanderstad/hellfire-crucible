mod physics;

use anyhow::Result;
use clap::Parser;
use futures_util::{SinkExt, StreamExt};
use hellfire_crucible_shared::{
    ClientMessage, GameState, PlayerId, PlayerState, ServerMessage, Vec2, ARENA_SIZE, PLAYER_FORCE,
    TICK_DURATION,
};
use physics::PhysicsWorld;
use std::collections::HashMap;
use std::sync::Arc;
use std::time::{Duration, Instant};
use tokio::sync::{broadcast, Mutex};
use tokio::time;
use tokio_tungstenite::{accept_async, tungstenite::Message};

#[derive(Parser, Debug)]
#[command(name = "hellfire-crucible-server")]
#[command(about = "Hellfire Crucible game server")]
struct Args {
    #[arg(long, default_value = "443")]
    port: u16,
}

struct Player {
    is_dead: bool,
    input_movement: Vec2,
}

struct GameServer {
    physics: PhysicsWorld,
    players: HashMap<PlayerId, Player>,
    next_id: PlayerId,
}

impl GameServer {
    fn new() -> Self {
        Self {
            physics: PhysicsWorld::new(),
            players: HashMap::new(),
            next_id: 1,
        }
    }

    fn add_player(&mut self) -> PlayerId {
        let id = self.next_id;
        self.next_id += 1;

        let spawn_pos = self.find_spawn_position();
        self.physics.add_player(id, spawn_pos);
        // Ensure clean spawn state
        self.physics.respawn_player(id, spawn_pos);

        let player = Player {
            is_dead: false,
            input_movement: Vec2::zero(),
        };
        self.players.insert(id, player);

        id
    }

    fn remove_player(&mut self, id: PlayerId) {
        self.physics.remove_player(id);
        self.players.remove(&id);
    }

    fn handle_input(&mut self, id: PlayerId, movement: Vec2) {
        if let Some(player) = self.players.get_mut(&id) {
            // Only process input from living players
            if !player.is_dead {
                player.input_movement = movement;
            }
        }
    }

    fn respawn_player(&mut self, id: PlayerId) {
        // Check if player is dead first
        let is_dead = self.players.get(&id).map(|p| p.is_dead).unwrap_or(false);
        if !is_dead {
            return;
        }

        // Find spawn position before mutable borrow
        let spawn_pos = self.find_spawn_position();

        // Now update the player
        if let Some(player) = self.players.get_mut(&id) {
            player.is_dead = false;
            // Clear input state on respawn
            player.input_movement = Vec2::zero();
        }

        self.physics.respawn_player(id, spawn_pos);
        // Recreate blade on respawn
        self.physics.recreate_blade(id, spawn_pos);
    }

    fn update(&mut self, dt: f32) {
        // Apply forces based on player inputs
        for (&id, player) in &self.players {
            if !player.is_dead {
                // Apply movement force
                let force = Vec2::new(
                    player.input_movement.x * PLAYER_FORCE,
                    player.input_movement.y * PLAYER_FORCE,
                );
                self.physics.apply_player_force(id, force);
            }
        }

        // Step physics
        self.physics.step(dt);

        // Check for hits
        let hits = self.physics.check_blade_hits();
        for (attacker_id, victim_id) in hits {
            // Only living players can kill
            if let Some(attacker) = self.players.get(&attacker_id) {
                if !attacker.is_dead {
                    if let Some(victim) = self.players.get_mut(&victim_id) {
                        victim.is_dead = true;
                        // Clear input state when player dies
                        victim.input_movement = Vec2::zero();
                        // Remove blade when player dies
                        self.physics.remove_blade(victim_id);
                    }
                }
            }
        }
    }

    fn get_game_state(&self) -> GameState {
        let mut player_states = Vec::new();

        for (&id, player) in &self.players {
            if let Some((pos, rotation)) = self.physics.get_player_transform(id) {
                // Dead players don't have blades
                if player.is_dead {
                    player_states.push(PlayerState {
                        id,
                        position: pos,
                        rotation,
                        blade_angle: 0.0,
                        blade_position: pos,
                        is_dead: true,
                    });
                } else {
                    // Living players must have blades
                    if let Some(blade_angle) = self.physics.get_blade_angle(id) {
                        if let Some(blade_pos) = self.physics.get_blade_position(id) {
                            player_states.push(PlayerState {
                                id,
                                position: pos,
                                rotation,
                                blade_angle: blade_angle - rotation,
                                blade_position: blade_pos,
                                is_dead: false,
                            });
                        }
                    }
                }
            }
        }

        GameState {
            players: player_states,
        }
    }

    fn find_spawn_position(&self) -> Vec2 {
        // Simple spawn logic - try to spawn away from other players
        let mut best_pos = Vec2::new(0.0, 0.0);
        let mut best_distance = 0.0;

        for _ in 0..20 {
            // Need 2.5m buffer: 0.25m (half player) + 2.0m (blade length) + 0.25m (safety)
            let spawn_range = ARENA_SIZE - 5.0;
            let x = (rand::random::<f32>() - 0.5) * spawn_range;
            let y = (rand::random::<f32>() - 0.5) * spawn_range;
            let candidate = Vec2::new(x, y);

            let mut min_distance = f32::MAX;
            for (&id, player) in &self.players {
                if !player.is_dead {
                    if let Some((pos, _)) = self.physics.get_player_transform(id) {
                        let dx = candidate.x - pos.x;
                        let dy = candidate.y - pos.y;
                        let distance = (dx * dx + dy * dy).sqrt();
                        min_distance = min_distance.min(distance);
                    }
                }
            }

            if min_distance > best_distance {
                best_distance = min_distance;
                best_pos = candidate;
            }
        }

        best_pos
    }
}

type SharedGameServer = Arc<Mutex<GameServer>>;

async fn handle_connection(
    game_server: SharedGameServer,
    raw_stream: tokio::net::TcpStream,
    broadcast_tx: broadcast::Sender<ServerMessage>,
) -> Result<()> {
    let ws_stream = accept_async(raw_stream).await?;
    let (mut ws_sender, mut ws_receiver) = ws_stream.split();

    // Add player
    let player_id = {
        let mut server = game_server.lock().await;
        server.add_player()
    };

    // Send welcome message
    let welcome = ServerMessage::Welcome { your_id: player_id };
    ws_sender
        .send(Message::text(serde_json::to_string(&welcome)?))
        .await?;
    println!("Player {player_id} joined");

    // Notify others
    broadcast_tx.send(ServerMessage::PlayerJoined { id: player_id })?;

    // Listen for broadcasts
    let mut broadcast_rx = broadcast_tx.subscribe();
    let send_task = tokio::spawn(async move {
        while let Ok(msg) = broadcast_rx.recv().await {
            if ws_sender
                .send(Message::text(serde_json::to_string(&msg).unwrap()))
                .await
                .is_err()
            {
                break;
            }
        }
    });

    // Handle incoming messages
    while let Some(msg) = ws_receiver.next().await {
        match msg {
            Ok(Message::Text(text)) => {
                if let Ok(client_msg) = serde_json::from_str::<ClientMessage>(&text) {
                    let mut server = game_server.lock().await;
                    match client_msg {
                        ClientMessage::Join => {
                            // Already handled
                        }
                        ClientMessage::Input {
                            movement,
                            blade_target: _,
                        } => {
                            server.handle_input(player_id, movement);
                        }
                        ClientMessage::Respawn => {
                            server.respawn_player(player_id);
                        }
                    }
                }
            }
            Ok(Message::Close(_)) => break,
            _ => {}
        }
    }

    send_task.abort();

    // Remove player
    {
        let mut server = game_server.lock().await;
        server.remove_player(player_id);
    }

    broadcast_tx.send(ServerMessage::PlayerLeft { id: player_id })?;
    println!("Player {player_id} left");

    Ok(())
}

async fn game_loop(game_server: SharedGameServer, broadcast_tx: broadcast::Sender<ServerMessage>) {
    let mut interval = time::interval(Duration::from_secs_f32(TICK_DURATION));
    let mut last_time = Instant::now();

    loop {
        interval.tick().await;

        let now = Instant::now();
        let dt = (now - last_time).as_secs_f32();
        last_time = now;

        let game_state = {
            let mut server = game_server.lock().await;
            server.update(dt);
            server.get_game_state()
        };

        let _ = broadcast_tx.send(ServerMessage::GameState(game_state));
    }
}

#[tokio::main]
async fn main() -> Result<()> {
    let args = Args::parse();
    let game_server = Arc::new(Mutex::new(GameServer::new()));
    let (broadcast_tx, mut _broadcast_rx) = broadcast::channel(1000);
    let broadcast_tx_clone = broadcast_tx.clone();

    // Start game loop
    let game_server_clone = game_server.clone();
    tokio::spawn(async move {
        game_loop(game_server_clone, broadcast_tx_clone).await;
    });

    // Start TCP server
    let addr = format!("0.0.0.0:{}", args.port);
    let listener = tokio::net::TcpListener::bind(&addr).await?;
    println!("Server listening on {}", addr);

    while let Ok((stream, _)) = listener.accept().await {
        let game_server_clone = game_server.clone();
        let broadcast_tx_clone = broadcast_tx.clone();
        tokio::spawn(async move {
            if let Err(e) = handle_connection(game_server_clone, stream, broadcast_tx_clone).await {
                eprintln!("Connection error: {e}");
            }
        });
    }

    Ok(())
}

mod rand {
    use rand::distributions::{Distribution, Standard};
    use rand::rngs::StdRng;
    use rand::SeedableRng;
    use std::cell::RefCell;
    use std::time::{SystemTime, UNIX_EPOCH};

    pub fn random<T>() -> T
    where
        Standard: Distribution<T>,
    {
        thread_local! {
            static RNG: RefCell<StdRng> = RefCell::new({
                let seed = SystemTime::now()
                    .duration_since(UNIX_EPOCH)
                    .unwrap()
                    .as_nanos() as u64;
                StdRng::seed_from_u64(seed)
            });
        }

        RNG.with(|rng| Standard.sample(&mut *rng.borrow_mut()))
    }
}
