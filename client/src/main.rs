use anyhow::Result;
use bytemuck::{Pod, Zeroable};
use clap::Parser;
use hellfire_crucible_shared::{
    ClientMessage, GameState, PlayerId, ServerMessage, Vec2, ARENA_SIZE, BLADE_LENGTH, BLADE_WIDTH,
    PLAYER_SIZE,
};
use std::sync::Arc;
use std::time::Duration;
use tokio::sync::{mpsc, Mutex};
use winit::{
    application::ApplicationHandler,
    event::{ElementState, WindowEvent},
    event_loop::{ControlFlow, EventLoop},
    keyboard::{KeyCode, PhysicalKey},
    window::Window,
};

// Macro to convert hex color to RGB float array
macro_rules! hex_color {
    ($hex:expr) => {{
        let hex = $hex.trim_start_matches('#');
        let r = u8::from_str_radix(&hex[0..2], 16).unwrap() as f32 / 255.0;
        let g = u8::from_str_radix(&hex[2..4], 16).unwrap() as f32 / 255.0;
        let b = u8::from_str_radix(&hex[4..6], 16).unwrap() as f32 / 255.0;
        [r, g, b]
    }};
}

#[derive(Parser, Debug)]
#[command(name = "hellfire-crucible-client")]
#[command(about = "Hellfire Crucible game client")]
struct Args {
    #[arg(long, default_value = "ws://localhost:8080", global = true)]
    server: String,

    #[command(subcommand)]
    mode: Option<GameMode>,
}

#[derive(Parser, Debug)]
enum GameMode {
    /// Play as an AI with random movements
    Ai {
        #[arg(long, default_value = "1")]
        count: usize,
    },
    /// Local multiplayer - control two players with one client
    Local,
}

#[repr(C)]
#[derive(Copy, Clone, Debug, Pod, Zeroable)]
struct Vertex {
    position: [f32; 2],
    color: [f32; 3],
}

struct Renderer {
    surface: wgpu::Surface<'static>,
    device: wgpu::Device,
    queue: wgpu::Queue,
    config: wgpu::SurfaceConfiguration,
    pipeline: wgpu::RenderPipeline,
    vertex_buffer: wgpu::Buffer,
}

impl Renderer {
    async fn new(window: Arc<Window>) -> Result<Self> {
        let size = window.inner_size();
        let instance = wgpu::Instance::default();
        let surface = instance.create_surface(window)?;
        let adapter = instance
            .request_adapter(&wgpu::RequestAdapterOptions {
                power_preference: wgpu::PowerPreference::default(),
                compatible_surface: Some(&surface),
                force_fallback_adapter: false,
            })
            .await
            .ok_or_else(|| anyhow::anyhow!("No adapter found"))?;

        let (device, queue) = adapter
            .request_device(
                &wgpu::DeviceDescriptor {
                    required_features: wgpu::Features::empty(),
                    required_limits: wgpu::Limits::default(),
                    label: None,
                    memory_hints: Default::default(),
                },
                None,
            )
            .await?;

        let config = surface
            .get_default_config(&adapter, size.width, size.height)
            .ok_or_else(|| anyhow::anyhow!("No default config"))?;
        surface.configure(&device, &config);

        let shader = device.create_shader_module(wgpu::ShaderModuleDescriptor {
            label: Some("Shader"),
            source: wgpu::ShaderSource::Wgsl(include_str!("shader.wgsl").into()),
        });

        let pipeline_layout = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
            label: Some("Pipeline Layout"),
            bind_group_layouts: &[],
            push_constant_ranges: &[],
        });

        let pipeline = device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
            label: Some("Render Pipeline"),
            layout: Some(&pipeline_layout),
            vertex: wgpu::VertexState {
                module: &shader,
                entry_point: "vs_main",
                buffers: &[wgpu::VertexBufferLayout {
                    array_stride: std::mem::size_of::<Vertex>() as wgpu::BufferAddress,
                    step_mode: wgpu::VertexStepMode::Vertex,
                    attributes: &[
                        wgpu::VertexAttribute {
                            offset: 0,
                            shader_location: 0,
                            format: wgpu::VertexFormat::Float32x2,
                        },
                        wgpu::VertexAttribute {
                            offset: std::mem::size_of::<[f32; 2]>() as wgpu::BufferAddress,
                            shader_location: 1,
                            format: wgpu::VertexFormat::Float32x3,
                        },
                    ],
                }],
                compilation_options: Default::default(),
            },
            fragment: Some(wgpu::FragmentState {
                module: &shader,
                entry_point: "fs_main",
                targets: &[Some(wgpu::ColorTargetState {
                    format: config.format,
                    blend: Some(wgpu::BlendState::REPLACE),
                    write_mask: wgpu::ColorWrites::ALL,
                })],
                compilation_options: Default::default(),
            }),
            primitive: wgpu::PrimitiveState {
                topology: wgpu::PrimitiveTopology::TriangleList,
                strip_index_format: None,
                front_face: wgpu::FrontFace::Ccw,
                cull_mode: Some(wgpu::Face::Back),
                polygon_mode: wgpu::PolygonMode::Fill,
                unclipped_depth: false,
                conservative: false,
            },
            depth_stencil: None,
            multisample: wgpu::MultisampleState::default(),
            multiview: None,
            cache: None,
        });

        let vertex_buffer = device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("Vertex Buffer"),
            size: 10000 * std::mem::size_of::<Vertex>() as u64,
            usage: wgpu::BufferUsages::VERTEX | wgpu::BufferUsages::COPY_DST,
            mapped_at_creation: false,
        });

        Ok(Self {
            surface,
            device,
            queue,
            config,
            pipeline,
            vertex_buffer,
        })
    }

    fn resize(&mut self, new_size: winit::dpi::PhysicalSize<u32>) {
        if new_size.width > 0 && new_size.height > 0 {
            self.config.width = new_size.width;
            self.config.height = new_size.height;
            self.surface.configure(&self.device, &self.config);
        }
    }

    fn render(
        &mut self,
        game_state: &GameState,
        my_id: Option<PlayerId>,
        my_id_2: Option<PlayerId>,
    ) -> Result<()> {
        let output = self.surface.get_current_texture()?;
        let view = output
            .texture
            .create_view(&wgpu::TextureViewDescriptor::default());

        let mut encoder = self
            .device
            .create_command_encoder(&wgpu::CommandEncoderDescriptor {
                label: Some("Render Encoder"),
            });

        let mut vertices = Vec::new();

        // Calculate pixels per meter based on window size with padding
        // This determines the scale at which world coordinates are rendered
        let padding = 100.0; // Screen space padding in pixels
        let window_width = self.config.width as f32;
        let window_height = self.config.height as f32;
        let usable_width = window_width - 2.0 * padding;
        let usable_height = window_height - 2.0 * padding;
        let usable_min = usable_width.min(usable_height).max(100.0); // Ensure minimum size
        let pixels_per_meter = usable_min / ARENA_SIZE; // Scale factor: screen pixels per world meter

        // Draw arena background first (darker grey)
        let arena_bg_color = hex_color!("#0D0D0D");
        self.add_rect(
            &mut vertices,
            0.0,
            0.0,
            ARENA_SIZE,
            ARENA_SIZE,
            arena_bg_color,
            pixels_per_meter,
        );

        // Add arena walls
        let wall_color = hex_color!("#4D4D4D");
        let half_arena = ARENA_SIZE / 2.0;
        self.add_rect(
            &mut vertices,
            0.0,
            -half_arena + 0.1,
            ARENA_SIZE,
            0.2,
            wall_color,
            pixels_per_meter,
        ); // Bottom
        self.add_rect(
            &mut vertices,
            0.0,
            half_arena - 0.1,
            ARENA_SIZE,
            0.2,
            wall_color,
            pixels_per_meter,
        ); // Top
        self.add_rect(
            &mut vertices,
            -half_arena + 0.1,
            0.0,
            0.2,
            ARENA_SIZE,
            wall_color,
            pixels_per_meter,
        ); // Left
        self.add_rect(
            &mut vertices,
            half_arena - 0.1,
            0.0,
            0.2,
            ARENA_SIZE,
            wall_color,
            pixels_per_meter,
        ); // Right

        // Draw players and blades
        for player in &game_state.players {
            let player_color = if player.is_dead {
                hex_color!("#4D4D4D")
            } else if Some(player.id) == my_id {
                hex_color!("#2DD296")
            } else if Some(player.id) == my_id_2 {
                hex_color!("#3D9EFF") // Blue for player 2
            } else {
                hex_color!("#DF2040")
            };

            // Draw player
            self.add_rotated_rect(
                &mut vertices,
                player.position.x,
                player.position.y,
                PLAYER_SIZE,
                PLAYER_SIZE,
                player.rotation,
                player_color,
                pixels_per_meter,
            );

            if !player.is_dead {
                // Draw blade
                // blade_angle is relative to player, so add player rotation for absolute angle
                let blade_absolute_angle = player.rotation + player.blade_angle;

                self.add_rotated_rect(
                    &mut vertices,
                    player.blade_position.x,
                    player.blade_position.y,
                    BLADE_WIDTH,
                    BLADE_LENGTH,
                    blade_absolute_angle,
                    hex_color!("#FFFFFF"),
                    pixels_per_meter,
                );
            }
        }

        self.queue
            .write_buffer(&self.vertex_buffer, 0, bytemuck::cast_slice(&vertices));

        {
            let mut render_pass = encoder.begin_render_pass(&wgpu::RenderPassDescriptor {
                label: Some("Render Pass"),
                color_attachments: &[Some(wgpu::RenderPassColorAttachment {
                    view: &view,
                    resolve_target: None,
                    ops: wgpu::Operations {
                        load: wgpu::LoadOp::Clear(wgpu::Color {
                            r: 0x26 as f64 / 255.0,
                            g: 0x26 as f64 / 255.0,
                            b: 0x26 as f64 / 255.0,
                            a: 1.0,
                        }),
                        store: wgpu::StoreOp::Store,
                    },
                })],
                depth_stencil_attachment: None,
                timestamp_writes: None,
                occlusion_query_set: None,
            });

            render_pass.set_pipeline(&self.pipeline);
            render_pass.set_vertex_buffer(0, self.vertex_buffer.slice(..));
            render_pass.draw(0..vertices.len() as u32, 0..1);
        }

        self.queue.submit(std::iter::once(encoder.finish()));
        output.present();

        Ok(())
    }

    /// Adds an axis-aligned rectangle to the vertex buffer
    /// x, y: Center position of the rectangle in world coordinates (meters)
    /// width, height: Size of the rectangle in world coordinates (meters)
    #[allow(clippy::too_many_arguments)]
    fn add_rect(
        &self,
        vertices: &mut Vec<Vertex>,
        x: f32,
        y: f32,
        width: f32,
        height: f32,
        color: [f32; 3],
        pixels_per_meter: f32,
    ) {
        self.add_rotated_rect(vertices, x, y, width, height, 0.0, color, pixels_per_meter);
    }

    /// Adds a rotated rectangle to the vertex buffer
    /// x, y: Center position of the rectangle in world coordinates (meters)
    /// width, height: Size of the rectangle in world coordinates (meters)
    /// angle: Rotation angle in radians (0 = no rotation)
    /// pixels_per_meter: Conversion factor from world to screen coordinates
    #[allow(clippy::too_many_arguments)]
    fn add_rotated_rect(
        &self,
        vertices: &mut Vec<Vertex>,
        x: f32,
        y: f32,
        width: f32,
        height: f32,
        angle: f32,
        color: [f32; 3],
        pixels_per_meter: f32,
    ) {
        let window_width = self.config.width as f32;
        let window_height = self.config.height as f32;

        let cos = angle.cos();
        let sin = angle.sin();

        // Define corners relative to the rectangle's center in world space
        let corners = [
            [x - width / 2.0, y - height / 2.0],
            [x + width / 2.0, y - height / 2.0],
            [x + width / 2.0, y + height / 2.0],
            [x - width / 2.0, y + height / 2.0],
        ];

        // Apply rotation around the center point
        let mut rotated_corners = [[0.0; 2]; 4];
        for i in 0..4 {
            let dx = corners[i][0] - x;
            let dy = corners[i][1] - y;
            rotated_corners[i][0] = x + dx * cos - dy * sin;
            rotated_corners[i][1] = y + dx * sin + dy * cos;
        }

        let indices = [0, 1, 2, 0, 2, 3];
        for &i in &indices {
            // Convert from world coordinates (meters) to screen pixels
            // World origin (0,0) is at the center of the screen
            let screen_x = rotated_corners[i][0] * pixels_per_meter;
            let screen_y = rotated_corners[i][1] * pixels_per_meter;

            // Convert to normalized device coordinates (NDC)
            // NDC range: -1 to 1, with (0,0) at screen center
            let ndc_x = screen_x / (window_width / 2.0);
            let ndc_y = screen_y / (window_height / 2.0);

            vertices.push(Vertex {
                position: [ndc_x, ndc_y],
                color,
            });
        }
    }
}

struct Game {
    window: Option<Arc<Window>>,
    renderer: Option<Renderer>,
    game_state: Arc<Mutex<GameState>>,
    my_id: Arc<Mutex<Option<PlayerId>>>,
    my_id_2: Arc<Mutex<Option<PlayerId>>>, // Second player ID for local multiplayer
    input_tx: Option<mpsc::UnboundedSender<ClientMessage>>,
    input_tx_2: Option<mpsc::UnboundedSender<ClientMessage>>, // Second player input
    keys_pressed: [bool; 4],                                  // W, A, S, D
    keys_pressed_2: [bool; 4],                                // Arrow keys for player 2
    blade_rotating_cw: bool,                                  // J key for player 1
    blade_rotating_ccw: bool,                                 // K key for player 1
    blade_rotating_cw_2: bool,                                // Comma key for player 2
    blade_rotating_ccw_2: bool,                               // Period key for player 2
    mouse_pos: Vec2,
    mouse_pos_2: Vec2, // Mouse position for player 2
    shutdown_rx: Option<mpsc::UnboundedReceiver<()>>,
    server_url: String,
    ai_mode: bool,
    local_mode: bool,
}

impl Game {
    fn new(server_url: String, ai_mode: bool, local_mode: bool) -> Self {
        Self {
            window: None,
            renderer: None,
            game_state: Arc::new(Mutex::new(GameState {
                players: Vec::new(),
            })),
            my_id: Arc::new(Mutex::new(None)),
            my_id_2: Arc::new(Mutex::new(None)),
            input_tx: None,
            input_tx_2: None,
            keys_pressed: [false; 4],
            keys_pressed_2: [false; 4],
            blade_rotating_cw: false,
            blade_rotating_ccw: false,
            blade_rotating_cw_2: false,
            blade_rotating_ccw_2: false,
            mouse_pos: Vec2::zero(),
            mouse_pos_2: Vec2::zero(),
            shutdown_rx: None,
            server_url,
            ai_mode,
            local_mode,
        }
    }

    fn update_input(&mut self) {
        if let Some(tx) = &self.input_tx {
            let mut movement = Vec2::zero();
            if self.keys_pressed[0] {
                movement.y += 1.0;
            }
            if self.keys_pressed[1] {
                movement.x -= 1.0;
            }
            if self.keys_pressed[2] {
                movement.y -= 1.0;
            }
            if self.keys_pressed[3] {
                movement.x += 1.0;
            }

            // Normalize diagonal movement
            let mag = (movement.x * movement.x + movement.y * movement.y).sqrt();
            if mag > 0.0 {
                movement.x /= mag;
                movement.y /= mag;
            }

            // Determine blade torque (J is CW, K is CCW)
            let blade_torque = if self.blade_rotating_cw {
                1.0
            } else if self.blade_rotating_ccw {
                -1.0
            } else {
                0.0
            };

            let _ = tx.send(ClientMessage::Input {
                movement,
                blade_target: self.mouse_pos,
                blade_torque,
            });
        }

        // Update player 2 if local multiplayer
        if self.local_mode {
            if let Some(tx) = &self.input_tx_2 {
                let mut movement = Vec2::zero();
                if self.keys_pressed_2[0] {
                    movement.y += 1.0;
                }
                if self.keys_pressed_2[1] {
                    movement.x -= 1.0;
                }
                if self.keys_pressed_2[2] {
                    movement.y -= 1.0;
                }
                if self.keys_pressed_2[3] {
                    movement.x += 1.0;
                }

                // Normalize diagonal movement
                let mag = (movement.x * movement.x + movement.y * movement.y).sqrt();
                if mag > 0.0 {
                    movement.x /= mag;
                    movement.y /= mag;
                }

                // Determine blade torque for player 2
                let blade_torque_2 = if self.blade_rotating_ccw_2 {
                    -1.0
                } else if self.blade_rotating_cw_2 {
                    1.0
                } else {
                    0.0
                };

                let _ = tx.send(ClientMessage::Input {
                    movement,
                    blade_target: self.mouse_pos_2,
                    blade_torque: blade_torque_2,
                });
            }
        }
    }
}

impl ApplicationHandler for Game {
    fn resumed(&mut self, event_loop: &winit::event_loop::ActiveEventLoop) {
        let window = Arc::new(
            event_loop
                .create_window(
                    winit::window::Window::default_attributes()
                        .with_title("Hellfire Crucible")
                        .with_inner_size(winit::dpi::LogicalSize::new(800, 800)),
                )
                .unwrap(),
        );

        self.window = Some(window.clone());
        window.request_redraw();

        let game_state = self.game_state.clone();
        let (input_tx, input_rx) = mpsc::unbounded_channel();
        self.input_tx = Some(input_tx.clone());

        let my_id = self.my_id.clone();

        // Create shutdown channel
        let (shutdown_tx, shutdown_rx) = mpsc::unbounded_channel();
        self.shutdown_rx = Some(shutdown_rx);

        // Initialize renderer
        let window_clone = window.clone();
        let renderer = pollster::block_on(Renderer::new(window_clone)).unwrap();
        self.renderer = Some(renderer);

        // Start network connection
        let server_url = self.server_url.clone();
        let ai_mode = self.ai_mode;
        let local_mode = self.local_mode;

        if local_mode {
            // Create second player connection
            let (input_tx_2, input_rx_2) = mpsc::unbounded_channel();
            self.input_tx_2 = Some(input_tx_2.clone());

            let game_state_2 = self.game_state.clone();
            let my_id_2 = self.my_id_2.clone();
            let server_url_2 = server_url.clone();
            let shutdown_tx_2 = shutdown_tx.clone();

            // Connect second player
            tokio::spawn(async move {
                if let Err(e) = connect_to_server(
                    game_state_2,
                    input_tx_2,
                    input_rx_2,
                    my_id_2,
                    server_url_2,
                    false,
                )
                .await
                {
                    eprintln!("Player 2 connection error: {e}");
                }
                let _ = shutdown_tx_2.send(());
            });
        }

        tokio::spawn(async move {
            if let Err(e) =
                connect_to_server(game_state, input_tx, input_rx, my_id, server_url, ai_mode).await
            {
                eprintln!("Connection error: {e}");
            }
            // Send shutdown signal
            let _ = shutdown_tx.send(());
        });
    }

    fn window_event(
        &mut self,
        event_loop: &winit::event_loop::ActiveEventLoop,
        _window_id: winit::window::WindowId,
        event: WindowEvent,
    ) {
        // Check for shutdown signal
        if let Some(shutdown_rx) = &mut self.shutdown_rx {
            if shutdown_rx.try_recv().is_ok() {
                eprintln!("Client shutting down due to lost connection");
                event_loop.exit();
                return;
            }
        }

        match event {
            WindowEvent::CloseRequested => {
                event_loop.exit();
            }
            WindowEvent::Resized(physical_size) => {
                if let Some(renderer) = &mut self.renderer {
                    renderer.resize(physical_size);
                }
            }
            WindowEvent::RedrawRequested => {
                if let Some(renderer) = &mut self.renderer {
                    let game_state = pollster::block_on(self.game_state.lock()).clone();
                    let my_id = *pollster::block_on(self.my_id.lock());
                    let my_id_2 = if self.local_mode {
                        *pollster::block_on(self.my_id_2.lock())
                    } else {
                        None
                    };
                    if let Err(e) = renderer.render(&game_state, my_id, my_id_2) {
                        eprintln!("Render error: {e}");
                    }
                }
                if let Some(window) = &self.window {
                    window.request_redraw();
                }
            }
            WindowEvent::KeyboardInput {
                event:
                    winit::event::KeyEvent {
                        physical_key: PhysicalKey::Code(keycode),
                        state,
                        ..
                    },
                ..
            } => {
                let pressed = state == ElementState::Pressed;
                match keycode {
                    KeyCode::KeyW => {
                        self.keys_pressed[0] = pressed;
                        self.update_input();
                    }
                    KeyCode::KeyA => {
                        self.keys_pressed[1] = pressed;
                        self.update_input();
                    }
                    KeyCode::KeyS => {
                        self.keys_pressed[2] = pressed;
                        self.update_input();
                    }
                    KeyCode::KeyD => {
                        self.keys_pressed[3] = pressed;
                        self.update_input();
                    }
                    KeyCode::Space => {
                        if pressed {
                            if let Some(tx) = &self.input_tx {
                                let _ = tx.send(ClientMessage::Respawn);
                            }
                        }
                    }
                    // Player 2 controls (arrow keys)
                    KeyCode::ArrowUp => {
                        if self.local_mode {
                            self.keys_pressed_2[0] = pressed;
                            self.update_input();
                        }
                    }
                    KeyCode::ArrowLeft => {
                        if self.local_mode {
                            self.keys_pressed_2[1] = pressed;
                            self.update_input();
                        }
                    }
                    KeyCode::ArrowDown => {
                        if self.local_mode {
                            self.keys_pressed_2[2] = pressed;
                            self.update_input();
                        }
                    }
                    KeyCode::ArrowRight => {
                        if self.local_mode {
                            self.keys_pressed_2[3] = pressed;
                            self.update_input();
                        }
                    }
                    KeyCode::Enter => {
                        if pressed && self.local_mode {
                            if let Some(tx) = &self.input_tx_2 {
                                let _ = tx.send(ClientMessage::Respawn);
                            }
                        }
                    }
                    KeyCode::KeyJ => {
                        self.blade_rotating_cw = pressed;
                        self.update_input();
                    }
                    KeyCode::KeyK => {
                        self.blade_rotating_ccw = pressed;
                        self.update_input();
                    }
                    KeyCode::Comma => {
                        if self.local_mode {
                            self.blade_rotating_cw_2 = pressed;
                            self.update_input();
                        }
                    }
                    KeyCode::Period => {
                        if self.local_mode {
                            self.blade_rotating_ccw_2 = pressed;
                            self.update_input();
                        }
                    }
                    _ => {}
                }
            }
            WindowEvent::CursorMoved { position, .. } => {
                if let Some(renderer) = &self.renderer {
                    // Convert screen coordinates to world coordinates
                    // Must use same padding and scale as rendering
                    let padding = 100.0;
                    let window_width = renderer.config.width as f32;
                    let window_height = renderer.config.height as f32;
                    let usable_width = window_width - 2.0 * padding;
                    let usable_height = window_height - 2.0 * padding;
                    let usable_min = usable_width.min(usable_height).max(100.0);
                    let pixels_per_meter = usable_min / ARENA_SIZE;

                    // Screen center is world origin (0,0)
                    let screen_center_x = window_width / 2.0;
                    let screen_center_y = window_height / 2.0;

                    // Convert mouse position to world coordinates
                    // Note: Y is flipped (screen Y increases downward, world Y increases upward)
                    self.mouse_pos.x = (position.x as f32 - screen_center_x) / pixels_per_meter;
                    self.mouse_pos.y = -(position.y as f32 - screen_center_y) / pixels_per_meter;

                    // In local mode, player 2 aims opposite of player 1
                    if self.local_mode {
                        self.mouse_pos_2.x = -self.mouse_pos.x;
                        self.mouse_pos_2.y = -self.mouse_pos.y;
                    }

                    self.update_input();
                }
            }
            _ => {}
        }
    }
}

async fn connect_to_server(
    game_state: Arc<Mutex<GameState>>,
    _input_tx: mpsc::UnboundedSender<ClientMessage>,
    mut input_rx: mpsc::UnboundedReceiver<ClientMessage>,
    my_id: Arc<Mutex<Option<PlayerId>>>,
    server_url: String,
    ai_mode: bool,
) -> Result<()> {
    use futures_util::{SinkExt, StreamExt};
    use tokio_tungstenite::connect_async;

    println!("Connecting to {server_url}...");
    let (ws_stream, _) = connect_async(&server_url).await?;
    let (mut write, mut read) = ws_stream.split();

    // Send join message
    write
        .send(tokio_tungstenite::tungstenite::Message::text(
            serde_json::to_string(&ClientMessage::Join)?,
        ))
        .await?;

    // Start AI controller if in AI mode
    if ai_mode {
        let ai_tx = _input_tx.clone();
        let ai_game_state = game_state.clone();
        let ai_my_id = my_id.clone();
        tokio::spawn(async move {
            ai_controller(ai_tx, ai_game_state, ai_my_id).await;
        });
    }

    // Handle messages
    loop {
        tokio::select! {
            msg = read.next() => {
                match msg {
                    Some(Ok(tokio_tungstenite::tungstenite::Message::Text(text))) => {
                        if let Ok(server_msg) = serde_json::from_str::<ServerMessage>(&text) {
                            match server_msg {
                                ServerMessage::Welcome { your_id } => {
                                    println!("Connected as player {your_id}");
                                    *my_id.lock().await = Some(your_id);
                                }
                                ServerMessage::GameState(state) => {
                                    *game_state.lock().await = state;
                                }
                                ServerMessage::PlayerJoined { id } => {
                                    println!("Player {id} joined");
                                }
                                ServerMessage::PlayerLeft { id } => {
                                    println!("Player {id} left");
                                }
                            }
                        }
                    }
                    Some(Ok(tokio_tungstenite::tungstenite::Message::Close(_))) | None => {
                        eprintln!("Connection closed by server");
                        return Err(anyhow::anyhow!("Connection closed"));
                    }
                    Some(Err(e)) => {
                        eprintln!("WebSocket error: {e}");
                        return Err(e.into());
                    }
                    _ => {}
                }
            }
            msg = input_rx.recv() => {
                if let Some(msg) = msg {
                    if write.send(tokio_tungstenite::tungstenite::Message::text(
                        serde_json::to_string(&msg)?
                    )).await.is_err() {
                        eprintln!("Failed to send message to server");
                        return Err(anyhow::anyhow!("Failed to send message"));
                    }
                }
            }
        }
    }
}

async fn ai_controller(
    input_tx: mpsc::UnboundedSender<ClientMessage>,
    game_state: Arc<Mutex<GameState>>,
    my_id: Arc<Mutex<Option<PlayerId>>>,
) {
    use rand::{rngs::StdRng, Rng, SeedableRng};
    let mut rng = StdRng::from_entropy();

    loop {
        tokio::time::sleep(Duration::from_millis(100)).await;

        // Check if we have an ID yet
        let id = *my_id.lock().await;
        if id.is_none() {
            continue;
        }

        // Get current game state
        let state = game_state.lock().await.clone();
        let my_player = state.players.iter().find(|p| Some(p.id) == id);

        if let Some(player) = my_player {
            if player.is_dead {
                // Respawn after a delay
                tokio::time::sleep(Duration::from_secs(1)).await;
                let _ = input_tx.send(ClientMessage::Respawn);
                continue;
            }

            // Movement biased toward center
            let to_center = Vec2::new(-player.position.x, -player.position.y);
            let distance_from_center = (player.position.x * player.position.x
                + player.position.y * player.position.y)
                .sqrt();

            // Only move toward center if far from it (> 5 meters)
            let center_bias = if distance_from_center > 5.0 {
                normalize(to_center)
            } else {
                Vec2::new(0.0, 0.0)
            };

            // Random movement with more variety
            let action = rng.gen_range(0..20);
            let random_movement = match action {
                0..=4 => Vec2::new(0.0, 0.0),           // Stop (25%)
                5..=6 => Vec2::new(0.0, 1.0),           // Up
                7..=8 => Vec2::new(0.0, -1.0),          // Down
                9..=10 => Vec2::new(-1.0, 0.0),         // Left
                11..=12 => Vec2::new(1.0, 0.0),         // Right
                13 => normalize(Vec2::new(1.0, 1.0)),   // Diagonal
                14 => normalize(Vec2::new(-1.0, 1.0)),  // Diagonal
                15 => normalize(Vec2::new(1.0, -1.0)),  // Diagonal
                16 => normalize(Vec2::new(-1.0, -1.0)), // Diagonal
                // Circling movements
                17 => normalize(Vec2::new(-player.position.y, player.position.x)), // Circle left
                18 => normalize(Vec2::new(player.position.y, -player.position.x)), // Circle right
                _ => normalize(Vec2::new(0.7, 0.7)), // Default diagonal
            };

            // Combine with less center bias (30% center, 70% random) when far from center
            let bias_strength = if distance_from_center > 7.0 { 0.3 } else { 0.1 };
            let combined = Vec2::new(
                center_bias.x * bias_strength + random_movement.x * (1.0 - bias_strength),
                center_bias.y * bias_strength + random_movement.y * (1.0 - bias_strength),
            );
            let movement = if combined.x.abs() > 0.001 || combined.y.abs() > 0.001 {
                normalize(combined)
            } else {
                Vec2::new(0.0, 0.0)
            };

            // Random blade target
            let blade_offset_x = rng.gen_range(-5.0..5.0);
            let blade_offset_y = rng.gen_range(-5.0..5.0);
            let blade_target = Vec2::new(
                player.position.x + blade_offset_x,
                player.position.y + blade_offset_y,
            );

            // Random blade torque for AI
            let blade_torque = match rng.gen_range(0..20) {
                0 => -1.0, // CCW
                1 => 1.0,  // CW
                _ => 0.0,  // No torque most of the time
            };

            let _ = input_tx.send(ClientMessage::Input {
                movement,
                blade_target,
                blade_torque,
            });
        }
    }
}

fn normalize(v: Vec2) -> Vec2 {
    let mag = (v.x * v.x + v.y * v.y).sqrt();
    if mag > 0.0 {
        Vec2::new(v.x / mag, v.y / mag)
    } else {
        v
    }
}

async fn run_headless_ai(server_url: String, ai_number: usize) {
    println!("AI {} connecting to {}", ai_number, server_url);

    loop {
        let game_state = Arc::new(Mutex::new(GameState {
            players: Vec::new(),
        }));
        let my_id = Arc::new(Mutex::new(None));
        let (input_tx, input_rx) = mpsc::unbounded_channel();

        match connect_to_server(
            game_state.clone(),
            input_tx.clone(),
            input_rx,
            my_id.clone(),
            server_url.clone(),
            true,
        )
        .await
        {
            Ok(_) => println!("AI {} disconnected normally", ai_number),
            Err(e) => println!("AI {} disconnected with error: {}", ai_number, e),
        }

        println!("AI {} reconnecting in 1 second...", ai_number);
        tokio::time::sleep(Duration::from_secs(1)).await;
    }
}

fn main() {
    let args = Args::parse();

    let (ai_mode, ai_count) = match args.mode {
        Some(GameMode::Ai { count }) => (true, count),
        _ => (false, 0),
    };
    let local_mode = matches!(args.mode, Some(GameMode::Local));

    // For production: cargo run -- --server wss://hellfirecrucible.com
    // For local dev: cargo run (uses ws://localhost:8080)
    // For AI mode: cargo run -- ai --count 5
    // For local multiplayer: cargo run -- local

    let rt = tokio::runtime::Runtime::new().unwrap();

    if ai_mode {
        // Run headless AI mode
        rt.block_on(async {
            for i in 0..ai_count {
                let server = args.server.clone();
                tokio::spawn(async move {
                    // Stagger spawn by 1 second each
                    tokio::time::sleep(Duration::from_secs(i as u64)).await;
                    run_headless_ai(server, i + 1).await;
                });
            }

            // Keep the main thread alive
            loop {
                tokio::time::sleep(Duration::from_secs(60)).await;
            }
        });
    } else {
        // Run normal windowed mode
        let _guard = rt.enter();

        let event_loop = EventLoop::new().unwrap();
        event_loop.set_control_flow(ControlFlow::Poll);

        let mut game = Game::new(args.server, false, local_mode);
        event_loop.run_app(&mut game).unwrap();
    }
}
