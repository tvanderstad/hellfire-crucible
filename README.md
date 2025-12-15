# Hellfire Crucible

A minimalist top-down multiplayer arena fighting game with physics-based blade combat.

## Running the Game

1. Start the server:
```bash
cargo run -p hellfire-crucible-server
```

2. In another terminal, start a client:
```bash
cargo run -p hellfire-crucible-client
```

3. Start additional clients for multiplayer (each in its own terminal).

## Controls

- **WASD**: Move your character
- **J**: Rotate blade clockwise
- **K**: Rotate blade counter-clockwise
- **Space**: Respawn when dead

## Gameplay

- Players control square characters with rotating blades
- Blades rotate with physics-based momentum
- Instant death on blade contact
- Last player standing wins

The game uses a physics engine where all movement is force-based, creating natural and responsive controls while maintaining realistic interactions between players and their blades.