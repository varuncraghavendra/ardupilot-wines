#!/usr/bin/env python3
import asyncio
import math
from mavsdk import System
from mavsdk.offboard import (OffboardError, PositionNedYaw)

# --- Helper: Calculate Distance in 3D ---
def get_distance(pos1, n2, e2, d2):
    dx = pos1.north_m - n2
    dy = pos1.east_m - e2
    dz = pos1.down_m - d2
    return math.sqrt(dx*dx + dy*dy + dz*dz)

async def run():
    drone = System()
    await drone.connect(system_address="udpin://0.0.0.0:14540")

    print("Waiting for drone...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print("-- Connected")
            break

    print("-- Arming")
    await drone.action.arm()

    # --- 1. SMART TAKEOFF ---
    target_alt = 30.0
    print(f"-- Setting takeoff altitude to {target_alt}m")
    await drone.action.set_takeoff_altitude(target_alt)
    
    print("-- Taking off")
    await drone.action.takeoff()

    print(f"-- Waiting to reach {target_alt}m...")
    async for position in drone.telemetry.position():
        if position.relative_altitude_m >= (target_alt - 0.5):
            print("-- Takeoff Complete!")
            break
            
    # --- 2. SETUP OFFBOARD ---
    print("-- Starting Offboard Mode")
    initial_pos = PositionNedYaw(0.0, 0.0, -target_alt, 0.0)
    await drone.offboard.set_position_ned(initial_pos)

    try:
        await drone.offboard.start()
    except OffboardError as error:
        print(f"Offboard failed: {error}")
        await drone.action.land()
        return

    # --- 3. DEFINE 100m SQUARE ---
    # Down is negative altitude (-10.0)
    # Changed 20.0 -> 100.0
    waypoints = [
        (100.0, 0.0, -target_alt),    # North 100m
        (100.0, 100.0, -target_alt),  # East 100m
        (0.0, 100.0, -target_alt),    # South (back to North 0)
        (0.0, 0.0, -target_alt)       # West (Home)
    ]

    # --- 4. EXECUTION LOOP ---
    for i, (t_north, t_east, t_down) in enumerate(waypoints):
        print(f"\n--- Moving to Point {i+1}: N={t_north}, E={t_east} ---")
        
        await drone.offboard.set_position_ned(PositionNedYaw(t_north, t_east, t_down, 0.0))
        
        async for position_ned in drone.telemetry.position_velocity_ned():
            pos = position_ned.position
            dist = get_distance(pos, t_north, t_east, t_down)
            
            # Check if we are within 2 meters (increased slightly for larger distances)
            if dist < 2.0:
                print(f"-- Reached Point {i+1}")
                break
            
            # Keep sending command (Failsafe requirement)
            await drone.offboard.set_position_ned(PositionNedYaw(t_north, t_east, t_down, 0.0))
            await asyncio.sleep(0.2)

    print("-- Mission Complete.")
    
    # Stop Offboard before switching modes
    try:
        await drone.offboard.stop()
    except OffboardError:
        pass

    # --- 5. RTL (RETURN TO LAUNCH) ---
    print("-- Returning to Launch (RTL)")
    await drone.action.return_to_launch()

if __name__ == "__main__":
    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)
    loop.run_until_complete(run())