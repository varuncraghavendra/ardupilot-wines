#!/usr/bin/env python3
import asyncio
import math
import sys
import numpy as np
from shapely.geometry import Point, Polygon
from mavsdk import System
from mavsdk.action import ActionError

# --- CONFIGURATION ---
TARGET_AREA_M2 = 5000.0
MIN_AREA_STOP_M2 = 100.0
DRONE_SPEED_MPS = 8.0     
MEASURE_HOLD_S = 3.0      
SHRINK_FACTOR = 0.7

# Fake Radio Parameters
P0_DBM = -50.0
PATHLOSS_N = 3.0
NOISE_SIGMA = 1.5
UE_CONN_RADIUS_M = 60.0

# --- MATH HELPER FUNCTIONS ---
def meters_to_degrees(meters, is_longitude=False, center_lat=None):
    if is_longitude and center_lat is not None:
        return meters / (111111.0 * math.cos(math.radians(center_lat)))
    return meters / 111111.0

def degrees_to_meters(deg, is_longitude=False, center_lat=None):
    if is_longitude and center_lat is not None:
        return deg * (111111.0 * math.cos(math.radians(center_lat)))
    return deg * 111111.0

def geo_distance_m(lat1, lng1, lat2, lng2):
    lat_dist = degrees_to_meters(abs(lat2 - lat1))
    lng_dist = degrees_to_meters(abs(lng2 - lng1), is_longitude=True, center_lat=(lat1 + lat2) / 2.0)
    return math.hypot(lat_dist, lng_dist)

def get_midpoint(p1, p2):
    return ((p1[0] + p2[0]) / 2.0, (p1[1] + p2[1]) / 2.0)

def weighted_point(p_from, p_to, w):
    return (w * p_from[0] + (1.0 - w) * p_to[0], w * p_from[1] + (1.0 - w) * p_to[1])

def get_polygon_area_meters(corners):
    poly = Polygon(corners)
    return poly.area * (111111.0 ** 2)

def cut_polygon_in_half(corners, best_edge_idx):
    c_idx = best_edge_idx
    d_idx = (best_edge_idx + 1) % 4
    a_idx = (best_edge_idx + 2) % 4
    b_idx = (best_edge_idx + 3) % 4

    point_c = corners[c_idx]
    point_d = corners[d_idx]
    point_a = corners[a_idx]
    point_b = corners[b_idx]

    shrunk_da = weighted_point(point_d, point_a, SHRINK_FACTOR)
    shrunk_cb = weighted_point(point_c, point_b, SHRINK_FACTOR)

    new_corners = [point_c, point_d, shrunk_da, shrunk_cb]
    return new_corners

# --- FAKE RADIO LOGIC ---
def cluster_average_rsrp(lat, lng, ue_coords):
    metrics = []
    for ue_lat, ue_lng in ue_coords:
        d = geo_distance_m(lat, lng, ue_lat, ue_lng)
        if d <= UE_CONN_RADIUS_M:
            d_eff = max(d, 1.0)
            base = P0_DBM - 10.0 * PATHLOSS_N * math.log10(d_eff)
            m_i = base + np.random.normal(0.0, NOISE_SIGMA)
            metrics.append(m_i)
    if not metrics:
        return -120.0
    return float(np.mean(metrics))

def generate_ue_cluster(center_lat, center_lng):
    ue_coords = []
    cluster_lat = center_lat + meters_to_degrees(30)
    cluster_lng = center_lng + meters_to_degrees(30, True, center_lat)
    for _ in range(5): 
        offset_lat = np.random.uniform(-0.0002, 0.0002)
        offset_lng = np.random.uniform(-0.0002, 0.0002)
        ue_coords.append((cluster_lat + offset_lat, cluster_lng + offset_lng))
    return ue_coords

# --- DRONE OPTIMIZER CLASS ---
class DroneOptimizer:
    def __init__(self, drone, initial_corners, ue_coords, fly_altitude):
        self.drone = drone
        self.corners = initial_corners
        self.ue_coords = ue_coords
        self.fly_alt = fly_altitude 
        self.measured_positions = {}
        self.iteration = 0

    async def move_drone_to(self, lat, lng, reason):
        print(f"--> Moving to {reason} at ({lat:.6f}, {lng:.6f})")
        
        try:
            # 1. Send Command
            await self.drone.action.goto_location(lat, lng, self.fly_alt, 0)
        
        except ActionError as e:
            # --- ABORT ON FAILURE ---
            print(f"\n!!! MOVEMENT REJECTED: {e} !!!")
            print("!!! GEOFENCE BREACH DETECTED !!!")
            print("!!! ABORTING MISSION & HOLDING POSITION (HOVER) !!!")
            
            try:
                await self.drone.action.hold() # HOLD = LOITER (HOVER)
            except ActionError:
                pass 
            
            # STOP SCRIPT
            raise RuntimeError("Mission Aborted due to Geofence/Movement Rejection")
        
        # 2. Monitor Distance
        max_wait_cycles = 60 
        cycles = 0
        
        while True:
            async for position in self.drone.telemetry.position():
                current_lat = position.latitude_deg
                current_lng = position.longitude_deg
                dist = geo_distance_m(current_lat, current_lng, lat, lng)
                
                if dist < 1.5:
                    return True # Success
                break 
            
            await asyncio.sleep(0.5)
            cycles += 1
            if cycles > max_wait_cycles:
                print("xx TIMEOUT: Drone didn't reach target. Aborting.")
                await self.drone.action.hold()
                raise RuntimeError("Mission Aborted: Timeout")

    async def measure_rsrp(self, lat, lng, label):
        await self.move_drone_to(lat, lng, label)
        
        print(f"    Holding {MEASURE_HOLD_S}s to measure...")
        await asyncio.sleep(MEASURE_HOLD_S)
        
        metric = cluster_average_rsrp(lat, lng, self.ue_coords)
        print(f"    Measured: {metric:.1f} dBm")
        
        pos_key = (round(lat, 8), round(lng, 8))
        self.measured_positions[pos_key] = metric
        return metric

    async def run_bisection(self):
        print("\n=== TRACING INITIAL SQUARE ===")
        measurements = []
        for i, (lng, lat) in enumerate(self.corners):
            val = await self.measure_rsrp(lat, lng, f"Corner_{i}")
            measurements.append(val)
            
        while True:
            self.iteration += 1
            area = get_polygon_area_meters(self.corners)
            print(f"\n=== ITERATION {self.iteration} (Area: {area:.0f} m²) ===")
            
            if area < MIN_AREA_STOP_M2:
                print("Target area reached. Stopping.")
                break

            edge_scores = []
            for i in range(4):
                j = (i + 1) % 4
                score = measurements[i] + measurements[j]
                edge_scores.append((score, i))
                print(f"    Edge {i}-{j} Score: {score:.1f}")

            best_score, best_idx = max(edge_scores, key=lambda x: x[0])
            print(f"    Best Edge is {best_idx} (Score {best_score:.1f})")

            c_idx = best_idx
            d_idx = (best_idx + 1) % 4
            mid_lng, mid_lat = get_midpoint(self.corners[c_idx], self.corners[d_idx])
            
            await self.measure_rsrp(mid_lat, mid_lng, "Midpoint")
            
            self.corners = cut_polygon_in_half(self.corners, best_idx)
            
            print("    Resampling new polygon corners...")
            measurements = []
            for i, (lng, lat) in enumerate(self.corners):
                pos_key = (round(lat, 8), round(lng, 8))
                if pos_key in self.measured_positions:
                     val = self.measured_positions[pos_key]
                     print(f"    Corner {i} (Cached): {val:.1f}")
                else:
                     val = await self.measure_rsrp(lat, lng, f"New_Corner_{i}")
                measurements.append(val)

        avg_lng = sum(c[0] for c in self.corners)/4
        avg_lat = sum(c[1] for c in self.corners)/4
        print("\n=== GOING TO OPTIMUM ===")
        await self.move_drone_to(avg_lat, avg_lng, "FINAL OPTIMUM")
        print("Mission Complete.")

# --- MAIN ASYNC ROUTINE ---
async def run():
    drone = System()
    await drone.connect(system_address="udpin://0.0.0.0:14540")

    print("Waiting for drone...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print("-- Connected")
            break

    # --- 1. TAKEOFF SEQUENCE ---
    print("-- Arming")
    await drone.action.arm()

    target_rel_alt = 15.0
    print(f"-- Taking off to {target_rel_alt}m")
    await drone.action.set_takeoff_altitude(target_rel_alt)
    await drone.action.takeoff()

    print("-- Waiting for altitude...")
    abs_alt = 0.0
    async for position in drone.telemetry.position():
        if position.relative_altitude_m >= (target_rel_alt - 0.5):
            abs_alt = position.absolute_altitude_m
            print(f"-- Altitude Reached: {abs_alt:.1f}m AMSL")
            break
    # --------------------------------------

    # 2. CONFIG MANUAL CORNERS
    # Order: Bottom-Left -> Bottom-Right -> Top-Right -> Top-Left
    initial_corners = [
        (-71.0862127, 42.3388219),
        (-71.0853061, 42.3382420),
        (-71.0841514, 42.3390202),
        (-71.0851666, 42.3396268)
    ]
    ue_coords = generate_ue_cluster(42.3388, -71.0862) 

    # 3. RUN MISSION
    optimizer = DroneOptimizer(drone, initial_corners, ue_coords, abs_alt)
    
    try:
        print("-- Starting Optimization")
        await optimizer.run_bisection()
        print("-- Mission Done. Landing.")
        await drone.action.land() # Only land if successful
    
    except RuntimeError as e:
        print(f"\n>> SCRIPT STOPPED: {e}")
        # NO LAND COMMAND HERE. IT WILL HOLD.
    
    except Exception as e:
        print(f"\n>> UNEXPECTED ERROR: {e}")
        # NO LAND COMMAND HERE. IT WILL HOLD.
    
    finally:
        # 4. FINAL SAFETY
        print("\n>> ENDING: Switching to LOITER (HOVER)")
        try:
            await drone.action.hold()
        except:
            pass

if __name__ == "__main__":
    try:
        asyncio.run(run())
    except KeyboardInterrupt:
        print("\nUser Cancelled.")