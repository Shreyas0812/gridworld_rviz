#!/usr/bin/env python3
"""
Warehouse Trajectory Generator

Generates trajectories for a single agent in a warehouse environment where:
- Each trajectory starts at an induct (inject) station
- Each trajectory ends at an eject station
- Robots cannot move directly between adjacent eject stations
- Output is continuous timesteps for a single agent
"""

import csv
import random
from typing import List, Tuple, Set
import heapq


class WarehouseConfig:
    """Warehouse configuration based on gridworld_warehouse_small.yaml"""
    
    def __init__(self):
        self.grid_width = 30
        self.grid_height = 30
        
        # Induct stations (x, y, station_id) - where robots pick up items
        self.induct_stations = [
            (2, 4, 1),
            (2, 12, 2),
            (2, 20, 3),
            (2, 26, 4),
            (26, 4, 5),
            (27, 4, 5),
            (26, 25, 6),
            (27, 25, 6),
        ]
        
        # Eject stations (x, y, station_id) - where robots drop off items
        self.eject_stations = [
            (11, 4, 1), (11, 5, 2), (12, 4, 3), (12, 5, 4),
            (13, 4, 5), (13, 5, 6), (14, 4, 7), (14, 5, 8),
            (15, 4, 9), (15, 5, 10), (16, 4, 11), (16, 5, 12),
            (17, 4, 13), (17, 5, 14), (18, 4, 15), (18, 5, 16),
            (19, 4, 17), (19, 5, 18),
            (11, 12, 19), (12, 12, 20), (13, 12, 21), (14, 12, 22),
            (15, 12, 23), (16, 12, 24),
            (15, 19, 25), (16, 19, 26), (17, 19, 27), (18, 19, 28), (19, 19, 29),
            (11, 26, 30), (12, 26, 31), (13, 26, 32), (14, 26, 33),
            (15, 26, 34), (16, 26, 35), (17, 26, 36), (18, 26, 37), (19, 26, 38)
        ]
        
        # Obstacle regions - cells that cannot be traversed
        self.obstacles = self._parse_obstacles()
        
    def _parse_obstacles(self) -> Set[Tuple[int, int]]:
        """Parse obstacle regions into a set of blocked cells"""
        obstacle_regions = [
            # At the ends of eject stations
            (10, 4, 10, 5), (20, 4, 20, 5), (10, 12, 10, 12),
            (17, 12, 17, 12), (14, 19, 14, 19), (20, 19, 20, 19),
            (10, 26, 10, 26), (20, 26, 20, 26),
            # Warehouse perimeter walls
            (0, 0, 29, 1), (0, 28, 29, 29), (0, 0, 1, 29), (28, 0, 29, 29),
            # Structural columns
            (8, 8, 9, 9), (21, 8, 22, 9), (8, 21, 9, 22),
            (21, 21, 22, 22), (15, 15, 16, 16),
            # Maintenance bay
            (26, 5, 27, 24),
            # Dividers
            (2, 7, 5, 8), (2, 15, 5, 16), (2, 23, 5, 24)
        ]
        
        obstacles = set()
        for region in obstacle_regions:
            start_x, start_y, end_x, end_y = region
            for x in range(start_x, end_x + 1):
                for y in range(start_y, end_y + 1):
                    obstacles.add((x, y))
        
        return obstacles
    
    def is_valid_cell(self, x: int, y: int) -> bool:
        """Check if a cell is within bounds and not an obstacle"""
        if x < 0 or x >= self.grid_width or y < 0 or y >= self.grid_height:
            return False
        return (x, y) not in self.obstacles
    
    def are_adjacent(self, pos1: Tuple[int, int], pos2: Tuple[int, int]) -> bool:
        """Check if two positions are adjacent (Manhattan distance = 1)"""
        return abs(pos1[0] - pos2[0]) + abs(pos1[1] - pos2[1]) == 1


class PathPlanner:
    """A* pathfinding with warehouse constraints"""
    
    def __init__(self, config: WarehouseConfig):
        self.config = config
        self.eject_positions = {(x, y) for x, y, _ in config.eject_stations}
    
    def heuristic(self, pos1: Tuple[int, int], pos2: Tuple[int, int]) -> float:
        """Manhattan distance heuristic"""
        return abs(pos1[0] - pos2[0]) + abs(pos1[1] - pos2[1])
    
    def get_neighbors(self, pos: Tuple[int, int], goal: Tuple[int, int]) -> List[Tuple[int, int]]:
        """Get valid neighboring cells with warehouse constraints"""
        x, y = pos
        neighbors = [
            (x + 1, y), (x - 1, y),
            (x, y + 1), (x, y - 1)
        ]
        
        valid_neighbors = []
        for nx, ny in neighbors:
            if not self.config.is_valid_cell(nx, ny):
                continue
            
            # Check if neighbor is an eject station
            if (nx, ny) in self.eject_positions:
                # Only allow moving to eject station if it's the goal
                if (nx, ny) == goal:
                    valid_neighbors.append((nx, ny))
                # Don't allow moving through eject stations
                continue
            
            # Check if current position is an eject station
            if pos in self.eject_positions:
                # If we're at an eject station, we can only leave (not move to adjacent eject)
                if (nx, ny) not in self.eject_positions:
                    valid_neighbors.append((nx, ny))
                # This prevents moving from one eject to adjacent eject
                continue
            
            valid_neighbors.append((nx, ny))
        
        return valid_neighbors
    
    def find_path(self, start: Tuple[int, int], goal: Tuple[int, int]) -> List[Tuple[int, int]]:
        """Find path from start to goal using A* with warehouse constraints"""
        
        if start == goal:
            return [start]
        
        # Priority queue: (f_score, counter, position, path)
        counter = 0
        heap = [(0, counter, start, [start])]
        visited = {start: 0}
        
        while heap:
            f_score, _, current, path = heapq.heappop(heap)
            
            if current == goal:
                return path
            
            # Get g_score for current
            g_score = len(path) - 1
            
            for neighbor in self.get_neighbors(current, goal):
                new_g_score = g_score + 1
                
                # Check if we found a better path to this neighbor
                if neighbor not in visited or new_g_score < visited[neighbor]:
                    visited[neighbor] = new_g_score
                    h_score = self.heuristic(neighbor, goal)
                    new_f_score = new_g_score + h_score
                    counter += 1
                    new_path = path + [neighbor]
                    heapq.heappush(heap, (new_f_score, counter, neighbor, new_path))
        
        # No path found
        return None


class TrajectoryGenerator:
    """Generate warehouse trajectories for a single agent"""
    
    def __init__(self, config: WarehouseConfig):
        self.config = config
        self.planner = PathPlanner(config)
    
    def generate_trajectory(self, num_trips: int, agent_id: int = 0) -> List[dict]:
        """
        Generate a continuous trajectory with multiple trips
        
        Args:
            num_trips: Number of induct->eject trips to generate
            agent_id: ID of the agent
            
        Returns:
            List of trajectory points with agent, timestep, location, x, y, orientation
        """
        trajectory = []
        timestep = 0
        
        for trip in range(num_trips):
            # Randomly select induct and eject stations
            induct = random.choice(self.config.induct_stations)
            eject = random.choice(self.config.eject_stations)
            
            induct_pos = (induct[0], induct[1])
            eject_pos = (eject[0], eject[1])
            
            # Find path from induct to eject
            path = self.planner.find_path(induct_pos, eject_pos)
            
            if path is None:
                print(f"Warning: No path found from {induct_pos} to {eject_pos}, skipping trip {trip}")
                continue
            
            # Add path to trajectory
            for pos in path:
                x, y = pos
                location = y * self.config.grid_width + x  # Convert to location ID
                
                trajectory.append({
                    'agent': agent_id,
                    'timestep': timestep,
                    'location': location,
                    'x': x,
                    'y': y,
                    'orientation': -1  # Not used, set to -1
                })
                timestep += 1
            
            # Optional: Add waiting time at eject station
            wait_time = random.randint(1, 3)
            for _ in range(wait_time):
                x, y = eject_pos
                location = y * self.config.grid_width + x
                trajectory.append({
                    'agent': agent_id,
                    'timestep': timestep,
                    'location': location,
                    'x': x,
                    'y': y,
                    'orientation': -1
                })
                timestep += 1
        
        return trajectory
    
    def save_trajectory(self, trajectory: List[dict], filename: str):
        """Save trajectory to CSV file"""
        with open(filename, 'w', newline='') as csvfile:
            fieldnames = ['agent', 'timestep', 'location', 'x', 'y', 'orientation']
            writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
            
            writer.writeheader()
            for point in trajectory:
                writer.writerow(point)
        
        print(f"Saved trajectory with {len(trajectory)} timesteps to {filename}")


def main():
    """Generate example trajectories"""
    
    config = WarehouseConfig()
    generator = TrajectoryGenerator(config)
    
    # Generate trajectories for multiple agents
    for agent_id in range(0, 6):  # Agents 0-5
        num_trips = random.randint(5, 10)  # Random number of trips per agent
        trajectory = generator.generate_trajectory(num_trips=num_trips, agent_id=agent_id)
        
        filename = f"agent_{agent_id}_trajectory.csv"
        generator.save_trajectory(trajectory, filename)
    
    print("\nTrajectory generation complete!")
    print("Files generated: agent_0_trajectory.csv through agent_5_trajectory.csv")
    print("\nEach file contains:")
    print("- Continuous timesteps for a single agent")
    print("- Multiple trips from induct stations to eject stations")
    print("- Paths that respect warehouse obstacles")
    print("- No direct movement between adjacent eject stations")


if __name__ == "__main__":
    main()
