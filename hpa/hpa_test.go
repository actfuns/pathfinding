package hpa

import (
	"testing"

	"github.com/actfuns/navpath/finder"
	"github.com/actfuns/navpath/grid"
)

// TestHPASingleChunk verifies HPA* falls back to A* when start and end are in the same chunk.
func TestHPASingleChunk(t *testing.T) {
	grid := grid.NewOrthogonalGrid([][]int{
		{0, 0, 0, 0, 0},
		{0, 0, 0, 0, 0},
		{0, 0, 0, 0, 0},
		{0, 0, 0, 0, 0},
		{0, 0, 0, 0, 0},
	})
	world := NewHPABuilder(HPAConfig{ChunkSize: 8}).Build(grid)
	f := NewHPAStarFinder(nil)
	result := f.FindPath(0, 0, 4, 4, grid, world)
	if result.Waypoints == nil {
		t.Fatal("expected path, got nil")
	}
	if result.Waypoints[0] != [2]int{0, 0} {
		t.Errorf("expected start (0,0), got %v", result.Waypoints[0])
	}
	if result.Waypoints[len(result.Waypoints)-1] != [2]int{4, 4} {
		t.Errorf("expected end (4,4), got %v", result.Waypoints[len(result.Waypoints)-1])
	}
	if len(result.PortalKeys) != 0 {
		t.Errorf("expected no portal keys for single chunk, got %v", result.PortalKeys)
	}
	verifyWalkable(t, grid, result.Waypoints)
}

// TestHPATwoChunks verifies a path crossing chunk boundaries on an open grid.
func TestHPATwoChunks(t *testing.T) {
	grid := grid.NewOrthogonalGrid([][]int{
		{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
		{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
		{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
		{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
	})
	world := NewHPABuilder(HPAConfig{ChunkSize: 8}).Build(grid)
	f := NewHPAStarFinder(nil)
	result := f.FindPath(0, 0, 19, 3, grid, world)
	if result.Waypoints == nil {
		t.Fatal("expected path, got nil")
	}
	assertStartEnd(t, result.Waypoints, 0, 0, 19, 3)
	verifyWalkable(t, grid, result.Waypoints)
	if len(result.PortalKeys) == 0 {
		t.Error("expected portal path crossing chunks")
	}
	t.Logf("portal path length: %d, waypoint path length: %d", len(result.PortalKeys), len(result.Waypoints))
}

// TestHPAWithObstacles verifies HPA* on a grid with obstacles.
func TestHPAWithObstacles(t *testing.T) {
	grid := grid.NewOrthogonalGrid([][]int{
		{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
		{0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0},
		{0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0},
		{0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0},
		{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
		{0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0},
		{0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0},
		{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
	})
	world := NewHPABuilder(HPAConfig{ChunkSize: 8}).Build(grid)
	f := NewHPAStarFinder(nil)
	result := f.FindPath(0, 0, 15, 7, grid, world)
	if result.Waypoints == nil {
		t.Fatal("expected path, got nil")
	}
	assertStartEnd(t, result.Waypoints, 0, 0, 15, 7)
	verifyWalkable(t, grid, result.Waypoints)
	t.Logf("portal path length: %d, waypoint path length: %d", len(result.PortalKeys), len(result.Waypoints))
}

// TestHPAUnreachable verifies HPA* returns empty when there's no path.
func TestHPAUnreachable(t *testing.T) {
	grid := grid.NewOrthogonalGrid([][]int{
		{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
		{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
		{1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1},
		{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
		{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
	})
	world := NewHPABuilder(HPAConfig{ChunkSize: 8}).Build(grid)
	f := NewHPAStarFinder(nil)
	result := f.FindPath(0, 2, 19, 2, grid, world)
	if result.Waypoints != nil {
		t.Error("expected nil path for unreachable destination, got a path")
	}
}

// TestHPAStartIsWall verifies wall start returns empty.
func TestHPAStartIsWall(t *testing.T) {
	grid := grid.NewOrthogonalGrid([][]int{
		{1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
		{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
	})
	world := NewHPABuilder(HPAConfig{ChunkSize: 8}).Build(grid)
	f := NewHPAStarFinder(nil)
	result := f.FindPath(0, 0, 19, 1, grid, world)
	if result.Waypoints != nil {
		t.Error("expected nil for wall start")
	}
}

// TestHPASameChunkDiagonal verifies HPA* with diagonal movement for a same-chunk path.
func TestHPASameChunkDiagonal(t *testing.T) {
	grid := grid.NewOrthogonalGrid([][]int{
		{0, 0, 0, 0, 0},
		{0, 0, 0, 0, 0},
		{0, 0, 0, 0, 0},
		{0, 0, 0, 0, 0},
		{0, 0, 0, 0, 0},
	})
	world := NewHPABuilder(HPAConfig{ChunkSize: 8}).Build(grid)
	f := NewHPAStarFinder(&finder.FinderOptions{
		AllowDiagonal:    true,
		DontCrossCorners: false,
	})
	result := f.FindPath(0, 0, 4, 4, grid, world)
	if result.Waypoints == nil {
		t.Fatal("expected path, got nil")
	}
	assertStartEnd(t, result.Waypoints, 0, 0, 4, 4)
	verifyWalkable(t, grid, result.Waypoints)
}

// TestHPAMediumGrid verifies HPA* on a medium-sized open grid.
func TestHPAMediumGrid(t *testing.T) {
	grid := grid.NewOrthogonalGrid([][]int{
		{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
		{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
		{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
		{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
	})
	world := NewHPABuilder(HPAConfig{ChunkSize: 8}).Build(grid)
	f := NewHPAStarFinder(nil)
	result := f.FindPath(0, 0, 19, 3, grid, world)
	if result.Waypoints == nil {
		t.Fatal("expected path, got nil")
	}
	assertStartEnd(t, result.Waypoints, 0, 0, 19, 3)
	verifyWalkable(t, grid, result.Waypoints)
	t.Logf("portal path length: %d, waypoint path length: %d", len(result.PortalKeys), len(result.Waypoints))
}

// --- helpers ---

func assertStartEnd(t *testing.T, path [][2]int, sx, sy, ex, ey int) {
	t.Helper()
	if path[0] != [2]int{sx, sy} {
		t.Errorf("expected start (%d,%d), got %v", sx, sy, path[0])
	}
	if path[len(path)-1] != [2]int{ex, ey} {
		t.Errorf("expected end (%d,%d), got %v", ex, ey, path[len(path)-1])
	}
}

func verifyWalkable(t *testing.T, grid finder.Grid, path [][2]int) {
	t.Helper()
	for i, wp := range path {
		if !grid.IsWalkableAt(wp[0], wp[1]) {
			t.Errorf("waypoint %d (%v) is not walkable", i, wp)
		}
	}
}
