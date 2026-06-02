package hpa

import (
	"testing"

	"github.com/actfuns/pathfinding/finder"
	"github.com/actfuns/pathfinding/grid"
)

func TestDefaultHPAConfig(t *testing.T) {
	cfg := DefaultHPAConfig()
	if cfg.ChunkSize != 16 {
		t.Errorf("expected ChunkSize=16, got %d", cfg.ChunkSize)
	}
}

func TestChunkOf(t *testing.T) {
	tests := []struct {
		x, y, cs int
		cx, cy   int
	}{
		{0, 0, 8, 0, 0},
		{7, 7, 8, 0, 0},
		{8, 0, 8, 1, 0},
		{0, 8, 8, 0, 1},
		{15, 15, 8, 1, 1},
		{16, 16, 8, 2, 2},
	}
	for _, tt := range tests {
		cx, cy := ChunkOf(tt.x, tt.y, tt.cs)
		if cx != tt.cx || cy != tt.cy {
			t.Errorf("ChunkOf(%d,%d,%d) = (%d,%d), want (%d,%d)",
				tt.x, tt.y, tt.cs, cx, cy, tt.cx, tt.cy)
		}
	}
}

func TestHPAWorldChunkIDOf(t *testing.T) {
	grid := grid.NewOrthogonalGrid([][]int{
		{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
		{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
	})
	world := NewHPABuilder(HPAConfig{ChunkSize: 8}).Build(grid)

	// 16x2 grid, padded to 16x8, chunks: (16/8)*(8/8)=2x1=2 chunks
	if world.ChunkMapX != 2 || world.ChunkMapY != 1 {
		t.Errorf("expected 2x1 chunks, got %dx%d", world.ChunkMapX, world.ChunkMapY)
	}

	tests := []struct {
		x, y int
		id   int
	}{
		{0, 0, 0},
		{7, 0, 0},
		{8, 0, 1},
		{15, 0, 1},
	}
	for _, tt := range tests {
		id := world.ChunkIDOf(tt.x, tt.y)
		if id != tt.id {
			t.Errorf("ChunkIDOf(%d,%d) = %d, want %d", tt.x, tt.y, id, tt.id)
		}
	}
}

func TestHPAWorldIsSingleChunk(t *testing.T) {
	grid := grid.NewOrthogonalGrid([][]int{
		{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
		{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
	})
	world := NewHPABuilder(HPAConfig{ChunkSize: 8}).Build(grid)

	if !world.IsSingleChunk(0, 0, 7, 0) {
		t.Error("expected same chunk for (0,0) and (7,0)")
	}
	if world.IsSingleChunk(0, 0, 8, 0) {
		t.Error("expected different chunks for (0,0) and (8,0)")
	}
}

func TestNewHPAStarFinderWithOptions(t *testing.T) {
	// Diagonal movement
	f := NewHPAStarFinder(finder.WithAllowDiagonal(false))
	if f.DiagonalMovement != finder.DiagonalIfAtMostOneObstacle {
		t.Errorf("expected DiagonalIfAtMostOneObstacle, got %v", f.DiagonalMovement)
	}

	// Diagonal + no cross corners
	f2 := NewHPAStarFinder(finder.WithAllowDiagonal(true))
	if f2.DiagonalMovement != finder.DiagonalOnlyWhenNoObstacles {
		t.Errorf("expected DiagonalOnlyWhenNoObstacles, got %v", f2.DiagonalMovement)
	}

	// Explicit DiagonalMovement + custom heuristic
	customHeuristic := func(dx, dy float64) float64 { return dx + dy + 1 }
	f3 := NewHPAStarFinder(
		finder.WithDiagonal(finder.DiagonalAlways),
		finder.WithHeuristic(customHeuristic),
		finder.WithWeight(1.5),
	)
	if f3.DiagonalMovement != finder.DiagonalAlways {
		t.Errorf("expected DiagonalAlways, got %v", f3.DiagonalMovement)
	}
	if f3.Weight != 1.5 {
		t.Errorf("expected Weight 1.5, got %v", f3.Weight)
	}
}

func TestHPAWorldChunkMapSize(t *testing.T) {
	grid := grid.NewOrthogonalGrid([][]int{
		{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
		{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
	})
	// 24x2 grid, chunk size 10 → padded 30x10 → 3x1 chunks
	world := NewHPABuilder(HPAConfig{ChunkSize: 10}).Build(grid)
	if world.ChunkMapX != 3 || world.ChunkMapY != 1 {
		t.Errorf("expected 3x1 chunks for padded 30x10 with cs=10, got %dx%d",
			world.ChunkMapX, world.ChunkMapY)
	}
	if world.PaddedWidth != 30 || world.PaddedHeight != 10 {
		t.Errorf("expected padded 30x10, got %dx%d", world.PaddedWidth, world.PaddedHeight)
	}
}
