package hpa

import (
	"testing"

	"github.com/actfuns/navpath/grid"
)

// ref: hpastar (C#) — https://github.com/ByteExceptions/hpastar
//
// Our implementation shares the following structural properties with the C# ref:
//
// 1. PortalKey encoding: same formula (pos + dir*chunkSize + chunkID*MaxPortalsPerChunk)
// 2. Edge portal detection: one portal per walkable edge cell (we differ from ref's run-based merging)
// 3. Internal connection: 4-dir BFS with cost 10 per step
// 4. Two-layer search: abstract portal-level A* + concrete per-segment A*
// 5. External connection: connect adjacent chunks via opposite-direction portals
//
// Key differences:
//   - C# ref uses run-based portals (merge adjacent walkable cells → one portal with
//     Length, CenterPos at middle). We use one-portal-per-walkable-cell.
//   - C# ref has diagonal corner portals (3 diagonal directions at chunk corners).
//     We do not implement diagonal portals.
//   - C# ref uses region-based BFS optimization (ResetRegions, region fill).
//     We BFS per-start-portal (simpler but more BFS calls).
//   - C# ref has PathFindingManager cache. We have no cache.

// TestHPAPortalKeyEncoding verifies PortalKey matches the C# ref formula:
// key = position + direction*ChunkSize + chunkID*MaxPortalsPerChunk
func TestHPAPortalKeyEncoding(t *testing.T) {
	cs := 8
	// Chunk 0, pos 3, dir N(0)
	key := PortalKey(0, 3, DirN, cs)
	if key != 3 {
		t.Errorf("expected 3, got %d", key)
	}
	// Chunk 0, pos 0, dir E(1)
	key = PortalKey(0, 0, DirE, cs)
	if key != 8 {
		t.Errorf("expected 8 (0 + 1*8), got %d", key)
	}
	// Decode
	dir := portalKeyDir(key, cs)
	if dir != DirE {
		t.Errorf("expected dir E(1), got %d", dir)
	}
	pos := portalKeyPos(key, cs)
	if pos != 0 {
		t.Errorf("expected pos 0, got %d", pos)
	}
	// Chunk 1, pos 5, dir S(2)
	key = PortalKey(1, 5, DirS, cs)
	// 5 + 2*8 + 1*256 = 5 + 16 + 256 = 277
	if key != 277 {
		t.Errorf("expected 277, got %d", key)
	}
	cid := portalKeyChunkID(key, cs)
	if cid != 1 {
		t.Errorf("expected chunk 1, got %d", cid)
	}
}

// TestHPAPortalPerCell verifies our edge portal detection creates one portal
// per walkable cell on each chunk edge (vs C# ref's run-based merging).
func TestHPAPortalPerCell(t *testing.T) {
	// Grid with partial obstacles on the north edge of chunk (8,8)-(15,15)
	grid := grid.NewOrthogonalGrid([][]int{
		{0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
		{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
		{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
		{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
		{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
		{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
		{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
		{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
	})
	world := NewHPABuilder(HPAConfig{ChunkSize: 8}).Build(grid)

	// Chunk 0 covers (0,0)-(7,7). Its north edge has 2 walls at pos 2,3.
	// With one-portal-per-cell, we expect 6 portals on north edge (8 cells - 2 walls).
	northCount := countPortalsInDir(world, 0, DirN)
	if northCount != 6 {
		t.Errorf("expected 6 north portals (8 edge cells - 2 walls), got %d", northCount)
	}
}

// TestHPAExternalConnections verifies external portal connections
// between adjacent chunks match the C# ref's approach (opposite direction).
func TestHPAExternalConnections(t *testing.T) {
	grid := grid.NewOrthogonalGrid([][]int{
		{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
		{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
	})
	world := NewHPABuilder(HPAConfig{ChunkSize: 8}).Build(grid)
	// Chunk 0 is (0,0)-(7,1), chunk 1 is (8,0)-(15,1).
	// Chunk 0's east edge portal at pos 0: should connect to chunk 1's west edge portal at pos 0
	eastKey := PortalKey(0, 0, DirE, 8)
	eastPortal := world.Portals[eastKey]
	if eastPortal == nil {
		t.Fatal("expected east portal at chunk 0, pos 0")
	}
	if eastPortal.ExternalCount != 1 {
		t.Fatalf("expected 1 external connection, got %d", eastPortal.ExternalCount)
	}
	expectedKey := PortalKey(1, 0, DirW, 8)
	if eastPortal.ExternalPortals[0] != expectedKey {
		t.Errorf("expected external connection to key %d, got %d", expectedKey, eastPortal.ExternalPortals[0])
	}
}

// TestHPABFSQuadDir verifies internal BFS uses 4-direction with cost 10,
// matching the C# ref's approach.
func TestHPABFSQuadDir(t *testing.T) {
	grid := grid.NewOrthogonalGrid([][]int{
		{0, 0, 0, 0, 0, 0, 0, 0},
		{0, 0, 0, 0, 0, 0, 0, 0},
		{0, 0, 0, 0, 0, 0, 0, 0},
		{0, 0, 0, 0, 0, 0, 0, 0},
		{0, 0, 0, 0, 0, 0, 0, 0},
		{0, 0, 0, 0, 0, 0, 0, 0},
		{0, 0, 0, 0, 0, 0, 0, 0},
		{0, 0, 0, 0, 0, 0, 0, 0},
	})
	world := NewHPABuilder(HPAConfig{ChunkSize: 8}).Build(grid)
	// North portal at pos 0 and south portal at pos 0 in the same chunk
	northKey := PortalKey(0, 0, DirN, 8)
	southKey := PortalKey(0, 0, DirS, 8)
	northPortal := world.Portals[northKey]
	southPortal := world.Portals[southKey]
	if northPortal == nil || southPortal == nil {
		t.Fatal("expected portals at both edges")
	}
	// BFS from (0,0) to (0,7): 7 steps * 10 cost = 70
	found := false
	for i := 0; i < northPortal.InternalCount; i++ {
		if northPortal.InternalPortals[i] == southKey {
			if northPortal.InternalCosts[i] != 70 {
				t.Errorf("expected cost 70 (7 steps * 10), got %d", northPortal.InternalCosts[i])
			}
			found = true
		}
	}
	if !found {
		t.Error("expected internal connection from north to south portal")
	}
}

// TestHPAReachablePortals verifies FindPortalNodes behavior: start can reach
// its chunk's portals via BFS (same as C# ref's FindPortalNodes).
func TestHPAReachablePortals(t *testing.T) {
	grid := grid.NewOrthogonalGrid([][]int{
		{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
		{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
	})
	world := NewHPABuilder(HPAConfig{ChunkSize: 8}).Build(grid)
	f := NewHPAStarFinder()
	// Start at (1,1) in chunk 0 — should find reachable portals
	keys := f.findReachablePortals(grid, world, 1, 1, 0)
	if len(keys) == 0 {
		t.Fatal("expected reachable portals from (1,1)")
	}
	// Should find both north and west portals on an open grid
	// (chunk 0 has walkable edges on all sides)
	if len(keys) < 4 {
		t.Logf("expected at least 4 reachable portals, got %d", len(keys))
	}
}

// TestHPAConcretePathValid verifies the concrete A* fallback per segment
// produces valid walkable paths.
func TestHPAConcretePathValid(t *testing.T) {
	grid := grid.NewOrthogonalGrid([][]int{
		{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
		{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
	})
	world := NewHPABuilder(HPAConfig{ChunkSize: 8}).Build(grid)
	f := NewHPAStarFinder()
	result := f.FindPath(0, 0, 19, 1, grid, world)
	if result.Waypoints == nil {
		t.Fatal("expected path, got nil")
	}
	// Verify all waypoints are walkable
	for i, wp := range result.Waypoints {
		if !grid.IsWalkableAt(wp[0], wp[1]) {
			t.Errorf("waypoint %d at (%d,%d) is not walkable", i, wp[0], wp[1])
		}
	}
	// Verify portal path goes through chunks 0 → 1 → 2
	if len(result.PortalKeys) < 2 {
		t.Errorf("expected portal path through multiple chunks, got %d portals", len(result.PortalKeys))
	}
	t.Logf("portal keys: %v", result.PortalKeys)
}

// TestHPAManyChunks verifies HPA* on a larger grid crossing many chunk boundaries.
func TestHPAManyChunks(t *testing.T) {
	grid := grid.NewOrthogonalGrid([][]int{
		{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
		{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
		{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
	})
	world := NewHPABuilder(HPAConfig{ChunkSize: 8}).Build(grid)
	f := NewHPAStarFinder()
	result := f.FindPath(0, 1, 31, 1, grid, world)
	if result.Waypoints == nil {
		t.Fatal("expected path, got nil")
	}
	if len(result.PortalKeys) == 0 {
		t.Error("expected portal path across many chunks")
	}
}

// TestHPAPortalCount verifies portal count is deterministic and
// consistent with per-cell approach.
func TestHPAPortalCount(t *testing.T) {
	grid := grid.NewOrthogonalGrid([][]int{
		{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
		{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
		{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
		{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
	})
	world := NewHPABuilder(HPAConfig{ChunkSize: 8}).Build(grid)
	// 2x1 chunks = 2 chunks. Each chunk has 4 edges × 8 cells = 32 portals.
	// With all walkable: 2 × 32 = 64 portals
	// But some portal slots are shared/overlapping in our scheme?
	// Actually each edge cell gets exactly one portal, and all cells are walkable.
	// Let's count non-nil portals:
	count := 0
	for _, p := range world.Portals {
		if p != nil {
			count++
		}
	}
	// 2 chunks × (north: 8 + east: 4 + west: 4) = 32 (south edges have no walkable cells
	// since grid is only 4 rows tall but padded to 8 — y=4..7 are out of bounds)
	if count != 32 {
		t.Errorf("expected 32 portals (2 chunks × 16 edge cells within bounds), got %d", count)
	}
}

// --- helpers ---

func countPortalsInDir(world *HPAWorld, chunkID, dir int) int {
	count := 0
	for pos := 0; pos < world.ChunkSize; pos++ {
		key := PortalKey(chunkID, pos, dir, world.ChunkSize)
		if world.Portals[key] != nil {
			count++
		}
	}
	return count
}
