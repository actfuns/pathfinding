package hpa

import (
	"testing"

	"github.com/actfuns/pathfinding/grid"
)

func TestChunkOf(t *testing.T) {
	tests := []struct {
		x, y, cs int32
		cx, cy   int32
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
	g := grid.NewOrthogonalGrid([][]int32{
		{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
		{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
	})
	world := BuildWorld(g, 8)
	if world.ChunkMapX != 2 || world.ChunkMapY != 1 {
		t.Errorf("expected 2x1 chunks, got %dx%d", world.ChunkMapX, world.ChunkMapY)
	}
	tests := []struct {
		x, y int32
		id   int32
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
	g := grid.NewOrthogonalGrid([][]int32{
		{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
		{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
	})
	world := BuildWorld(g, 8)
	if !world.IsSingleChunk(0, 0, 7, 0) {
		t.Error("expected same chunk for (0,0) and (7,0)")
	}
	if world.IsSingleChunk(0, 0, 8, 0) {
		t.Error("expected different chunks for (0,0) and (8,0)")
	}
}

func TestHPAWorldChunkMapSize(t *testing.T) {
	g := grid.NewOrthogonalGrid([][]int32{
		{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
		{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
	})
	world := BuildWorld(g, 10)
	if world.ChunkMapX != 3 || world.ChunkMapY != 1 {
		t.Errorf("expected 3x1 chunks, got %dx%d", world.ChunkMapX, world.ChunkMapY)
	}
	if world.PaddedWidth != 30 || world.PaddedHeight != 10 {
		t.Errorf("expected padded 30x10, got %dx%d", world.PaddedWidth, world.PaddedHeight)
	}
}

func TestHPAPortalKeyEncoding(t *testing.T) {
	cs := int32(8)
	key := PortalKey(0, 3, DirN, cs)
	if key != 3 {
		t.Errorf("expected 3, got %d", key)
	}
	key = PortalKey(0, 0, DirE, cs)
	if key != 8 {
		t.Errorf("expected 8, got %d", key)
	}
	dir := portalKeyDir(key, cs)
	if dir != DirE {
		t.Errorf("expected dir E(1), got %d", dir)
	}
	pos := portalKeyPos(key, cs)
	if pos != 0 {
		t.Errorf("expected pos 0, got %d", pos)
	}
	key = PortalKey(1, 5, DirS, cs)
	if key != 53 {
		t.Errorf("expected 53, got %d", key)
	}
	cid := portalKeyChunkID(key, cs)
	if cid != 1 {
		t.Errorf("expected chunk 1, got %d", cid)
	}
}

func TestHPAPortalPerCell(t *testing.T) {
	g := grid.NewOrthogonalGrid([][]int32{
		{0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
		{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
		{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
		{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
		{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
		{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
		{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
		{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
	})
	world := BuildWorld(g, 8)
	northCount := countPortalsInDir(world, 0, DirN)
	if northCount != 2 {
		t.Errorf("expected 2 compressed north portals, got %d", northCount)
	}
}

func TestHPAExternalConnections(t *testing.T) {
	g := grid.NewOrthogonalGrid([][]int32{
		{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
		{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
	})
	world := BuildWorld(g, 8)
	eastKey := PortalKey(0, 0, DirE, 8)
	eastPortal := world.Portals[eastKey]
	if eastPortal == nil {
		t.Fatal("expected east portal at chunk 0, pos 0")
	}
	if eastPortal.ExternalCount != 1 {
		t.Fatalf("expected 1 external connection, got %d", eastPortal.ExternalCount)
	}
	if eastPortal.ExternalPortals[0] != PortalKey(1, 0, DirW, 8) {
		t.Errorf("expected external connection to key %d, got %d", PortalKey(1, 0, DirW, 8), eastPortal.ExternalPortals[0])
	}
}

func TestHPABFSQuadDir(t *testing.T) {
	g := grid.NewOrthogonalGrid([][]int32{
		{0, 0, 0, 0, 0, 0, 0, 0},
		{0, 0, 0, 0, 0, 0, 0, 0},
		{0, 0, 0, 0, 0, 0, 0, 0},
		{0, 0, 0, 0, 0, 0, 0, 0},
		{0, 0, 0, 0, 0, 0, 0, 0},
		{0, 0, 0, 0, 0, 0, 0, 0},
		{0, 0, 0, 0, 0, 0, 0, 0},
		{0, 0, 0, 0, 0, 0, 0, 0},
	})
	world := BuildWorld(g, 8)
	northKey := PortalKey(0, 0, DirN, 8)
	southKey := PortalKey(0, 0, DirS, 8)
	northPortal := world.Portals[northKey]
	southPortal := world.Portals[southKey]
	if northPortal == nil || southPortal == nil {
		t.Fatal("expected portals at both edges")
	}
	found := false
	for i := 0; i < northPortal.InternalCount; i++ {
		if northPortal.InternalPortals[i] == southKey {
			if northPortal.InternalCosts[i] != 70 {
				t.Errorf("expected cost 70, got %d", northPortal.InternalCosts[i])
			}
			found = true
		}
	}
	if !found {
		t.Error("expected internal connection from north to south portal")
	}
}

func TestHPAPortalCount(t *testing.T) {
	g := grid.NewOrthogonalGrid([][]int32{
		{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
		{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
		{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
		{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
	})
	world := BuildWorld(g, 8)
	count := 0
	for _, p := range world.Portals {
		if p != nil {
			count++
		}
	}
	if count != 6 {
		t.Errorf("expected 6 compressed portals, got %d", count)
	}
}

// --- helpers ---

func countPortalsInDir(world *HPAWorld, chunkID, dir int32) int {
	count := 0
	for pos := int32(0); pos < world.ChunkSize; pos++ {
		key := PortalKey(chunkID, pos, dir, world.ChunkSize)
		if world.Portals[key] != nil {
			count++
		}
	}
	return count
}

// A chunk size above 64 used to overflow the fixed 256-key-per-chunk stride,
// overlapping chunks and indexing past Portals. The stride must scale with the
// configured chunk size.
func TestHPAPortalKeyLargeChunk(t *testing.T) {
	const cs = 100
	g := grid.NewOrthogonalGrid(grid.NewMatrix(200, 200))
	w := BuildWorld(g, cs)

	want := w.ChunkMapX * w.ChunkMapY * cs * 4
	if int32(len(w.Portals)) != want {
		t.Fatalf("Portals length = %d, want %d", len(w.Portals), want)
	}

	chunks := w.ChunkMapX * w.ChunkMapY
	for cid := int32(0); cid < chunks; cid++ {
		for pos := int32(0); pos < cs; pos++ {
			for dir := int32(0); dir < 4; dir++ {
				key := PortalKey(cid, pos, dir, cs)
				if key < 0 || key >= int32(len(w.Portals)) {
					t.Fatalf("key %d out of range [0,%d) for cid=%d pos=%d dir=%d",
						key, len(w.Portals), cid, pos, dir)
				}
				if got := portalKeyChunkID(key, cs); got != cid {
					t.Fatalf("chunkID = %d, want %d", got, cid)
				}
				if got := portalKeyPos(key, cs); got != pos {
					t.Fatalf("pos = %d, want %d", got, pos)
				}
				if got := portalKeyDir(key, cs); got != dir {
					t.Fatalf("dir = %d, want %d", got, dir)
				}
			}
		}
	}
}
