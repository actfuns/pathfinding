package hpa

import (
	"github.com/actfuns/pathfinding/finder"
)

// HPABuilder builds the hierarchical portal graph from a Grid.
type HPABuilder struct {
	cfg HPAConfig
}

// NewHPABuilder creates an HPABuilder.
func NewHPABuilder(cfg HPAConfig) *HPABuilder {
	return &HPABuilder{cfg: cfg}
}

// Build constructs the HPA world: partition grid into chunks, detect edge
// portals (one per walkable cell on each chunk edge), connect internal
// portals (BFS within chunk), and connect external portals (adjacent chunk).
func (b *HPABuilder) Build(grid finder.Grid) *HPAWorld {
	cs := b.cfg.ChunkSize
	if cs < 2 {
		cs = 16
	}
	pw, ph := padSize(grid.Width(), grid.Height(), cs)
	cmx := pw / cs
	cmy := ph / cs

	w := &HPAWorld{
		Portals:      make([]*Portal, cmx*cmy*MaxPortalsPerChunk),
		ChunkMapX:    cmx,
		ChunkMapY:    cmy,
		PaddedWidth:  pw,
		PaddedHeight: ph,
		ChunkSize:    cs,
	}

	// Pre-allocate neighbor buffer for all GetNeighbors calls during build.
	neighborBuf := make([]*finder.Node, 0, 8)

	for cy := 0; cy < cmy; cy++ {
		for cx := 0; cx < cmx; cx++ {
			b.buildChunkPortals(grid, w, cx, cy, neighborBuf)
		}
	}
	for cy := 0; cy < cmy; cy++ {
		for cx := 0; cx < cmx; cx++ {
			b.connectInternals(grid, w, cx, cy, neighborBuf)
		}
	}
	return w
}

func padSize(w, h, cs int) (int, int) {
	rw := w
	if rem := w % cs; rem != 0 {
		rw = w + cs - rem
	}
	rh := h
	if rem := h % cs; rem != 0 {
		rh = h + cs - rem
	}
	return rw, rh
}

const (
	DirN = 0
	DirE = 1
	DirS = 2
	DirW = 3
)

// buildChunkPortals creates one portal per walkable edge cell.
func (b *HPABuilder) buildChunkPortals(grid finder.Grid, w *HPAWorld, cx, cy int, nb []*finder.Node) {
	cs := w.ChunkSize
	chunkID := cy*w.ChunkMapX + cx
	ox := cx * cs
	oy := cy * cs

	// North edge: dir=0, cells (ox..ox+cs-1, oy), opposite dir=S=2
	b.edgePortals(grid, w, chunkID, ox, oy, cs, DirN, DirS, false, nb)
	// South edge: dir=2, cells (ox..ox+cs-1, oy+cs-1), opposite dir=N=0
	b.edgePortals(grid, w, chunkID, ox, oy+cs-1, cs, DirS, DirN, false, nb)
	// West edge: dir=3, cells (ox, oy..oy+cs-1), opposite dir=E=1
	b.edgePortals(grid, w, chunkID, ox, oy, cs, DirW, DirE, true, nb)
	// East edge: dir=1, cells (ox+cs-1, oy..oy+cs-1), opposite dir=W=3
	b.edgePortals(grid, w, chunkID, ox+cs-1, oy, cs, DirE, DirW, true, nb)
}

// edgePortals creates portals for each walkable cell on a chunk edge,
// then finds external (cross-chunk) neighbors using grid.GetNeighbors.
func (b *HPABuilder) edgePortals(grid finder.Grid, w *HPAWorld, chunkID, ox, oy, cs, dir, oppDir int, vertical bool, nb []*finder.Node) {
	for pos := 0; pos < cs; pos++ {
		var cx, cy int
		if vertical {
			cx, cy = ox, oy+pos
		} else {
			cx, cy = ox+pos, oy
		}
		if !grid.IsWalkableAt(cx, cy) {
			continue
		}

		key := PortalKey(chunkID, pos, dir, cs)
		p := newPortal(cx, cy)
		p.Length = 1
		p.Offset = pos
		w.Portals[key] = p
		w.NumPortals++

		// External connection: find neighbors via grid.GetNeighbors that
		// lie in a different chunk. This works for all grid topologies
		// (orthogonal, hex, staggered).
		tmpNode := &finder.Node{X: cx, Y: cy, Walkable: true}
		neighbors := grid.GetNeighbors(tmpNode, finder.DiagonalNever, nb)
		for _, n := range neighbors {
			nx, ny := n.X, n.Y
			if nx < 0 || nx >= grid.Width() || ny < 0 || ny >= grid.Height() {
				continue
			}
			nChunk := (ny/cs)*w.ChunkMapX + nx/cs
			if nChunk == chunkID {
				continue
			}
			if !grid.IsWalkableAt(nx, ny) {
				continue
			}
			ekey := PortalKey(nChunk, pos, oppDir, cs)
			// Avoid duplicates (possible at chunk corners where a cell
			// might have >1 external neighbor to the same chunk)
			dup := false
			for ei := 0; ei < p.ExternalCount; ei++ {
				if p.ExternalPortals[ei] == ekey {
					dup = true
					break
				}
			}
			if dup {
				continue
			}
			if p.ExternalCount < len(p.ExternalPortals) {
				p.ExternalPortals[p.ExternalCount] = ekey
				p.ExternalCount++
			}
		}
	}
}

// connectInternals runs BFS between all pairs of portals in the same chunk.
func (b *HPABuilder) connectInternals(grid finder.Grid, w *HPAWorld, cx, cy int, nb []*finder.Node) {
	cs := w.ChunkSize
	chunkID := cy*w.ChunkMapX + cx
	ox := cx * cs
	oy := cy * cs

	// Collect non-nil portal keys for this chunk
	var keys []int
	for dir := 0; dir < 4; dir++ {
		for pos := 0; pos < cs; pos++ {
			key := PortalKey(chunkID, pos, dir, cs)
			if w.Portals[key] != nil {
				keys = append(keys, key)
			}
		}
	}
	if len(keys) < 2 {
		return
	}

	for i := 0; i < len(keys)-1; i++ {
		pi := w.Portals[keys[i]]
		costs := bfsCostsInChunk(grid, pi.CenterX, pi.CenterY, ox, oy, cs, nb)
		for j := i + 1; j < len(keys); j++ {
			pj := w.Portals[keys[j]]
			cost := getBfsCost(costs, pj.CenterX, pj.CenterY, ox, oy, cs)
			if cost < 0 {
				continue
			}
			// Bidirectional
			pi.InternalPortals[pi.InternalCount] = keys[j]
			pi.InternalCosts[pi.InternalCount] = cost
			pi.InternalCount++
			pj.InternalPortals[pj.InternalCount] = keys[i]
			pj.InternalCosts[pj.InternalCount] = cost
			pj.InternalCount++
		}
	}
}

// bfsCostsInChunk runs BFS within a chunk bounds using the grid's actual
// neighbor topology (works for orthogonal, hex, and staggered grids).
func bfsCostsInChunk(grid finder.Grid, sx, sy, ox, oy, cs int, nb []*finder.Node) []int {
	if !grid.IsWalkableAt(sx, sy) {
		return nil
	}
	size := cs * cs
	costs := make([]int, size)
	for i := range costs {
		costs[i] = -1
	}
	toKey := func(x, y int) int {
		return (y-oy)*cs + (x - ox)
	}
	inside := func(x, y int) bool {
		return x >= ox && x < ox+cs && y >= oy && y < oy+cs
	}

	queue := make([][2]int, 0, size)
	sKey := toKey(sx, sy)
	costs[sKey] = 0
	queue = append(queue, [2]int{sx, sy})

	// Use a reusable tmp node for GetNeighbors calls
	tmpNode := &finder.Node{Walkable: true}

	for len(queue) > 0 {
		cur := queue[0]
		queue = queue[1:]
		cx, cy := cur[0], cur[1]
		ck := toKey(cx, cy)
		cc := costs[ck]

		tmpNode.X = cx
		tmpNode.Y = cy
		neighbors := grid.GetNeighbors(tmpNode, finder.DiagonalNever, nb)
		for _, n := range neighbors {
			nx, ny := n.X, n.Y
			if !inside(nx, ny) || !grid.IsWalkableAt(nx, ny) {
				continue
			}
			nk := toKey(nx, ny)
			nc := cc + 10
			if costs[nk] < 0 || nc < costs[nk] {
				costs[nk] = nc
				queue = append(queue, [2]int{nx, ny})
			}
		}
	}
	return costs
}

func getBfsCost(costs []int, wx, wy, ox, oy, cs int) int {
	dx, dy := wx-ox, wy-oy
	if dx < 0 || dx >= cs || dy < 0 || dy >= cs || costs == nil {
		return -1
	}
	return costs[dy*cs+dx]
}

// ChunkOf returns the chunk coordinates for a world position.
func ChunkOf(x, y, chunkSize int) (int, int) {
	return x / chunkSize, y / chunkSize
}

// ChunkIDOf returns the chunk ID for a world position.
func (w *HPAWorld) ChunkIDOf(x, y int) int {
	cx, cy := ChunkOf(x, y, w.ChunkSize)
	return cy*w.ChunkMapX + cx
}

// IsSingleChunk reports whether two positions are in the same chunk.
func (w *HPAWorld) IsSingleChunk(x1, y1, x2, y2 int) bool {
	return w.ChunkIDOf(x1, y1) == w.ChunkIDOf(x2, y2)
}
