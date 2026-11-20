package hpa

import (
	"math"

	"github.com/actfuns/pathfinding/finder"
)

// Portal represents a portal on a chunk border or a diagonal corner portal.
type Portal struct {
	CenterX int32 // world X of portal center
	CenterY int32 // world Y of portal center
	Length  int32 // number of cells this portal spans
	Offset  int32 // offset along the chunk edge (0-based)

	// External connections to neighboring chunks (portal keys)
	ExternalCount   int
	ExternalPortals []int32
	ExternalCosts   []int32

	// Internal connections to other portals in the same chunk
	InternalCount   int
	InternalPortals []int32 // portal key of connected portal
	InternalCosts   []int32 // movement cost (octile * 10)
}

func newPortal(cx, cy int32) *Portal {
	return &Portal{CenterX: cx, CenterY: cy}
}

// PortalKey encodes chunk ID, edge position, and direction.
// key = position + direction*chunkSize + chunkID*chunkSize*4
func PortalKey(chunkID, pos, dir, chunkSize int32) int32 {
	return pos + dir*chunkSize + chunkID*chunkSize*4
}

func portalKeyChunkID(key, chunkSize int32) int32 {
	return key / (chunkSize * 4)
}

func portalKeyPos(key, chunkSize int32) int32 {
	return key % (chunkSize * 4) % chunkSize
}

func portalKeyDir(key, chunkSize int32) int32 {
	return key % (chunkSize * 4) / chunkSize
}

// HPAWorld holds the hierarchical portal graph for a Grid.
type HPAWorld struct {
	// Portals is the flat array of all portals indexed by PortalKey.
	// Unused slots remain nil.
	Portals []*Portal
	// NumPortals is the total number of allocated portals.
	NumPortals int
	// ChunkMapX is the number of chunks along the X axis.
	ChunkMapX int32
	// ChunkMapY is the number of chunks along the Y axis.
	ChunkMapY int32
	// PaddedWidth is the grid width rounded up to a multiple of ChunkSize.
	PaddedWidth int32
	// PaddedHeight is the grid height rounded up to a multiple of ChunkSize.
	PaddedHeight int32
	// ChunkSize is the edge length of a single chunk in cells.
	ChunkSize int32
	// Diagonal is the movement rule used for portal connectivity.
	Diagonal finder.DiagonalMovement
}

// Cardinal direction constants used for portal orientation.
const (
	DirN = 0 // north (top edge)
	DirE = 1 // east (right edge)
	DirS = 2 // south (bottom edge)
	DirW = 3 // west (left edge)
)

// BuildWorld constructs the HPA world: partition grid into chunks, detect edge
// portals (compressed — one per run of consecutive walkable cells), connect
// external portals (adjacent chunk), and connect internal portals (BFS within chunk).
func BuildWorld(grid finder.Grid, chunkSize int32) *HPAWorld {
	return BuildWorldWithDiagonal(grid, chunkSize, finder.DiagonalNever)
}

// BuildWorldWithDiagonal builds the portal graph using the given diagonal
// movement rule for cross-chunk and intra-chunk connectivity.
func BuildWorldWithDiagonal(grid finder.Grid, chunkSize int32, diagonal finder.DiagonalMovement) *HPAWorld {
	cs := chunkSize
	if cs < 2 {
		cs = 16
	}
	pw, ph := padSize(grid.Width(), grid.Height(), cs)
	cmx := pw / cs
	cmy := ph / cs

	w := &HPAWorld{
		Portals:      make([]*Portal, int(cmx*cmy*cs*4)),
		ChunkMapX:    cmx,
		ChunkMapY:    cmy,
		PaddedWidth:  pw,
		PaddedHeight: ph,
		ChunkSize:    cs,
		Diagonal:     diagonal,
	}

	neighborBuf := make([]*finder.Node, 0, 8)
	tmpNode := &finder.Node{}

	// Phase 1: create compressed portals per chunk edge
	for cy := int32(0); cy < cmy; cy++ {
		for cx := int32(0); cx < cmx; cx++ {
			buildChunkPortals(grid, w, cx, cy)
		}
	}
	// Phase 1b: resolve external connections (after all portals exist)
	connectExternals(grid, w, neighborBuf)
	// Phase 2: connect portals within the same chunk via BFS
	for cy := int32(0); cy < cmy; cy++ {
		for cx := int32(0); cx < cmx; cx++ {
			connectInternals(grid, w, cx, cy, neighborBuf, tmpNode)
		}
	}
	return w
}

func padSize(w, h, cs int32) (int32, int32) {
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

func buildChunkPortals(grid finder.Grid, w *HPAWorld, cx, cy int32) {
	cs := w.ChunkSize
	chunkID := cy*w.ChunkMapX + cx
	ox := cx * cs
	oy := cy * cs

	edgePortals(grid, w, chunkID, ox, oy, cs, DirN, DirS, false)
	edgePortals(grid, w, chunkID, ox, oy+cs-1, cs, DirS, DirN, false)
	edgePortals(grid, w, chunkID, ox, oy, cs, DirW, DirE, true)
	edgePortals(grid, w, chunkID, ox+cs-1, oy, cs, DirE, DirW, true)
}

// edgePortals creates compressed portals along a chunk edge.
// Consecutive walkable cells are merged into a single portal.
func edgePortals(grid finder.Grid, w *HPAWorld, chunkID, ox, oy, cs, dir, _ int32, vertical bool) {
	pos := int32(0)
	for pos < cs {
		var cx, cy int32
		if vertical {
			cx, cy = ox, oy+pos
		} else {
			cx, cy = ox+pos, oy
		}
		if !grid.IsWalkableAt(cx, cy) {
			pos++
			continue
		}

		runStart := pos
		// Extend run while consecutive cells are walkable
		for pos < cs {
			var nx, ny int32
			if vertical {
				nx, ny = ox, oy+pos
			} else {
				nx, ny = ox+pos, oy
			}
			if !grid.IsWalkableAt(nx, ny) {
				break
			}
			pos++
		}
		runEnd := pos - 1
		runLength := runEnd - runStart + 1
		centerPos := runStart + runLength/2

		var centerX, centerY int32
		if vertical {
			centerX, centerY = ox, oy+centerPos
		} else {
			centerX, centerY = ox+centerPos, oy
		}

		key := PortalKey(chunkID, runStart, dir, cs)
		p := newPortal(centerX, centerY)
		p.Length = runLength
		p.Offset = runStart
		w.Portals[key] = p
		w.NumPortals++
	}
}

// findPortalAtEdgeOffset finds the compressed portal key on the given chunk
// edge whose offset range covers the specified offset. Returns -1 if not found.
func findPortalAtEdgeOffset(w *HPAWorld, chunkID, offset, dir int32) int32 {
	cs := w.ChunkSize
	for pos := int32(0); pos < cs; pos++ {
		key := PortalKey(chunkID, pos, dir, cs)
		p := w.Portals[key]
		if p != nil && offset >= p.Offset && offset < p.Offset+p.Length {
			return key
		}
	}
	return -1
}

// connectExternals resolves all external (cross-chunk) connections for every
// portal. Must be called after all portals have been created (Phase 1).
func connectExternals(grid finder.Grid, w *HPAWorld, nb []*finder.Node) {
	cs := w.ChunkSize
	totalChunks := w.ChunkMapX * w.ChunkMapY
	dirs := []int32{DirN, DirE, DirS, DirW}
	oppDirs := []int32{DirS, DirW, DirN, DirE}

	for chunkID := int32(0); chunkID < totalChunks; chunkID++ {
		chunkCy := chunkID / w.ChunkMapX
		chunkCx := chunkID - chunkCy*w.ChunkMapX
		ox := chunkCx * cs
		oy := chunkCy * cs

		for di := int32(0); di < 4; di++ {
			dir := dirs[int(di)]
			oppDir := oppDirs[int(di)]

			for pos := int32(0); pos < cs; pos++ {
				key := PortalKey(chunkID, pos, dir, cs)
				p := w.Portals[key]
				if p == nil {
					continue
				}

				// Check each cell in this compressed portal's range for cross-chunk neighbors
				for cellPos := p.Offset; cellPos < p.Offset+p.Length; cellPos++ {
					var cx, cy int32
					switch dir {
					case DirN:
						cx, cy = ox+cellPos, oy
					case DirS:
						cx, cy = ox+cellPos, oy+cs-1
					case DirW:
						cx, cy = ox, oy+cellPos
					case DirE:
						cx, cy = ox+cs-1, oy+cellPos
					}

					tmpNode := &finder.Node{X: cx, Y: cy, Walkable: true}
					neighbors := grid.GetNeighbors(tmpNode, w.Diagonal, nb)
					for _, n := range neighbors {
						nx, ny := n.X, n.Y
						if !grid.IsInside(nx, ny) || !grid.IsWalkableAt(nx, ny) {
							continue
						}
						nChunk := (ny/cs)*w.ChunkMapX + nx/cs
						if nChunk == chunkID {
							continue
						}

						// Use the neighbour's offset on the opposite edge.
						// For diagonal crossings this differs from cellPos.
						nCx, nCy := nx/cs, ny/cs
						nOffset := nx - nCx*cs
						if oppDir == DirE || oppDir == DirW {
							nOffset = ny - nCy*cs
						}
						ekey := findPortalAtEdgeOffset(w, nChunk, nOffset, oppDir)
						if ekey < 0 {
							continue
						}

						// Deduplicate
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
						ep := w.Portals[ekey]
						if ep == nil {
							continue
						}
						p.ExternalPortals = append(p.ExternalPortals, ekey)
						p.ExternalCosts = append(p.ExternalCosts, octile10(ep.CenterX-p.CenterX, ep.CenterY-p.CenterY))
						p.ExternalCount++
					}
				}
			}
		}
	}
}

func connectInternals(grid finder.Grid, w *HPAWorld, cx, cy int32, nb []*finder.Node, tmpNode *finder.Node) {
	cs := w.ChunkSize
	chunkID := cy*w.ChunkMapX + cx
	ox := cx * cs
	oy := cy * cs

	var keys []int32
	for dir := int32(0); dir < 4; dir++ {
		for pos := int32(0); pos < cs; pos++ {
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
		costs, _ := bfsCostsInChunk(grid, pi.CenterX, pi.CenterY, ox, oy, cs, w.Diagonal, nb, nil, nil, tmpNode)
		for j := i + 1; j < len(keys); j++ {
			pj := w.Portals[keys[j]]
			cost := getBfsCost(costs, pj.CenterX, pj.CenterY, ox, oy, cs)
			if cost < 0 {
				continue
			}
			pi.InternalPortals = append(pi.InternalPortals, keys[j])
			pi.InternalCosts = append(pi.InternalCosts, cost)
			pi.InternalCount++
			pj.InternalPortals = append(pj.InternalPortals, keys[i])
			pj.InternalCosts = append(pj.InternalCosts, cost)
			pj.InternalCount++
		}
	}
}

func bfsCostsInChunk(grid finder.Grid, sx, sy, ox, oy, cs int32, diagonal finder.DiagonalMovement,
	nb []*finder.Node, costsBuf []int32, queueBuf [][2]int32, tmpNode *finder.Node) ([]int32, [][2]int32) {
	if !grid.IsWalkableAt(sx, sy) {
		return nil, nil
	}
	size := cs * cs

	costs := costsBuf
	if cap(costs) < int(size) {
		costs = make([]int32, int(size))
	}
	costs = costs[:int(size)]
	for i := range costs {
		costs[i] = -1
	}

	queue := queueBuf[:0]
	if cap(queue) < int(size) {
		queue = make([][2]int32, 0, int(size))
	}

	toKey := func(x, y int32) int32 {
		return (y-oy)*cs + (x - ox)
	}
	inside := func(x, y int32) bool {
		return x >= ox && x < ox+cs && y >= oy && y < oy+cs
	}

	n := tmpNode
	if n == nil {
		n = &finder.Node{}
	}

	sKey := toKey(sx, sy)
	costs[sKey] = 0
	queue = append(queue, [2]int32{sx, sy})

	for head := 0; head < len(queue); head++ {
		cur := queue[head]
		cx, cy := cur[0], cur[1]
		ck := toKey(cx, cy)
		cc := costs[ck]

		n.X = cx
		n.Y = cy
		neighbors := grid.GetNeighbors(n, diagonal, nb)
		for _, neighbor := range neighbors {
			nx, ny := neighbor.X, neighbor.Y
			if !inside(nx, ny) || !grid.IsWalkableAt(nx, ny) {
				continue
			}
			nk := toKey(nx, ny)
			nc := cc + stepCost10(nx-cx, ny-cy)
			if costs[nk] < 0 || nc < costs[nk] {
				costs[nk] = nc
				queue = append(queue, [2]int32{nx, ny})
			}
		}
	}
	return costs, queue[:0]
}

func stepCost10(dx, dy int32) int32 {
	if dx != 0 && dy != 0 {
		return 14
	}
	return 10
}

// octile10 returns the octile distance between two points in HPA cost units.
func octile10(dx, dy int32) int32 {
	if dx < 0 {
		dx = -dx
	}
	if dy < 0 {
		dy = -dy
	}
	if dx < dy {
		dx, dy = dy, dx
	}
	return int32(math.Round(float64((float32(dx) + (finder.SQRT2-1)*float32(dy)) * 10)))
}

func getBfsCost(costs []int32, wx, wy, ox, oy, cs int32) int32 {
	dx, dy := wx-ox, wy-oy
	if dx < 0 || dx >= cs || dy < 0 || dy >= cs || costs == nil {
		return -1
	}
	return costs[dy*cs+dx]
}

// ChunkOf returns the chunk coordinates for a world position.
func ChunkOf(x, y, chunkSize int32) (int32, int32) {
	return x / chunkSize, y / chunkSize
}

// ChunkIDOf returns the chunk ID for a world position.
func (w *HPAWorld) ChunkIDOf(x, y int32) int32 {
	cx, cy := ChunkOf(x, y, w.ChunkSize)
	return cy*w.ChunkMapX + cx
}

// IsSingleChunk reports whether two positions are in the same chunk.
func (w *HPAWorld) IsSingleChunk(x1, y1, x2, y2 int32) bool {
	return w.ChunkIDOf(x1, y1) == w.ChunkIDOf(x2, y2)
}
