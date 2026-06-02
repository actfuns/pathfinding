package hpa

import (
	"github.com/actfuns/pathfinding/finder"
)

// Portal represents a portal on a chunk border or a diagonal corner portal.
type Portal struct {
	CenterX int // world X of portal center
	CenterY int // world Y of portal center
	Length  int // number of cells this portal spans
	Offset  int // offset along the chunk edge (0-based)

	// External connections to neighboring chunks (portal keys)
	ExternalCount   int
	ExternalPortals [5]int

	// Internal connections to other portals in the same chunk
	InternalCount   int
	InternalPortals [255]int // portal key of connected portal
	InternalCosts   [255]int // movement cost (octile * 10)
}

func newPortal(cx, cy int) *Portal {
	return &Portal{CenterX: cx, CenterY: cy}
}

// PortalKey encodes chunk ID, edge position, and direction.
// key = position + direction*chunkSize + chunkID*MaxPortalsPerChunk
func PortalKey(chunkID, pos, dir, chunkSize int) int {
	return pos + dir*chunkSize + chunkID*MaxPortalsPerChunk
}

func portalKeyChunkID(key int) int {
	return key / MaxPortalsPerChunk
}

func portalKeyPos(key, chunkSize int) int {
	return key % (chunkSize * 4) % chunkSize
}

func portalKeyDir(key, chunkSize int) int {
	return key % (chunkSize * 4) / chunkSize
}

// MaxPortalsPerChunk = ChunkSize * 4 (N/E/S/W)
const MaxPortalsPerChunk = 256

// HPAWorld holds the hierarchical portal graph for a Grid.
type HPAWorld struct {
	// Portals is the flat array of all portals indexed by PortalKey.
	// Unused slots remain nil.
	Portals []*Portal
	// NumPortals is the total number of allocated portals.
	NumPortals int
	// ChunkMapX is the number of chunks along the X axis.
	ChunkMapX int
	// ChunkMapY is the number of chunks along the Y axis.
	ChunkMapY int
	// PaddedWidth is the grid width rounded up to a multiple of ChunkSize.
	PaddedWidth int
	// PaddedHeight is the grid height rounded up to a multiple of ChunkSize.
	PaddedHeight int
	// ChunkSize is the edge length of a single chunk in cells.
	ChunkSize int
}

// Cardinal direction constants used for portal orientation.
const (
	DirN = 0 // north (top edge)
	DirE = 1 // east (right edge)
	DirS = 2 // south (bottom edge)
	DirW = 3 // west (left edge)
)

// BuildWorld constructs the HPA world: partition grid into chunks, detect edge
// portals (one per walkable cell on each chunk edge), connect internal portals
// (BFS within chunk), and connect external portals (adjacent chunk).
func BuildWorld(grid finder.Grid, chunkSize int) *HPAWorld {
	cs := chunkSize
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

	neighborBuf := make([]*finder.Node, 0, 8)
	tmpNode := &finder.Node{}

	for cy := 0; cy < cmy; cy++ {
		for cx := 0; cx < cmx; cx++ {
			buildChunkPortals(grid, w, cx, cy, neighborBuf)
		}
	}
	for cy := 0; cy < cmy; cy++ {
		for cx := 0; cx < cmx; cx++ {
			connectInternals(grid, w, cx, cy, neighborBuf, tmpNode)
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

func buildChunkPortals(grid finder.Grid, w *HPAWorld, cx, cy int, nb []*finder.Node) {
	cs := w.ChunkSize
	chunkID := cy*w.ChunkMapX + cx
	ox := cx * cs
	oy := cy * cs

	edgePortals(grid, w, chunkID, ox, oy, cs, DirN, DirS, false, nb)
	edgePortals(grid, w, chunkID, ox, oy+cs-1, cs, DirS, DirN, false, nb)
	edgePortals(grid, w, chunkID, ox, oy, cs, DirW, DirE, true, nb)
	edgePortals(grid, w, chunkID, ox+cs-1, oy, cs, DirE, DirW, true, nb)
}

func edgePortals(grid finder.Grid, w *HPAWorld, chunkID, ox, oy, cs, dir, oppDir int, vertical bool, nb []*finder.Node) {
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

func connectInternals(grid finder.Grid, w *HPAWorld, cx, cy int, nb []*finder.Node, tmpNode *finder.Node) {
	cs := w.ChunkSize
	chunkID := cy*w.ChunkMapX + cx
	ox := cx * cs
	oy := cy * cs

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
		costs, _ := bfsCostsInChunk(grid, pi.CenterX, pi.CenterY, ox, oy, cs, nb, nil, nil, tmpNode)
		for j := i + 1; j < len(keys); j++ {
			pj := w.Portals[keys[j]]
			cost := getBfsCost(costs, pj.CenterX, pj.CenterY, ox, oy, cs)
			if cost < 0 {
				continue
			}
			pi.InternalPortals[pi.InternalCount] = keys[j]
			pi.InternalCosts[pi.InternalCount] = cost
			pi.InternalCount++
			pj.InternalPortals[pj.InternalCount] = keys[i]
			pj.InternalCosts[pj.InternalCount] = cost
			pj.InternalCount++
		}
	}
}

func bfsCostsInChunk(grid finder.Grid, sx, sy, ox, oy, cs int,
	nb []*finder.Node, costsBuf []int, queueBuf [][2]int, tmpNode *finder.Node) ([]int, [][2]int) {
	if !grid.IsWalkableAt(sx, sy) {
		return nil, nil
	}
	size := cs * cs

	costs := costsBuf
	if cap(costs) < size {
		costs = make([]int, size)
	}
	costs = costs[:size]
	for i := range costs {
		costs[i] = -1
	}

	queue := queueBuf[:0]
	if cap(queue) < size {
		queue = make([][2]int, 0, size)
	}

	toKey := func(x, y int) int {
		return (y-oy)*cs + (x - ox)
	}
	inside := func(x, y int) bool {
		return x >= ox && x < ox+cs && y >= oy && y < oy+cs
	}

	n := tmpNode
	if n == nil {
		n = &finder.Node{}
	}

	sKey := toKey(sx, sy)
	costs[sKey] = 0
	queue = append(queue, [2]int{sx, sy})

	for len(queue) > 0 {
		cur := queue[0]
		queue = queue[1:]
		cx, cy := cur[0], cur[1]
		ck := toKey(cx, cy)
		cc := costs[ck]

		n.X = cx
		n.Y = cy
		neighbors := grid.GetNeighbors(n, finder.DiagonalNever, nb)
		for _, neighbor := range neighbors {
			nx, ny := neighbor.X, neighbor.Y
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
	return costs, queue[:cap(queue)]
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
