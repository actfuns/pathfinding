package hpa

import (
	"github.com/actfuns/pathfinding/finder"
)

// HPAStarFinder implements Hierarchical Pathfinding A*.
//
// It uses a two-layer approach:
//  1. Abstract graph (portal-level) A* — finds the sequence of portals
//     connecting the start and end chunks.
//  2. Concrete pathfinding — A* within each cluster between consecutive
//     portal centers (plus start→first portal, last portal→end).
type HPAStarFinder struct {
	Heuristic        finder.HeuristicFunc
	Weight           float64
	DiagonalMovement finder.DiagonalMovement
}

// NewHPAStarFinder creates an HPA* finder.
func NewHPAStarFinder(opts ...finder.Option) *HPAStarFinder {
	f := &HPAStarFinder{
		Heuristic:        finder.Manhattan,
		Weight:           1,
		DiagonalMovement: finder.DiagonalNever,
	}
	opt := finder.ApplyOptions(opts)
	if opt.DiagonalMovement != 0 {
		f.DiagonalMovement = opt.DiagonalMovement
	} else if opt.AllowDiagonal {
		if opt.DontCrossCorners {
			f.DiagonalMovement = finder.DiagonalOnlyWhenNoObstacles
		} else {
			f.DiagonalMovement = finder.DiagonalIfAtMostOneObstacle
		}
	}
	if opt.Heuristic != nil {
		f.Heuristic = opt.Heuristic
	}
	if opt.Weight > 0 {
		f.Weight = opt.Weight
	}
	if f.DiagonalMovement != finder.DiagonalNever {
		f.Heuristic = finder.Octile
	}
	return f
}

// hpaNode is a node in the abstract portal graph.
type hpaNode struct {
	portalKey int
	g, h, f   float64
	parent    *hpaNode
	closed    bool
	heapIdx   int
}

// HPAFindResult holds the result of an HPA* query.
type HPAFindResult struct {
	PortalKeys []int    // sequence of portal keys (can be empty)
	Waypoints  [][2]int // full concrete path (can be nil if no path)
}

// FindPath performs HPA* on the given grid, using the pre-built HPAWorld.
// Returns both the abstract portal sequence and the concrete path.
func (f *HPAStarFinder) FindPath(startX, startY, endX, endY int, grid finder.Grid, world *HPAWorld) *HPAFindResult {
	startChunk := world.ChunkIDOf(startX, startY)
	endChunk := world.ChunkIDOf(endX, endY)

	// Same chunk: fall back to flat A* directly
	if startChunk == endChunk {
		path := f.concreteFind(grid, startX, startY, endX, endY)
		return &HPAFindResult{Waypoints: path}
	}

	// Check start/end aren't walls
	if !grid.IsWalkableAt(startX, startY) || !grid.IsWalkableAt(endX, endY) {
		return &HPAFindResult{}
	}

	// Find start portals: portals in start's chunk reachable from start
	startPortalKeys := f.findReachablePortals(grid, world, startX, startY, startChunk)
	if len(startPortalKeys) == 0 {
		return &HPAFindResult{}
	}

	// Find end portals: portals in end's chunk that can reach end
	endPortalKeys := f.findReachablePortals(grid, world, endX, endY, endChunk)
	if len(endPortalKeys) == 0 {
		return &HPAFindResult{}
	}

	// Build a set of goal portal keys for fast lookup
	endSet := make(map[int]bool, len(endPortalKeys))
	for _, k := range endPortalKeys {
		endSet[k] = true
	}

	// Pre-compute goal heuristic reference
	goalPortal := &Portal{CenterX: endX, CenterY: endY}

	// A* on abstract portal graph
	open := make([]*hpaNode, 0, 64)
	allNodes := make(map[int]*hpaNode)

	// Enqueue start portals
	for _, pk := range startPortalKeys {
		p := world.Portals[pk]
		if p == nil {
			continue
		}
		n := &hpaNode{
			portalKey: pk,
			g:         0,
			h:         f.heuristicCost(p, goalPortal),
			heapIdx:   -1,
		}
		n.f = n.g + n.h*f.Weight
		allNodes[pk] = n
		open = f.push(open, n)
	}

	var found *hpaNode
	for len(open) > 0 {
		var cur *hpaNode
		open, cur = f.pop(open)
		if cur.closed {
			continue
		}
		cur.closed = true

		if endSet[cur.portalKey] {
			found = cur
			break
		}

		cp := world.Portals[cur.portalKey]
		if cp == nil {
			continue
		}

		// Expand external connections
		for i := 0; i < cp.ExternalCount; i++ {
			ek := cp.ExternalPortals[i]
			ep := world.Portals[ek]
			if ep == nil {
				continue
			}
			edgeG := cur.g + 10 // cost of stepping to neighbor chunk
			open = f.relaxEdge(allNodes, open, ek, edgeG, ep, goalPortal, cur)
		}

		// Expand internal connections
		for i := 0; i < cp.InternalCount; i++ {
			ik := cp.InternalPortals[i]
			ip := world.Portals[ik]
			if ip == nil {
				continue
			}
			edgeG := cur.g + float64(cp.InternalCosts[i])
			open = f.relaxEdge(allNodes, open, ik, edgeG, ip, goalPortal, cur)
		}
	}

	if found == nil {
		return &HPAFindResult{}
	}

	// Backtrace portal keys
	var portalPath []int
	for n := found; n != nil; n = n.parent {
		portalPath = append(portalPath, n.portalKey)
	}
	// Reverse
	for i, j := 0, len(portalPath)-1; i < j; i, j = i+1, j-1 {
		portalPath[i], portalPath[j] = portalPath[j], portalPath[i]
	}

	// Build concrete path
	var waypoints [][2]int

	// Start → first portal
	firstPortal := world.Portals[portalPath[0]]
	seg := f.concreteFind(grid, startX, startY, firstPortal.CenterX, firstPortal.CenterY)
	if seg == nil {
		return &HPAFindResult{PortalKeys: portalPath}
	}
	waypoints = append(waypoints, seg[:len(seg)-1]...) // exclude duplicate endpoint

	// Portal → portal
	for i := 0; i < len(portalPath)-1; i++ {
		p1 := world.Portals[portalPath[i]]
		p2 := world.Portals[portalPath[i+1]]
		seg = f.concreteFind(grid, p1.CenterX, p1.CenterY, p2.CenterX, p2.CenterY)
		if seg == nil {
			return &HPAFindResult{PortalKeys: portalPath, Waypoints: waypoints}
		}
		waypoints = append(waypoints, seg[1:]...) // skip first (already added)
	}

	// Last portal → end
	lastPortal := world.Portals[portalPath[len(portalPath)-1]]
	seg = f.concreteFind(grid, lastPortal.CenterX, lastPortal.CenterY, endX, endY)
	if seg == nil {
		return &HPAFindResult{PortalKeys: portalPath, Waypoints: waypoints}
	}
	waypoints = append(waypoints, seg[1:]...)

	return &HPAFindResult{
		PortalKeys: portalPath,
		Waypoints:  waypoints,
	}
}

func (f *HPAStarFinder) relaxEdge(allNodes map[int]*hpaNode, open []*hpaNode, ik int, edgeG float64, portal, goal *Portal, parent *hpaNode) []*hpaNode {
	existing, ok := allNodes[ik]
	if !ok {
		n := &hpaNode{
			portalKey: ik,
			g:         edgeG,
			h:         f.heuristicCost(portal, goal),
			parent:    parent,
			heapIdx:   -1,
		}
		n.f = n.g + n.h*f.Weight
		allNodes[ik] = n
		return f.push(open, n)
	}
	if existing.closed {
		return open
	}
	if edgeG < existing.g {
		existing.g = edgeG
		existing.f = existing.g + existing.h*f.Weight
		existing.parent = parent
		open = f.push(open, existing)
	}
	return open
}

func (f *HPAStarFinder) heuristicCost(a, b *Portal) float64 {
	dx := float64(b.CenterX - a.CenterX)
	if dx < 0 {
		dx = -dx
	}
	dy := float64(b.CenterY - a.CenterY)
	if dy < 0 {
		dy = -dy
	}
	return f.Heuristic(dx, dy)
}

// findReachablePortals finds portals in the same chunk as (sx, sy) that
// are reachable via BFS within the chunk.
func (f *HPAStarFinder) findReachablePortals(grid finder.Grid, world *HPAWorld, sx, sy int, chunkID int) []int {
	cs := world.ChunkSize
	cx := (sx / cs) * cs
	cy := (sy / cs) * cs
	nb := make([]*finder.Node, 0, 8)

	costs := bfsCostsInChunk(grid, sx, sy, cx, cy, cs, nb)

	dirs := []int{DirN, DirE, DirS, DirW}
	var result []int
	for _, dir := range dirs {
		for pos := 0; pos < cs; pos++ {
			key := PortalKey(chunkID, pos, dir, cs)
			p := world.Portals[key]
			if p == nil {
				continue
			}
			cost := getBfsCost(costs, p.CenterX, p.CenterY, cx, cy, cs)
			if cost >= 0 {
				result = append(result, key)
			}
		}
	}
	return result
}

// concreteFind runs A* on the concrete grid between two points (within a chunk).
func (f *HPAStarFinder) concreteFind(grid finder.Grid, sx, sy, ex, ey int) [][2]int {
	astar := &finder.AStarFinder{
		Heuristic:        f.Heuristic,
		Weight:           f.Weight,
		DiagonalMovement: f.DiagonalMovement,
	}
	return astar.FindPath(sx, sy, ex, ey, grid)
}

// --- Min-heap helpers for abstract A* ---

func (f *HPAStarFinder) push(heap []*hpaNode, n *hpaNode) []*hpaNode {
	if n.heapIdx >= 0 && n.heapIdx < len(heap) && heap[n.heapIdx] == n {
		f.siftUp(heap, n.heapIdx)
		return heap
	}
	n.heapIdx = len(heap)
	heap = append(heap, n)
	return f.siftUp(heap, n.heapIdx)
}

func (f *HPAStarFinder) pop(heap []*hpaNode) ([]*hpaNode, *hpaNode) {
	n := heap[0]
	last := len(heap) - 1
	if last > 0 {
		heap[0] = heap[last]
		heap[0].heapIdx = 0
	}
	heap = heap[:last]
	n.heapIdx = -1
	if len(heap) > 0 {
		heap = f.siftDown(heap, 0)
	}
	return heap, n
}

func (f *HPAStarFinder) siftUp(heap []*hpaNode, idx int) []*hpaNode {
	for idx > 0 {
		parent := (idx - 1) / 2
		if heap[idx].f >= heap[parent].f {
			break
		}
		heap[idx], heap[parent] = heap[parent], heap[idx]
		heap[idx].heapIdx = idx
		heap[parent].heapIdx = parent
		idx = parent
	}
	return heap
}

func (f *HPAStarFinder) siftDown(heap []*hpaNode, idx int) []*hpaNode {
	n := len(heap)
	for {
		smallest := idx
		left := 2*idx + 1
		right := 2*idx + 2
		if left < n && heap[left].f < heap[smallest].f {
			smallest = left
		}
		if right < n && heap[right].f < heap[smallest].f {
			smallest = right
		}
		if smallest == idx {
			break
		}
		heap[idx], heap[smallest] = heap[smallest], heap[idx]
		heap[idx].heapIdx = idx
		heap[smallest].heapIdx = smallest
		idx = smallest
	}
	return heap
}
