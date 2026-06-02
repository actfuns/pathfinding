package hpa

import (
	"github.com/actfuns/pathfinding/finder"
)

// HPAFinder implements Hierarchical Pathfinding A*.
//
// It implements finder.Finder, so it can be used wherever a
// Finder is expected. Call Build(grid) to pre-build the
// hierarchical portal graph before FindPath.
type HPAFinder struct {
	// Heuristic is the heuristic function used for estimating path cost.
	Heuristic finder.HeuristicFunc
	// Weight is the heuristic weight (higher values bias toward faster
	// but potentially suboptimal paths).
	Weight float64
	// DiagonalMovement controls whether diagonal moves are permitted
	// and under what conditions.
	DiagonalMovement finder.DiagonalMovement

	chunkSize      int
	world          *HPAWorld
	concreteFinder finder.Finder
	stringPulling  bool // apply LOS smoothing to final path

	// Reusable buffers for FindPath (zero-alloc after warm-up)
	neighborBuf     []*finder.Node
	bfsCostsBuf     []int
	bfsQueueBuf     [][2]int
	tmpNode         *finder.Node
	allNodes        map[int]*hpaNode
	openSlice       []*hpaNode
	hpaNodePool     []*hpaNode       // free list for hpaNode reuse
	portalPathBuf   []int            // reusable portal key accumulation
	waypointsBuf    [][2]int         // reusable waypoint accumulation
	waypointsResult [][2]int         // reusable result buffer (P2)
	portalKeyBuf    []int            // reusable result in findReachablePortals (start)
	portalKeyEndBuf []int            // reusable result in findReachablePortals (end)
	endSetMap       map[int]struct{} // reusable end-portal set
	pathCache       map[[2]int][]int // portal path cache (P3): key=[startChunk,endChunk]
}

// HPAOption configures an HPAFinder.
type HPAOption func(*HPAFinder)

// WithChunkSize sets the chunk size for world partitioning.
func WithChunkSize(cs int) HPAOption {
	return func(f *HPAFinder) { f.chunkSize = cs }
}

// WithConcreteFinder sets the finder used for segment pathfinding.
func WithConcreteFinder(fnd finder.Finder) HPAOption {
	return func(f *HPAFinder) { f.concreteFinder = fnd }
}

// WithDiagonal sets the diagonal movement rule.
func WithDiagonal(d finder.DiagonalMovement) HPAOption {
	return func(f *HPAFinder) { f.DiagonalMovement = d }
}

// WithWeight sets the heuristic weight.
func WithWeight(w float64) HPAOption {
	return func(f *HPAFinder) { f.Weight = w }
}

// WithHeuristic sets the heuristic function.
func WithHeuristic(h finder.HeuristicFunc) HPAOption {
	return func(f *HPAFinder) { f.Heuristic = h }
}

// WithAllowDiagonal enables diagonal movement.
func WithAllowDiagonal(dontCrossCorners bool) HPAOption {
	return func(f *HPAFinder) {
		if dontCrossCorners {
			f.DiagonalMovement = finder.DiagonalOnlyWhenNoObstacles
		} else {
			f.DiagonalMovement = finder.DiagonalIfAtMostOneObstacle
		}
	}
}

// WithStringPulling enables or disables path smoothing via LOS string pulling.
// Enabled by default.
func WithStringPulling(enabled bool) HPAOption {
	return func(f *HPAFinder) { f.stringPulling = enabled }
}

// NewHPAFinder creates an HPA* finder.
// Call Build(grid) to pre-build the hierarchical portal graph before
// calling FindPath.
func NewHPAFinder(opts ...HPAOption) *HPAFinder {
	f := &HPAFinder{
		Heuristic:        finder.Manhattan,
		Weight:           1,
		DiagonalMovement: finder.DiagonalNever,
		chunkSize:        16,
		stringPulling:    true,
	}
	for _, opt := range opts {
		opt(f)
	}
	if f.DiagonalMovement != finder.DiagonalNever {
		f.Heuristic = finder.Octile
	}
	// Default concrete finder if not set
	if f.concreteFinder == nil {
		f.concreteFinder = finder.NewAStarFinder(
			finder.WithDiagonal(f.DiagonalMovement),
			finder.WithHeuristic(f.Heuristic),
			finder.WithWeight(f.Weight),
		)
	}
	// Pre-allocate buffers
	f.neighborBuf = make([]*finder.Node, 0, 8)
	f.tmpNode = &finder.Node{}
	f.allNodes = make(map[int]*hpaNode, 64)
	f.openSlice = make([]*hpaNode, 0, 64)
	f.endSetMap = make(map[int]struct{})
	f.pathCache = make(map[[2]int][]int, 64)
	return f
}

// Build pre-builds the hierarchical portal graph (HPAWorld) from the
// given grid. Must be called before FindPath. Can be called again to
// rebuild after grid changes. Clears the portal path cache on rebuild.
func (f *HPAFinder) Build(grid finder.Grid) {
	f.world = BuildWorld(grid, f.chunkSize)
	// Invalidate path cache on rebuild
	for k := range f.pathCache {
		delete(f.pathCache, k)
	}
}

// hpaNode is a node in the abstract portal graph.
type hpaNode struct {
	portalKey int
	g, h, f   float64
	parent    *hpaNode
	closed    bool
	heapIdx   int
}

// getHPANode returns a node from the pool, or allocates a new one.
func (f *HPAFinder) getHPANode() *hpaNode {
	if n := len(f.hpaNodePool); n > 0 {
		node := f.hpaNodePool[n-1]
		f.hpaNodePool = f.hpaNodePool[:n-1]
		return node
	}
	return &hpaNode{heapIdx: -1}
}

// recycleNodes clears the allNodes map and returns all hpaNodes to the pool.
func (f *HPAFinder) recycleNodes() {
	for _, n := range f.allNodes {
		*n = hpaNode{heapIdx: -1}
		f.hpaNodePool = append(f.hpaNodePool, n)
	}
	for k := range f.allNodes {
		delete(f.allNodes, k)
	}
}

// FindPath performs HPA* on the given grid. The hierarchical graph must
// have been pre-built via Build(grid).
//
// It implements finder.Finder. Panics if Build() has not been called.
func (f *HPAFinder) FindPath(startX, startY, endX, endY int, grid finder.Grid) [][2]int {
	if f.world == nil {
		panic("HPAFinder: Build(grid) must be called before FindPath")
	}

	world := f.world
	startChunk := world.ChunkIDOf(startX, startY)
	endChunk := world.ChunkIDOf(endX, endY)

	if startChunk == endChunk {
		return f.concreteFind(startX, startY, endX, endY, grid)
	}

	if !grid.IsWalkableAt(startX, startY) || !grid.IsWalkableAt(endX, endY) {
		return nil
	}

	startPortalKeys := f.findReachablePortals(world, startX, startY, startChunk, grid, f.portalKeyBuf)
	if len(startPortalKeys) == 0 {
		return nil
	}

	endPortalKeys := f.findReachablePortals(world, endX, endY, endChunk, grid, f.portalKeyEndBuf)
	if len(endPortalKeys) == 0 {
		return nil
	}

	// Reuse endSetMap
	for k := range f.endSetMap {
		delete(f.endSetMap, k)
	}
	for _, k := range endPortalKeys {
		f.endSetMap[k] = struct{}{}
	}

	// --- P3: Portal Path Cache ---
	cacheKey := [2]int{startChunk, endChunk}
	cachedPath, cacheHit := f.pathCache[cacheKey]
	useCache := false
	if cacheHit && len(cachedPath) > 0 {
		firstInStart := false
		for _, pk := range startPortalKeys {
			if pk == cachedPath[0] {
				firstInStart = true
				break
			}
		}
		_, lastInEnd := f.endSetMap[cachedPath[len(cachedPath)-1]]
		if firstInStart && lastInEnd {
			useCache = true
		}
	}

	if !useCache {
		// Run abstract A* on portal graph
		f.openSlice = f.openSlice[:0]

		for _, pk := range startPortalKeys {
			p := world.Portals[pk]
			if p == nil {
				continue
			}
			n := f.getHPANode()
			n.portalKey = pk
			n.g = 0
			n.h = f.heuristicCost(p.CenterX, p.CenterY, endX, endY)
			n.f = n.h * f.Weight
			n.parent = nil
			n.closed = false
			n.heapIdx = -1
			f.allNodes[pk] = n
			f.openSlice = f.push(f.openSlice, n)
		}

		var found *hpaNode
		for len(f.openSlice) > 0 {
			var cur *hpaNode
			f.openSlice, cur = f.pop(f.openSlice)
			if cur.closed {
				continue
			}
			cur.closed = true

			if _, ok := f.endSetMap[cur.portalKey]; ok {
				found = cur
				break
			}

			cp := world.Portals[cur.portalKey]
			if cp == nil {
				continue
			}

			for i := 0; i < cp.ExternalCount; i++ {
				ek := cp.ExternalPortals[i]
				ep := world.Portals[ek]
				if ep == nil {
					continue
				}
				edgeG := cur.g + 10
				f.openSlice = f.relaxEdge(ek, edgeG, ep, endX, endY, cur)
			}

			for i := 0; i < cp.InternalCount; i++ {
				ik := cp.InternalPortals[i]
				ip := world.Portals[ik]
				if ip == nil {
					continue
				}
				edgeG := cur.g + float64(cp.InternalCosts[i])
				f.openSlice = f.relaxEdge(ik, edgeG, ip, endX, endY, cur)
			}
		}

		if found == nil {
			f.recycleNodes()
			return nil
		}

		// Build portal path (reversed from found linked list)
		f.portalPathBuf = f.portalPathBuf[:0]
		for n := found; n != nil; n = n.parent {
			f.portalPathBuf = append(f.portalPathBuf, n.portalKey)
		}
		// Reverse
		for i, j := 0, len(f.portalPathBuf)-1; i < j; i, j = i+1, j-1 {
			f.portalPathBuf[i], f.portalPathBuf[j] = f.portalPathBuf[j], f.portalPathBuf[i]
		}

		// Store in cache
		cached := make([]int, len(f.portalPathBuf))
		copy(cached, f.portalPathBuf)
		f.pathCache[cacheKey] = cached
	} else {
		// Reuse cached portal path
		f.portalPathBuf = append(f.portalPathBuf[:0], cachedPath...)
	}

	// Build concrete waypoints (shared by cache hit and miss)
	f.waypointsBuf = f.waypointsBuf[:0]

	firstPortal := world.Portals[f.portalPathBuf[0]]
	seg := f.concreteFind(startX, startY, firstPortal.CenterX, firstPortal.CenterY, grid)
	if seg == nil {
		f.recycleNodes()
		return nil
	}
	f.waypointsBuf = append(f.waypointsBuf, seg[:len(seg)-1]...)

	for i := 0; i < len(f.portalPathBuf)-1; i++ {
		p1 := world.Portals[f.portalPathBuf[i]]
		p2 := world.Portals[f.portalPathBuf[i+1]]
		seg = f.concreteFind(p1.CenterX, p1.CenterY, p2.CenterX, p2.CenterY, grid)
		if seg == nil {
			f.recycleNodes()
			return nil
		}
		f.waypointsBuf = append(f.waypointsBuf, seg[1:]...)
	}

	lastPortal := world.Portals[f.portalPathBuf[len(f.portalPathBuf)-1]]
	seg = f.concreteFind(lastPortal.CenterX, lastPortal.CenterY, endX, endY, grid)
	if seg == nil {
		f.recycleNodes()
		return nil
	}
	f.waypointsBuf = append(f.waypointsBuf, seg[1:]...)

	// P1: String pulling — smooth path
	if f.stringPulling {
		f.waypointsBuf = f.smoothPath(f.waypointsBuf, grid)
	}

	// P2: Reusable result buffer
	if cap(f.waypointsResult) >= len(f.waypointsBuf) {
		f.waypointsResult = f.waypointsResult[:len(f.waypointsBuf)]
	} else {
		f.waypointsResult = make([][2]int, len(f.waypointsBuf))
	}
	copy(f.waypointsResult, f.waypointsBuf)

	f.recycleNodes()
	return f.waypointsResult
}

func (f *HPAFinder) relaxEdge(ik int, edgeG float64, portal *Portal, endX, endY int, parent *hpaNode) []*hpaNode {
	existing, ok := f.allNodes[ik]
	if !ok {
		n := f.getHPANode()
		n.portalKey = ik
		n.g = edgeG
		n.h = f.heuristicCost(portal.CenterX, portal.CenterY, endX, endY)
		n.f = n.g + n.h*f.Weight
		n.parent = parent
		n.closed = false
		n.heapIdx = -1
		f.allNodes[ik] = n
		return f.push(f.openSlice, n)
	}
	if existing.closed {
		return f.openSlice
	}
	if edgeG < existing.g {
		existing.g = edgeG
		existing.f = existing.g + existing.h*f.Weight
		existing.parent = parent
		return f.push(f.openSlice, existing)
	}
	return f.openSlice
}

func (f *HPAFinder) heuristicCost(ax, ay, bx, by int) float64 {
	dx := float64(bx - ax)
	if dx < 0 {
		dx = -dx
	}
	dy := float64(by - ay)
	if dy < 0 {
		dy = -dy
	}
	return f.Heuristic(dx, dy)
}

func (f *HPAFinder) findReachablePortals(world *HPAWorld, sx, sy int, chunkID int, grid finder.Grid, buf []int) []int {
	cs := world.ChunkSize
	cx := (sx / cs) * cs
	cy := (sy / cs) * cs

	f.neighborBuf = f.neighborBuf[:0]
	costs, qBuf := bfsCostsInChunk(grid, sx, sy, cx, cy, cs, f.neighborBuf, f.bfsCostsBuf, f.bfsQueueBuf, f.tmpNode)
	if cap(costs) > cap(f.bfsCostsBuf) {
		f.bfsCostsBuf = costs
	}
	if cap(qBuf) > cap(f.bfsQueueBuf) {
		f.bfsQueueBuf = qBuf
	}

	dirs := []int{DirN, DirE, DirS, DirW}
	buf = buf[:0]
	for _, dir := range dirs {
		for pos := 0; pos < cs; pos++ {
			key := PortalKey(chunkID, pos, dir, cs)
			p := world.Portals[key]
			if p == nil {
				continue
			}
			cost := getBfsCost(costs, p.CenterX, p.CenterY, cx, cy, cs)
			if cost >= 0 {
				buf = append(buf, key)
			}
		}
	}
	return buf
}

func (f *HPAFinder) concreteFind(sx, sy, ex, ey int, grid finder.Grid) [][2]int {
	return f.concreteFinder.FindPath(sx, sy, ex, ey, grid)
}

// smoothPath removes unnecessary waypoints from a path by checking
// line-of-sight between each point. Only points where the direct line
// is blocked by obstacles are kept. Result is written in-place over the
// input buffer.
func (f *HPAFinder) smoothPath(path [][2]int, grid finder.Grid) [][2]int {
	if len(path) < 3 {
		return path
	}
	writeIdx := 1
	for i := 2; i < len(path); i++ {
		if !tileLineOfSight(path[writeIdx-1], path[i], grid) {
			path[writeIdx] = path[i-1]
			writeIdx++
		}
	}
	if path[writeIdx-1] != path[len(path)-1] {
		path[writeIdx] = path[len(path)-1]
		writeIdx++
	}
	return path[:writeIdx]
}

// tileLineOfSight checks walkability along a Bresenham line between two
// tiles. Returns true if every cell on the line is walkable.
func tileLineOfSight(a, b [2]int, grid finder.Grid) bool {
	x0, y0 := a[0], a[1]
	x1, y1 := b[0], b[1]
	dx := x1 - x0
	dy := y1 - y0
	var sx, sy int
	if dx < 0 {
		dx = -dx
		sx = -1
	} else {
		sx = 1
	}
	if dy < 0 {
		dy = -dy
		sy = -1
	} else {
		sy = 1
	}
	err := dx - dy

	for {
		if !grid.IsWalkableAt(x0, y0) {
			return false
		}
		if x0 == x1 && y0 == y1 {
			break
		}
		e2 := 2 * err
		if e2 > -dy {
			err -= dy
			x0 += sx
		}
		if e2 < dx {
			err += dx
			y0 += sy
		}
	}
	return true
}

// --- Min-heap helpers ---

func (f *HPAFinder) push(heap []*hpaNode, n *hpaNode) []*hpaNode {
	if n.heapIdx >= 0 && n.heapIdx < len(heap) && heap[n.heapIdx] == n {
		f.siftUp(heap, n.heapIdx)
		return heap
	}
	n.heapIdx = len(heap)
	heap = append(heap, n)
	return f.siftUp(heap, n.heapIdx)
}

func (f *HPAFinder) pop(heap []*hpaNode) ([]*hpaNode, *hpaNode) {
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

func (f *HPAFinder) siftUp(heap []*hpaNode, idx int) []*hpaNode {
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

func (f *HPAFinder) siftDown(heap []*hpaNode, idx int) []*hpaNode {
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
