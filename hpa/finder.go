package hpa

import (
	"sync"

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
	Weight float32
	// DiagonalMovement controls whether diagonal moves are permitted
	// and under what conditions.
	DiagonalMovement finder.DiagonalMovement

	chunkSize      int32
	world          *HPAWorld
	concreteFinder finder.Finder
	stringPulling  bool // apply LOS smoothing to final path

	// Reusable buffers for FindPath (zero-alloc after warm-up)
	neighborBuf     []*finder.Node
	bfsCostsBuf     []int32
	bfsQueueBuf     [][2]int32
	tmpNode         *finder.Node
	allNodes        map[int32]*hpaNode
	openSlice       []*hpaNode
	hpaNodePool     []*hpaNode           // free list for hpaNode reuse
	portalPathBuf   []int32              // reusable portal key accumulation
	waypointsBuf    [][2]int32           // reusable waypoint accumulation
	waypointsResult [][2]int32           // reusable result buffer (P2)
	portalKeyBuf    []int32              // reusable result in findReachablePortals (start)
	portalKeyEndBuf []int32              // reusable result in findReachablePortals (end)
	endSetMap       map[int32]struct{}   // reusable end-portal set
	pathCache       map[[2]int32][]int32 // portal path cache (P3): key=[startChunk,endChunk]
	cacheMu         *sync.RWMutex
}

// HPAOption configures an HPAFinder.
type HPAOption func(*HPAFinder)

// WithChunkSize sets the chunk size for world partitioning.
func WithChunkSize(cs int32) HPAOption {
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
func WithWeight(w float32) HPAOption {
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
	f.allNodes = make(map[int32]*hpaNode, 64)
	f.openSlice = make([]*hpaNode, 0, 64)
	f.endSetMap = make(map[int32]struct{})
	f.pathCache = make(map[[2]int32][]int32, 64)
	f.cacheMu = new(sync.RWMutex)
	return f
}

// Build pre-builds the hierarchical portal graph (HPAWorld) from the given
// grid. Must be called before FindPath. Can be called again to rebuild after
// grid changes. Clears the portal path cache on rebuild.
//
// HPA assumes an axis-aligned (orthogonal) grid and uniform tile weights. It
// returns finder.ErrUnsupportedGrid for hex/staggered grids; use A*/Dijkstra
// for weighted maps.
func (f *HPAFinder) Build(grid finder.Grid) error {
	if !supportsHPA(grid) {
		return finder.ErrUnsupportedGrid
	}
	if f.cacheMu == nil {
		f.cacheMu = new(sync.RWMutex)
	}
	if f.pathCache == nil {
		f.pathCache = make(map[[2]int32][]int32, 64)
	}
	f.world = BuildWorldWithDiagonal(grid, f.chunkSize, f.DiagonalMovement)
	// Invalidate path cache on rebuild
	f.cacheMu.Lock()
	for k := range f.pathCache {
		delete(f.pathCache, k)
	}
	f.cacheMu.Unlock()
	return nil
}

// supportsHPA reports whether the grid has the axis-aligned topology HPA uses.
func supportsHPA(grid finder.Grid) bool {
	g, ok := grid.(interface{ SupportsJPSCanonicalPruning() bool })
	return ok && g.SupportsJPSCanonicalPruning()
}

// hpaNode is a node in the abstract portal graph.
type hpaNode struct {
	portalKey int32
	g, h, f   float32
	parent    *hpaNode
	closed    bool
	heapIdx   int32
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
// HPAFinder is a Finder that searches an abstract chunk graph, falling back to
// a concrete search within a chunk.
var _ finder.Finder = (*HPAFinder)(nil)

func (f *HPAFinder) FindPath(startX, startY, endX, endY int32, grid finder.Grid, dst [][2]int32) ([][2]int32, error) {
	local := *f
	local.neighborBuf = nil
	local.bfsCostsBuf = nil
	local.bfsQueueBuf = nil
	local.tmpNode = &finder.Node{}
	local.allNodes = make(map[int32]*hpaNode)
	local.openSlice = nil
	local.hpaNodePool = nil
	local.portalPathBuf = nil
	local.waypointsBuf = nil
	if dst != nil {
		local.waypointsResult = dst[:0]
	} else {
		local.waypointsResult = nil
	}
	local.portalKeyBuf = nil
	local.portalKeyEndBuf = nil
	local.endSetMap = make(map[int32]struct{})
	p, err := local.findPath(startX, startY, endX, endY, grid)
	if dst == nil || p == nil {
		return p, err
	}
	return append(dst[:0], p...), err
}

func (f *HPAFinder) findPath(startX, startY, endX, endY int32, grid finder.Grid) ([][2]int32, error) {
	if f.world == nil {
		panic("HPAFinder: Build(grid) must be called before FindPath")
	}
	if !supportsHPA(grid) {
		return nil, finder.ErrUnsupportedGrid
	}
	// Search would otherwise answer "no route" for a request it never ran.
	if !grid.IsInside(startX, startY) || !grid.IsInside(endX, endY) {
		return nil, finder.ErrOutOfBounds
	}

	world := f.world
	startChunk := world.ChunkIDOf(startX, startY)
	endChunk := world.ChunkIDOf(endX, endY)

	if startChunk == endChunk {
		return f.concreteFind(startX, startY, endX, endY, grid)
	}

	if !grid.IsWalkableAt(endX, endY) {
		return nil, finder.ErrNotWalkable
	}
	if !grid.IsWalkableAt(startX, startY) {
		return nil, nil
	}

	startPortalKeys := f.findReachablePortals(world, startX, startY, startChunk, grid, f.portalKeyBuf)
	if len(startPortalKeys) == 0 {
		return nil, nil
	}

	endPortalKeys := f.findReachablePortals(world, endX, endY, endChunk, grid, f.portalKeyEndBuf)
	if len(endPortalKeys) == 0 {
		return nil, nil
	}

	// Reuse endSetMap
	for k := range f.endSetMap {
		delete(f.endSetMap, k)
	}
	for _, k := range endPortalKeys {
		f.endSetMap[k] = struct{}{}
	}

	// --- P3: Portal Path Cache ---
	cacheKey := [2]int32{startChunk, endChunk}
	f.cacheMu.RLock()
	cachedPath, cacheHit := f.pathCache[cacheKey]
	f.cacheMu.RUnlock()
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

			for i := 0; i < int(cp.ExternalCount); i++ {
				ek := cp.ExternalPortals[i]
				ep := world.Portals[ek]
				if ep == nil {
					continue
				}
				edgeG := cur.g + float32(cp.ExternalCosts[i])
				f.openSlice = f.relaxEdge(ek, edgeG, ep, endX, endY, cur)
			}

			for i := 0; i < int(cp.InternalCount); i++ {
				ik := cp.InternalPortals[i]
				ip := world.Portals[ik]
				if ip == nil {
					continue
				}
				edgeG := cur.g + float32(cp.InternalCosts[i])
				f.openSlice = f.relaxEdge(ik, edgeG, ip, endX, endY, cur)
			}
		}

		if found == nil {
			f.recycleNodes()
			return nil, nil
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
		cached := make([]int32, len(f.portalPathBuf))
		copy(cached, f.portalPathBuf)
		f.cacheMu.Lock()
		f.pathCache[cacheKey] = cached
		f.cacheMu.Unlock()
	} else {
		// Reuse cached portal path
		f.portalPathBuf = append(f.portalPathBuf[:0], cachedPath...)
	}

	// Build concrete waypoints (shared by cache hit and miss)
	f.waypointsBuf = f.waypointsBuf[:0]

	firstPortal := world.Portals[f.portalPathBuf[0]]
	seg, err := f.concreteFind(startX, startY, firstPortal.CenterX, firstPortal.CenterY, grid)
	if err != nil {
		f.recycleNodes()
		return nil, err
	}
	if seg == nil {
		f.recycleNodes()
		return nil, nil
	}
	f.waypointsBuf = append(f.waypointsBuf, seg[:len(seg)-1]...)

	for i := 0; i < len(f.portalPathBuf)-1; i++ {
		p1 := world.Portals[f.portalPathBuf[i]]
		p2 := world.Portals[f.portalPathBuf[i+1]]
		seg, err = f.concreteFind(p1.CenterX, p1.CenterY, p2.CenterX, p2.CenterY, grid)
		if err != nil {
			f.recycleNodes()
			return nil, err
		}
		if seg == nil {
			f.recycleNodes()
			return nil, nil
		}
		f.waypointsBuf = append(f.waypointsBuf, seg[1:]...)
	}

	lastPortal := world.Portals[f.portalPathBuf[len(f.portalPathBuf)-1]]
	seg, err = f.concreteFind(lastPortal.CenterX, lastPortal.CenterY, endX, endY, grid)
	if err != nil {
		f.recycleNodes()
		return nil, err
	}
	if seg == nil {
		f.recycleNodes()
		return nil, nil
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
		f.waypointsResult = make([][2]int32, len(f.waypointsBuf))
	}
	copy(f.waypointsResult, f.waypointsBuf)

	f.recycleNodes()
	return f.waypointsResult, nil
}

func (f *HPAFinder) relaxEdge(ik int32, edgeG float32, portal *Portal, endX, endY int32, parent *hpaNode) []*hpaNode {
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

func (f *HPAFinder) heuristicCost(ax, ay, bx, by int32) float32 {
	dx := float32(bx) - float32(ax)
	if dx < 0 {
		dx = -dx
	}
	dy := float32(by) - float32(ay)
	if dy < 0 {
		dy = -dy
	}
	return f.Heuristic(dx, dy) * 10
}

func (f *HPAFinder) findReachablePortals(world *HPAWorld, sx, sy int32, chunkID int32, grid finder.Grid, buf []int32) []int32 {
	cs := world.ChunkSize
	cx := (sx / cs) * cs
	cy := (sy / cs) * cs

	f.neighborBuf = f.neighborBuf[:0]
	costs, qBuf := bfsCostsInChunk(grid, sx, sy, cx, cy, cs, world.Diagonal, f.neighborBuf, f.bfsCostsBuf, f.bfsQueueBuf, f.tmpNode)
	if cap(costs) > cap(f.bfsCostsBuf) {
		f.bfsCostsBuf = costs
	}
	if cap(qBuf) > cap(f.bfsQueueBuf) {
		f.bfsQueueBuf = qBuf
	}

	dirs := []int32{DirN, DirE, DirS, DirW}
	buf = buf[:0]
	for _, dir := range dirs {
		for pos := 0; pos < int(cs); pos++ {
			key := PortalKey(chunkID, int32(pos), dir, cs)
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

func (f *HPAFinder) concreteFind(sx, sy, ex, ey int32, grid finder.Grid) ([][2]int32, error) {
	return f.concreteFinder.FindPath(sx, sy, ex, ey, grid, nil)
}

// smoothPath removes unnecessary waypoints from a path by checking
// line-of-sight between each point. Only points where the direct line
// is blocked by obstacles are kept. Result is written in-place over the
// input buffer.
func (f *HPAFinder) smoothPath(path [][2]int32, grid finder.Grid) [][2]int32 {
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
func tileLineOfSight(a, b [2]int32, grid finder.Grid) bool {
	x0, y0 := a[0], a[1]
	x1, y1 := b[0], b[1]
	dx := x1 - x0
	dy := y1 - y0
	var sx, sy int32
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
	if n.heapIdx >= 0 && int(n.heapIdx) < len(heap) && heap[int(n.heapIdx)] == n {
		f.siftUp(heap, int(n.heapIdx))
		return heap
	}
	n.heapIdx = int32(len(heap))
	heap = append(heap, n)
	return f.siftUp(heap, int(n.heapIdx))
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
		heap[idx].heapIdx = int32(idx)
		heap[parent].heapIdx = int32(parent)
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
		heap[idx].heapIdx = int32(idx)
		heap[smallest].heapIdx = int32(smallest)
		idx = smallest
	}
	return heap
}
