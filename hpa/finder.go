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

	pathCache map[[2]int32][]int32 // portal path cache (P3): key=[startChunk,endChunk]
	cacheMu   *sync.RWMutex
}

// hpaSearch holds the per-call scratch and caches for one HPA search. Shared
// state such as the portal path cache stays on HPAFinder.
type hpaSearch struct {
	*HPAFinder
	neighborBuf     []*finder.Node
	bfsCostsBuf     []int32
	bfsQueueBuf     [][2]int32
	tmpNode         *finder.Node
	allNodes        map[int32]*hpaNode
	openSlice       []*hpaNode
	hpaNodePool     []*hpaNode
	portalPathBuf   []int32
	waypointsBuf    [][2]int32
	waypointsResult [][2]int32
	portalKeyBuf    []int32
	portalKeyEndBuf []int32
	endSetMap       map[int32]struct{}
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
	heapIdx   int
}

// getHPANode returns a node from the pool, or allocates a new one.
func (s *hpaSearch) getHPANode() *hpaNode {
	if n := len(s.hpaNodePool); n > 0 {
		node := s.hpaNodePool[n-1]
		s.hpaNodePool = s.hpaNodePool[:n-1]
		return node
	}
	return &hpaNode{heapIdx: -1}
}

// recycleNodes clears the allNodes map and returns all hpaNodes to the pool.
func (s *hpaSearch) recycleNodes() {
	for _, n := range s.allNodes {
		*n = hpaNode{heapIdx: -1}
		s.hpaNodePool = append(s.hpaNodePool, n)
	}
	for k := range s.allNodes {
		delete(s.allNodes, k)
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
	s := &hpaSearch{
		HPAFinder: f,
		tmpNode:   &finder.Node{},
		allNodes:  make(map[int32]*hpaNode),
		endSetMap: make(map[int32]struct{}),
	}
	if dst != nil {
		s.waypointsResult = dst[:0]
	}
	p, err := s.findPath(startX, startY, endX, endY, grid)
	if dst == nil || p == nil {
		return p, err
	}
	return append(dst[:0], p...), err
}

func (s *hpaSearch) findPath(startX, startY, endX, endY int32, grid finder.Grid) ([][2]int32, error) {
	if s.world == nil {
		panic("HPAFinder: Build(grid) must be called before FindPath")
	}
	if !supportsHPA(grid) {
		return nil, finder.ErrUnsupportedGrid
	}
	// Search would otherwise answer "no route" for a request it never ran.
	if !grid.IsInside(startX, startY) || !grid.IsInside(endX, endY) {
		return nil, finder.ErrOutOfBounds
	}

	world := s.world
	startChunk := world.ChunkIDOf(startX, startY)
	endChunk := world.ChunkIDOf(endX, endY)

	if startChunk == endChunk {
		return s.concreteFind(startX, startY, endX, endY, grid)
	}

	if !grid.IsWalkableAt(endX, endY) {
		return nil, finder.ErrNotWalkable
	}
	if !grid.IsWalkableAt(startX, startY) {
		return nil, nil
	}

	startPortalKeys := s.findReachablePortals(world, startX, startY, startChunk, grid, s.portalKeyBuf)
	if len(startPortalKeys) == 0 {
		return nil, nil
	}

	endPortalKeys := s.findReachablePortals(world, endX, endY, endChunk, grid, s.portalKeyEndBuf)
	if len(endPortalKeys) == 0 {
		return nil, nil
	}

	// Reuse endSetMap
	for k := range s.endSetMap {
		delete(s.endSetMap, k)
	}
	for _, k := range endPortalKeys {
		s.endSetMap[k] = struct{}{}
	}

	// --- P3: Portal Path Cache ---
	cacheKey := [2]int32{startChunk, endChunk}
	s.cacheMu.RLock()
	cachedPath, cacheHit := s.pathCache[cacheKey]
	s.cacheMu.RUnlock()
	useCache := false
	if cacheHit && len(cachedPath) > 0 {
		firstInStart := false
		for _, pk := range startPortalKeys {
			if pk == cachedPath[0] {
				firstInStart = true
				break
			}
		}
		_, lastInEnd := s.endSetMap[cachedPath[len(cachedPath)-1]]
		if firstInStart && lastInEnd {
			useCache = true
		}
	}

	if !useCache {
		// Run abstract A* on portal graph
		s.openSlice = s.openSlice[:0]

		for _, pk := range startPortalKeys {
			p := world.Portals[pk]
			if p == nil {
				continue
			}
			n := s.getHPANode()
			n.portalKey = pk
			n.g = 0
			n.h = s.heuristicCost(p.CenterX, p.CenterY, endX, endY)
			n.f = n.h * s.Weight
			n.parent = nil
			n.closed = false
			n.heapIdx = -1
			s.allNodes[pk] = n
			s.openSlice = s.push(s.openSlice, n)
		}

		var found *hpaNode
		for len(s.openSlice) > 0 {
			var cur *hpaNode
			s.openSlice, cur = s.pop(s.openSlice)
			if cur.closed {
				continue
			}
			cur.closed = true

			if _, ok := s.endSetMap[cur.portalKey]; ok {
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
				edgeG := cur.g + float32(cp.ExternalCosts[i])
				s.openSlice = s.relaxEdge(ek, edgeG, ep, endX, endY, cur)
			}

			for i := 0; i < cp.InternalCount; i++ {
				ik := cp.InternalPortals[i]
				ip := world.Portals[ik]
				if ip == nil {
					continue
				}
				edgeG := cur.g + float32(cp.InternalCosts[i])
				s.openSlice = s.relaxEdge(ik, edgeG, ip, endX, endY, cur)
			}
		}

		if found == nil {
			s.recycleNodes()
			return nil, nil
		}

		// Build portal path (reversed from found linked list)
		s.portalPathBuf = s.portalPathBuf[:0]
		for n := found; n != nil; n = n.parent {
			s.portalPathBuf = append(s.portalPathBuf, n.portalKey)
		}
		// Reverse
		for i, j := 0, len(s.portalPathBuf)-1; i < j; i, j = i+1, j-1 {
			s.portalPathBuf[i], s.portalPathBuf[j] = s.portalPathBuf[j], s.portalPathBuf[i]
		}

		// Store in cache
		cached := make([]int32, len(s.portalPathBuf))
		copy(cached, s.portalPathBuf)
		s.cacheMu.Lock()
		s.pathCache[cacheKey] = cached
		s.cacheMu.Unlock()
	} else {
		// Reuse cached portal path
		s.portalPathBuf = append(s.portalPathBuf[:0], cachedPath...)
	}

	// Build concrete waypoints (shared by cache hit and miss)
	s.waypointsBuf = s.waypointsBuf[:0]

	firstPortal := world.Portals[s.portalPathBuf[0]]
	seg, err := s.concreteFind(startX, startY, firstPortal.CenterX, firstPortal.CenterY, grid)
	if err != nil {
		s.recycleNodes()
		return nil, err
	}
	if seg == nil {
		s.recycleNodes()
		return nil, nil
	}
	s.waypointsBuf = append(s.waypointsBuf, seg[:len(seg)-1]...)

	for i := 0; i < len(s.portalPathBuf)-1; i++ {
		p1 := world.Portals[s.portalPathBuf[i]]
		p2 := world.Portals[s.portalPathBuf[i+1]]
		seg, err = s.concreteFind(p1.CenterX, p1.CenterY, p2.CenterX, p2.CenterY, grid)
		if err != nil {
			s.recycleNodes()
			return nil, err
		}
		if seg == nil {
			s.recycleNodes()
			return nil, nil
		}
		s.waypointsBuf = append(s.waypointsBuf, seg[1:]...)
	}

	lastPortal := world.Portals[s.portalPathBuf[len(s.portalPathBuf)-1]]
	seg, err = s.concreteFind(lastPortal.CenterX, lastPortal.CenterY, endX, endY, grid)
	if err != nil {
		s.recycleNodes()
		return nil, err
	}
	if seg == nil {
		s.recycleNodes()
		return nil, nil
	}
	s.waypointsBuf = append(s.waypointsBuf, seg[1:]...)

	// P1: String pulling — smooth path
	if s.stringPulling {
		s.waypointsBuf = s.smoothPath(s.waypointsBuf, grid)
	}

	// P2: Reusable result buffer
	if cap(s.waypointsResult) >= len(s.waypointsBuf) {
		s.waypointsResult = s.waypointsResult[:len(s.waypointsBuf)]
	} else {
		s.waypointsResult = make([][2]int32, len(s.waypointsBuf))
	}
	copy(s.waypointsResult, s.waypointsBuf)

	s.recycleNodes()
	return s.waypointsResult, nil
}

func (s *hpaSearch) relaxEdge(ik int32, edgeG float32, portal *Portal, endX, endY int32, parent *hpaNode) []*hpaNode {
	existing, ok := s.allNodes[ik]
	if !ok {
		n := s.getHPANode()
		n.portalKey = ik
		n.g = edgeG
		n.h = s.heuristicCost(portal.CenterX, portal.CenterY, endX, endY)
		n.f = n.g + n.h*s.Weight
		n.parent = parent
		n.closed = false
		n.heapIdx = -1
		s.allNodes[ik] = n
		return s.push(s.openSlice, n)
	}
	if existing.closed {
		return s.openSlice
	}
	if edgeG < existing.g {
		existing.g = edgeG
		existing.f = existing.g + existing.h*s.Weight
		existing.parent = parent
		return s.push(s.openSlice, existing)
	}
	return s.openSlice
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

func (s *hpaSearch) findReachablePortals(world *HPAWorld, sx, sy int32, chunkID int32, grid finder.Grid, buf []int32) []int32 {
	cs := world.ChunkSize
	cx := (sx / cs) * cs
	cy := (sy / cs) * cs

	s.neighborBuf = s.neighborBuf[:0]
	costs, qBuf := bfsCostsInChunk(grid, sx, sy, cx, cy, cs, world.Diagonal, s.neighborBuf, s.bfsCostsBuf, s.bfsQueueBuf, s.tmpNode)
	if cap(costs) > cap(s.bfsCostsBuf) {
		s.bfsCostsBuf = costs
	}
	if cap(qBuf) > cap(s.bfsQueueBuf) {
		s.bfsQueueBuf = qBuf
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
