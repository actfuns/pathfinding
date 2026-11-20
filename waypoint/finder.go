package waypoint

import (
	"math"
	"sync"
	"sync/atomic"

	"github.com/actfuns/pathfinding/finder"
)

// WaypointFinder implements finder.Finder on a waypoint graph.
//
// It finds the closest waypoint nodes with line-of-sight to the start/end
// positions (via the grid parameter) and runs A* on the graph.
type waypointScratch struct {
	seq     int64
	allMap  map[*WaypointNode]*aNode
	heap    []*aNode
	pathBuf [][2]int32
}

var (
	waypointSeq         atomic.Int64
	waypointScratchPool = sync.Pool{New: func() any { return &waypointScratch{allMap: make(map[*WaypointNode]*aNode)} }}
)

type WaypointFinder struct {
	graph *WaypointGraph
}

// WaypointOption configures a WaypointFinder.
type WaypointOption func(*WaypointFinder)

// WithGraph sets the waypoint graph. If not provided, a new empty graph
// is created internally. Use this when sharing a graph between multiple finders
// or when loading a pre-built graph.
func WithGraph(g *WaypointGraph) WaypointOption {
	return func(f *WaypointFinder) { f.graph = g }
}

// NewWaypointFinder creates a WaypointFinder with the given options.
// By default it creates an internal WaypointGraph. Use WithGraph to
// inject a pre-built graph.
func NewWaypointFinder(opts ...WaypointOption) *WaypointFinder {
	f := &WaypointFinder{
		graph: NewWaypointGraph(),
	}
	for _, opt := range opts {
		opt(f)
	}
	return f
}

// Graph returns the underlying waypoint graph for node/edge operations.
func (f *WaypointFinder) Graph() *WaypointGraph { return f.graph }

// WaypointFinder is a Finder over a prebuilt waypoint graph.
var _ finder.Finder = (*WaypointFinder)(nil)

// FindPath implements finder.Finder by searching the waypoint graph.
// The grid is used for LOS-based node selection — only nodes with a clear
// line of sight from the start/end position are considered.
// An unwalkable destination returns finder.ErrNotWalkable; an unwalkable start
// is still snapped to the closest node with line of sight.
// Path is automatically smoothed via string pulling to remove redundant waypoints.
func (f *WaypointFinder) FindPath(startX, startY, endX, endY int32, grid finder.Grid, dst [][2]int32) ([][2]int32, error) {
	sc := waypointScratchPool.Get().(*waypointScratch)
	sc.seq = waypointSeq.Add(1)
	sc.heap = sc.heap[:0]
	if dst != nil {
		sc.pathBuf = dst[:0]
	} else {
		sc.pathBuf = nil
	}
	defer waypointScratchPool.Put(sc)
	p, err := f.findPath(startX, startY, endX, endY, grid, sc)
	if dst == nil || p == nil {
		return p, err
	}
	return append(dst[:0], p...), err
}

func (f *WaypointFinder) findPath(startX, startY, endX, endY int32, grid finder.Grid, sc *waypointScratch) ([][2]int32, error) {
	// FindClosestWithLOS would fall back to "no node found" for a request that
	// never pointed at the graph at all.
	if !grid.IsInside(startX, startY) || !grid.IsInside(endX, endY) {
		return nil, finder.ErrOutOfBounds
	}
	if !grid.IsWalkableAt(endX, endY) {
		return nil, finder.ErrNotWalkable
	}
	sc.seq++

	startNode := f.graph.FindClosestWithLOS(startX, startY, grid)
	endNode := f.graph.FindClosestWithLOS(endX, endY, grid)
	if startNode == nil || endNode == nil {
		// No usable endpoint: the closest nodes are unreachable from here, which
		// is a completed search with no route, not a failure.
		return nil, nil
	}
	if startNode == endNode {
		// Both endpoints snapped to the same node, but they may not see each
		// other (e.g. a wall between them), so route through the node instead
		// of returning a straight, possibly wall-crossing segment.
		raw := make([][2]int32, 0, 3)
		add := func(x, y int32) {
			if len(raw) > 0 && raw[len(raw)-1] == [2]int32{x, y} {
				return
			}
			raw = append(raw, [2]int32{x, y})
		}
		add(startX, startY)
		add(startNode.X, startNode.Y)
		add(endX, endY)
		return f.SmoothPath(raw, grid), nil
	}

	path := f.findPathInternal(startNode, endNode, startX, startY, endX, endY, sc)
	return f.SmoothPath(path, grid), nil
}

// SmoothPath removes unnecessary waypoints from a path by checking
// line-of-sight between each point. Only points where the direct line
// is blocked by obstacles are kept. The result is written in-place
// over the input buffer.
func (f *WaypointFinder) SmoothPath(path [][2]int32, grid finder.Grid) [][2]int32 {
	if len(path) < 3 {
		return path
	}
	writeIdx := 1
	for i := 2; i < len(path); i++ {
		if !hasLineOfSight(path[writeIdx-1][0], path[writeIdx-1][1],
			path[i][0], path[i][1], grid) {
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

// findPathInternal runs A* between two graph nodes.
func (f *WaypointFinder) findPathInternal(startNode, endNode *WaypointNode, startX, startY, endX, endY int32, sc *waypointScratch) [][2]int32 {
	open := &nodeHeap{nodes: sc.heap[:0]}
	defer func() { sc.heap = open.nodes[:0] }()

	seq := sc.seq

	an := getANode(sc.allMap, startNode)
	an.seq = seq
	an.g = 0
	an.h = heuristic(startNode, endNode)
	an.f = an.h
	an.parent = nil
	an.closed = false
	open.Push(an)

	for open.Len() > 0 {
		cur := open.Pop()
		if cur.closed {
			continue
		}
		cur.closed = true

		if cur.node == endNode {
			// Build node path into pathBuf (reverse, then reverse-back)
			sc.pathBuf = sc.pathBuf[:0]
			for c := cur; c != nil; c = c.parent {
				sc.pathBuf = append(sc.pathBuf, [2]int32{c.node.X, c.node.Y})
			}
			for i, j := 0, len(sc.pathBuf)-1; i < j; i, j = i+1, j-1 {
				sc.pathBuf[i], sc.pathBuf[j] = sc.pathBuf[j], sc.pathBuf[i]
			}

			// Prepend start (skip if same as first waypoint)
			if sc.pathBuf[0][0] != startX || sc.pathBuf[0][1] != startY {
				sc.pathBuf = append(sc.pathBuf, [2]int32{})
				copy(sc.pathBuf[1:], sc.pathBuf[:len(sc.pathBuf)-1])
				sc.pathBuf[0] = [2]int32{startX, startY}
			}
			// Append end (skip if same as last waypoint)
			last := sc.pathBuf[len(sc.pathBuf)-1]
			if last[0] != endX || last[1] != endY {
				sc.pathBuf = append(sc.pathBuf, [2]int32{endX, endY})
			}
			return sc.pathBuf
		}

		for _, edge := range cur.node.Edges {
			if edge.State == EdgeStateNull {
				continue
			}
			ng := cur.g + edge.Cost
			an := getANode(sc.allMap, edge.To)
			if an.seq != seq {
				// First visit this search — initialize fresh
				an.seq = seq
				an.g = ng
				an.h = heuristic(edge.To, endNode)
				an.f = an.g + an.h
				an.parent = cur
				an.closed = false
				open.Push(an)
			} else if !an.closed && ng < an.g {
				// Already visited this search but found better path
				an.g = ng
				an.f = an.g + an.h
				an.parent = cur
				open.Push(an)
			}
		}
	}

	return nil
}

// getANode returns the aNode for a given WaypointNode, creating it lazily.
func getANode(all map[*WaypointNode]*aNode, n *WaypointNode) *aNode {
	an, ok := all[n]
	if !ok {
		an = &aNode{node: n}
		all[n] = an
	}
	return an
}

// heuristic computes Euclidean distance between two waypoint nodes.
func heuristic(a, b *WaypointNode) float32 {
	dx := float32(a.X) - float32(b.X)
	dy := float32(a.Y) - float32(b.Y)
	return float32(math.Sqrt(float64(dx*dx + dy*dy)))
}

// aNode is an internal A* node wrapping a waypoint node, reused across searches.
type aNode struct {
	node    *WaypointNode
	g, h, f float32
	parent  *aNode
	closed  bool
	seq     int64 // matches finder searchSeq to detect stale state
	heapIdx int
}

// nodeHeap is a min-heap for aNode.
type nodeHeap struct {
	nodes []*aNode
}

// Len returns the number of nodes in the heap.
func (h *nodeHeap) Len() int { return len(h.nodes) }

// Push adds a node to the heap and maintains the min-heap invariant.
func (h *nodeHeap) Push(n *aNode) {
	n.heapIdx = len(h.nodes)
	h.nodes = append(h.nodes, n)
	h.siftUp(n.heapIdx)
}

// Pop removes and returns the node with the smallest f value.
func (h *nodeHeap) Pop() *aNode {
	n := len(h.nodes)
	last := h.nodes[n-1]
	h.nodes = h.nodes[:n-1]
	if len(h.nodes) == 0 {
		last.heapIdx = -1
		return last
	}
	root := h.nodes[0]
	h.nodes[0] = last
	last.heapIdx = 0
	root.heapIdx = -1
	h.siftDown(0)
	return root
}

func (h *nodeHeap) siftUp(idx int) {
	for idx > 0 {
		parent := (idx - 1) / 2
		if h.nodes[idx].f >= h.nodes[parent].f {
			break
		}
		h.nodes[idx], h.nodes[parent] = h.nodes[parent], h.nodes[idx]
		h.nodes[idx].heapIdx = idx
		h.nodes[parent].heapIdx = parent
		idx = parent
	}
}

func (h *nodeHeap) siftDown(idx int) {
	n := len(h.nodes)
	for {
		smallest := idx
		left := 2*idx + 1
		right := 2*idx + 2
		if left < n && h.nodes[left].f < h.nodes[smallest].f {
			smallest = left
		}
		if right < n && h.nodes[right].f < h.nodes[smallest].f {
			smallest = right
		}
		if smallest == idx {
			break
		}
		h.nodes[idx], h.nodes[smallest] = h.nodes[smallest], h.nodes[idx]
		h.nodes[idx].heapIdx = idx
		h.nodes[smallest].heapIdx = smallest
		idx = smallest
	}
}
