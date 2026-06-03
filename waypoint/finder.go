package waypoint

import (
	"math"

	"github.com/actfuns/pathfinding/finder"
)

// WaypointFinder implements finder.Finder on a waypoint graph.
//
// It finds the closest waypoint nodes with line-of-sight to the start/end
// positions (via the grid parameter) and runs A* on the graph.
type WaypointFinder struct {
	graph *WaypointGraph

	// Reusable buffers for FindPath (zero-alloc after warm-up)
	searchSeq int
	heapSlice []*aNode
	allMap    map[*WaypointNode]*aNode
	pathBuf   [][2]int
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
		graph:  NewWaypointGraph(),
		allMap: make(map[*WaypointNode]*aNode),
	}
	for _, opt := range opts {
		opt(f)
	}
	return f
}

// Graph returns the underlying waypoint graph for node/edge operations.
func (f *WaypointFinder) Graph() *WaypointGraph { return f.graph }

// FindPath implements finder.Finder by searching the waypoint graph.
// The grid is used for LOS-based node selection — only nodes with a clear
// line of sight from the start/end position are considered.
// Path is automatically smoothed via string pulling to remove redundant waypoints.
func (f *WaypointFinder) FindPath(startX, startY, endX, endY int, grid finder.Grid) [][2]int {
	f.searchSeq++

	startNode := f.graph.FindClosestWithLOS(startX, startY, grid)
	endNode := f.graph.FindClosestWithLOS(endX, endY, grid)
	if startNode == nil || endNode == nil {
		return nil
	}
	if startNode == endNode {
		return [][2]int{{startX, startY}, {endX, endY}}
	}

	path := f.findPathInternal(startNode, endNode, startX, startY, endX, endY)
	return f.SmoothPath(path, grid)
}

// SmoothPath removes unnecessary waypoints from a path by checking
// line-of-sight between each point. Only points where the direct line
// is blocked by obstacles are kept. The result is written in-place
// over the input buffer.
func (f *WaypointFinder) SmoothPath(path [][2]int, grid finder.Grid) [][2]int {
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
func (f *WaypointFinder) findPathInternal(startNode, endNode *WaypointNode, startX, startY, endX, endY int) [][2]int {
	open := &nodeHeap{nodes: f.heapSlice[:0]}
	defer func() { f.heapSlice = open.nodes[:0] }()
	all := f.allMap

	seq := f.searchSeq

	an := getANode(all, startNode)
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
			f.pathBuf = f.pathBuf[:0]
			for c := cur; c != nil; c = c.parent {
				f.pathBuf = append(f.pathBuf, [2]int{c.node.X, c.node.Y})
			}
			for i, j := 0, len(f.pathBuf)-1; i < j; i, j = i+1, j-1 {
				f.pathBuf[i], f.pathBuf[j] = f.pathBuf[j], f.pathBuf[i]
			}

			// Prepend start (skip if same as first waypoint)
			if f.pathBuf[0][0] != startX || f.pathBuf[0][1] != startY {
				f.pathBuf = append(f.pathBuf, [2]int{})
				copy(f.pathBuf[1:], f.pathBuf[:len(f.pathBuf)-1])
				f.pathBuf[0] = [2]int{startX, startY}
			}
			// Append end (skip if same as last waypoint)
			last := f.pathBuf[len(f.pathBuf)-1]
			if last[0] != endX || last[1] != endY {
				f.pathBuf = append(f.pathBuf, [2]int{endX, endY})
			}
			return f.pathBuf
		}

		for _, edge := range cur.node.Edges {
			if edge.State == EdgeStateNull {
				continue
			}
			ng := cur.g + edge.Cost
			an := getANode(all, edge.To)
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
func heuristic(a, b *WaypointNode) float64 {
	dx := a.X - b.X
	dy := a.Y - b.Y
	return math.Sqrt(float64(dx*dx + dy*dy))
}

// aNode is an internal A* node wrapping a waypoint node, reused across searches.
type aNode struct {
	node    *WaypointNode
	g, h, f float64
	parent  *aNode
	closed  bool
	seq     int // matches finder searchSeq to detect stale state
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
