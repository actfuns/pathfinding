package finder

// MinHeap is a binary heap matching the JS `heap` npm package behavior.
type MinHeap struct {
	nodes []*Node
	less  func(a, b *Node) bool
}

func NewMinHeap(less func(a, b *Node) bool) *MinHeap {
	return &MinHeap{
		nodes: make([]*Node, 0, 64),
		less:  less,
	}
}

func (h *MinHeap) Len() int    { return len(h.nodes) }
func (h *MinHeap) Empty() bool { return len(h.nodes) == 0 }

func (h *MinHeap) Push(n *Node) {
	n.HeapIndex = len(h.nodes)
	h.nodes = append(h.nodes, n)
	h.siftDownFrom(len(h.nodes)-1, 0)
}

func (h *MinHeap) Pop() *Node {
	n := len(h.nodes)
	last := h.nodes[n-1]
	h.nodes = h.nodes[:n-1]
	if len(h.nodes) == 0 {
		last.HeapIndex = -1
		return last
	}
	root := h.nodes[0]
	h.nodes[0] = last
	last.HeapIndex = 0
	root.HeapIndex = -1
	h.siftUpFrom(0)
	return root
}

func (h *MinHeap) UpdateItem(node *Node) {
	pos := node.HeapIndex
	if pos < 0 || pos >= len(h.nodes) || h.nodes[pos] != node {
		return
	}
	h.siftDownFrom(pos, 0)
	h.siftUpFrom(pos)
}

func (h *MinHeap) siftDownFrom(pos, start int) {
	for pos > start {
		parent := (pos - 1) >> 1
		if !h.less(h.nodes[pos], h.nodes[parent]) {
			break
		}
		h.swap(pos, parent)
		pos = parent
	}
}

func (h *MinHeap) siftUpFrom(pos int) {
	n := len(h.nodes)
	start := pos
	node := h.nodes[pos]
	child := 2*pos + 1
	for child < n {
		right := child + 1
		if right < n && !h.less(h.nodes[child], h.nodes[right]) {
			child = right
		}
		h.nodes[pos] = h.nodes[child]
		h.nodes[pos].HeapIndex = pos
		pos = child
		child = 2*pos + 1
	}
	h.nodes[pos] = node
	node.HeapIndex = pos
	h.siftDownFrom(pos, start)
}

func (h *MinHeap) swap(i, j int) {
	h.nodes[i], h.nodes[j] = h.nodes[j], h.nodes[i]
	h.nodes[i].HeapIndex = i
	h.nodes[j].HeapIndex = j
}
